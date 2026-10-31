#!/usr/bin/env python3
"""External vision inference server for the Thesis simulator (PLAN.md C.3).

The UE FVisionBridge connects over loopback TCP and sends one request per
drone capture; this server runs a person detector and returns detections.
The sim blocks on the response at a fixed tick (fence philosophy), so this
process being slow only stretches wall-clock time, never sim correctness.

Protocol (all integers big-endian uint32):
  request:  [4B header_len][header JSON][pixel bytes]
            header = {"type": "detect", "drone": int, "tick": int,
                      "width": int, "height": int, "format": "bgra8"}
            pixels = width*height*4 bytes, row-major BGRA.
  response: [4B body_len][body JSON]
            body = {"detections": [{"cls": int, "conf": float,
                                    "x0": f, "y0": f, "x1": f, "y1": f}]}
            bbox coords normalized to [0,1] in image space.

Backends:
  --backend dummy        no model; returns zero detections (pipeline testing).
  --backend ultralytics  YOLO via ultralytics (pip install ultralytics);
                         --model picks the weights (default yolov8n.pt).
                         Only the person class (COCO 0) is returned.

Example:
  python Tools/vision_server.py --port 8765 --backend ultralytics --model yolov8n.pt
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import queue
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import socket
import struct
import sys
import threading


MAX_HEADER_BYTES = 1_000_000
MAX_IMAGE_PIXELS = 16_777_216  # Bound per-request allocation to 64 MiB BGRA.
MAX_IMAGE_SIDE = 8192


def recv_all(conn: socket.socket, n: int) -> bytearray | None:
    if n < 0:
        raise ValueError("negative receive length")
    buf = bytearray(n)
    view = memoryview(buf)
    offset = 0
    while offset < n:
        received = conn.recv_into(view[offset:])
        if not received:
            return None
        offset += received
    return buf


def recv_all_into(conn: socket.socket, buf: bytearray) -> bool:
    """Fill an existing buffer completely, preserving the connection framing."""
    view = memoryview(buf)
    offset = 0
    while offset < len(view):
        received = conn.recv_into(view[offset:])
        if not received:
            return False
        offset += received
    return True


class DummyDetector:
    name = "dummy"

    def detect(self, bgra: bytes, width: int, height: int) -> list[dict]:
        return []


class SerializedDetector:
    """Serialize calls into a stateful detector shared by client threads."""

    def __init__(self, detector):
        self.detector = detector
        self._lock = threading.Lock()

    @property
    def name(self):
        return self.detector.name

    @property
    def supports_reusable_input_buffer(self):
        return getattr(self.detector, "supports_reusable_input_buffer", False)

    def detect(self, bgra: bytes, width: int, height: int) -> list[dict]:
        with self._lock:
            return self.detector.detect(bgra, width, height)


class CudaStreamDetector:
    """An independently owned predictor with its own per-thread CUDA stream."""

    def __init__(self, detector):
        import torch
        self.detector = detector
        self.torch = torch
        self.stream = torch.cuda.Stream(device=detector.device)

    @property
    def name(self):
        return self.detector.name

    supports_reusable_input_buffer = True

    def detect(self, bgra, width, height):
        with self.torch.cuda.stream(self.stream):
            # The underlying detector transfers the output to CPU before return,
            # so each simulator still consumes a completed response synchronously.
            return self.detector.detect(bgra, width, height)


def initialize_process_detector(model, confidence, device, imgsz):
    global process_detector
    process_detector = UltralyticsDetector(model, confidence, device, imgsz)


def process_detect(pixels, width, height):
    return process_detector.detect(pixels, width, height)


class ProcessDetector:
    """One model in one fixed child process; never redispatch to another model."""
    name = "ultralytics-drone-process"
    supports_reusable_input_buffer = True

    def __init__(self, model, confidence, device, imgsz):
        self.executor = ProcessPoolExecutor(max_workers=1,
            mp_context=multiprocessing.get_context('spawn'),
            initializer=initialize_process_detector,
            initargs=(model, confidence, device, imgsz))

    def detect(self, pixels, width, height):
        return self.executor.submit(process_detect, pixels, width, height).result()

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)


class DroneAffinityDetector:
    """Four fixed models for drone indices 0..3 in one Unreal connection."""
    name = "ultralytics-drone-affinity"
    supports_reusable_input_buffer = True

    def __init__(self, detectors, slot):
        if len(detectors) != 4:
            raise ValueError('Drone affinity requires four fixed predictors')
        self.detectors = detectors
        self.slot = slot

    def detect_for_drone(self, drone, pixels, width, height):
        if type(drone) is not int or not 0 <= drone < 4:
            raise ValueError('Drone affinity requires an integer drone index 0..3')
        return self.detectors[drone].detect(pixels, width, height)



class UltralyticsDetector:
    name = "ultralytics"
    # detect() consumes the input synchronously through frombuffer and predict.
    supports_reusable_input_buffer = True

    def __init__(self, model_path: str, conf_threshold: float, device: str,
                 imgsz: int = 640):
        if imgsz <= 0:
            raise ValueError("imgsz must be positive")
        # CUDA requires this before torch import / CUDA context creation.
        os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
        import numpy as np
        import torch

        random.seed(0)
        np.random.seed(0)
        torch.manual_seed(0)
        torch.use_deterministic_algorithms(True, warn_only=False)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cuda.matmul.allow_tf32 = False
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(0)
        self.device = ("cuda:0" if torch.cuda.is_available() else "cpu") if device == "auto" else device
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable in this PyTorch installation")

        # Seed and configure deterministic kernels before constructing the model.
        from ultralytics import YOLO

        self.np = np
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.imgsz = imgsz
        print(f"YOLO model={model_path} device={self.device} imgsz={imgsz} "
              f"conf={conf_threshold} torch={torch.__version__} deterministic=True", flush=True)

    def detect(self, bgra: bytes, width: int, height: int) -> list[dict]:
        img = self.np.frombuffer(bgra, dtype=self.np.uint8).reshape(height, width, 4)
        bgr = img[:, :, :3]  # YOLO expects BGR (OpenCV convention)

        results = self.model.predict(
            bgr, conf=self.conf_threshold, classes=[0],  # COCO 0 = person
            device=self.device, imgsz=self.imgsz, verbose=False)

        out = []
        for r in results:
            if r.boxes is None:
                continue
            # One transfer per result instead of synchronizing the GPU once
            # for every coordinate, confidence and class of every detection.
            boxes = r.boxes.cpu()
            for coords, conf, cls in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()):
                x0, y0, x1, y1 = (float(v) for v in coords)
                if not all(math.isfinite(v) for v in (x0, y0, x1, y1, conf, cls)):
                    raise ValueError("model returned non-finite detection values")
                out.append({
                    "cls": int(cls),
                    "conf": float(conf),
                    "x0": x0 / width, "y0": y0 / height,
                    "x1": x1 / width, "y1": y1 / height,
                })
        # Deterministic response order regardless of model internals.
        out.sort(key=lambda d: (-d["conf"], d["x0"], d["y0"], d["x1"], d["y1"], d["cls"]))
        return out


def send_response(conn: socket.socket, payload: dict) -> None:
    body = json.dumps(payload, allow_nan=False).encode("utf-8")
    conn.sendall(struct.pack(">I", len(body)) + body)


def validate_header(header: object) -> tuple[int, int]:
    if not isinstance(header, dict) or header.get("type") != "detect":
        raise ValueError("expected a detect request object")
    if header.get("format") != "bgra8":
        raise ValueError("only bgra8 pixel format is supported")
    w, h = header.get("width"), header.get("height")
    if type(w) is not int or type(h) is not int:
        raise ValueError("width and height must be integers")
    if not (0 < w <= MAX_IMAGE_SIDE and 0 < h <= MAX_IMAGE_SIDE) or w * h > MAX_IMAGE_PIXELS:
        raise ValueError("image dimensions exceed supported bounds")
    return w, h


def handle_connection(conn: socket.socket, detector, verbose: bool, quiet: bool = False,
                      reuse_pixel_buffer: bool | None = None) -> None:
    n_requests = 0
    if reuse_pixel_buffer is None:
        reuse_pixel_buffer = getattr(detector, "supports_reusable_input_buffer", False) is True
    pixel_buffer = bytearray()
    while True:
        raw_len = recv_all(conn, 4)
        if raw_len is None:
            break
        header_len = struct.unpack(">I", raw_len)[0]
        if header_len == 0 or header_len > MAX_HEADER_BYTES:
            send_response(conn, {"error": "invalid header length"})
            print(f"bad header length {header_len}; closing", file=sys.stderr)
            break

        header_bytes = recv_all(conn, header_len)
        if header_bytes is None:
            break
        try:
            header = json.loads(header_bytes.decode("utf-8"))
            w, h = validate_header(header)
        except (ValueError, UnicodeError) as exc:
            send_response(conn, {"error": f"invalid request: {exc}"})
            break  # Payload size is untrusted; do not reuse this stream.
        pixel_bytes = w * h * 4
        if reuse_pixel_buffer:
            if len(pixel_buffer) != pixel_bytes:
                pixel_buffer = bytearray(pixel_bytes)
            if not recv_all_into(conn, pixel_buffer):
                break
            pixels = pixel_buffer
        else:
            received_pixels = recv_all(conn, pixel_bytes)
            if received_pixels is None:
                break
            pixels = received_pixels

        try:
            if isinstance(detector, DroneAffinityDetector):
                detections = detector.detect_for_drone(header.get('drone'), pixels, w, h)
            else:
                detections = detector.detect(pixels, w, h)
            send_response(conn, {"detections": detections})
        except Exception as exc:
            print(f"inference failed at drone {header.get('drone')} tick {header.get('tick')}: "
                  f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
            send_response(conn, {"error": f"inference failed: {type(exc).__name__}: {exc}"})
            break

        n_requests += 1
        if verbose:
            print(f"drone {header.get('drone')} tick {header.get('tick')}: "
                  f"{len(detections)} detection(s)")
    if not quiet:
        print(f"connection closed after {n_requests} request(s)")


def serve_client(conn: socket.socket, addr, detector, verbose: bool, quiet: bool,
                 reuse_pixel_buffer: bool | None = None) -> None:
    """Serve one persistent client; keep transport errors visible on stderr."""
    try:
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            # Socket pairs used by local protocol tests have no TCP options.
            pass
        if not quiet:
            print(f"client connected: {addr}")
        handle_connection(conn, detector, verbose, quiet, reuse_pixel_buffer)
    except OSError as exc:
        print(f"client transport failure: {exc}", file=sys.stderr)
    finally:
        conn.close()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--backend", choices=["dummy", "ultralytics"], default="dummy")
    p.add_argument("--model", default="yolov8n.pt")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--device", default="cpu", help="cpu | cuda | cuda:0 | auto (CUDA when available)")
    p.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size (default: 640)")
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--quiet", action="store_true", help="Suppress successful connection and request messages.")
    p.add_argument("--no-reuse-pixel-buffer", action="store_true",
                   help="Disable the Ultralytics input-buffer reuse optimization for A/B comparisons.")
    p.add_argument("--max-clients", type=int, default=2,
                   help="Maximum simultaneous persistent simulator clients (default: 2).")
    p.add_argument("--model-replicas", type=int, choices=[1, 2, 3, 8], default=1,
                   help="1 shared, 2 per-client, or 8 fixed child processes (four drones per each of two clients).")
    args = p.parse_args()
    if args.imgsz <= 0:
        p.error("--imgsz must be positive")
    if args.max_clients <= 0:
        p.error("--max-clients must be positive")
    if not math.isfinite(args.conf) or not 0 <= args.conf <= 1:
        p.error("--conf must be between 0 and 1")
    if args.model_replicas == 8 and args.max_clients != 2:
        p.error("Eight fixed drone models require exactly two Unreal clients")
    if args.model_replicas != 8 and args.model_replicas > args.max_clients:
        p.error("--model-replicas cannot exceed --max-clients")
    if args.model_replicas > 1 and (args.backend != "ultralytics" or not args.device.startswith("cuda")):
        p.error("Multiple replicas require an explicit CUDA ultralytics backend")

    if args.model_replicas > 1:
        detector = DummyDetector()  # All inference happens in the fixed children.
    elif args.backend == "ultralytics":
        detector = UltralyticsDetector(args.model, args.conf, args.device, args.imgsz)
    else:
        detector = DummyDetector()
    # Ultralytics' predictor is stateful. Sessions may connect concurrently,
    # but each per-image inference is serialized on the shared model.
    detector = SerializedDetector(detector)
    replica_pool = None
    process_detectors = []
    if args.model_replicas == 8:
        replica_pool = queue.Queue(maxsize=2)
        try:
            for _ in range(8):
                process_detectors.append(ProcessDetector(args.model,args.conf,args.device,args.imgsz))
            # Warm concurrently, then wait for every owned model before listening.
            warmups=[d.executor.submit(process_detect,bytes(args.imgsz*args.imgsz*4),
                                       args.imgsz,args.imgsz) for d in process_detectors]
            for future in warmups:
                future.result(timeout=180)
            for slot in range(2):
                replica_pool.put(DroneAffinityDetector(process_detectors[slot*4:(slot+1)*4],slot))
            print('Ready: eight fixed YOLO processes, four drone models per Unreal client',flush=True)
        except BaseException:
            for d in process_detectors:
                d.close()
            raise
    elif args.model_replicas > 1:
        # A fixed child process/model per active Unreal connection.
        replica_pool = queue.Queue(maxsize=args.model_replicas)
        try:
            process_detectors = [ProcessDetector(args.model, args.conf, args.device, args.imgsz)
                                 for _ in range(args.model_replicas)]
            warmups = [d.executor.submit(process_detect, bytes(args.imgsz * args.imgsz * 4),
                                         args.imgsz, args.imgsz) for d in process_detectors]
            for future in warmups:
                future.result(timeout=180)
            for d in process_detectors:
                replica_pool.put(d)
            print(f"Ready: {args.model_replicas} fixed deterministic YOLO processes", flush=True)
        except BaseException:
            for d in process_detectors:
                d.close()
            raise

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(args.max_clients)
    if not args.quiet:
        print(f"vision server ({detector.name}) listening on {args.host}:{args.port}")

    client_slots = threading.BoundedSemaphore(args.max_clients)
    active_connections: set[socket.socket] = set()
    active_threads: set[threading.Thread] = set()
    active_lock = threading.Lock()

    def client_worker(conn: socket.socket, addr) -> None:
        owned_detector = replica_pool.get() if replica_pool is not None else detector
        if isinstance(owned_detector, DroneAffinityDetector):
            print(f'Client {addr} assigned fixed model slot {owned_detector.slot}; '
                  f'drones 0..3 use models {owned_detector.slot*4}..{owned_detector.slot*4+3}',flush=True)
        try:
            serve_client(conn, addr, owned_detector, args.verbose, args.quiet,
                         not args.no_reuse_pixel_buffer)
        finally:
            if replica_pool is not None:
                replica_pool.put(owned_detector)
            with active_lock:
                active_connections.discard(conn)
                active_threads.discard(threading.current_thread())
            client_slots.release()

    try:
        while True:
            client_slots.acquire()
            try:
                conn, addr = srv.accept()
            except BaseException:
                client_slots.release()
                raise
            worker = threading.Thread(
                target=client_worker,
                args=(conn, addr),
                name=f"vision-client-{addr[0]}:{addr[1]}",
                daemon=True,
            )
            with active_lock:
                active_connections.add(conn)
                active_threads.add(worker)
            try:
                worker.start()
            except BaseException:
                with active_lock:
                    active_connections.discard(conn)
                    active_threads.discard(worker)
                conn.close()
                client_slots.release()
                raise
    except KeyboardInterrupt:
        if not args.quiet:
            print("shutting down")
    finally:
        srv.close()
        with active_lock:
            connections = list(active_connections)
            workers = list(active_threads)
        for conn in connections:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            conn.close()
        for worker in workers:
            worker.join(timeout=2.0)
        for owned_process in process_detectors:
            owned_process.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
