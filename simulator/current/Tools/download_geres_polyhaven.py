#!/usr/bin/env python3
"""Download CC0 Poly Haven models that replace missing Fab forest assets.

This is a Geres-focused companion to Tools/download_flora.py. It downloads
FBX models plus the textures referenced by each FBX into RawAssets/polyhaven,
then merges them into the manifest used by Tools/import_polyhaven.py.

Good replacements for the Fab assets that often arrive as references only:
  - mossy/granite-like rocks and rock faces
  - fallen trunks, roots, branches, bark debris, stumps
  - ferns, moss, grass, and low shrubs
  - optional pine/fir/tree models if you want canopy variation

Usage:
  python Tools/download_geres_polyhaven.py
  python Tools/download_geres_polyhaven.py --set rocks
  python Tools/download_geres_polyhaven.py --set full --res 1k
  python Tools/download_geres_polyhaven.py --asset rock_moss_set_01 --asset fern_02

Then import in Unreal:
  Tools > Execute Python Script... > Tools/import_polyhaven.py

All assets are CC0 from Poly Haven: https://polyhaven.com/license
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any

API_FILES = "https://api.polyhaven.com/files/{asset}"
API_ASSETS = "https://api.polyhaven.com/assets?t=models"

# Poly Haven sits behind Cloudflare, which can reject Python's default UA.
HEADERS = {
    "User-Agent": (
        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
        "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0 Safari/537.36"
    )
}

RESOLUTION_ORDER = ["1k", "2k", "4k", "8k"]
TEXTURE_EXTS = {".bmp", ".exr", ".hdr", ".jpeg", ".jpg", ".png", ".tga", ".tif", ".tiff"}
MAP_SUFFIXES = ("diff", "nor_gl", "rough", "alpha", "mask", "ao", "arm")


@dataclass(frozen=True)
class AssetSpec:
    asset: str
    role: str
    note: str


GERES_ASSETS: dict[str, AssetSpec] = {
    # Similar to Fab "mossy rock", "granite rock", "Nordic rock cluster".
    "rock_moss_set_01": AssetSpec("rock_moss_set_01", "cover", "mossy rock formation"),
    "rock_moss_set_02": AssetSpec("rock_moss_set_02", "cover", "mossy rock formation"),
    "rock_07": AssetSpec("rock_07", "cover", "small granite-like rock"),
    "rock_09": AssetSpec("rock_09", "cover", "small granite-like rock"),
    "rock_face_01": AssetSpec("rock_face_01", "cover", "exposed rock face"),
    "rock_face_02": AssetSpec("rock_face_02", "cover", "exposed rock face"),
    "mountainside": AssetSpec("mountainside", "cover", "terrain/rock outcrop"),

    # Similar to fallen logs, forest deadwood, roots, and forest floor clutter.
    "dead_tree_trunk": AssetSpec("dead_tree_trunk", "cover", "fallen/dead trunk"),
    "dead_tree_trunk_02": AssetSpec("dead_tree_trunk_02", "cover", "fallen/dead trunk"),
    "dry_branches_medium_01": AssetSpec("dry_branches_medium_01", "cover", "dry branch pile"),
    "pine_roots": AssetSpec("pine_roots", "cover", "exposed roots"),
    "tree_stump_02": AssetSpec("tree_stump_02", "cover", "tree stump"),
    "bark_debris_01": AssetSpec("bark_debris_01", "cover", "bark/leaf debris"),

    # Similar to understory, ferns, moss, and low ground vegetation.
    "fern_02": AssetSpec("fern_02", "bush", "fern understory"),
    "moss_01": AssetSpec("moss_01", "bush", "moss patch"),
    "grass_medium_01": AssetSpec("grass_medium_01", "bush", "forest grass clump"),
    "grass_medium_02": AssetSpec("grass_medium_02", "bush", "forest grass clump"),
    "shrub_02": AssetSpec("shrub_02", "bush", "low shrub"),
    "shrub_03": AssetSpec("shrub_03", "bush", "low shrub"),
    "shrub_04": AssetSpec("shrub_04", "bush", "low shrub"),

    # Optional canopy variation. The Beech pack should stay primary.
    "fir_tree_01": AssetSpec("fir_tree_01", "tree", "evergreen canopy variation"),
    "pine_tree_01": AssetSpec("pine_tree_01", "tree", "pine canopy variation"),
    "fir_sapling_medium": AssetSpec("fir_sapling_medium", "tree", "sapling canopy variation"),
    "pine_sapling_medium": AssetSpec("pine_sapling_medium", "tree", "sapling canopy variation"),
    "tree_small_02": AssetSpec("tree_small_02", "tree", "small broadleaf tree"),
}

ASSET_SETS = {
    "rocks": [
        "rock_moss_set_01", "rock_moss_set_02", "rock_07", "rock_09",
        "rock_face_01", "rock_face_02", "mountainside",
    ],
    "deadwood": [
        "dead_tree_trunk", "dead_tree_trunk_02", "dry_branches_medium_01",
        "pine_roots", "tree_stump_02", "bark_debris_01",
    ],
    "understory": [
        "fern_02", "moss_01", "grass_medium_01", "grass_medium_02",
        "shrub_02", "shrub_03", "shrub_04",
    ],
    "trees": [
        "fir_tree_01", "pine_tree_01", "fir_sapling_medium",
        "pine_sapling_medium", "tree_small_02",
    ],
}
ASSET_SETS["core"] = (
    ASSET_SETS["rocks"][:4]
    + ASSET_SETS["deadwood"]
    + ASSET_SETS["understory"][:5]
)
ASSET_SETS["full"] = (
    ASSET_SETS["rocks"]
    + ASSET_SETS["deadwood"]
    + ASSET_SETS["understory"]
    + ASSET_SETS["trees"]
)


def open_url(url: str, timeout: int):
    request = urllib.request.Request(url, headers=HEADERS)
    return urllib.request.urlopen(request, timeout=timeout)


def fetch_json(url: str, timeout: int = 60) -> dict[str, Any]:
    with open_url(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def download(url: str, dest: Path, timeout: int = 300) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0:
        print(f"    [skip] {dest.name}")
        return

    tmp = dest.with_suffix(dest.suffix + ".part")
    print(f"    {dest.name} ...", end="", flush=True)
    with open_url(url, timeout=timeout) as response, tmp.open("wb") as out:
        while True:
            chunk = response.read(1 << 20)
            if not chunk:
                break
            out.write(chunk)
    tmp.replace(dest)
    print(f" {dest.stat().st_size // 1024} KB")


def choose_resolution(available: dict[str, Any], wanted: str) -> str | None:
    if wanted in available:
        return wanted
    wanted_idx = RESOLUTION_ORDER.index(wanted)
    by_distance = sorted(
        (abs(RESOLUTION_ORDER.index(res) - wanted_idx), res)
        for res in available
        if res in RESOLUTION_ORDER
    )
    return by_distance[0][1] if by_distance else next(iter(available), None)


def choose_fbx(files: dict[str, Any], wanted_res: str) -> tuple[str, dict[str, Any]] | None:
    tiers = files.get("fbx")
    if not isinstance(tiers, dict) or not tiers:
        return None

    res = choose_resolution(tiers, wanted_res)
    if not res:
        return None

    tier = tiers.get(res)
    if isinstance(tier, dict) and isinstance(tier.get("fbx"), dict):
        return res, tier["fbx"]
    return None


def normalized_texture_name(source_name: str, asset: str) -> str | None:
    path = Path(source_name.replace("\\", "/"))
    suffix = path.suffix.lower()
    if suffix not in TEXTURE_EXTS:
        return None

    stem = path.stem.lower()
    stem = re.sub(r"_(1k|2k|4k|8k|16k)$", "", stem)
    stem = re.sub(r"[^a-z0-9_]+", "_", stem).strip("_")

    if not any(stem.endswith("_" + s) or stem == s or s in stem for s in MAP_SUFFIXES):
        return None

    # Keep material-set prefixes for multi-material assets, but remove asset
    # slug noise when Poly Haven repeats it in every texture filename.
    asset_prefix = re.sub(r"[^a-z0-9_]+", "_", asset.lower()).strip("_") + "_"
    if stem.startswith(asset_prefix) and len(stem) > len(asset_prefix):
        stem = stem[len(asset_prefix):]
    return f"{stem}{suffix}"


def texture_includes(fbx_entry: dict[str, Any], asset: str) -> list[tuple[str, str]]:
    include = fbx_entry.get("include", {})
    if not isinstance(include, dict):
        return []

    textures: list[tuple[str, str]] = []
    for source_name, info in sorted(include.items()):
        if not isinstance(info, dict) or "url" not in info:
            continue
        normalized = normalized_texture_name(source_name, asset)
        if normalized:
            textures.append((normalized, info["url"]))
    return textures


def load_manifest(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def save_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")


def selected_assets(args: argparse.Namespace) -> list[str]:
    if args.asset:
        ids = args.asset
    else:
        ids = ASSET_SETS[args.set]

    seen = set()
    ordered = []
    for asset in ids:
        if asset not in seen:
            ordered.append(asset)
            seen.add(asset)
    return ordered


def print_list() -> None:
    for set_name in ["core", "rocks", "deadwood", "understory", "trees", "full"]:
        print(f"\n[{set_name}]")
        for asset in ASSET_SETS[set_name]:
            spec = GERES_ASSETS[asset]
            print(f"  {asset:24s} {spec.role:5s} {spec.note}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", default="core", choices=sorted(ASSET_SETS),
                        help="Curated replacement set to download.")
    parser.add_argument("--asset", action="append",
                        help="Download a specific Poly Haven asset id. Can be repeated.")
    parser.add_argument("--res", default="2k", choices=RESOLUTION_ORDER,
                        help="FBX texture resolution tier. 2k is usually enough.")
    parser.add_argument("--out", type=Path,
                        default=Path(__file__).resolve().parent.parent / "RawAssets" / "polyhaven")
    parser.add_argument("--no-merge", action="store_true",
                        help="Write a fresh manifest instead of merging with the existing one.")
    parser.add_argument("--list", action="store_true",
                        help="Print curated asset sets and exit.")
    parser.add_argument("--check-api", action="store_true",
                        help="Verify selected asset ids exist before downloading.")
    args = parser.parse_args()

    if args.list:
        print_list()
        return 0

    assets = selected_assets(args)
    if args.check_api:
        api_assets = fetch_json(API_ASSETS)
        missing = [asset for asset in assets if asset not in api_assets]
        if missing:
            print("Missing from Poly Haven API: " + ", ".join(missing), file=sys.stderr)
            return 2

    manifest_path = args.out / "manifest.json"
    manifest = {} if args.no_merge else load_manifest(manifest_path)

    failures = 0
    for asset in assets:
        spec = GERES_ASSETS.get(asset, AssetSpec(asset, "cover", "custom asset"))
        print(f"== {asset} ({spec.role}: {spec.note}) ==")
        try:
            files = fetch_json(API_FILES.format(asset=asset))
            chosen = choose_fbx(files, args.res)
            if not chosen:
                print(f"    !! no FBX published for {asset}")
                failures += 1
                continue

            res, fbx_entry = chosen
            asset_dir = args.out / asset
            download(fbx_entry["url"], asset_dir / f"{asset}.fbx")

            textures = []
            for filename, url in texture_includes(fbx_entry, asset):
                download(url, asset_dir / filename)
                textures.append(filename)

            manifest[asset] = {
                "role": spec.role,
                "fbx": f"{asset}.fbx",
                "textures": textures,
                "source": "Poly Haven",
                "polyhaven_asset": asset,
                "resolution": res,
                "note": spec.note,
            }
        except (urllib.error.URLError, TimeoutError, OSError, KeyError, json.JSONDecodeError) as exc:
            print(f"    !! failed: {exc}")
            failures += 1

    save_manifest(manifest_path, manifest)
    (args.out / "LICENSE.txt").write_text(
        "Assets in this folder are from Poly Haven (https://polyhaven.com)\n"
        "and are licensed CC0 (public domain): https://polyhaven.com/license\n",
        encoding="utf-8",
    )

    print(f"\nManifest: {manifest_path}")
    print("Next: in Unreal, run Tools/import_polyhaven.py, then Tools/setup_geres_scene.py.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
