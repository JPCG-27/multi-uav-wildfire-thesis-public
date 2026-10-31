#!/usr/bin/env python3
"""Download CC0 photoscanned flora from Poly Haven (no account needed).

Fetches FBX models + 2k PBR textures for a curated set of wildfire-biome
assets into RawAssets/polyhaven/<asset>/, ready for Tools/import_polyhaven.py
to import into the project (with Nanite + auto-built foliage materials).

All assets are CC0 (public domain) — see https://polyhaven.com/license.

Usage:
  python Tools/download_flora.py            # download the default set
  python Tools/download_flora.py --res 1k   # smaller textures
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from pathlib import Path

API = "https://api.polyhaven.com/files/{asset}"

# Poly Haven sits behind Cloudflare, which 403s Python's default user agent.
HEADERS = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
                         "AppleWebKit/537.36 (KHTML, like Gecko) Chrome/124.0 Safari/537.36"}


def open_url(url: str, timeout: int):
    req = urllib.request.Request(url, headers=HEADERS)
    return urllib.request.urlopen(req, timeout=timeout)

# Curated set for a dry-summer wildfire biome.
ASSETS = {
    # name: role (used by the import script for naming only)
    "fir_sapling":        "tree",   # 3 young fir variations in one FBX
    "pine_sapling_small": "tree",
    "quiver_tree_02":     "tree",
    "searsia_lucida":     "bush",   # dry Karoo shrub — classic fire fuel
    "tree_stump_01":      "cover",
    "boulder_01":         "cover",
}

# Texture map suffixes worth importing (others like displacement are skipped).
MAP_SUFFIXES = ("diff", "nor_gl", "rough", "alpha", "mask", "ao", "arm", "translucent")


def fetch_json(url: str):
    with open_url(url, timeout=60) as r:
        return json.loads(r.read().decode("utf-8"))


def download(url: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if dest.exists() and dest.stat().st_size > 0:
        print(f"    [skip] {dest.name} (exists)")
        return
    print(f"    {dest.name} ...", end="", flush=True)
    with open_url(url, timeout=300) as r, dest.open("wb") as f:
        while True:
            chunk = r.read(1 << 20)
            if not chunk:
                break
            f.write(chunk)
    print(f" {dest.stat().st_size // 1024} KB")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--res", default="2k", choices=["1k", "2k", "4k"],
                   help="Texture resolution (2k is plenty for 640px drone captures)")
    p.add_argument("--out", type=Path,
                   default=Path(__file__).resolve().parent.parent / "RawAssets" / "polyhaven")
    args = p.parse_args()

    manifest = {}
    for asset, role in ASSETS.items():
        print(f"== {asset} ({role}) ==")
        files = fetch_json(API.format(asset=asset))
        adir = args.out / asset

        # Model: FBX at the chosen texture resolution tier.
        fbx = files.get("fbx", {}).get(args.res, {}).get("fbx")
        if not fbx:
            # some assets only publish one fbx tier
            tiers = files.get("fbx", {})
            fbx = next(iter(tiers.values()))["fbx"] if tiers else None
        if not fbx:
            print(f"    !! no FBX published for {asset}, skipping")
            continue
        download(fbx["url"], adir / f"{asset}.fbx")

        # Textures: every published map whose key ends with a wanted suffix.
        textures = []
        for key, tiers in files.items():
            if not isinstance(tiers, dict) or not any(key.endswith(s) for s in MAP_SUFFIXES):
                continue
            tier = tiers.get(args.res) or next(iter(tiers.values()), None)
            if not isinstance(tier, dict):
                continue
            # tier holds format entries: {"png": {...}} or {"jpg": {...}}
            entry = tier.get("png") or tier.get("jpg") or next(iter(tier.values()), None)
            if isinstance(entry, dict) and "url" in entry:
                ext = entry["url"].rsplit(".", 1)[-1]
                fname = f"{key}.{ext}"
                download(entry["url"], adir / fname)
                textures.append(fname)

        manifest[asset] = {"role": role, "fbx": f"{asset}.fbx", "textures": textures}

    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    (args.out / "LICENSE.txt").write_text(
        "All assets in this folder are from Poly Haven (https://polyhaven.com)\n"
        "and are licensed CC0 (public domain): https://polyhaven.com/license\n",
        encoding="utf-8")
    print(f"\nDone. Manifest: {args.out / 'manifest.json'}")
    print("Next: in the Unreal editor run Tools/import_polyhaven.py "
          "(Tools menu -> Execute Python Script).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
