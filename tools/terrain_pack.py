#!/usr/bin/env python3
"""
Pack extracted terrain PNG layers into Battlezone-style HG2, MAT, and LGT files.

Inputs:
- height PNG (8-bit or 16-bit grayscale)
- paint PNG (8-bit grayscale where levels map to 0..15 material indices)
- light PNG (8-bit grayscale)

Outputs:
- .hg2 (13-bit height values in zoned layout)
- .mat (16-bit packed material tile entries, half-res from paint vertices)
- .lgt (border chunk + zoned grayscale lightmap chunks)
"""

from __future__ import annotations

import argparse
import math
import struct
from pathlib import Path

import numpy as np
from PIL import Image


def _load_gray(path: Path) -> np.ndarray:
    img = Image.open(path)
    if img.mode in ("I;16", "I;16B", "I;16L", "I"):
        arr = np.array(img.convert("I;16"), dtype=np.uint16)
    else:
        arr = np.array(img.convert("L"), dtype=np.uint8)
    if arr.ndim != 2:
        raise ValueError(f"{path} is not a single-channel image.")
    return arr


def _resolve_grid(width: int, height: int, zone_size: int, zones_w: int | None, zones_h: int | None) -> tuple[int, int]:
    if zone_size <= 0:
        raise ValueError("--zone-size must be > 0.")
    if zones_w is not None and zones_h is not None:
        if width % zones_w != 0 or height % zones_h != 0:
            raise ValueError("Image dimensions are not divisible by --zones-w/--zones-h.")
        zs_w = width // zones_w
        zs_h = height // zones_h
        if zs_w != zs_h:
            raise ValueError("Non-square zone layout detected from --zones-w/--zones-h.")
        if zs_w != zone_size:
            raise ValueError(f"Computed zone size {zs_w} does not match --zone-size {zone_size}.")
        return zones_w, zones_h
    if width % zone_size != 0 or height % zone_size != 0:
        raise ValueError("Image dimensions are not divisible by --zone-size.")
    return width // zone_size, height // zone_size


def _height_to_u13(arr: np.ndarray, mode: str) -> np.ndarray:
    if mode == "u13":
        out = arr.astype(np.uint32)
    elif mode == "u16":
        out = (arr.astype(np.uint32) // 8)
    elif mode == "u8":
        out = np.round((arr.astype(np.float32) / 255.0) * 8191.0).astype(np.uint32)
    elif mode == "auto":
        if arr.dtype == np.uint8:
            out = np.round((arr.astype(np.float32) / 255.0) * 8191.0).astype(np.uint32)
        else:
            mx = int(arr.max()) if arr.size else 0
            if mx <= 8191:
                out = arr.astype(np.uint32)
            else:
                out = (arr.astype(np.uint32) // 8)
    else:
        raise ValueError(f"Unsupported height mode: {mode}")
    return np.clip(out, 0, 8191).astype(np.uint16)


def _write_hg2(path: Path, u13: np.ndarray, zone_size: int, zones_w: int, zones_h: int) -> None:
    depth = int(math.log2(zone_size))
    if (1 << depth) != zone_size:
        raise ValueError("Zone size must be a power of two for HG2.")
    if u13.shape != (zones_h * zone_size, zones_w * zone_size):
        raise ValueError("HG2 data shape does not match zones and zone size.")

    header = struct.pack("<HHHHHH", 1, depth, zones_w, zones_h, 10, 0)
    out = bytearray(header)

    data = u13.astype("<u2", copy=False)
    for zy in range(zones_h):
        for zx in range(zones_w):
            zone = data[zy * zone_size : (zy + 1) * zone_size, zx * zone_size : (zx + 1) * zone_size]
            out.extend(zone.tobytes())

    path.write_bytes(out)


def _encode_mat_entry(base: int, next_mat: int, cap: int, flip: int, rot: int, variant: int = 0) -> int:
    entry = 0
    entry |= (variant & 0x3)
    entry |= ((rot & 0x3) << 4)
    entry |= ((flip & 0x1) << 6)
    entry |= ((cap & 0x1) << 7)
    entry |= ((next_mat & 0xF) << 8)
    entry |= ((base & 0xF) << 12)
    return entry


def _paint_to_idx(paint: np.ndarray) -> np.ndarray:
    # Extracted paint uses CI4-expanded grayscale levels (0,17,...,255).
    idx = np.rint(paint.astype(np.float32) / 17.0).astype(np.int32)
    return np.clip(idx, 0, 15).astype(np.uint8)


def _build_mat_from_paint(paint_idx: np.ndarray) -> np.ndarray:
    h, w = paint_idx.shape
    if (h % 2) != 0 or (w % 2) != 0:
        raise ValueError("Paint dimensions must be even to build MAT tiles.")
    mat_h, mat_w = h // 2, w // 2
    mat = np.zeros((mat_h, mat_w), dtype=np.uint16)

    for y in range(mat_h):
        for x in range(mat_w):
            colors = [
                int(paint_idx[y * 2, x * 2]),
                int(paint_idx[y * 2, x * 2 + 1]),
                int(paint_idx[y * 2 + 1, x * 2 + 1]),
                int(paint_idx[y * 2 + 1, x * 2]),
            ]
            unique_mats = sorted(set(colors))
            base_mat = unique_mats[0]
            next_mat = unique_mats[-1]

            if base_mat == next_mat:
                mat[y, x] = _encode_mat_entry(base_mat, base_mat, cap=0, flip=0, rot=0, variant=0)
                continue

            mask = 0
            if colors[0] == next_mat:
                mask |= 8
            if colors[1] == next_mat:
                mask |= 4
            if colors[2] == next_mat:
                mask |= 2
            if colors[3] == next_mat:
                mask |= 1

            cap = 0
            flip = 0
            rot = 0

            if mask == 15:
                base_mat = next_mat
                next_mat = base_mat
            elif mask == 8:
                cap = 0
                flip = 0
                rot = 0
            elif mask == 4:
                cap = 0
                flip = 0
                rot = 3
            elif mask == 2:
                cap = 0
                flip = 0
                rot = 2
            elif mask == 1:
                cap = 0
                flip = 0
                rot = 1
            elif mask == 12:
                cap = 0
                flip = 1
                rot = 0
            elif mask == 6:
                cap = 0
                flip = 1
                rot = 3
            elif mask == 3:
                cap = 0
                flip = 1
                rot = 2
            elif mask == 9:
                cap = 0
                flip = 1
                rot = 1
            elif mask == 10:
                cap = 1
                flip = 0
                rot = 0
            elif mask == 5:
                cap = 1
                flip = 1
                rot = 0
            elif mask in (7, 11, 13, 14):
                base_mat, next_mat = next_mat, base_mat
                inv_mask = (~mask) & 15
                if inv_mask == 8:
                    cap = 0
                    flip = 0
                    rot = 0
                elif inv_mask == 4:
                    cap = 0
                    flip = 0
                    rot = 3
                elif inv_mask == 2:
                    cap = 0
                    flip = 0
                    rot = 2
                elif inv_mask == 1:
                    cap = 0
                    flip = 0
                    rot = 1

            mat[y, x] = _encode_mat_entry(base_mat, next_mat, cap, flip, rot, variant=0)

    return mat


def _write_mat(path: Path, mat_data: np.ndarray) -> None:
    path.write_bytes(mat_data.astype("<u2", copy=False).tobytes())


def _write_lgt(path: Path, light: np.ndarray, zone_size: int, zones_w: int, zones_h: int) -> None:
    if light.shape != (zones_h * zone_size, zones_w * zone_size):
        raise ValueError("LGT lightmap shape does not match zones and zone size.")
    if light.dtype != np.uint8:
        light = np.clip(light, 0, 255).astype(np.uint8)

    # Match TextureManager behavior: flip N/S before storage, then write border + chunks.
    img = np.flipud(light)
    border = int(img[0, 0])

    out = bytearray(bytes([border]) * (zone_size * zone_size))
    for zy in range(zones_h):
        for zx in range(zones_w):
            zone = img[zy * zone_size : (zy + 1) * zone_size, zx * zone_size : (zx + 1) * zone_size]
            out.extend(zone.tobytes())
    path.write_bytes(out)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pack extracted terrain PNGs into HG2/MAT/LGT.")
    p.add_argument("--height", required=True, type=Path, help="Height PNG path.")
    p.add_argument("--paint", required=True, type=Path, help="Paint PNG path.")
    p.add_argument("--light", required=True, type=Path, help="Light PNG path.")
    p.add_argument("--outdir", type=Path, default=None, help="Output folder (default: height file folder).")
    p.add_argument("--name", type=str, default=None, help="Base output name (default: height stem).")
    p.add_argument("--zone-size", type=int, default=128, help="Zone side in pixels (default: 128).")
    p.add_argument("--zones-w", type=int, default=None, help="Map width in zones.")
    p.add_argument("--zones-h", type=int, default=None, help="Map height in zones.")
    p.add_argument(
        "--height-mode",
        choices=("auto", "u8", "u16", "u13"),
        default="auto",
        help="Height conversion: auto|u8|u16|u13 (default: auto).",
    )
    p.add_argument("--skip-hg2", action="store_true", help="Do not write HG2 output.")
    p.add_argument("--skip-mat", action="store_true", help="Do not write MAT output.")
    p.add_argument("--skip-lgt", action="store_true", help="Do not write LGT output.")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    for p in (args.height, args.paint, args.light):
        if not p.exists():
            raise FileNotFoundError(f"Input file not found: {p}")

    height = _load_gray(args.height)
    paint = _load_gray(args.paint)
    light = _load_gray(args.light)

    hh, hw = height.shape
    ph, pw = paint.shape
    lh, lw = light.shape
    if (hw, hh) != (pw, ph) or (hw, hh) != (lw, lh):
        raise ValueError(
            f"Input dimensions must match. height={hw}x{hh}, paint={pw}x{ph}, light={lw}x{lh}"
        )

    zones_w, zones_h = _resolve_grid(hw, hh, args.zone_size, args.zones_w, args.zones_h)

    outdir = args.outdir if args.outdir is not None else args.height.parent
    outdir.mkdir(parents=True, exist_ok=True)
    base = args.name if args.name else args.height.stem

    wrote: list[str] = []

    if not args.skip_hg2:
        hg2_path = outdir / f"{base}.hg2"
        h_u13 = _height_to_u13(height, args.height_mode)
        _write_hg2(hg2_path, h_u13, args.zone_size, zones_w, zones_h)
        wrote.append(str(hg2_path))

    if not args.skip_mat:
        mat_path = outdir / f"{base}.mat"
        p_idx = _paint_to_idx(paint.astype(np.uint8, copy=False))
        mat_data = _build_mat_from_paint(p_idx)
        _write_mat(mat_path, mat_data)
        wrote.append(str(mat_path))

    if not args.skip_lgt:
        lgt_path = outdir / f"{base}.lgt"
        _write_lgt(lgt_path, light.astype(np.uint8, copy=False), args.zone_size, zones_w, zones_h)
        wrote.append(str(lgt_path))

    print("Generated:")
    for path in wrote:
        print(f"- {path}")
    print(f"Grid: {zones_w}x{zones_h} zones, zone_size={args.zone_size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
