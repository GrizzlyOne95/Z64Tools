#!/usr/bin/env python3
"""
Battlezone 64 (N64) quick extraction helpers.

Current capabilities:
- Scan ROM for candidate BZn64Model headers and mission/script filenames.
- Export a BZn64Model binary blob to OBJ/MTL (+ optional PNG texture).

This is intentionally conservative and aims to be correct for discovered
sample format(s) before broadening to all content archives.
"""

from __future__ import annotations

import argparse
import binascii
from collections import Counter, defaultdict
import dataclasses
import csv
import fnmatch
import math
import json
import hashlib
import os
import pathlib
import re
import struct
import statistics
import wave
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple, cast

try:
    from PIL import Image  # type: ignore
except Exception:  # pragma: no cover
    Image = None

try:
    import yaml  # type: ignore
except Exception:  # pragma: no cover
    yaml = None


MODEL_SIG = bytes.fromhex("D700000280008000")


@dataclasses.dataclass
class Vtx:
    x: int
    y: int
    z: int
    s: int
    t: int
    r: int
    g: int
    b: int
    a: int


@dataclasses.dataclass
class Tri:
    i0: int
    i1: int
    i2: int


TexState = Tuple[int, int, int, int, int, int, int]
UvInfo = Tuple[int, int, int, int]


def _tex_scale_to_div(scale: int) -> int:
    # Map G_TEXTURE scale (16-bit) to an effective divisor for s/t.
    # Common defaults use 0x8000, which maps closer to a 2048 divisor in this
    # microcode path (legacy 4096 produced consistently half-sized UV spans).
    if scale <= 0:
        return 2048
    return max(1, int(round(2048.0 * (0x8000 / float(scale)))))


def _apply_shift_to_div(div: int, shift: int) -> int:
    # RDP shift: 0..10 shift right, 11..15 shift left by (16-shift).
    if shift <= 10:
        return div * (1 << shift)
    return max(1, div // (1 << (16 - shift)))


def _convert_loadblock_texels(texels: int, load_siz: Optional[int], render_siz: Optional[int]) -> int:
    if texels <= 0:
        return texels
    if load_siz is None or render_siz is None:
        return texels
    # siz encodes bits-per-texel as 4,8,16,32 for 0..3.
    load_bpt = 4 << int(load_siz)
    render_bpt = 4 << int(render_siz)
    if load_bpt <= 0 or render_bpt <= 0:
        return texels
    return max(1, int(round(float(texels) * (float(load_bpt) / float(render_bpt)))))


def _tri_uv_area(va: Vtx, vb: Vtx, vc: Vtx, u_div: float, v_div: float, u_off: float, v_off: float) -> float:
    au = (va.s - u_off) / u_div
    av = (va.t - v_off) / v_div
    bu = (vb.s - u_off) / u_div
    bv = (vb.t - v_off) / v_div
    cu = (vc.s - u_off) / u_div
    cv = (vc.t - v_off) / v_div
    return abs((bu - au) * (cv - av) - (cu - au) * (bv - av)) * 0.5


def _tri_uv_areas(verts: Sequence[Vtx], tris: Sequence[Tri], tri_uv_info: Sequence[UvInfo]) -> List[float]:
    areas: List[float] = []
    for i, t in enumerate(tris):
        info = tri_uv_info[i] if i < len(tri_uv_info) else (4096, 4096, 0, 0)
        areas.append(
            _tri_uv_area(
                verts[t.i0],
                verts[t.i1],
                verts[t.i2],
                float(info[0]),
                float(info[1]),
                float(info[2]),
                float(info[3]),
            )
        )
    return areas


def _uv_out_of_range_ratio(
    verts: Sequence[Vtx],
    tris: Sequence[Tri],
    tri_uv_info: Sequence[UvInfo],
    limit: float = 8.0,
) -> float:
    total = 0
    out = 0
    for i, t in enumerate(tris):
        info = tri_uv_info[i] if i < len(tri_uv_info) else (4096, 4096, 0, 0)
        u_div = float(info[0])
        v_div = float(info[1])
        u_off = float(info[2])
        v_off = float(info[3])
        for vi in (t.i0, t.i1, t.i2):
            v = verts[vi]
            u = (v.s - u_off) / u_div
            vv = (v.t - v_off) / v_div
            total += 1
            if abs(u) > limit or abs(vv) > limit:
                out += 1
    if total == 0:
        return 0.0
    return out / total


def be32(b: bytes, off: int) -> int:
    return struct.unpack_from(">I", b, off)[0]


def s16(b: bytes, off: int) -> int:
    return struct.unpack_from(">h", b, off)[0]


def parse_display_list(data: bytes, dl_off: int = 8) -> List[Tuple[int, int, int, int]]:
    cmds: List[Tuple[int, int, int, int]] = []
    for off in range(dl_off, len(data), 8):
        if off + 8 > len(data):
            break
        w0 = be32(data, off)
        w1 = be32(data, off + 4)
        op = (w0 >> 24) & 0xFF
        cmds.append((off, op, w0, w1))
        if op == 0xDF:  # End display list
            break
    return cmds


def _iter_display_list_exec(
    data: bytes,
    dl_off: int = 8,
    max_cmds: int = 200000,
    seg_bases: Optional[Dict[int, int]] = None,
) -> Iterable[Tuple[int, int, int, int]]:
    # Execute nested display lists via G_DL calls so geometry commands are read
    # in the same flow as the RSP command stream.
    stack: List[int] = []
    off = dl_off
    steps = 0
    while 0 <= off <= (len(data) - 8) and steps < max_cmds:
        w0 = be32(data, off)
        w1 = be32(data, off + 4)
        op = (w0 >> 24) & 0xFF
        yield (off, op, w0, w1)
        steps += 1

        if seg_bases is not None and op == 0xDB:  # G_MOVEWORD (segment base updates)
            _update_segment_bases(seg_bases, w0, w1, len(data))

        if op == 0xDE:  # G_DL
            target = _resolve_dl_addr(data, w1, seg_bases=seg_bases)
            if target is not None and 0 <= target <= (len(data) - 8):
                stack.append(off + 8)
                off = target
                continue

        if op == 0xDF:  # EndDL / return
            if stack:
                off = stack.pop()
                continue
            break

        off += 8


def _resolve_segment_offset(data: bytes, seg_addr: int, seg_bases: Optional[Dict[int, int]] = None) -> Optional[int]:
    seg = (seg_addr >> 24) & 0xFF
    off = seg_addr & 0x00FFFFFF
    if seg_bases is not None:
        base = seg_bases.get(seg)
        if base is not None:
            target = base + off
            if 0 <= target < len(data):
                return target
        # Heuristic: if the offset is already in-range, treat this segment as base 0.
        if 0 <= off < len(data):
            seg_bases[seg] = 0
            return off
    # In current captures, segment ids 0x08+ often point into the same
    # decompressed blob; treat them as local when in range.
    if seg in (0x08, 0x09, 0x0A, 0x0B) and 0 <= off < len(data):
        return off
    return None


def _resolve_dl_addr(data: bytes, seg_addr: int, seg_bases: Optional[Dict[int, int]] = None) -> Optional[int]:
    # Prefer strict segment decode first, then best-effort local low-24 fallback.
    off = _resolve_segment_offset(data, seg_addr, seg_bases=seg_bases)
    if off is not None:
        return off
    lo = seg_addr & 0x00FFFFFF
    if 0 <= lo < len(data):
        return lo
    return None


def _update_segment_bases(seg_bases: Dict[int, int], w0: int, w1: int, data_len: int) -> None:
    # gSPSegment is encoded as G_MOVEWORD with index G_MW_SEGMENT (0x06).
    index = (w0 >> 16) & 0xFF
    if index != 0x06:
        return
    seg = (w0 & 0xFFFF) // 4
    if seg < 0 or seg > 0x0F:
        return
    base = w1 & 0x00FFFFFF
    if 0 <= base < data_len:
        seg_bases[seg] = base


def iter_model_headers(rom: bytes) -> Iterable[int]:
    pos = 0
    while True:
        i = rom.find(MODEL_SIG, pos)
        if i < 0:
            break
        yield i
        pos = i + 1


def scan_rom(rom: bytes) -> Dict[str, object]:
    mission_names = sorted({m.group().decode("ascii") for m in re.finditer(rb"[A-Za-z0-9_\-]{3,20}\.bzn", rom)})
    model_offsets: List[int] = []
    for off in iter_model_headers(rom):
        chunk = rom[off : off + 0x4000]
        cmds = parse_display_list(chunk, dl_off=8)
        if not cmds:
            continue
        ops = {c[1] for c in cmds}
        # Require a minimal plausible model display list profile.
        if 0xDF in ops and 0x01 in ops and (0x05 in ops or 0x06 in ops or 0x07 in ops):
            model_offsets.append(off)
    return {
        "model_header_count": len(model_offsets),
        "model_offsets": model_offsets,
        "mission_like_bzn_count": len(mission_names),
        "mission_like_bzn": mission_names,
    }


def find_yay0_headers(rom: bytes) -> List[int]:
    offs: List[int] = []
    pos = 0
    n = len(rom)
    while True:
        i = rom.find(b"Yay0", pos)
        if i < 0:
            break
        if i + 16 <= n:
            dec_size = be32(rom, i + 4)
            link_off = be32(rom, i + 8)
            chunk_off = be32(rom, i + 12)
            if 0 < dec_size <= 0x4000000 and 16 <= link_off < chunk_off < 0x1000000:
                offs.append(i)
        pos = i + 1
    return offs


def yay0_decompress(rom: bytes, off: int) -> Tuple[bytes, int]:
    if rom[off : off + 4] != b"Yay0":
        raise ValueError("Not a Yay0 stream")
    dec_size = be32(rom, off + 4)
    link_off = be32(rom, off + 8)
    chunk_off = be32(rom, off + 12)

    mask_pos = off + 16
    link_pos = off + link_off
    chunk_pos = off + chunk_off
    out = bytearray()

    mask = 0
    bits_left = 0
    n = len(rom)

    while len(out) < dec_size:
        if bits_left == 0:
            if mask_pos + 4 > n:
                raise ValueError("Yay0 mask out of range")
            mask = be32(rom, mask_pos)
            mask_pos += 4
            bits_left = 32

        take_raw = (mask & 0x80000000) != 0
        mask = (mask << 1) & 0xFFFFFFFF
        bits_left -= 1

        if take_raw:
            if chunk_pos >= n:
                raise ValueError("Yay0 chunk out of range")
            out.append(rom[chunk_pos])
            chunk_pos += 1
            continue

        if link_pos + 2 > n:
            raise ValueError("Yay0 link out of range")
        code = (rom[link_pos] << 8) | rom[link_pos + 1]
        link_pos += 2

        count = code >> 12
        dist = (code & 0x0FFF) + 1
        if count == 0:
            if chunk_pos >= n:
                raise ValueError("Yay0 chunk count out of range")
            count = rom[chunk_pos] + 18
            chunk_pos += 1
        else:
            count += 2

        src = len(out) - dist
        if src < 0:
            raise ValueError("Yay0 invalid back-reference")
        for _ in range(count):
            out.append(out[src])
            src += 1
            if len(out) >= dec_size:
                break

    comp_end = max(mask_pos, link_pos, chunk_pos)
    return bytes(out), comp_end - off


def parse_vertices(data: bytes, seg_off: int, count: int) -> List[Vtx]:
    verts: List[Vtx] = []
    for i in range(count):
        o = seg_off + i * 16
        if o + 16 > len(data):
            break
        x = s16(data, o + 0)
        y = s16(data, o + 2)
        z = s16(data, o + 4)
        s = s16(data, o + 8)
        t = s16(data, o + 10)
        r = data[o + 12]
        g = data[o + 13]
        b = data[o + 14]
        a = data[o + 15]
        verts.append(Vtx(x, y, z, s, t, r, g, b, a))
    return verts


def _mtx_identity() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _decode_n64_mtx(data: bytes, off: int) -> List[List[float]]:
    if off < 0 or (off + 64) > len(data):
        raise ValueError("matrix out of range")
    ints = [struct.unpack_from(">h", data, off + (i * 2))[0] for i in range(16)]
    fracs = [struct.unpack_from(">H", data, off + 32 + (i * 2))[0] for i in range(16)]
    vals: List[float] = []
    for i in range(16):
        raw = ((ints[i] & 0xFFFF) << 16) | fracs[i]
        if raw & 0x80000000:
            raw -= 0x100000000
        vals.append(raw / 65536.0)
    return [
        [vals[0], vals[1], vals[2], vals[3]],
        [vals[4], vals[5], vals[6], vals[7]],
        [vals[8], vals[9], vals[10], vals[11]],
        [vals[12], vals[13], vals[14], vals[15]],
    ]


def _apply_mtx(v: Vtx, m: Sequence[Sequence[float]]) -> Vtx:
    x = float(v.x)
    y = float(v.y)
    z = float(v.z)
    tx = (m[0][0] * x) + (m[0][1] * y) + (m[0][2] * z) + m[0][3]
    ty = (m[1][0] * x) + (m[1][1] * y) + (m[1][2] * z) + m[1][3]
    tz = (m[2][0] * x) + (m[2][1] * y) + (m[2][2] * z) + m[2][3]
    return Vtx(int(round(tx)), int(round(ty)), int(round(tz)), v.s, v.t, v.r, v.g, v.b, v.a)


def parse_model(data: bytes, dl_off: int = 8, tri_div2: bool = True) -> Tuple[List[Vtx], List[Tri], Dict[str, int]]:
    seg_bases: Dict[int, int] = {}
    cmds = list(_iter_display_list_exec(data, dl_off=dl_off, seg_bases=seg_bases))
    if not cmds:
        raise ValueError("No display-list commands found")

    all_verts: List[Vtx] = []
    all_tris: List[Tri] = []
    vert_lut: Dict[Tuple[int, int, int, int, int, int, int, int, int], int] = {}
    vtx_loads = 0
    da_count = 0
    de_count = 0
    external_matrix_refs = 0
    external_dl_refs = 0
    unresolved_setimg = 0
    unresolved_setimg = 0

    # RSP vertex cache slots. 0..31 are usually used; keep extra space for safety.
    cache: List[Optional[Vtx]] = [None] * 64
    current_mtx: List[List[float]] = _mtx_identity()
    mtx_stack: List[List[List[float]]] = []

    for _, op, w0, w1 in cmds:
        if op == 0xDA:  # G_MTX
            da_count += 1
            flags = w0 & 0xFF
            # Common setup uses DA380003 (projection load). Applying projection
            # matrix directly to OBJ vertex positions causes severe distortion.
            if (flags & 0x03) == 0x03:
                continue
            mtx_off = _resolve_segment_offset(data, w1, seg_bases=seg_bases)
            if mtx_off is None:
                external_matrix_refs += 1
                continue
            try:
                decoded = _decode_n64_mtx(data, mtx_off)
            except Exception:
                continue
            # Observed values suggest low bit acts as push in this content.
            if (w0 & 1) != 0:
                mtx_stack.append(current_mtx)
            current_mtx = decoded
            continue

        if op == 0xD8:  # POPMTX (best-effort)
            if mtx_stack:
                current_mtx = mtx_stack.pop()
            continue

        if op == 0xDE:  # G_DL
            de_count += 1
            if _resolve_segment_offset(data, w1, seg_bases=seg_bases) is None:
                external_dl_refs += 1
            continue

        if op == 0x01:  # VTX
            # F3DEX2-style decode:
            # n   = bits [19:12]
            # end = bits [11:1]  (slot index after last loaded vert)
            # v0  = end - n
            n = (w0 >> 12) & 0xFF
            end = (w0 & 0x0FFF) >> 1
            v0 = end - n
            if n <= 0 or v0 < 0:
                continue

            voff = _resolve_segment_offset(data, w1, seg_bases=seg_bases)
            if voff is None:
                continue
            block_verts = parse_vertices(data, voff, n)
            for i, v in enumerate(block_verts):
                slot = v0 + i
                if slot < 0 or slot >= len(cache):
                    continue
                key = (v.x, v.y, v.z, v.s, v.t, v.r, v.g, v.b, v.a)
                cache[slot] = v
            vtx_loads += 1
            continue

        if op == 0x05:  # TRI1
            if tri_div2:
                a = ((w0 >> 16) & 0xFF) // 2
                b = ((w0 >> 8) & 0xFF) // 2
                c = (w0 & 0xFF) // 2
            else:
                a = (w0 >> 16) & 0xFF
                b = (w0 >> 8) & 0xFF
                c = w0 & 0xFF
            if a >= len(cache) or b >= len(cache) or c >= len(cache):
                continue
            va = cache[a]
            vb = cache[b]
            vc = cache[c]
            if va is None or vb is None or vc is None:
                continue
            tv = [_apply_mtx(va, current_mtx), _apply_mtx(vb, current_mtx), _apply_mtx(vc, current_mtx)]
            idxs: List[int] = []
            for vv in tv:
                key = (vv.x, vv.y, vv.z, vv.s, vv.t, vv.r, vv.g, vv.b, vv.a)
                gi = vert_lut.get(key)
                if gi is None:
                    gi = len(all_verts)
                    all_verts.append(vv)
                    vert_lut[key] = gi
                idxs.append(gi)
            all_tris.append(Tri(idxs[0], idxs[1], idxs[2]))
            continue

        if op == 0x06:  # TRI2 (two triangles packed)
            if tri_div2:
                a0 = ((w0 >> 16) & 0xFF) // 2
                b0 = ((w0 >> 8) & 0xFF) // 2
                c0 = (w0 & 0xFF) // 2
                a1 = ((w1 >> 16) & 0xFF) // 2
                b1 = ((w1 >> 8) & 0xFF) // 2
                c1 = (w1 & 0xFF) // 2
            else:
                a0 = (w0 >> 16) & 0xFF
                b0 = (w0 >> 8) & 0xFF
                c0 = w0 & 0xFF
                a1 = (w1 >> 16) & 0xFF
                b1 = (w1 >> 8) & 0xFF
                c1 = w1 & 0xFF
            for a, b, c in ((a0, b0, c0), (a1, b1, c1)):
                if a >= len(cache) or b >= len(cache) or c >= len(cache):
                    continue
                va = cache[a]
                vb = cache[b]
                vc = cache[c]
                if va is None or vb is None or vc is None:
                    continue
                tv = [_apply_mtx(va, current_mtx), _apply_mtx(vb, current_mtx), _apply_mtx(vc, current_mtx)]
                idxs: List[int] = []
                for vv in tv:
                    key = (vv.x, vv.y, vv.z, vv.s, vv.t, vv.r, vv.g, vv.b, vv.a)
                    gi = vert_lut.get(key)
                    if gi is None:
                        gi = len(all_verts)
                        all_verts.append(vv)
                        vert_lut[key] = gi
                    idxs.append(gi)
                all_tris.append(Tri(idxs[0], idxs[1], idxs[2]))
            continue

        if op == 0x07:  # QUAD (best-effort: two triangles)
            if tri_div2:
                a = ((w0 >> 16) & 0xFF) // 2
                b = ((w0 >> 8) & 0xFF) // 2
                c = (w0 & 0xFF) // 2
                d = ((w1 >> 16) & 0xFF) // 2
            else:
                a = (w0 >> 16) & 0xFF
                b = (w0 >> 8) & 0xFF
                c = w0 & 0xFF
                d = (w1 >> 16) & 0xFF
            for i0, i1, i2 in ((a, b, c), (a, c, d)):
                if i0 >= len(cache) or i1 >= len(cache) or i2 >= len(cache):
                    continue
                va = cache[i0]
                vb = cache[i1]
                vc = cache[i2]
                if va is None or vb is None or vc is None:
                    continue
                tv = [_apply_mtx(va, current_mtx), _apply_mtx(vb, current_mtx), _apply_mtx(vc, current_mtx)]
                idxs: List[int] = []
                for vv in tv:
                    key = (vv.x, vv.y, vv.z, vv.s, vv.t, vv.r, vv.g, vv.b, vv.a)
                    gi = vert_lut.get(key)
                    if gi is None:
                        gi = len(all_verts)
                        all_verts.append(vv)
                        vert_lut[key] = gi
                    idxs.append(gi)
                all_tris.append(Tri(idxs[0], idxs[1], idxs[2]))
            continue

        if op == 0xBE:  # G_CULLDL (ignored)
            continue

    stats: Dict[str, int] = {
        "tri_count": 0,
        "vert_count": 0,
        "vtx_loads": vtx_loads,
        "da_count": da_count,
        "de_count": de_count,
        "external_matrix_refs": external_matrix_refs,
        "external_dl_refs": external_dl_refs,
    }
    stats["tri_count"] = len(all_tris)
    stats["vert_count"] = len(all_verts)
    return all_verts, all_tris, stats


def parse_model_with_texstates(
    data: bytes,
    dl_off: int = 8,
    tri_div2: bool = True,
    seg_bases_init: Optional[Dict[int, int]] = None,
) -> Tuple[List[Vtx], List[Tri], Dict[str, int], List[Optional[TexState]], List[UvInfo]]:
    seg_bases: Dict[int, int] = dict(seg_bases_init or {})
    cmds = list(_iter_display_list_exec(data, dl_off=dl_off, seg_bases=seg_bases))
    if not cmds:
        raise ValueError("No display-list commands found")

    all_verts: List[Vtx] = []
    all_tris: List[Tri] = []
    tri_states: List[Optional[TexState]] = []
    tri_uv_info: List[UvInfo] = []
    vert_lut: Dict[Tuple[int, int, int, int, int, int, int, int, int], int] = {}
    vtx_loads = 0
    da_count = 0
    de_count = 0
    external_matrix_refs = 0
    external_dl_refs = 0
    unresolved_setimg = 0

    cache: List[Optional[Vtx]] = [None] * 64
    current_mtx: List[List[float]] = _mtx_identity()
    mtx_stack: List[List[List[float]]] = []

    current_pal: Optional[int] = None
    current_img: Optional[int] = None
    current_fmt: Optional[int] = None
    current_siz: Optional[int] = None
    current_pal_bank = 0
    current_wh: Tuple[int, int] = (32, 32)
    base_uv_div: Tuple[int, int] = (4096, 4096)
    current_uv_div: Tuple[int, int] = (4096, 4096)
    current_uv_off: Tuple[int, int] = (0, 0)
    current_tlut_entries: Optional[int] = None
    last_tlut_candidate: Optional[int] = None
    tile_fmt: List[Optional[int]] = [None] * 8
    tile_siz: List[Optional[int]] = [None] * 8
    tile_pal_bank: List[int] = [0] * 8
    tile_tmem: List[int] = [0] * 8
    tile_line: List[int] = [0] * 8
    tile_wh: List[Optional[Tuple[int, int]]] = [None] * 8
    tile_load_texels: List[Optional[int]] = [None] * 8
    tile_load_siz: List[Optional[int]] = [None] * 8
    tile_load_img: List[Optional[int]] = [None] * 8
    tile_shift_s: List[int] = [0] * 8
    tile_shift_t: List[int] = [0] * 8
    active_render_tile = 0
    texture_tile_locked = False

    def _resolve_state_img() -> Optional[int]:
        # Prefer TMEM-aware mapping from load tile image -> render tile sub-image.
        # This is a lightweight approximation and not a full TMEM emulator.
        render = int(active_render_tile)
        render_tmem = int(tile_tmem[render]) if 0 <= render < 8 else 0
        current = current_img

        # Commonly, tile 7 is load tile and non-7 tile is render tile.
        for load_tile in (7, render):
            if not (0 <= load_tile < 8):
                continue
            base = tile_load_img[load_tile]
            if base is None:
                continue
            load_tmem = int(tile_tmem[load_tile])
            delta_words = render_tmem - load_tmem
            if delta_words < 0:
                continue
            # TMEM address is in 64-bit words.
            img = int(base) + (delta_words * 8)
            if 0 <= img < len(data):
                return img
        return current

    def _make_state() -> Optional[TexState]:
        pal = current_pal
        if pal is None and current_fmt == 2 and last_tlut_candidate is not None:
            pal = last_tlut_candidate
        state_img = _resolve_state_img()
        if state_img is None or current_fmt is None or current_siz is None:
            return None
        pal_bank = current_pal_bank
        if current_fmt == 2 and current_siz == 0 and current_tlut_entries is not None and current_tlut_entries <= 16:
            pal_bank = 0
        return (
            -1 if pal is None else int(pal),
            int(state_img),
            int(current_wh[0]),
            int(current_wh[1]),
            int(current_fmt),
            int(current_siz),
            int(pal_bank),
        )

    for _, op, w0, w1 in cmds:
        if op == 0xDA:  # G_MTX
            da_count += 1
            flags = w0 & 0xFF
            if (flags & 0x03) == 0x03:
                continue
            mtx_off = _resolve_segment_offset(data, w1, seg_bases=seg_bases)
            if mtx_off is None:
                external_matrix_refs += 1
                continue
            try:
                decoded = _decode_n64_mtx(data, mtx_off)
            except Exception:
                continue
            if (w0 & 1) != 0:
                mtx_stack.append(current_mtx)
            current_mtx = decoded
            continue

        if op == 0xD8:  # POPMTX
            if mtx_stack:
                current_mtx = mtx_stack.pop()
            continue

        if op == 0xDE:  # G_DL
            de_count += 1
            if _resolve_segment_offset(data, w1, seg_bases=seg_bases) is None:
                external_dl_refs += 1
            continue

        if op == 0xFD:  # G_SETTIMG
            addr = _resolve_dl_addr(data, w1, seg_bases=seg_bases)
            if addr is None:
                unresolved_setimg += 1
                continue
            fmt = (w0 >> 21) & 0x7
            siz = (w0 >> 19) & 0x3
            if fmt == 0 and siz == 2 and (addr + 32) <= len(data):
                last_tlut_candidate = addr
            current_img = addr
            continue

        if op == 0xF0:  # G_LOADTLUT
            if last_tlut_candidate is not None:
                current_pal = last_tlut_candidate
            tlut_entries = ((w1 >> 14) & 0x3FF) + 1
            if tlut_entries > 0:
                current_tlut_entries = int(tlut_entries)
            continue

        if op == 0xF5:  # G_SETTILE
            tile = (w1 >> 24) & 0x7
            tile_fmt[tile] = (w0 >> 21) & 0x7
            tile_siz[tile] = (w0 >> 19) & 0x3
            tile_line[tile] = (w0 >> 9) & 0x1FF
            tile_tmem[tile] = w0 & 0x1FF
            tile_pal_bank[tile] = (w1 >> 20) & 0xF
            tile_shift_t[tile] = (w1 >> 10) & 0xF
            tile_shift_s[tile] = w1 & 0xF
            if not texture_tile_locked and tile != 7:
                active_render_tile = tile
            if tile == active_render_tile:
                current_fmt = tile_fmt[tile]
                current_siz = tile_siz[tile]
                current_pal_bank = tile_pal_bank[tile]
                if tile_wh[tile] is not None:
                    current_wh = tile_wh[tile]  # type: ignore[assignment]
                elif tile_load_texels[tile] is not None:
                    guessed_texels = _convert_loadblock_texels(
                        int(tile_load_texels[tile]),
                        tile_load_siz[tile],
                        tile_siz[tile],
                    )
                    guess = _guess_wh_from_texels(guessed_texels)
                    if guess is not None:
                        tile_wh[tile] = guess
                        current_wh = guess
                current_uv_div = (
                    _apply_shift_to_div(base_uv_div[0], tile_shift_s[tile]),
                    _apply_shift_to_div(base_uv_div[1], tile_shift_t[tile]),
                )
            continue

        if op == 0xF2:  # G_SETTILESIZE
            # Tile index is encoded in w1[26:24].
            tile = (w1 >> 24) & 0x7
            uls, ult, lrs, lrt = _decode_settilesize_range(w0, w1)
            wh = _decode_settilesize_wh(w0, w1)
            if wh is not None:
                tile_wh[tile] = wh
            if tile == active_render_tile:
                # Convert 10.2 texel offsets to 5-bit frac space (x8).
                current_uv_off = (uls * 8, ult * 8)
                if wh is not None:
                    current_wh = wh
                current_uv_div = (
                    _apply_shift_to_div(base_uv_div[0], tile_shift_s[tile]),
                    _apply_shift_to_div(base_uv_div[1], tile_shift_t[tile]),
                )
            continue

        if op == 0xF3:  # G_LOADBLOCK
            tile = (w1 >> 24) & 0x7
            texels = ((w1 >> 12) & 0x0FFF) + 1
            if texels > 0:
                tile_load_texels[tile] = texels
                tile_load_siz[tile] = tile_siz[tile]
                tile_load_img[tile] = current_img
                if tile == active_render_tile and tile_wh[tile] is None:
                    guessed_texels = _convert_loadblock_texels(texels, tile_load_siz[tile], tile_siz[tile])
                    guess = _guess_wh_from_texels(guessed_texels)
                    if guess is not None:
                        tile_wh[tile] = guess
                        current_wh = guess
            continue

        if op == 0xD7:  # G_TEXTURE
            s_scale = (w1 >> 16) & 0xFFFF
            t_scale = w1 & 0xFFFF
            active_render_tile = (w0 >> 8) & 0x7
            texture_tile_locked = True
            if tile_fmt[active_render_tile] is not None:
                current_fmt = tile_fmt[active_render_tile]
                current_siz = tile_siz[active_render_tile]
                current_pal_bank = tile_pal_bank[active_render_tile]
            base_uv_div = (_tex_scale_to_div(s_scale), _tex_scale_to_div(t_scale))
            if tile_wh[active_render_tile] is not None:
                tw, th = tile_wh[active_render_tile]  # type: ignore[misc]
                current_wh = (tw, th)
            current_uv_div = (
                _apply_shift_to_div(base_uv_div[0], tile_shift_s[active_render_tile]),
                _apply_shift_to_div(base_uv_div[1], tile_shift_t[active_render_tile]),
            )
            continue

        if op == 0x01:  # VTX
            n = (w0 >> 12) & 0xFF
            end = (w0 & 0x0FFF) >> 1
            v0 = end - n
            if n <= 0 or v0 < 0:
                continue
            voff = _resolve_segment_offset(data, w1, seg_bases=seg_bases)
            if voff is None:
                continue
            block_verts = parse_vertices(data, voff, n)
            for i, v in enumerate(block_verts):
                slot = v0 + i
                if 0 <= slot < len(cache):
                    cache[slot] = v
            vtx_loads += 1
            continue

        if op == 0x05:  # TRI1
            if tri_div2:
                a = ((w0 >> 16) & 0xFF) // 2
                b = ((w0 >> 8) & 0xFF) // 2
                c = (w0 & 0xFF) // 2
            else:
                a = (w0 >> 16) & 0xFF
                b = (w0 >> 8) & 0xFF
                c = w0 & 0xFF
            if a >= len(cache) or b >= len(cache) or c >= len(cache):
                continue
            va = cache[a]
            vb = cache[b]
            vc = cache[c]
            if va is None or vb is None or vc is None:
                continue
            tv = [_apply_mtx(va, current_mtx), _apply_mtx(vb, current_mtx), _apply_mtx(vc, current_mtx)]
            idxs: List[int] = []
            for vv in tv:
                key = (vv.x, vv.y, vv.z, vv.s, vv.t, vv.r, vv.g, vv.b, vv.a)
                gi = vert_lut.get(key)
                if gi is None:
                    gi = len(all_verts)
                    all_verts.append(vv)
                    vert_lut[key] = gi
                idxs.append(gi)
            all_tris.append(Tri(idxs[0], idxs[1], idxs[2]))
            tri_states.append(_make_state())
            tri_uv_info.append((current_uv_div[0], current_uv_div[1], current_uv_off[0], current_uv_off[1]))
            continue

        if op == 0x06:  # TRI2
            if tri_div2:
                a0 = ((w0 >> 16) & 0xFF) // 2
                b0 = ((w0 >> 8) & 0xFF) // 2
                c0 = (w0 & 0xFF) // 2
                a1 = ((w1 >> 16) & 0xFF) // 2
                b1 = ((w1 >> 8) & 0xFF) // 2
                c1 = (w1 & 0xFF) // 2
            else:
                a0 = (w0 >> 16) & 0xFF
                b0 = (w0 >> 8) & 0xFF
                c0 = w0 & 0xFF
                a1 = (w1 >> 16) & 0xFF
                b1 = (w1 >> 8) & 0xFF
                c1 = w1 & 0xFF
            for a, b, c in ((a0, b0, c0), (a1, b1, c1)):
                if a >= len(cache) or b >= len(cache) or c >= len(cache):
                    continue
                va = cache[a]
                vb = cache[b]
                vc = cache[c]
                if va is None or vb is None or vc is None:
                    continue
                tv = [_apply_mtx(va, current_mtx), _apply_mtx(vb, current_mtx), _apply_mtx(vc, current_mtx)]
                idxs: List[int] = []
                for vv in tv:
                    key = (vv.x, vv.y, vv.z, vv.s, vv.t, vv.r, vv.g, vv.b, vv.a)
                    gi = vert_lut.get(key)
                    if gi is None:
                        gi = len(all_verts)
                        all_verts.append(vv)
                        vert_lut[key] = gi
                    idxs.append(gi)
                all_tris.append(Tri(idxs[0], idxs[1], idxs[2]))
                tri_states.append(_make_state())
                tri_uv_info.append((current_uv_div[0], current_uv_div[1], current_uv_off[0], current_uv_off[1]))
            continue

        if op == 0x07:  # QUAD (best-effort: two triangles)
            if tri_div2:
                a = ((w0 >> 16) & 0xFF) // 2
                b = ((w0 >> 8) & 0xFF) // 2
                c = (w0 & 0xFF) // 2
                d = ((w1 >> 16) & 0xFF) // 2
            else:
                a = (w0 >> 16) & 0xFF
                b = (w0 >> 8) & 0xFF
                c = w0 & 0xFF
                d = (w1 >> 16) & 0xFF
            for i0, i1, i2 in ((a, b, c), (a, c, d)):
                if i0 >= len(cache) or i1 >= len(cache) or i2 >= len(cache):
                    continue
                va = cache[i0]
                vb = cache[i1]
                vc = cache[i2]
                if va is None or vb is None or vc is None:
                    continue
                tv = [_apply_mtx(va, current_mtx), _apply_mtx(vb, current_mtx), _apply_mtx(vc, current_mtx)]
                idxs: List[int] = []
                for vv in tv:
                    key = (vv.x, vv.y, vv.z, vv.s, vv.t, vv.r, vv.g, vv.b, vv.a)
                    gi = vert_lut.get(key)
                    if gi is None:
                        gi = len(all_verts)
                        all_verts.append(vv)
                        vert_lut[key] = gi
                    idxs.append(gi)
                all_tris.append(Tri(idxs[0], idxs[1], idxs[2]))
                tri_states.append(_make_state())
                tri_uv_info.append((current_uv_div[0], current_uv_div[1], current_uv_off[0], current_uv_off[1]))
            continue

        if op == 0xBE:  # G_CULLDL (ignored)
            continue

    stats: Dict[str, int] = {
        "tri_count": len(all_tris),
        "vert_count": len(all_verts),
        "vtx_loads": vtx_loads,
        "da_count": da_count,
        "de_count": de_count,
        "external_matrix_refs": external_matrix_refs,
        "external_dl_refs": external_dl_refs,
        "unresolved_setimg": unresolved_setimg,
    }
    return all_verts, all_tris, stats, tri_states, tri_uv_info


def _mesh_topology_penalty(verts: Sequence[Vtx], tris: Sequence[Tri]) -> float:
    if not verts or not tris:
        return 1e9
    deg = 0
    edge_use: Dict[Tuple[int, int], int] = {}
    for t in tris:
        idx = (t.i0, t.i1, t.i2)
        if idx[0] == idx[1] or idx[1] == idx[2] or idx[0] == idx[2]:
            deg += 1
        for a, b in ((idx[0], idx[1]), (idx[1], idx[2]), (idx[2], idx[0])):
            e = (a, b) if a < b else (b, a)
            edge_use[e] = edge_use.get(e, 0) + 1
    n_edges = max(1, len(edge_use))
    edge_single = sum(1 for c in edge_use.values() if c == 1) / n_edges
    edge_over = sum(1 for c in edge_use.values() if c > 2) / n_edges
    deg_r = deg / max(1, len(tris))
    spaghetti = _mesh_spaghetti_metrics(verts, tris)
    long_r = float(spaghetti["long_edge_ratio"])
    return (4.0 * deg_r) + (2.4 * edge_over) + (1.4 * long_r) + (0.8 * edge_single)


def mesh_quality_ok(verts: Sequence[Vtx], tris: Sequence[Tri]) -> bool:
    if len(verts) < 3 or len(tris) < 1:
        return False

    # Degenerate triangle ratio must stay low.
    deg = 0
    for t in tris:
        if t.i0 == t.i1 or t.i1 == t.i2 or t.i0 == t.i2:
            deg += 1
    if deg / max(1, len(tris)) > 0.05:
        return False

    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    ex = max(xs) - min(xs)
    ey = max(ys) - min(ys)
    ez = max(zs) - min(zs)
    # Reject collapsed or absurd extents.
    if ex <= 0 or ey <= 0 or ez <= 0:
        return False
    if max(ex, ey, ez) > 200000:
        return False

    return True


def _mesh_spaghetti_metrics(verts: Sequence[Vtx], tris: Sequence[Tri]) -> Dict[str, float]:
    if not verts or not tris:
        return {"diag": 0.0, "long_edge_ratio": 1.0, "stretch": 1e9}

    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    ex = float(max(xs) - min(xs))
    ey = float(max(ys) - min(ys))
    ez = float(max(zs) - min(zs))
    diag = (ex * ex + ey * ey + ez * ez) ** 0.5
    mn = min(ex, ey, ez)
    mx = max(ex, ey, ez)
    stretch = (mx / max(1.0, mn))

    if diag <= 0.0:
        return {"diag": diag, "long_edge_ratio": 1.0, "stretch": stretch}

    long_edges = 0
    edge_cnt = 0
    thr = 0.75 * diag
    for t in tris:
        ids = [t.i0, t.i1, t.i2]
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a = verts[ids[i]]
            b = verts[ids[j]]
            dx = float(a.x - b.x)
            dy = float(a.y - b.y)
            dz = float(a.z - b.z)
            d = (dx * dx + dy * dy + dz * dz) ** 0.5
            if d > thr:
                long_edges += 1
            edge_cnt += 1
    long_edge_ratio = (long_edges / edge_cnt) if edge_cnt else 1.0
    return {"diag": diag, "long_edge_ratio": long_edge_ratio, "stretch": stretch}


def mesh_quality_strict_ok(verts: Sequence[Vtx], tris: Sequence[Tri]) -> bool:
    if not mesh_quality_ok(verts, tris):
        return False
    m = _mesh_spaghetti_metrics(verts, tris)
    if float(m["long_edge_ratio"]) > 0.08:
        return False
    if float(m["stretch"]) > 96.0:
        return False
    return True


def _compact_mesh(verts: Sequence[Vtx], tris: Sequence[Tri]) -> Tuple[List[Vtx], List[Tri]]:
    used = sorted({i for t in tris for i in (t.i0, t.i1, t.i2)})
    remap = {old: new for new, old in enumerate(used)}
    out_verts = [verts[i] for i in used]
    out_tris = [Tri(remap[t.i0], remap[t.i1], remap[t.i2]) for t in tris]
    return out_verts, out_tris


def _compact_mesh_with_states(
    verts: Sequence[Vtx],
    tris: Sequence[Tri],
    tri_states: Sequence[Optional[TexState]],
) -> Tuple[List[Vtx], List[Tri], List[Optional[TexState]]]:
    used = sorted({i for t in tris for i in (t.i0, t.i1, t.i2)})
    remap = {old: new for new, old in enumerate(used)}
    out_verts = [verts[i] for i in used]
    out_tris = [Tri(remap[t.i0], remap[t.i1], remap[t.i2]) for t in tris]
    out_states = list(tri_states[: len(out_tris)])
    return out_verts, out_tris, out_states


def _prune_outlier_tris(verts: Sequence[Vtx], tris: Sequence[Tri], mode: str = "full") -> Tuple[List[Vtx], List[Tri], int]:
    if not verts or not tris:
        return list(verts), list(tris), 0
    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    ex = float(max(xs) - min(xs))
    ey = float(max(ys) - min(ys))
    ez = float(max(zs) - min(zs))
    diag = (ex * ex + ey * ey + ez * ez) ** 0.5
    if diag <= 0.0:
        return list(verts), list(tris), 0

    max_edges: List[float] = []
    tri_areas: List[float] = []
    tri_aspects: List[float] = []
    tri_metrics: List[Tuple[Tri, float, float, float]] = []
    for t in tris:
        ids = [t.i0, t.i1, t.i2]
        edges: List[float] = []
        pts = [verts[ids[0]], verts[ids[1]], verts[ids[2]]]
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a = verts[ids[i]]
            b = verts[ids[j]]
            dx = float(a.x - b.x)
            dy = float(a.y - b.y)
            dz = float(a.z - b.z)
            edges.append((dx * dx + dy * dy + dz * dz) ** 0.5)
        me = max(edges)
        mine = max(1e-6, min(edges))
        aspect = me / mine
        ux = float(pts[1].x - pts[0].x)
        uy = float(pts[1].y - pts[0].y)
        uz = float(pts[1].z - pts[0].z)
        vx = float(pts[2].x - pts[0].x)
        vy = float(pts[2].y - pts[0].y)
        vz = float(pts[2].z - pts[0].z)
        cx = (uy * vz) - (uz * vy)
        cy = (uz * vx) - (ux * vz)
        cz = (ux * vy) - (uy * vx)
        area = 0.5 * ((cx * cx + cy * cy + cz * cz) ** 0.5)
        max_edges.append(me)
        tri_areas.append(area)
        tri_aspects.append(aspect)
        tri_metrics.append((t, me, area, aspect))

    if not max_edges:
        return list(verts), list(tris), 0
    if mode == "degenerate":
        kept = [t for (t, _me, area, aspect) in tri_metrics if area > 1e-4 and aspect < 1e6]
        removed = len(tris) - len(kept)
        if removed <= 0:
            return list(verts), list(tris), 0
        out_verts, out_tris = _compact_mesh(verts, kept)
        return out_verts, out_tris, removed
    se = sorted(max_edges)
    sa = sorted(tri_areas)
    ss = sorted(tri_aspects)
    med_e = statistics.median(max_edges)
    med_a = statistics.median(tri_areas)
    p95_e = se[min(len(se) - 1, int(0.95 * len(se)))]
    p98_a = sa[min(len(sa) - 1, int(0.98 * len(sa)))]
    p95_s = ss[min(len(ss) - 1, int(0.95 * len(ss)))]
    edge_thresh = min(0.70 * diag, max(3.5 * med_e, 1.4 * p95_e))
    area_thresh = max(8.0 * med_a, 1.25 * p98_a)
    aspect_thresh = max(14.0, 1.4 * p95_s)
    kept = [t for (t, me, area, aspect) in tri_metrics if (me <= edge_thresh and area <= area_thresh and aspect <= aspect_thresh)]
    if len(kept) < max(1, len(tris) // 3):
        # Guardrail: if thresholds are too aggressive, keep original mesh.
        return list(verts), list(tris), 0
    removed = len(tris) - len(kept)
    if removed <= 0:
        return list(verts), list(tris), 0
    out_verts, out_tris = _compact_mesh(verts, kept)
    return out_verts, out_tris, removed


def _prune_outlier_tris_with_states(
    verts: Sequence[Vtx],
    tris: Sequence[Tri],
    tri_states: Sequence[Optional[TexState]],
    tri_uv_info: Optional[Sequence[UvInfo]] = None,
    mode: str = "full",
) -> Tuple[List[Vtx], List[Tri], List[Optional[TexState]], List[UvInfo], int]:
    if not verts or not tris:
        uv = list(tri_uv_info[: len(tris)]) if tri_uv_info is not None else []
        return list(verts), list(tris), list(tri_states[: len(tris)]), uv, 0
    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    ex = float(max(xs) - min(xs))
    ey = float(max(ys) - min(ys))
    ez = float(max(zs) - min(zs))
    diag = (ex * ex + ey * ey + ez * ez) ** 0.5
    if diag <= 0.0:
        uv = list(tri_uv_info[: len(tris)]) if tri_uv_info is not None else []
        return list(verts), list(tris), list(tri_states[: len(tris)]), uv, 0

    max_edges: List[float] = []
    tri_areas: List[float] = []
    tri_aspects: List[float] = []
    tri_metrics: List[Tuple[int, Tri, float, float, float]] = []
    for ti, t in enumerate(tris):
        ids = [t.i0, t.i1, t.i2]
        edges: List[float] = []
        pts = [verts[ids[0]], verts[ids[1]], verts[ids[2]]]
        for i, j in ((0, 1), (1, 2), (2, 0)):
            a = verts[ids[i]]
            b = verts[ids[j]]
            dx = float(a.x - b.x)
            dy = float(a.y - b.y)
            dz = float(a.z - b.z)
            edges.append((dx * dx + dy * dy + dz * dz) ** 0.5)
        me = max(edges)
        mine = max(1e-6, min(edges))
        aspect = me / mine
        ux = float(pts[1].x - pts[0].x)
        uy = float(pts[1].y - pts[0].y)
        uz = float(pts[1].z - pts[0].z)
        vx = float(pts[2].x - pts[0].x)
        vy = float(pts[2].y - pts[0].y)
        vz = float(pts[2].z - pts[0].z)
        cx = (uy * vz) - (uz * vy)
        cy = (uz * vx) - (ux * vz)
        cz = (ux * vy) - (uy * vx)
        area = 0.5 * ((cx * cx + cy * cy + cz * cz) ** 0.5)
        max_edges.append(me)
        tri_areas.append(area)
        tri_aspects.append(aspect)
        tri_metrics.append((ti, t, me, area, aspect))

    if not max_edges:
        uv = list(tri_uv_info[: len(tris)]) if tri_uv_info is not None else []
        return list(verts), list(tris), list(tri_states[: len(tris)]), uv, 0
    if mode == "degenerate":
        keep_ids = [ti for (ti, _t, _me, area, aspect) in tri_metrics if area > 1e-4 and aspect < 1e6]
        removed = len(tris) - len(keep_ids)
        if removed <= 0:
            uv = list(tri_uv_info[: len(tris)]) if tri_uv_info is not None else []
            return list(verts), list(tris), list(tri_states[: len(tris)]), uv, 0
        kept_tris = [tris[i] for i in keep_ids]
        kept_states = [tri_states[i] if i < len(tri_states) else None for i in keep_ids]
        kept_uvs = [tri_uv_info[i] for i in keep_ids] if tri_uv_info is not None else []
        out_verts, out_tris, out_states = _compact_mesh_with_states(verts, kept_tris, kept_states)
        return out_verts, out_tris, out_states, kept_uvs, removed

    se = sorted(max_edges)
    sa = sorted(tri_areas)
    ss = sorted(tri_aspects)
    med_e = statistics.median(max_edges)
    med_a = statistics.median(tri_areas)
    p95_e = se[min(len(se) - 1, int(0.95 * len(se)))]
    p98_a = sa[min(len(sa) - 1, int(0.98 * len(sa)))]
    p95_s = ss[min(len(ss) - 1, int(0.95 * len(ss)))]
    edge_thresh = min(0.70 * diag, max(3.5 * med_e, 1.4 * p95_e))
    area_thresh = max(8.0 * med_a, 1.25 * p98_a)
    aspect_thresh = max(14.0, 1.4 * p95_s)
    keep_ids = [ti for (ti, _t, me, area, aspect) in tri_metrics if (me <= edge_thresh and area <= area_thresh and aspect <= aspect_thresh)]
    if len(keep_ids) < max(1, len(tris) // 3):
        uv = list(tri_uv_info[: len(tris)]) if tri_uv_info is not None else []
        return list(verts), list(tris), list(tri_states[: len(tris)]), uv, 0
    removed = len(tris) - len(keep_ids)
    if removed <= 0:
        uv = list(tri_uv_info[: len(tris)]) if tri_uv_info is not None else []
        return list(verts), list(tris), list(tri_states[: len(tris)]), uv, 0
    kept_tris = [tris[i] for i in keep_ids]
    kept_states = [tri_states[i] if i < len(tri_states) else None for i in keep_ids]
    kept_uvs = [tri_uv_info[i] for i in keep_ids] if tri_uv_info is not None else []
    out_verts, out_tris, out_states = _compact_mesh_with_states(verts, kept_tris, kept_states)
    return out_verts, out_tris, out_states, kept_uvs, removed


def rgba5551_to_rgba8888(v: int) -> Tuple[int, int, int, int]:
    r = ((v >> 11) & 0x1F) * 255 // 31
    g = ((v >> 6) & 0x1F) * 255 // 31
    b = ((v >> 1) & 0x1F) * 255 // 31
    a = 255 if (v & 1) else 0
    return r, g, b, a


def extract_ci4_texture(data: bytes) -> Tuple[List[Tuple[int, int, int, int]], int, int]:
    # The discovered sample uses TLUT at 0x0BA0 and CI4 image at 0x0BC0 with 32x32 pixels.
    pal_off = 0x0BA0
    img_off = 0x0BC0
    width = 32
    height = 32
    if img_off + (width * height) // 2 > len(data):
        raise ValueError("Texture region out of range for CI4 32x32 assumption")

    palette: List[Tuple[int, int, int, int]] = []
    for i in range(16):
        p = be32(b"\x00\x00" + data[pal_off + i * 2 : pal_off + i * 2 + 2], 0) & 0xFFFF
        palette.append(rgba5551_to_rgba8888(p))

    pixels: List[Tuple[int, int, int, int]] = []
    blob = data[img_off : img_off + (width * height) // 2]
    for byte in blob:
        hi = (byte >> 4) & 0xF
        lo = byte & 0xF
        pixels.append(palette[hi])
        pixels.append(palette[lo])
    return pixels, width, height


def _extract_ci4_texture_at(data: bytes, pal_off: int, img_off: int, width: int = 32, height: int = 32) -> Tuple[List[Tuple[int, int, int, int]], int, int]:
    if pal_off < 0 or (pal_off + 32) > len(data):
        raise ValueError("palette region out of range")
    if img_off < 0 or (img_off + (width * height) // 2) > len(data):
        raise ValueError("image region out of range")
    palette: List[Tuple[int, int, int, int]] = []
    for i in range(16):
        p = be32(b"\x00\x00" + data[pal_off + i * 2 : pal_off + i * 2 + 2], 0) & 0xFFFF
        palette.append(rgba5551_to_rgba8888(p))
    pixels: List[Tuple[int, int, int, int]] = []
    blob = data[img_off : img_off + (width * height) // 2]
    for byte in blob:
        hi = (byte >> 4) & 0xF
        lo = byte & 0xF
        pixels.append(palette[hi])
        pixels.append(palette[lo])
    return pixels, width, height


def _decode_settilesize_wh(w0: int, w1: int) -> Optional[Tuple[int, int]]:
    # F2 (G_SETTILESIZE): w0 packs uls/ult, w1 packs lrs/lrt in 10.2 fields.
    uls = (w0 >> 12) & 0x0FFF
    ult = w0 & 0x0FFF
    lrs = (w1 >> 12) & 0x0FFF
    lrt = w1 & 0x0FFF
    width = ((lrs - uls) >> 2) + 1
    height = ((lrt - ult) >> 2) + 1
    if width <= 0 or height <= 0:
        return None
    if width > 1024 or height > 1024:
        return None
    return width, height


def _decode_settilesize_range(w0: int, w1: int) -> Tuple[int, int, int, int]:
    uls = (w0 >> 12) & 0x0FFF
    ult = w0 & 0x0FFF
    lrs = (w1 >> 12) & 0x0FFF
    lrt = w1 & 0x0FFF
    return uls, ult, lrs, lrt


def _guess_wh_from_texels(texels: int) -> Optional[Tuple[int, int]]:
    if texels <= 0:
        return None
    root = int(round(math.sqrt(texels)))
    if root > 0 and root * root == texels:
        if root <= 1024:
            return root, root
    for w in (8, 16, 32, 64, 128, 256, 512, 1024):
        if texels % w == 0:
            h = texels // w
            if 1 <= h <= 1024:
                return w, h
    return None


def _decode_rgba16_pixels(data: bytes, img_off: int, width: int, height: int) -> List[Tuple[int, int, int, int]]:
    need = width * height * 2
    if img_off < 0 or (img_off + need) > len(data):
        raise ValueError("RGBA16 image out of range")
    out: List[Tuple[int, int, int, int]] = []
    blob = data[img_off : img_off + need]
    for i in range(0, len(blob), 2):
        v = (blob[i] << 8) | blob[i + 1]
        out.append(rgba5551_to_rgba8888(v))
    return out


def _decode_i_pixels(data: bytes, img_off: int, width: int, height: int, siz: int) -> List[Tuple[int, int, int, int]]:
    n = width * height
    if siz == 0:  # I4
        need = (n + 1) // 2
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("I4 image out of range")
        out: List[Tuple[int, int, int, int]] = []
        for b in data[img_off : img_off + need]:
            hi = (b >> 4) & 0xF
            lo = b & 0xF
            i0 = hi * 17
            i1 = lo * 17
            out.append((i0, i0, i0, 255))
            out.append((i1, i1, i1, 255))
        return out[:n]
    if siz == 1:  # I8
        need = n
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("I8 image out of range")
        out = []
        for b in data[img_off : img_off + need]:
            out.append((b, b, b, 255))
        return out
    raise ValueError("unsupported I size")


def _decode_ia_pixels(data: bytes, img_off: int, width: int, height: int, siz: int) -> List[Tuple[int, int, int, int]]:
    n = width * height
    out: List[Tuple[int, int, int, int]] = []
    if siz == 0:  # IA4: 3-bit intensity + 1-bit alpha per texel
        need = (n + 1) // 2
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("IA4 image out of range")
        for b in data[img_off : img_off + need]:
            for v in ((b >> 4) & 0xF, b & 0xF):
                i = ((v >> 1) & 0x7) * 255 // 7
                a = 255 if (v & 1) else 0
                out.append((i, i, i, a))
        return out[:n]
    if siz == 1:  # IA8: 4-bit intensity + 4-bit alpha
        need = n
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("IA8 image out of range")
        for b in data[img_off : img_off + need]:
            i = ((b >> 4) & 0xF) * 17
            a = (b & 0xF) * 17
            out.append((i, i, i, a))
        return out
    if siz == 2:  # IA16: 8-bit intensity + 8-bit alpha
        need = n * 2
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("IA16 image out of range")
        blob = data[img_off : img_off + need]
        for i in range(0, len(blob), 2):
            inten = blob[i]
            alpha = blob[i + 1]
            out.append((inten, inten, inten, alpha))
        return out
    raise ValueError("unsupported IA size")


def _decode_ci_pixels(
    data: bytes,
    pal_off: int,
    img_off: int,
    width: int,
    height: int,
    siz: int,
    pal_bank: int = 0,
) -> List[Tuple[int, int, int, int]]:
    n = width * height
    if siz == 0:  # CI4
        # If multiple 16-color banks are packed contiguously, select by pal_bank.
        bank_off = pal_off + (max(0, pal_bank) * 32)
        if (bank_off + 32) > len(data):
            bank_off = pal_off
        palette: List[Tuple[int, int, int, int]] = []
        for i in range(16):
            p = be32(b"\x00\x00" + data[bank_off + i * 2 : bank_off + i * 2 + 2], 0) & 0xFFFF
            palette.append(rgba5551_to_rgba8888(p))
        need = (n + 1) // 2
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("CI4 image out of range")
        out: List[Tuple[int, int, int, int]] = []
        for b in data[img_off : img_off + need]:
            out.append(palette[(b >> 4) & 0xF])
            out.append(palette[b & 0xF])
        return out[:n]
    if siz == 1:  # CI8
        # Assume 256 RGBA16 entries.
        if pal_off < 0 or (pal_off + 512) > len(data):
            raise ValueError("CI8 palette out of range")
        palette: List[Tuple[int, int, int, int]] = []
        for i in range(256):
            p = be32(b"\x00\x00" + data[pal_off + i * 2 : pal_off + i * 2 + 2], 0) & 0xFFFF
            palette.append(rgba5551_to_rgba8888(p))
        need = n
        if img_off < 0 or (img_off + need) > len(data):
            raise ValueError("CI8 image out of range")
        out = []
        for idx in data[img_off : img_off + need]:
            out.append(palette[idx])
        return out
    raise ValueError("unsupported CI size")


def _decode_texture_pixels(
    data: bytes,
    fmt: int,
    siz: int,
    img_off: int,
    width: int,
    height: int,
    pal_off: Optional[int] = None,
    pal_bank: int = 0,
) -> List[Tuple[int, int, int, int]]:
    if fmt == 2:  # CI
        if pal_off is None:
            raise ValueError("CI decode requires palette")
        return _decode_ci_pixels(data, pal_off, img_off, width, height, siz, pal_bank=pal_bank)
    if fmt == 4:  # I
        return _decode_i_pixels(data, img_off, width, height, siz)
    if fmt == 3:  # IA
        return _decode_ia_pixels(data, img_off, width, height, siz)
    if fmt == 0 and siz == 2:  # RGBA16
        return _decode_rgba16_pixels(data, img_off, width, height)
    raise ValueError("unsupported texture format")


def _decode_texture_pixels_best_palbank(
    data: bytes,
    fmt: int,
    siz: int,
    img_off: int,
    width: int,
    height: int,
    pal_off: Optional[int] = None,
    pal_bank: int = 0,
) -> List[Tuple[int, int, int, int]]:
    def _decode_pick_score(px: Sequence[Tuple[int, int, int, int]]) -> float:
        s = _texture_visual_score(px, width, height)
        a0_ratio = _alpha0_ratio(px)
        # Strongly discourage heavily transparent decodes. These are often
        # false CI/TLUT interpretations that look colorful but map badly.
        if a0_ratio >= 0.98:
            s += 1.6
        elif a0_ratio >= 0.90:
            s += 1.25
        elif a0_ratio >= 0.80:
            s += 0.9
        elif a0_ratio >= 0.65:
            s += 0.55
        return s

    def _alpha0_ratio(px: Sequence[Tuple[int, int, int, int]]) -> float:
        n = max(1, len(px))
        return sum(1 for p in px if p[3] == 0) / n

    base_pixels = _decode_texture_pixels(data, fmt, siz, img_off, width, height, pal_off, pal_bank=pal_bank)
    base_score = _decode_pick_score(base_pixels)
    base_a0_ratio = _alpha0_ratio(base_pixels)

    # For CI4, resolve palette-bank ambiguity only when base decode looks
    # clearly invalid. In this corpus, brute-force bank hopping can produce
    # vivid but wrong textures; trust parsed pal_bank by default.
    if fmt == 2 and siz == 0 and pal_off is not None:
        base_invalid = base_score >= 1e8
        if base_invalid or base_a0_ratio >= 0.85:
            best: Tuple[float, List[Tuple[int, int, int, int]], int] = (base_score, base_pixels, int(pal_bank))
            tried = {int(pal_bank)}
            for bank in (0, int(pal_bank), 1, 2, 3, 4, 5, 6, 7):
                if bank in tried:
                    continue
                tried.add(bank)
                try:
                    pixels = _decode_texture_pixels(data, fmt, siz, img_off, width, height, pal_off, pal_bank=bank)
                except Exception:
                    continue
                score = _decode_pick_score(pixels)
                if score < best[0]:
                    best = (score, pixels, int(bank))
            best_score, best_pixels, _best_bank = best
            if (best_score + 0.20) < base_score or (base_a0_ratio >= 0.85 and (best_score + 0.08) < base_score):
                base_score, base_pixels = best_score, best_pixels

    # CI4 state can be mislabeled in some streams; compare against I/IA variants.
    # Keep CI4 unless an alternate decode is clearly better.
    if fmt == 2 and siz == 0:
        # When a CI palette pointer is present and the decode is reasonably
        # opaque, prefer CI4. Aggressive IA/I fallback causes grayscale/alpha
        # artifacts on valid paletted models in this corpus.
        allow_alt_fmt = (pal_off is None) or (base_a0_ratio >= 0.65) or (base_score >= 3.0)
        if allow_alt_fmt:
            alt_best: Optional[Tuple[float, List[Tuple[int, int, int, int]]]] = None
            for alt_fmt, alt_siz in ((4, 1), (3, 1), (4, 0), (3, 0)):
                try:
                    alt_pixels = _decode_texture_pixels(
                        data, alt_fmt, alt_siz, img_off, width, height, pal_off=None, pal_bank=0
                    )
                except Exception:
                    continue
                alt_score = _decode_pick_score(alt_pixels)
                if alt_best is None or alt_score < alt_best[0]:
                    alt_best = (alt_score, alt_pixels)
            if alt_best is not None:
                alt_score, alt_pixels = alt_best
                # Make fallback to I/IA strict; only use it when CI4 decode is
                # clearly implausible or substantially worse.
                if (alt_score + 0.30) < base_score or (base_a0_ratio >= 0.70 and (alt_score + 0.08) < base_score):
                    return alt_pixels

    return base_pixels


def _texture_state_pref_score(fmt: int, siz: int) -> int:
    # Prefer paletted/colored states first for single-texture OBJ export.
    if fmt == 2 and siz == 0:  # CI4
        return 6
    if fmt == 2 and siz == 1:  # CI8
        return 5
    if fmt == 0 and siz == 2:  # RGBA16
        return 4
    if fmt == 3:  # IA*
        return 3
    if fmt == 4:  # I*
        return 2
    return 1


def extract_ci4_texture_from_dl(data: bytes, dl_off: int) -> Tuple[List[Tuple[int, int, int, int]], int, int]:
    # Heuristic decode from RDP command stream:
    # - FD100000 commonly points to RGBA16 TLUT (16 entries)
    # - FD500000 / FD900000 commonly point to CI4 image payloads
    seg_bases: Dict[int, int] = {}
    cmds = list(_iter_display_list_exec(data, dl_off=dl_off, seg_bases=seg_bases))
    pal_offs: List[int] = []
    img_offs: List[int] = []
    current_pal: Optional[int] = None
    current_img: Optional[int] = None
    current_wh: Optional[Tuple[int, int]] = None
    current_fmt: Optional[int] = None
    current_siz: Optional[int] = None
    current_pal_bank: int = 0
    current_tlut_entries: Optional[int] = None
    last_tlut_candidate: Optional[int] = None
    tile_fmt: List[Optional[int]] = [None] * 8
    tile_siz: List[Optional[int]] = [None] * 8
    tile_pal_bank: List[int] = [0] * 8
    tile_wh: List[Optional[Tuple[int, int]]] = [None] * 8
    tile_load_texels: List[Optional[int]] = [None] * 8
    active_render_tile = 0
    usage: Dict[Tuple[int, int, int, int, int, int, int], int] = {}

    for _off, op, w0, w1 in cmds:
        if op == 0xFD:
            addr = _resolve_dl_addr(data, w1, seg_bases=seg_bases)
            if addr is None:
                continue
            fmt = (w0 >> 21) & 0x7
            siz = (w0 >> 19) & 0x3
            if fmt == 0 and siz == 2 and (addr + 32) <= len(data):
                pal_offs.append(addr)
                last_tlut_candidate = addr
            img_offs.append(addr)
            current_img = addr
            continue

        if op == 0xF0:  # G_LOADTLUT
            if last_tlut_candidate is not None:
                current_pal = last_tlut_candidate
            tlut_entries = ((w1 >> 14) & 0x3FF) + 1
            if tlut_entries > 0:
                current_tlut_entries = int(tlut_entries)
            continue

        if op == 0xF5:
            # Track render-tile fmt/siz/palette bank. Tile 7 is often load tile.
            tile = (w1 >> 24) & 0x7
            tile_fmt[tile] = (w0 >> 21) & 0x7
            tile_siz[tile] = (w0 >> 19) & 0x3
            tile_pal_bank[tile] = (w1 >> 20) & 0xF
            if tile != 7:
                active_render_tile = tile
                current_fmt = tile_fmt[tile]
                current_siz = tile_siz[tile]
                current_pal_bank = tile_pal_bank[tile]
                if tile_wh[tile] is not None:
                    current_wh = tile_wh[tile]
            continue

        if op == 0xF2:
            # Tile index is encoded in w1[26:24].
            tile = (w1 >> 24) & 0x7
            wh = _decode_settilesize_wh(w0, w1)
            if wh is not None:
                tile_wh[tile] = wh
                if tile == active_render_tile:
                    current_wh = wh
            continue

        if op == 0xF3:  # G_LOADBLOCK
            tile = (w1 >> 24) & 0x7
            texels = ((w1 >> 12) & 0x0FFF) + 1
            if texels > 0:
                tile_load_texels[tile] = texels
                if tile == active_render_tile and tile_wh[tile] is None:
                    guess = _guess_wh_from_texels(texels)
                    if guess is not None:
                        tile_wh[tile] = guess
                        current_wh = guess
            continue

        if op in (0x05, 0x06):
            pal = current_pal
            if pal is None and current_fmt == 2 and last_tlut_candidate is not None:
                pal = last_tlut_candidate
            pal_bank = current_pal_bank
            if current_fmt == 2 and current_siz == 0 and current_tlut_entries is not None and current_tlut_entries <= 16:
                pal_bank = 0
            if pal is None or current_img is None:
                # Non-CI formats may not require a palette.
                if current_img is None:
                    continue
            if current_fmt is None or current_siz is None:
                continue
            width, height = current_wh if current_wh is not None else (32, 32)
            if width <= 0 or height <= 0:
                continue
            tri_inc = 2 if op == 0x06 else 1
            key = (
                -1 if pal is None else pal,
                current_img,
                width,
                height,
                current_fmt,
                current_siz,
                pal_bank,
            )
            usage[key] = usage.get(key, 0) + tri_inc

    if not img_offs:
        raise ValueError("no texture image addresses inferred from DL")

    candidate_dims: List[Tuple[int, int]] = [(32, 32), (16, 16), (64, 32), (32, 64), (64, 64), (128, 128)]

    candidates: List[Tuple[int, int, int, int, int, int, int, int]] = []
    if usage:
        # Prefer state pair that was active for most drawn triangles.
        for (pal, img, width, height, fmt, siz, pal_bank), tri_hits in usage.items():
            candidates.append((tri_hits, pal, img, width, height, fmt, siz, pal_bank))
        candidates.sort(
            key=lambda c: (
                int(c[0]),  # tri hits
                _texture_state_pref_score(int(c[5]), int(c[6])),  # format preference
                -int(c[7]),  # lower palette bank as tie-breaker
                -int(c[3] * c[4]),  # smaller tile area preferred for tiny tiled assets
            ),
            reverse=True,
        )
    else:
        # Fallback to most recent palette/image pair; this tends to reflect
        # final bound state in compact model display lists.
        pal = pal_offs[-1] if pal_offs else -1
        img = img_offs[-1]
        for width, height in candidate_dims:
            candidates.append((0, pal, img, width, height, 2, 0, 0))

    best: Optional[Tuple[float, List[Tuple[int, int, int, int]], int, int]] = None
    # Try best-ranked pair first, then alternate dimensions if needed.
    for _hits, pal, img, width, height, fmt, siz, pal_bank in candidates:
        trial_dims = [(width, height)] + [d for d in candidate_dims if d != (width, height)]
        for tw, th in trial_dims:
            try:
                pixels = _decode_texture_pixels_best_palbank(
                    data=data,
                    fmt=fmt,
                    siz=siz,
                    img_off=img,
                    width=tw,
                    height=th,
                    pal_off=None if pal < 0 else pal,
                    pal_bank=pal_bank,
                )
                score = _texture_visual_score(pixels, tw, th)
                # Slightly favor states with more triangle hits.
                score -= min(0.5, (_hits * 0.02))
                if best is None or score < best[0]:
                    best = (score, pixels, tw, th)
            except Exception:
                continue

    if best is None:
        raise ValueError("failed to decode texture from inferred DL state")
    return best[1], best[2], best[3]


def _build_materials_from_tri_states(
    data: bytes,
    out_base: pathlib.Path,
    tri_states: Sequence[Optional[TexState]],
    tri_uv_areas: Optional[Sequence[float]] = None,
) -> Tuple[Optional[List[str]], Optional[Dict[str, str]], int]:
    if Image is None:
        return None, None, 0
    usage: Dict[TexState, int] = {}
    area_usage: Dict[TexState, float] = {}
    for i, st in enumerate(tri_states):
        if st is None:
            continue
        usage[st] = usage.get(st, 0) + 1
        if tri_uv_areas is not None and i < len(tri_uv_areas):
            area_usage[st] = area_usage.get(st, 0.0) + float(tri_uv_areas[i])
    if not usage:
        return None, None, 0

    decoded: List[Tuple[TexState, int, float, float, int, int, List[Tuple[int, int, int, int]]]] = []
    for st, hits in usage.items():
        pal, img, w, h, fmt, siz, pal_bank = st
        try:
            pixels = _decode_texture_pixels_best_palbank(
                data=data,
                fmt=fmt,
                siz=siz,
                img_off=img,
                width=w,
                height=h,
                pal_off=None if pal < 0 else pal,
                pal_bank=pal_bank,
            )
        except Exception:
            continue
        vis_score = _texture_visual_score(pixels, w, h)
        area = area_usage.get(st, 0.0)
        decoded.append((st, hits, vis_score, area, w, h, pixels))

    if not decoded:
        return None, None, 0

    area_norm = max(1e-6, sum(kv[3] for kv in decoded))
    ordered = sorted(
        decoded,
        key=lambda kv: (
            float(kv[1]) + (kv[3] / area_norm) * 3.0,  # tri hits + uv area coverage
            -float(kv[2]),  # lower visual score is better
            _texture_state_pref_score(int(kv[0][4]), int(kv[0][5])),
            -int(kv[0][6]),
            int(kv[4] * kv[5]),
        ),
        reverse=True,
    )

    mat_tex: Dict[str, str] = {}
    state_to_mat: Dict[TexState, str] = {}
    for i, (st, _hits, _vis, _area, w, h, pixels) in enumerate(ordered):
        mat_name = f"{out_base.stem}_mat{i:02d}"
        tex_name = f"{out_base.stem}_tex{i:02d}.png"
        _save_texture_png(out_base.with_name(f"{out_base.stem}_tex{i:02d}.png"), pixels, w, h)
        mat_tex[mat_name] = tex_name
        state_to_mat[st] = mat_name

    if not mat_tex:
        return None, None, 0
    default_mat = next(iter(mat_tex.keys()))
    tri_mats = [state_to_mat.get(st, default_mat) if st is not None else default_mat for st in tri_states]
    return tri_mats, mat_tex, len(mat_tex)


def _best_texture_from_tri_states(
    data: bytes,
    tri_states: Sequence[Optional[TexState]],
    tri_uv_areas: Optional[Sequence[float]] = None,
) -> Optional[Tuple[List[Tuple[int, int, int, int]], int, int]]:
    usage: Dict[TexState, int] = {}
    area_usage: Dict[TexState, float] = {}
    for i, st in enumerate(tri_states):
        if st is None:
            continue
        usage[st] = usage.get(st, 0) + 1
        if tri_uv_areas is not None and i < len(tri_uv_areas):
            area_usage[st] = area_usage.get(st, 0.0) + float(tri_uv_areas[i])
    if not usage:
        return None

    best: Optional[Tuple[float, List[Tuple[int, int, int, int]], int, int]] = None
    area_norm = max(1e-6, sum(area_usage.values()))
    for st, hits in usage.items():
        pal, img, w, h, fmt, siz, pal_bank = st
        try:
            pixels = _decode_texture_pixels_best_palbank(
                data=data,
                fmt=fmt,
                siz=siz,
                img_off=img,
                width=w,
                height=h,
                pal_off=None if pal < 0 else pal,
                pal_bank=pal_bank,
            )
        except Exception:
            continue
        vis_score = _texture_visual_score(pixels, w, h)
        area = area_usage.get(st, 0.0)
        score = vis_score - (hits * 0.02) - ((area / area_norm) * 0.5)
        if best is None or score < best[0]:
            best = (score, pixels, w, h)

    if best is None:
        return None
    return best[1], best[2], best[3]


def _apply_known_texture_fixes(
    yay0_offset: Optional[int],
    rom_off: int,
    verts: Sequence[Vtx],
    tris: Sequence[Tri],
    tri_states: Sequence[Optional[TexState]],
    tri_mats: List[str],
    tri_uv_info: List[UvInfo],
) -> int:
    # Model-specific fix: y0515_00B22068_m00_000010 has right-side texstates that
    # should mirror/reuse the left-side image family.
    if yay0_offset is None or int(yay0_offset) != 0x00B22068 or int(rom_off) != 0x10:
        return 0
    if not tri_mats or not tri_states or not tris or not tri_uv_info:
        return 0

    state_to_mat: Dict[TexState, str] = {}
    for i, st in enumerate(tri_states):
        if st is None or i >= len(tri_mats):
            continue
        if st not in state_to_mat:
            state_to_mat[st] = tri_mats[i]

    mat_remap: Dict[str, str] = {}
    for st, mat in state_to_mat.items():
        pal, img, w, h, fmt, siz, pal_bank = st
        if int(img) != 0x1210:
            continue
        target = (int(pal), 0x1410, int(w), int(h), int(fmt), int(siz), int(pal_bank))
        mat2 = state_to_mat.get(target)
        if mat2 and mat2 != mat:
            mat_remap[mat] = mat2

    if not mat_remap:
        return 0

    fixed = 0
    for i, t in enumerate(tris):
        if i >= len(tri_mats) or i >= len(tri_uv_info):
            continue
        src_mat = tri_mats[i]
        dst_mat = mat_remap.get(src_mat)
        if not dst_mat:
            continue
        cx = (verts[t.i0].x + verts[t.i1].x + verts[t.i2].x) / 3.0
        if cx <= 0.0:
            continue
        tri_mats[i] = dst_mat
        du, dv, ou, ov = tri_uv_info[i]
        # Mirror U by replacing u with (1 - u):
        # u=(s-ou)/du -> u'=(s-(ou+du))/(-du)=1-u
        if du != 0:
            tri_uv_info[i] = (-int(du), int(dv), int(ou + du), int(ov))
        fixed += 1
    return fixed


def _texture_visual_score(pixels: Sequence[Tuple[int, int, int, int]], w: int, h: int) -> float:
    # Lower score is better. Rejects flat/noisy garbage heuristically.
    n = len(pixels)
    if n <= 0 or w <= 0 or h <= 0:
        return 1e9
    uniq = len(set(pixels))
    uniq_ratio = uniq / max(1, n)
    if uniq <= 1:
        return 1e9

    def lum(px: Tuple[int, int, int, int]) -> float:
        return (0.2126 * px[0]) + (0.7152 * px[1]) + (0.0722 * px[2])

    vals = [lum(p) for p in pixels]
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    std = var ** 0.5

    # Edge activity on luma: too low=flat; too high=random noise.
    edge_sum = 0.0
    edge_cnt = 0
    for y in range(h):
        row = y * w
        for x in range(w):
            i = row + x
            if x + 1 < w:
                edge_sum += abs(vals[i] - vals[i + 1])
                edge_cnt += 1
            if y + 1 < h:
                edge_sum += abs(vals[i] - vals[i + w])
                edge_cnt += 1
    edge = (edge_sum / max(1, edge_cnt))
    l8 = [max(0, min(255, int(round(v)))) for v in vals]

    parity_even = 0
    parity_odd = 0
    ce = 0
    co = 0
    for y in range(h):
        row = y * w
        for x in range(w):
            v = l8[row + x]
            if ((x + y) & 1) == 0:
                parity_even += v
                ce += 1
            else:
                parity_odd += v
                co += 1
    pmean_e = (parity_even / ce) if ce else 0.0
    pmean_o = (parity_odd / co) if co else 0.0
    parity_bias = abs(pmean_e - pmean_o) / 255.0
    seam = _grid_seam_penalty(l8, w, h, periods=[4, 8, 16, 32])
    interlace = _row_interlace_penalty(l8, w, h, 255.0)

    # Moderate complexity and moderate edge energy tend to be useful textures.
    score = 0.0
    score += abs(edge - 24.0) / 24.0
    score += abs(std - 48.0) / 48.0
    score += 0.6 * abs(uniq_ratio - 0.18) / 0.18
    score += 0.8 * parity_bias
    score += 0.8 * interlace
    score += 0.5 * seam
    # Penalize mostly/fully transparent outputs; these are often bad TLUT picks.
    alpha0_ratio = sum(1 for p in pixels if p[3] == 0) / max(1, n)
    if alpha0_ratio >= 0.98:
        score += 1.25
    elif alpha0_ratio >= 0.90:
        score += 0.35
    return score


def _sanitize_texture_alpha(pixels: Sequence[Tuple[int, int, int, int]]) -> List[Tuple[int, int, int, int]]:
    out = list(pixels)
    if not out:
        return out
    # If every alpha channel is zero but RGB carries data, treat as opaque for usability.
    if all(px[3] == 0 for px in out) and any((px[0] | px[1] | px[2]) != 0 for px in out):
        return [(px[0], px[1], px[2], 255) for px in out]
    return out


def _save_texture_png(out_path: pathlib.Path, pixels: Sequence[Tuple[int, int, int, int]], w: int, h: int) -> None:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    px = _sanitize_texture_alpha(pixels)
    img = Image.new("RGBA", (w, h))
    img.putdata(px)
    img.save(out_path)


def _collect_unresolved_setimg_seg_offsets(data: bytes, dl_off: int = 8, seg_id: int = 0x04) -> List[int]:
    seg_bases: Dict[int, int] = {}
    offs: List[int] = []
    for _off, op, w0, w1 in _iter_display_list_exec(data, dl_off=dl_off, seg_bases=seg_bases):
        if op != 0xFD:
            continue
        seg = (w1 >> 24) & 0xFF
        if seg != seg_id:
            continue
        addr = _resolve_dl_addr(data, w1, seg_bases=seg_bases)
        if addr is None:
            offs.append(w1 & 0x00FFFFFF)
    return sorted(set(offs))


def _seg4_bridge_candidates(
    local_blob: bytes,
    full_rom: bytes,
    yay0_off: int,
    dl_off: int,
) -> List[Tuple[bytes, Dict[int, int], int]]:
    # Build local+external composite candidates for unresolved seg4 image pointers.
    offs = _collect_unresolved_setimg_seg_offsets(local_blob, dl_off=dl_off, seg_id=0x04)
    if not offs:
        return []

    min_off = int(min(offs))
    max_off = int(max(offs))
    if min_off < 0 or max_off < min_off:
        return local_blob, None

    # Candidate bases near this Yay0 region.
    hi = int(yay0_off) & 0x00FF0000
    est = (int(yay0_off) - min_off) & 0x00FFFF0000
    cand_bases = []
    for b in (est, hi - 0x20000, hi - 0x10000, hi, hi + 0x10000):
        if b not in cand_bases:
            cand_bases.append(b)

    span = max(0x2000, (max_off - min_off) + 0x800)
    out: List[Tuple[bytes, Dict[int, int], int]] = []
    for b in cand_bases:
        ext_start = int(b) + min_off
        ext_end = ext_start + span
        if ext_start < 0 or ext_end > len(full_rom):
            continue
        ext = full_rom[ext_start:ext_end]
        if not ext:
            continue
        append_at = len(local_blob)
        composite = local_blob + ext
        seg4_local_base = append_at - min_off
        out.append((composite, {0x04: int(seg4_local_base)}, int(b)))
    return out


def _extract_textures_from_model_chunk(
    data: bytes,
    chunk_label: str,
    out_dir: pathlib.Path,
    min_tri_hits: int = 2,
) -> List[Dict[str, object]]:
    out: List[Dict[str, object]] = []
    for mi, moff in enumerate(iter_model_headers(data)):
        dl_off = moff + 8
        for tri_div2 in (True, False):
            try:
                _v, _t, _s, tri_states, _uv = parse_model_with_texstates(data, dl_off=dl_off, tri_div2=tri_div2)
            except Exception:
                continue
            usage: Dict[TexState, int] = {}
            for st in tri_states:
                if st is None:
                    continue
                usage[st] = usage.get(st, 0) + 1
            if not usage:
                continue
            ordered = sorted(
                usage.items(),
                key=lambda kv: (
                    int(kv[1]),
                    _texture_state_pref_score(int(kv[0][4]), int(kv[0][5])),
                    -int(kv[0][6]),
                    -int(kv[0][2] * kv[0][3]),
                ),
                reverse=True,
            )
            for si, (st, hits) in enumerate(ordered):
                if hits < min_tri_hits:
                    continue
                pal, img, w, h, fmt, siz, pal_bank = st
                try:
                    pixels = _decode_texture_pixels_best_palbank(
                        data=data,
                        fmt=fmt,
                        siz=siz,
                        img_off=img,
                        width=w,
                        height=h,
                        pal_off=None if pal < 0 else pal,
                        pal_bank=pal_bank,
                    )
                except Exception:
                    continue
                score = _texture_visual_score(pixels, w, h)
                tex_name = (
                    f"{chunk_label}_m{mi:03d}_o{moff:06X}_"
                    f"s{si:02d}_f{fmt}z{siz}_p{pal_bank}_i{img:06X}_{w}x{h}.png"
                )
                out_path = out_dir / tex_name
                _save_texture_png(out_path, pixels, w, h)
                out.append(
                    {
                        "source": "model_dl",
                        "chunk": chunk_label,
                        "model_offset": moff,
                        "tri_div2": bool(tri_div2),
                        "tri_hits": int(hits),
                        "texture_state": {
                            "pal": pal,
                            "img": img,
                            "w": w,
                            "h": h,
                            "fmt": fmt,
                            "siz": siz,
                            "pal_bank": pal_bank,
                        },
                        "score": float(score),
                        "png": str(out_path),
                    }
                )
    return out


def _tri_state_decode_score(data: bytes, tri_states: Sequence[Optional[TexState]]) -> float:
    usage: Dict[TexState, int] = {}
    for st in tri_states:
        if st is None:
            continue
        usage[st] = usage.get(st, 0) + 1
    if not usage:
        return 1e9
    total = 0.0
    hits_total = 0
    for st, hits in usage.items():
        pal, img, w, h, fmt, siz, pal_bank = st
        try:
            pixels = _decode_texture_pixels_best_palbank(
                data=data,
                fmt=fmt,
                siz=siz,
                img_off=img,
                width=w,
                height=h,
                pal_off=None if pal < 0 else pal,
                pal_bank=pal_bank,
            )
            s = _texture_visual_score(pixels, w, h)
        except Exception:
            s = 5.0
        total += float(s) * float(hits)
        hits_total += int(hits)
    return total / max(1, hits_total)


def _candidate_pal16_offsets(data: bytes, step: int = 0x20) -> List[int]:
    offs: List[int] = []
    for o in range(0, max(0, len(data) - 32), max(2, step)):
        try:
            cols = []
            for i in range(16):
                v = (data[o + i * 2] << 8) | data[o + i * 2 + 1]
                cols.append(rgba5551_to_rgba8888(v))
            uniq = len(set(cols))
            if uniq >= 6:
                offs.append(o)
        except Exception:
            continue
    return offs


def _extract_generic_texture_candidates(
    data: bytes,
    chunk_label: str,
    out_dir: pathlib.Path,
    top_per_chunk: int = 10,
) -> List[Dict[str, object]]:
    dims = [(16, 16), (32, 32), (64, 64), (64, 32), (32, 64), (128, 128)]
    cand: List[Tuple[float, Dict[str, object], List[Tuple[int, int, int, int]], int, int, pathlib.Path]] = []
    pals = _candidate_pal16_offsets(data, step=0x20)

    # Heuristic CI4 sprite layouts: TLUT followed by image payload.
    for pal in pals[:512]:
        img = pal + 32
        for w, h in dims:
            need = (w * h + 1) // 2
            if img + need > len(data):
                continue
            try:
                pixels = _decode_ci_pixels(data, pal, img, w, h, siz=0, pal_bank=0)
            except Exception:
                continue
            s = _texture_visual_score(pixels, w, h)
            if s > 2.1:
                continue
            meta = {
                "source": "generic_ci4",
                "chunk": chunk_label,
                "texture_state": {"pal": pal, "img": img, "w": w, "h": h, "fmt": 2, "siz": 0, "pal_bank": 0},
                "score": float(s),
            }
            name = f"{chunk_label}_g_ci4_p{pal:06X}_i{img:06X}_{w}x{h}.png"
            cand.append((s, meta, pixels, w, h, out_dir / name))

    # Raw non-paletted candidates at common sprite sizes.
    for img in range(0, max(0, len(data) - 64), 0x40):
        for fmt, siz in ((4, 1), (4, 0), (3, 1), (3, 0), (0, 2)):
            for w, h in ((16, 16), (32, 32), (64, 32), (32, 64)):
                try:
                    pixels = _decode_texture_pixels(data, fmt, siz, img, w, h, pal_off=None, pal_bank=0)
                except Exception:
                    continue
                s = _texture_visual_score(pixels, w, h)
                if s > 1.8:
                    continue
                meta = {
                    "source": "generic_raw",
                    "chunk": chunk_label,
                    "texture_state": {"pal": -1, "img": img, "w": w, "h": h, "fmt": fmt, "siz": siz, "pal_bank": 0},
                    "score": float(s),
                }
                name = f"{chunk_label}_g_f{fmt}z{siz}_i{img:06X}_{w}x{h}.png"
                cand.append((s, meta, pixels, w, h, out_dir / name))

    cand.sort(key=lambda x: float(x[0]))
    out: List[Dict[str, object]] = []
    seen_keys: set[str] = set()
    for s, meta, pixels, w, h, path in cand:
        key = f"{meta['source']}:{meta['texture_state']['img']}:{meta['texture_state']['fmt']}:{meta['texture_state']['siz']}:{w}x{h}"
        if key in seen_keys:
            continue
        seen_keys.add(key)
        _save_texture_png(path, pixels, w, h)
        m = dict(meta)
        m["png"] = str(path)
        out.append(m)
        if len(out) >= top_per_chunk:
            break
    return out


def write_obj_mtl(
    out_base: pathlib.Path,
    verts: Sequence[Vtx],
    tris: Sequence[Tri],
    with_texture: bool,
    tri_materials: Optional[Sequence[str]] = None,
    material_textures: Optional[Dict[str, str]] = None,
    tri_uv_info: Optional[Sequence[UvInfo]] = None,
) -> None:
    obj_path = out_base.with_suffix(".obj")
    mtl_path = out_base.with_suffix(".mtl")
    tex_name = out_base.with_suffix(".png").name
    default_mat_name = f"{out_base.stem}_mat0"

    with obj_path.open("w", encoding="utf-8", newline="\n") as f:
        if with_texture:
            f.write(f"mtllib {mtl_path.name}\n")
            if not tri_materials:
                f.write(f"usemtl {default_mat_name}\n")
        for v in verts:
            f.write(f"v {v.x} {v.y} {v.z}\n")
        vt_lines: List[str] = []
        face_lines: List[str] = []
        if tri_uv_info:
            vt_index = 1
            current_mtl = None
            for ti, t in enumerate(tris):
                if with_texture and tri_materials and ti < len(tri_materials):
                    mtl = tri_materials[ti]
                    if mtl != current_mtl:
                        face_lines.append(f"usemtl {mtl}\n")
                        current_mtl = mtl
                info = tri_uv_info[ti] if ti < len(tri_uv_info) else (4096, 4096, 0, 0)
                idxs: List[int] = []
                for vi in (t.i0, t.i1, t.i2):
                    v = verts[vi]
                    u = (v.s - info[2]) / float(info[0])
                    vv = 1.0 - ((v.t - info[3]) / float(info[1]))
                    vt_lines.append(f"vt {u:.6f} {vv:.6f}\n")
                    idxs.append(vt_index)
                    vt_index += 1
                a = t.i0 + 1
                b = t.i1 + 1
                c = t.i2 + 1
                face_lines.append(f"f {a}/{idxs[0]} {b}/{idxs[1]} {c}/{idxs[2]}\n")
        else:
            # UV scaling observed in sample: 10.2 fixed-ish style with 4096 tile range.
            for v in verts:
                u = v.s / 4096.0
                vv = 1.0 - (v.t / 4096.0)
                vt_lines.append(f"vt {u:.6f} {vv:.6f}\n")
            current_mtl = None
            for ti, t in enumerate(tris):
                if with_texture and tri_materials and ti < len(tri_materials):
                    mtl = tri_materials[ti]
                    if mtl != current_mtl:
                        face_lines.append(f"usemtl {mtl}\n")
                        current_mtl = mtl
                a = t.i0 + 1
                b = t.i1 + 1
                c = t.i2 + 1
                face_lines.append(f"f {a}/{a} {b}/{b} {c}/{c}\n")
        for line in vt_lines:
            f.write(line)
        for line in face_lines:
            f.write(line)

    if with_texture:
        def _png_has_alpha(path: pathlib.Path) -> bool:
            if Image is None or (not path.exists()):
                return False
            try:
                im = Image.open(path).convert("RGBA")
                mn, mx = im.getchannel("A").getextrema()
                return mn < 255
            except Exception:
                return False

        with mtl_path.open("w", encoding="utf-8", newline="\n") as f:
            if material_textures:
                for mat, tex in material_textures.items():
                    f.write(f"newmtl {mat}\n")
                    f.write("Ka 1.0 1.0 1.0\n")
                    f.write("Kd 1.0 1.0 1.0\n")
                    f.write("Ks 0.0 0.0 0.0\n")
                    f.write("d 1.0\n")
                    f.write("illum 1\n")
                    f.write(f"map_Kd {tex}\n\n")
                    tex_path = out_base.with_name(tex)
                    if _png_has_alpha(tex_path):
                        f.write(f"map_d {tex}\n\n")
            else:
                f.write(f"newmtl {default_mat_name}\n")
                f.write("Ka 1.0 1.0 1.0\n")
                f.write("Kd 1.0 1.0 1.0\n")
                f.write("Ks 0.0 0.0 0.0\n")
                f.write("d 1.0\n")
                f.write("illum 1\n")
                f.write(f"map_Kd {tex_name}\n")
                tex_path = out_base.with_name(tex_name)
                if _png_has_alpha(tex_path):
                    f.write(f"map_d {tex_name}\n")


def _rewrite_obj_face_token(tok: str, base_v: int, base_vt: int, base_vn: int) -> str:
    parts = tok.split("/")
    out: List[str] = []
    for i, p in enumerate(parts):
        if p == "":
            out.append("")
            continue
        try:
            idx = int(p)
        except Exception:
            out.append(p)
            continue
        if idx <= 0:
            out.append(p)
            continue
        if i == 0:
            out.append(str(idx + base_v))
        elif i == 1:
            out.append(str(idx + base_vt))
        else:
            out.append(str(idx + base_vn))
    return "/".join(out)


def _merge_model_objs(models: Sequence[Dict[str, object]], out_base: pathlib.Path) -> Optional[Dict[str, object]]:
    valid: List[Dict[str, object]] = []
    for m in models:
        obj = pathlib.Path(str(m.get("obj", "")))
        if obj.exists():
            valid.append(m)
    if len(valid) < 2:
        return None

    out_base.parent.mkdir(parents=True, exist_ok=True)
    obj_out = out_base.with_suffix(".obj")
    mtl_out = out_base.with_suffix(".mtl")

    # Merge MTL blocks by unique material name.
    seen_mtls: set[str] = set()
    merged_mtl_lines: List[str] = []
    for m in valid:
        mtl_s = m.get("mtl")
        if not mtl_s:
            continue
        mtl_path = pathlib.Path(str(mtl_s))
        if not mtl_path.exists():
            continue
        cur_name: Optional[str] = None
        cur_lines: List[str] = []
        for ln in mtl_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            if ln.startswith("newmtl "):
                if cur_name is not None and cur_name not in seen_mtls and cur_lines:
                    merged_mtl_lines.extend(cur_lines)
                    merged_mtl_lines.append("")
                    seen_mtls.add(cur_name)
                cur_name = ln.split(" ", 1)[1].strip()
                cur_lines = [ln]
            else:
                if cur_name is not None:
                    cur_lines.append(ln)
        if cur_name is not None and cur_name not in seen_mtls and cur_lines:
            merged_mtl_lines.extend(cur_lines)
            merged_mtl_lines.append("")
            seen_mtls.add(cur_name)

    out_lines: List[str] = []
    if merged_mtl_lines:
        out_lines.append(f"mtllib {mtl_out.name}")

    total_v = 0
    total_vt = 0
    total_vn = 0
    merged_tris = 0

    for m in valid:
        name = str(m.get("name", "part"))
        obj_path = pathlib.Path(str(m.get("obj", "")))
        if not obj_path.exists():
            continue
        base_v = total_v
        base_vt = total_vt
        base_vn = total_vn
        out_lines.append(f"o {name}")
        for ln in obj_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            if ln.startswith("mtllib "):
                continue
            if ln.startswith("v "):
                out_lines.append(ln)
                total_v += 1
                continue
            if ln.startswith("vt "):
                out_lines.append(ln)
                total_vt += 1
                continue
            if ln.startswith("vn "):
                out_lines.append(ln)
                total_vn += 1
                continue
            if ln.startswith("f "):
                toks = ln.split()[1:]
                rt = [_rewrite_obj_face_token(tk, base_v, base_vt, base_vn) for tk in toks]
                out_lines.append("f " + " ".join(rt))
                merged_tris += 1
                continue
            if ln.startswith("usemtl ") or ln.startswith("g ") or ln.startswith("s "):
                out_lines.append(ln)

    obj_out.write_text("\n".join(out_lines) + "\n", encoding="utf-8")
    if merged_mtl_lines:
        mtl_out.write_text("\n".join(merged_mtl_lines) + "\n", encoding="utf-8")

    return {
        "name": out_base.name,
        "obj": str(obj_out),
        "mtl": str(mtl_out) if merged_mtl_lines else None,
        "source_models": [str(m.get("name")) for m in valid],
        "source_count": len(valid),
        "tri_count": merged_tris,
    }


def cmd_scan(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    report = scan_rom(rom)
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


def cmd_export_model(args: argparse.Namespace) -> int:
    blob = pathlib.Path(args.input).read_bytes()
    out_base = pathlib.Path(args.out)
    out_base.parent.mkdir(parents=True, exist_ok=True)

    tri_states: List[Optional[TexState]] = []
    tri_uv_info: List[UvInfo] = []
    if args.texture:
        verts, tris, stats, tri_states, tri_uv_info = parse_model_with_texstates(blob)
    else:
        verts, tris, stats = parse_model(blob)
    pruned_tris = 0
    if args.strict_mesh:
        if tri_states:
            verts, tris, tri_states, tri_uv_info, pruned_tris = _prune_outlier_tris_with_states(
                verts, tris, tri_states, tri_uv_info=tri_uv_info
            )
        else:
            verts, tris, pruned_tris = _prune_outlier_tris(verts, tris)
        if not mesh_quality_strict_ok(verts, tris):
            raise ValueError("strict mesh checks failed for this model")
    textured_tri_ratio = 0.0
    if tri_states:
        textured_tri_ratio = sum(1 for st in tri_states if st is not None) / max(1, len(tris))

    wrote_texture = False
    tri_mats: Optional[List[str]] = None
    mat_tex: Optional[Dict[str, str]] = None
    texture_materials = 0
    tri_uv_areas: Optional[List[float]] = None
    uv_out_ratio = 0.0
    if tri_uv_info:
        tri_uv_areas = _tri_uv_areas(verts, tris, tri_uv_info)
        uv_out_ratio = _uv_out_of_range_ratio(verts, tris, tri_uv_info)

    textured_tri_ratio = 0.0
    if tri_states:
        textured_tri_ratio = sum(1 for st in tri_states if st is not None) / max(1, len(tris))

    if args.texture:
        if Image is None:
            raise RuntimeError("Pillow is required for PNG texture export: pip install pillow")
        try:
            tri_mats, mat_tex, texture_materials = _build_materials_from_tri_states(
                blob, out_base, tri_states, tri_uv_areas=tri_uv_areas
            )
            if tri_mats and mat_tex:
                wrote_texture = True
            else:
                best = _best_texture_from_tri_states(blob, tri_states, tri_uv_areas=tri_uv_areas)
                if best is not None:
                    pixels, w, h = best
                else:
                    try:
                        pixels, w, h = extract_ci4_texture_from_dl(blob, dl_off=8)
                    except Exception:
                        pixels, w, h = extract_ci4_texture(blob)
                _save_texture_png(out_base.with_suffix(".png"), pixels, w, h)
                wrote_texture = True
        except Exception:
            wrote_texture = False

    write_obj_mtl(
        out_base,
        verts,
        tris,
        with_texture=wrote_texture,
        tri_materials=tri_mats,
        material_textures=mat_tex,
        tri_uv_info=tri_uv_info if tri_uv_info else None,
    )
    mesh_penalty = _mesh_topology_penalty(verts, tris)
    mesh_long = float(_mesh_spaghetti_metrics(verts, tris).get("long_edge_ratio", 1.0))
    mesh_conf = 1.0 / (1.0 + mesh_penalty + (2.0 * mesh_long) + (2.0 * uv_out_ratio))
    tex_conf = 0.0
    if wrote_texture:
        tex_conf = 0.9 if texture_materials > 0 else 0.6
    print(
        json.dumps(
            {
                "out": str(out_base),
                **stats,
                "tri_count": len(tris),
                "vert_count": len(verts),
                "pruned_outlier_tris": pruned_tris,
                "texture": wrote_texture,
                "texture_materials": texture_materials,
                "mesh_penalty": mesh_penalty,
                "mesh_long_edge_ratio": mesh_long,
                "mesh_confidence": mesh_conf,
                "uv_out_of_range_ratio": uv_out_ratio,
                "textured_tri_ratio": textured_tri_ratio,
                "texture_confidence": tex_conf,
            },
            indent=2,
        )
    )
    return 0


def _bbox(verts: Sequence[Vtx]) -> Dict[str, List[int]]:
    if not verts:
        return {"min": [0, 0, 0], "max": [0, 0, 0]}
    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    return {"min": [min(xs), min(ys), min(zs)], "max": [max(xs), max(ys), max(zs)]}


def _export_one_model(
    rom: bytes,
    rom_off: int,
    out_base: pathlib.Path,
    window: int,
    texture: bool,
    strict_mesh: bool = False,
    standalone_only: bool = False,
    forced_tri_mode: Optional[str] = None,
    disable_prune: bool = False,
    prune_mode: str = "full",
    full_rom: Optional[bytes] = None,
    yay0_offset: Optional[int] = None,
) -> Optional[Dict[str, object]]:
    starts: List[int] = []
    for s in [rom_off, max(0, rom_off - 8), max(0, rom_off - 16)]:
        if s in starts:
            continue
        starts.append(s)
    if rom_off < window and 0 not in starts:
        starts.append(0)

    best: Optional[Tuple[List[Vtx], List[Tri], Dict[str, int], int, int, str, float, float]] = None
    candidates: List[Tuple[List[Vtx], List[Tri], Dict[str, int], int, int, str, float, float]] = []
    for base in starts:
        chunk = rom[base : base + window + 16]
        if len(chunk) < 0x300:
            continue
        expected_sig = rom_off - base
        if expected_sig < 0 or (expected_sig + len(MODEL_SIG)) > len(chunk):
            continue
        if chunk[expected_sig : expected_sig + len(MODEL_SIG)] != MODEL_SIG:
            continue

        dl_candidates: List[int] = []
        for d in [expected_sig + 8, expected_sig, expected_sig - 8]:
            if d < 0 or d >= len(chunk):
                continue
            if d in dl_candidates:
                continue
            dl_candidates.append(d)

        for d in dl_candidates:
            for tri_div2 in (True, False):
                mode_name = "div2" if tri_div2 else "raw"
                if forced_tri_mode is not None and mode_name != forced_tri_mode:
                    continue
                try:
                    v_try, t_try, s_try = parse_model(chunk, dl_off=d, tri_div2=tri_div2)
                except Exception:
                    continue
                ok = mesh_quality_ok(v_try, t_try)
                penalty = _mesh_topology_penalty(v_try, t_try)
                long_r = float(_mesh_spaghetti_metrics(v_try, t_try).get("long_edge_ratio", 1.0))
                cand = (v_try, t_try, s_try, d, base, mode_name, penalty, long_r)
                candidates.append(cand)
                # Prefer candidates decoded against the original blob base when
                # possible; segmented offsets are typically blob-relative.
                rank = (1 if ok else 0, -penalty, -base, int(s_try.get("tri_count", 0)), int(s_try.get("vert_count", 0)))
                if best is None:
                    best = cand
                    best_rank = rank
                    continue
                if rank > best_rank:
                    best = cand
                    best_rank = rank

    if best is None:
        return None

    # Prefer base=0 when quality is comparable.
    if best[4] != 0:
        base0_alt: Optional[Tuple[List[Vtx], List[Tri], Dict[str, int], int, int, str, float, float]] = None
        for c in candidates:
            if c[4] == 0:
                if base0_alt is None:
                    base0_alt = c
                    continue
                # Keep better base0 candidate by same ranking axes.
                rank_c = (int(c[2].get("tri_count", 0)), -float(c[6]), int(c[2].get("vert_count", 0)))
                rank_b = (int(base0_alt[2].get("tri_count", 0)), -float(base0_alt[6]), int(base0_alt[2].get("vert_count", 0)))
                if rank_c > rank_b:
                    base0_alt = c
        if base0_alt is not None:
            pen_best = float(best[6])
            pen_b0 = float(base0_alt[6])
            tri_best = int(best[2].get("tri_count", 0))
            tri_b0 = int(base0_alt[2].get("tri_count", 0))
            # Accept base0 if it is not clearly worse.
            if pen_b0 <= (pen_best + 0.10) and tri_b0 >= int(tri_best * 0.85):
                best = base0_alt

    # Prefer div2 when it preserves substantially more topology and is not much
    # noisier than raw; this matches validated references and AFEBA0-like assets.
    if best[5] == "raw":
        raw = best
        div2_alt: Optional[Tuple[List[Vtx], List[Tri], Dict[str, int], int, int, str, float, float]] = None
        for c in candidates:
            if c[5] != "div2":
                continue
            # same parse window context is most comparable; otherwise still allow fallback
            if c[3] == raw[3] and c[4] == raw[4]:
                div2_alt = c
                break
            if div2_alt is None:
                div2_alt = c
        if div2_alt is not None:
            tri_raw = max(1, int(raw[2].get("tri_count", 0)))
            tri_div = int(div2_alt[2].get("tri_count", 0))
            pen_raw = float(raw[6])
            pen_div = float(div2_alt[6])
            long_raw = float(raw[7])
            long_div = float(div2_alt[7])
            if tri_div >= int(tri_raw * 1.20) and pen_div <= (pen_raw + 0.04) and long_div <= (long_raw + 0.01):
                best = div2_alt

    verts, tris, stats, dl_used, base_used, tri_mode_used, _pen_used, _long_used = best
    tri_states: List[Optional[TexState]] = []
    tri_uv_info: List[UvInfo] = []
    chunk_tex_for_decode: Optional[bytes] = None
    seg_base_hint: Optional[Dict[int, int]] = None
    if texture:
        try:
            chunk_tex = rom[base_used : base_used + window + 16]
            cand: List[Tuple[bytes, Optional[Dict[int, int]]]] = [(chunk_tex, None)]
            if full_rom is not None and yay0_offset is not None:
                for c_blob, c_hint, _c_base in _seg4_bridge_candidates(
                    local_blob=chunk_tex,
                    full_rom=full_rom,
                    yay0_off=int(yay0_offset),
                    dl_off=dl_used,
                ):
                    cand.append((c_blob, c_hint))

            best_pick: Optional[Tuple[float, bytes, Optional[Dict[int, int]], List[Optional[TexState]], List[UvInfo]]] = None
            for c_blob, c_hint in cand:
                try:
                    v2, t2, _s2, ts2, uv2 = parse_model_with_texstates(
                        c_blob,
                        dl_off=dl_used,
                        tri_div2=(tri_mode_used == "div2"),
                        seg_bases_init=c_hint,
                    )
                except Exception:
                    continue
                if len(v2) != len(verts) or len(t2) != len(tris):
                    continue
                score = _tri_state_decode_score(c_blob, ts2)
                if best_pick is None or score < best_pick[0]:
                    best_pick = (score, c_blob, c_hint, ts2, uv2)

            if best_pick is not None:
                _s, c_blob, c_hint, ts2, uv2 = best_pick
                chunk_tex_for_decode = c_blob
                seg_base_hint = c_hint
                tri_states = ts2
                tri_uv_info = uv2
            else:
                chunk_tex_for_decode = chunk_tex
        except Exception:
            tri_states = []
            tri_uv_info = []
    if stats["tri_count"] <= 0 or stats["vert_count"] <= 0:
        return None

    pruned_tris = 0
    if strict_mesh:
        if not disable_prune:
            if tri_states:
                verts, tris, tri_states, tri_uv_info, pruned_tris = _prune_outlier_tris_with_states(
                    verts, tris, tri_states, tri_uv_info=tri_uv_info, mode=prune_mode
                )
            else:
                verts, tris, pruned_tris = _prune_outlier_tris(verts, tris, mode=prune_mode)
        stats["tri_count"] = len(tris)
        stats["vert_count"] = len(verts)
        if stats["tri_count"] <= 0 or stats["vert_count"] <= 0:
            return None
    if strict_mesh:
        if not mesh_quality_strict_ok(verts, tris):
            return None
    elif not mesh_quality_ok(verts, tris):
        return None

    if standalone_only and (int(stats.get("external_matrix_refs", 0)) > 0 or int(stats.get("external_dl_refs", 0)) > 0):
        return None

    tri_uv_areas: Optional[List[float]] = None
    uv_out_ratio = 0.0
    if tri_uv_info:
        tri_uv_areas = _tri_uv_areas(verts, tris, tri_uv_info)
        uv_out_ratio = _uv_out_of_range_ratio(verts, tris, tri_uv_info)

    textured_tri_ratio = 0.0
    if tri_states:
        textured_tri_ratio = sum(1 for st in tri_states if st is not None) / max(1, len(tris))

    wrote_texture = False
    tri_mats: Optional[List[str]] = None
    mat_tex: Optional[Dict[str, str]] = None
    texture_materials = 0
    if texture and Image is not None:
        try:
            chunk_tex = chunk_tex_for_decode if chunk_tex_for_decode is not None else rom[base_used : base_used + window + 16]
            if tri_states:
                tri_mats, mat_tex, texture_materials = _build_materials_from_tri_states(
                    chunk_tex, out_base, tri_states, tri_uv_areas=tri_uv_areas
                )
                if tri_mats and tri_uv_info:
                    _apply_known_texture_fixes(
                        yay0_offset=yay0_offset,
                        rom_off=rom_off,
                        verts=verts,
                        tris=tris,
                        tri_states=tri_states,
                        tri_mats=tri_mats,
                        tri_uv_info=tri_uv_info,
                    )
            if tri_mats and mat_tex:
                wrote_texture = True
            else:
                best = _best_texture_from_tri_states(chunk_tex, tri_states, tri_uv_areas=tri_uv_areas)
                if best is not None:
                    pixels, w, h = best
                else:
                    try:
                        pixels, w, h = extract_ci4_texture_from_dl(chunk_tex, dl_off=dl_used)
                    except Exception:
                        pixels, w, h = extract_ci4_texture(chunk_tex)
                _save_texture_png(out_base.with_suffix(".png"), pixels, w, h)
                wrote_texture = True
        except Exception:
            wrote_texture = False

    write_obj_mtl(
        out_base,
        verts,
        tris,
        with_texture=wrote_texture,
        tri_materials=tri_mats,
        material_textures=mat_tex,
        tri_uv_info=tri_uv_info if tri_uv_info else None,
    )
    mesh_penalty = _mesh_topology_penalty(verts, tris)
    mesh_long = float(_mesh_spaghetti_metrics(verts, tris).get("long_edge_ratio", 1.0))
    mesh_conf = 1.0 / (1.0 + mesh_penalty + (2.0 * mesh_long) + (2.0 * uv_out_ratio))
    tex_conf = 0.0
    if wrote_texture:
        tex_conf = 0.9 if texture_materials > 0 else 0.6
    return {
        "rom_offset": rom_off,
        "name": out_base.name,
        "obj": str(out_base.with_suffix(".obj")),
        "mtl": str(out_base.with_suffix(".mtl")) if wrote_texture else None,
        "png": (
            str(out_base.with_suffix(".png"))
            if (wrote_texture and texture_materials == 0)
            else (str(out_base.with_name(f"{out_base.stem}_tex00.png")) if texture_materials > 0 else None)
        ),
        "tri_count": stats["tri_count"],
        "vert_count": stats["vert_count"],
        "da_count": stats.get("da_count", 0),
        "de_count": stats.get("de_count", 0),
        "external_matrix_refs": stats.get("external_matrix_refs", 0),
        "external_dl_refs": stats.get("external_dl_refs", 0),
        "pruned_outlier_tris": pruned_tris,
        "dl_offset_used": dl_used,
        "base_offset_used": base_used,
        "tri_index_mode": tri_mode_used,
        "texture_materials": texture_materials,
        "mesh_penalty": mesh_penalty,
        "mesh_long_edge_ratio": mesh_long,
        "mesh_confidence": mesh_conf,
        "uv_out_of_range_ratio": uv_out_ratio,
        "textured_tri_ratio": textured_tri_ratio,
        "texture_confidence": tex_conf,
        "bbox": _bbox(verts),
    }


def cmd_batch_models(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    scan = scan_rom(rom)
    offsets: List[int] = list(scan["model_offsets"])  # type: ignore[index]
    if args.limit:
        offsets = offsets[: args.limit]

    manifest: Dict[str, object] = {
        "rom": args.rom,
        "candidate_count": len(scan["model_offsets"]),  # type: ignore[index]
        "export_attempted": len(offsets),
        "exported": [],
        "failed_offsets": [],
    }
    exported: List[Dict[str, object]] = []
    failed: List[int] = []

    for i, off in enumerate(offsets):
        name = f"mdl_{i:04d}_{off:08X}"
        out_base = out_dir / name
        try:
            info = _export_one_model(
                rom=rom,
                rom_off=off,
                out_base=out_base,
                window=args.window,
                texture=args.texture,
                strict_mesh=args.strict_mesh,
                standalone_only=args.standalone_only,
                forced_tri_mode=None,
                disable_prune=False,
                prune_mode="full",
            )
            if info is None:
                failed.append(off)
                continue
            if int(info["tri_count"]) < args.min_tris:
                # Filter tiny/false-positive meshes.
                for p in [out_base.with_suffix(".obj"), out_base.with_suffix(".mtl"), out_base.with_suffix(".png")]:
                    if p.exists():
                        p.unlink()
                continue
            exported.append(info)
        except Exception:
            failed.append(off)

    manifest["exported"] = exported
    manifest["failed_offsets"] = failed
    manifest["exported_count"] = len(exported)
    manifest["failed_count"] = len(failed)

    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"outdir": str(out_dir), "manifest": str(out_manifest), "exported_count": len(exported), "failed_count": len(failed)}, indent=2))
    return 0


def cmd_batch_models_yay0(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    yay0_offsets = find_yay0_headers(rom)
    if args.limit:
        yay0_offsets = yay0_offsets[: args.limit]

    manifest: Dict[str, object] = {
        "rom": args.rom,
        "yay0_count": len(find_yay0_headers(rom)),
        "yay0_processed": len(yay0_offsets),
        "chunks": [],
        "exported_count": 0,
    }
    exported_count = 0

    for yi, yoff in enumerate(yay0_offsets):
        chunk_info: Dict[str, object] = {"yay0_offset": yoff, "models": [], "error": None}
        try:
            dec, comp_len = yay0_decompress(rom, yoff)
            chunk_info["decompressed_size"] = len(dec)
            chunk_info["compressed_size_est"] = comp_len

            sigs = list(iter_model_headers(dec))
            if not args.all_signatures:
                sigs = [s for s in sigs if s in (8, 16)]
            models: List[Dict[str, object]] = []
            for mi, moff in enumerate(sigs):
                name = f"y{yi:04d}_{yoff:08X}_m{mi:02d}_{moff:06X}"
                out_base = out_dir / name
                tri_mode_overrides: Dict[Tuple[int, int], str] = {
                    # User-validated decode choices.
                    (0x00AFEBA0, 0x10): "div2",
                    (0x00B00C4C, 0x10): "div2",
                    (0x00B22EB4, 0x10): "div2",
                    (0x00B22068, 0x10): "div2",
                    (0x00B22068, 0x1A0): "div2",
                    (0x00B22068, 0x2B0): "div2",
                }
                prune_disable_overrides: Dict[Tuple[int, int], bool] = {
                    # Keep pruning enabled for y0516, but with a gentler mode.
                    (0x00B22EB4, 0x10): False,
                }
                prune_mode_overrides: Dict[Tuple[int, int], str] = {
                    # Remove only clearly degenerate faces to fix local spikes
                    # without deleting valid detail.
                    (0x00B22EB4, 0x10): "degenerate",
                }
                forced_mode = tri_mode_overrides.get((int(yoff), int(moff)))
                disable_prune = bool(prune_disable_overrides.get((int(yoff), int(moff)), False))
                prune_mode = str(prune_mode_overrides.get((int(yoff), int(moff)), "full"))
                try:
                    info = _export_one_model(
                        rom=dec,
                        rom_off=moff,
                        out_base=out_base,
                        window=args.window,
                        texture=args.texture,
                        strict_mesh=args.strict_mesh,
                        standalone_only=args.standalone_only,
                        forced_tri_mode=forced_mode,
                        disable_prune=disable_prune,
                        prune_mode=prune_mode,
                        full_rom=rom,
                        yay0_offset=yoff,
                    )
                    if info is None:
                        continue
                    if int(info["tri_count"]) < args.min_tris:
                        for p in [out_base.with_suffix(".obj"), out_base.with_suffix(".mtl"), out_base.with_suffix(".png")]:
                            if p.exists():
                                p.unlink()
                        continue
                    info["yay0_offset"] = yoff
                    info["yay0_index"] = yi
                    info["decomp_model_offset"] = moff
                    models.append(info)
                    exported_count += 1
                except Exception:
                    continue
            chunk_info["models"] = models
            chunk_info["model_count"] = len(models)
            if args.merge_chunk_models and len(models) >= 2:
                merged_base = out_dir / f"y{yi:04d}_{yoff:08X}_merged"
                merged = _merge_model_objs(models, merged_base)
                chunk_info["merged"] = merged
        except Exception as ex:
            chunk_info["error"] = str(ex)
        cast_chunks = manifest["chunks"]
        assert isinstance(cast_chunks, list)
        cast_chunks.append(chunk_info)

    manifest["exported_count"] = exported_count
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"outdir": str(out_dir), "manifest": str(out_manifest), "exported_count": exported_count}, indent=2))
    return 0


def cmd_extract_bzn(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    entries: List[Dict[str, object]] = []
    seen_names: Dict[str, int] = {}
    pat = re.compile(rb"([A-Za-z0-9_\-]{3,24}\.bzn)", re.IGNORECASE)

    for off in find_yay0_headers(rom):
        try:
            dec, comp_len = yay0_decompress(rom, off)
        except Exception:
            continue
        m = pat.search(dec[:128])
        if not m:
            continue
        raw_name = m.group(1).decode("ascii", errors="ignore")
        norm = raw_name.lower()
        suffix = seen_names.get(norm, 0)
        seen_names[norm] = suffix + 1
        stem = raw_name[:-4]
        if suffix:
            out_name = f"{stem}__{off:08X}__{suffix}.bzn"
        else:
            out_name = f"{stem}__{off:08X}.bzn"
        out_path = out_dir / out_name
        out_path.write_bytes(dec)

        entries.append(
            {
                "name": raw_name,
                "rom_offset": off,
                "decompressed_size": len(dec),
                "compressed_size_est": comp_len,
                "name_offset": int(m.start()),
                "output": str(out_path),
            }
        )

    manifest = {
        "rom": args.rom,
        "count": len(entries),
        "entries": entries,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"outdir": str(out_dir), "manifest": str(out_manifest), "count": len(entries)}, indent=2))
    return 0


def _bzn_ref_record(path: pathlib.Path) -> Dict[str, object]:
    d = path.read_bytes()
    if len(d) < 0xD4:
        raise ValueError(f"File too small: {path}")
    name = d[0x0B : 0x1F].split(b"\x00", 1)[0].decode("ascii", errors="ignore")
    rec = {
        "file": str(path),
        "name": name,
        "size": len(d),
        "header_id": int(struct.unpack_from(">H", d, 4)[0]),
        # These fields are stable across the map container family and vary by map.
        "ref_a": int(struct.unpack_from(">H", d, 0x78)[0]),
        "ref_b": int(struct.unpack_from(">H", d, 0x7A)[0]),
        "ref_c": int(struct.unpack_from(">H", d, 0x7C)[0]),
        "ref_d": int(struct.unpack_from(">H", d, 0x7E)[0]),
        # Common vec3-ish block near 0x80.
        "vec0": float(struct.unpack_from(">f", d, 0x80)[0]),
        "vec1": float(struct.unpack_from(">f", d, 0x84)[0]),
        "vec2": float(struct.unpack_from(">f", d, 0x88)[0]),
        # Common orientation-ish block near 0xA4.
        "ang0": float(struct.unpack_from(">f", d, 0xA4)[0]),
        "ang1": float(struct.unpack_from(">f", d, 0xA8)[0]),
        "ang2": float(struct.unpack_from(">f", d, 0xAC)[0]),
    }
    return rec


def cmd_bzn_refs(args: argparse.Namespace) -> int:
    in_dir = pathlib.Path(args.indir)
    files = sorted(in_dir.glob("*.bzn"))
    records: List[Dict[str, object]] = []
    for p in files:
        try:
            records.append(_bzn_ref_record(p))
        except Exception:
            continue

    # Group files by the main reference tuple to highlight shared terrain/object refs.
    groups: Dict[str, List[str]] = {}
    for r in records:
        key = f"{r['ref_a']}:{r['ref_b']}:{r['ref_c']}:{r['ref_d']}"
        groups.setdefault(key, []).append(str(r["name"]))

    out = {
        "input_dir": str(in_dir),
        "count": len(records),
        "records": records,
        "groups": groups,
    }
    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps({"out": str(out_path), "count": len(records), "groups": len(groups)}, indent=2))
    return 0


def _extract_ascii_strings(data: bytes, min_len: int = 4, max_strings: int = 120) -> List[Dict[str, object]]:
    out: List[Dict[str, object]] = []
    start = -1
    n = len(data)
    for i in range(n):
        b = data[i]
        if 0x20 <= b <= 0x7E:
            if start < 0:
                start = i
            continue
        if start >= 0:
            ln = i - start
            if ln >= min_len:
                txt = data[start:i].decode("ascii", errors="ignore")
                out.append({"offset": start, "length": ln, "text": txt})
            start = -1
    if start >= 0:
        ln = n - start
        if ln >= min_len:
            txt = data[start:n].decode("ascii", errors="ignore")
            out.append({"offset": start, "length": ln, "text": txt})
    out.sort(key=lambda r: int(r["offset"]))
    return out[: max(1, int(max_strings))]


def _top_be_u16_counts(data: bytes, top_n: int = 32, min_count: int = 3) -> List[Dict[str, object]]:
    cnt: Counter[int] = Counter()
    for i in range(0, len(data) - 1, 2):
        cnt[struct.unpack_from(">H", data, i)[0]] += 1
    items = [(v, c) for v, c in cnt.items() if int(c) >= int(min_count)]
    items.sort(key=lambda t: (-int(t[1]), int(t[0])))
    out: List[Dict[str, object]] = []
    for v, c in items[: max(1, int(top_n))]:
        out.append({"value": int(v), "hex": f"0x{int(v):04X}", "count": int(c)})
    return out


def _top_be_u32_counts(data: bytes, top_n: int = 32, min_count: int = 2) -> List[Dict[str, object]]:
    cnt: Counter[int] = Counter()
    for i in range(0, len(data) - 3, 4):
        cnt[struct.unpack_from(">I", data, i)[0]] += 1
    items = [(v, c) for v, c in cnt.items() if int(c) >= int(min_count)]
    items.sort(key=lambda t: (-int(t[1]), int(t[0])))
    out: List[Dict[str, object]] = []
    for v, c in items[: max(1, int(top_n))]:
        out.append({"value": int(v), "hex": f"0x{int(v):08X}", "count": int(c)})
    return out


def _find_be_pointer_runs(data: bytes, min_run: int = 4, max_runs: int = 24) -> List[Dict[str, object]]:
    n = len(data)
    out: List[Dict[str, object]] = []
    i = 0
    while i + 4 <= n:
        v0 = be32(data, i)
        if not (0 < v0 < n and (v0 % 4) == 0):
            i += 4
            continue
        j = i + 4
        prev = v0
        deltas: List[int] = []
        while j + 4 <= n:
            v = be32(data, j)
            if not (0 < v < n and (v % 4) == 0 and v > prev and (v - prev) <= 0x40000):
                break
            deltas.append(v - prev)
            prev = v
            j += 4
        count = (j - i) // 4
        if count >= max(2, int(min_run)):
            step_hint = 0
            if deltas:
                step_hint = int(Counter(deltas).most_common(1)[0][0])
            out.append(
                {
                    "table_offset": i,
                    "entry_count": int(count),
                    "first_ptr": int(v0),
                    "last_ptr": int(prev),
                    "span": int(prev - v0),
                    "step_hint": int(step_hint),
                }
            )
            i = j
            continue
        i += 4
    out.sort(key=lambda r: (-int(r["entry_count"]), int(r["table_offset"])))
    return out[: max(1, int(max_runs))]


def _find_be_float_runs(
    data: bytes,
    min_run: int = 3,
    max_abs: float = 1000000.0,
    max_runs: int = 24,
) -> List[Dict[str, object]]:
    def _is_reasonable(v: float) -> bool:
        if not math.isfinite(v):
            return False
        if abs(v) > max_abs:
            return False
        return True

    n = len(data)
    out: List[Dict[str, object]] = []
    run_start = -1
    run_vals: List[float] = []
    for off in range(0, n - 3, 4):
        v = struct.unpack_from(">f", data, off)[0]
        if _is_reasonable(float(v)):
            if run_start < 0:
                run_start = off
                run_vals = []
            run_vals.append(float(v))
            continue
        if run_start >= 0:
            if len(run_vals) >= max(2, int(min_run)):
                nz = sum(1 for x in run_vals if abs(x) > 1e-6)
                if nz >= max(1, len(run_vals) // 4):
                    out.append(
                        {
                            "offset": int(run_start),
                            "count": int(len(run_vals)),
                            "min": float(min(run_vals)),
                            "max": float(max(run_vals)),
                            "sample": [float(x) for x in run_vals[:8]],
                        }
                    )
            run_start = -1
            run_vals = []
    if run_start >= 0 and len(run_vals) >= max(2, int(min_run)):
        nz = sum(1 for x in run_vals if abs(x) > 1e-6)
        if nz >= max(1, len(run_vals) // 4):
            out.append(
                {
                    "offset": int(run_start),
                    "count": int(len(run_vals)),
                    "min": float(min(run_vals)),
                    "max": float(max(run_vals)),
                    "sample": [float(x) for x in run_vals[:8]],
                }
            )
    out.sort(key=lambda r: (-int(r["count"]), int(r["offset"])))
    return out[: max(1, int(max_runs))]


def _byte_entropy(data: bytes) -> float:
    if not data:
        return 0.0
    n = float(len(data))
    cnt: Counter[int] = Counter(data)
    ent = 0.0
    for c in cnt.values():
        p = float(c) / n
        if p > 0.0:
            ent -= p * math.log2(p)
    return ent


_PC_BZN_FIELD_NAMES: Dict[int, str] = {
    0: "DATA_VOID",
    1: "DATA_BOOL",
    2: "DATA_CHAR",
    3: "DATA_SHORT",
    4: "DATA_LONG",
    5: "DATA_FLOAT",
    6: "DATA_DOUBLE",
    7: "DATA_ID",
    8: "DATA_PTR",
    9: "DATA_VEC3D",
    10: "DATA_VEC2D",
    11: "DATA_MAT3DOLD",
    12: "DATA_MAT3D",
    13: "DATA_STRING",
    14: "DATA_QUAT",
}


def _pc_bzn_default_priority_names() -> set[str]:
    out: set[str] = set()
    for i in range(1, 12):
        out.add(f"arcd{i:02d}")
    for i in range(1, 9):
        out.add(f"arcdb{i:02d}")
        out.add(f"arcds{i:02d}")
    for i in range(1, 7):
        out.add(f"n64dm{i:02d}")
    for i in range(1, 5):
        out.add(f"race{i:02d}")
    out.add("misn00")
    for i in range(20, 28):
        out.add(f"misn{i:02d}")
    return out


def _base_bzn_stem(path: pathlib.Path) -> str:
    # Extract canonical stem from filenames like n64dm01__00A9CFB4.bzn
    # while preserving plain names like misn00.bzn.
    stem = path.stem.lower()
    if "__" in stem:
        stem = stem.split("__", 1)[0]
    return stem


def _filter_bzn_files(
    files: Sequence[pathlib.Path],
    focus_patterns: Sequence[str],
    use_priority_set: bool,
) -> List[pathlib.Path]:
    pats = [p.strip().lower() for p in focus_patterns if str(p).strip()]
    priority = _pc_bzn_default_priority_names() if use_priority_set else set()
    if not pats and not priority:
        return list(files)
    out: List[pathlib.Path] = []
    for p in files:
        stem = _base_bzn_stem(p)
        if stem in priority:
            out.append(p)
            continue
        if stem.startswith("d") and stem[1:] in priority:
            out.append(p)
            continue
        if any(fnmatch.fnmatch(stem, pat) for pat in pats):
            out.append(p)
            continue
    return out


def _parse_pc_style_token_stream(
    data: bytes,
    start: int,
    max_fields: int = 512,
    max_payload_size: int = 1024,
    min_fields: int = 8,
) -> Optional[Dict[str, object]]:
    n = len(data)
    if start < 0 or start + 4 > n:
        return None

    pos = start
    fields = 0
    type_hist: Counter[int] = Counter()
    id_like: List[str] = []
    first_types: List[int] = []
    token_re = re.compile(r"^[A-Za-z0-9_\-\.]{3,64}$")

    while pos + 4 <= n and fields < max(1, int(max_fields)):
        t = int(data[pos])
        if t not in _PC_BZN_FIELD_NAMES:
            break
        sz = int(struct.unpack_from("<H", data, pos + 2)[0])
        if sz > max(0, int(max_payload_size)):
            break
        chunk = pos + 4
        end = chunk + sz
        if end > n:
            break

        type_hist[t] += 1
        if len(first_types) < 12:
            first_types.append(t)

        if t in (7, 13):
            raw = data[chunk:end]
            txt = raw.decode("ascii", errors="ignore").strip("\x00").strip()
            if token_re.match(txt) and not txt.isdigit():
                id_like.append(txt)

        fields += 1
        pos = end

    if fields < max(2, int(min_fields)):
        return None

    uniq_ids: List[str] = []
    seen: set[str] = set()
    for s in id_like:
        k = s.lower()
        if k in seen:
            continue
        seen.add(k)
        uniq_ids.append(s)

    non_void_fields = int(fields - int(type_hist.get(0, 0)))
    # Reject degenerate streams that are effectively only DATA_VOID tokens.
    if non_void_fields <= 0 and not uniq_ids:
        return None
    if non_void_fields < 2 and not uniq_ids:
        return None

    type_names = [
        {"type": int(k), "name": _PC_BZN_FIELD_NAMES.get(int(k), f"T{k}"), "count": int(v)}
        for k, v in sorted(type_hist.items(), key=lambda kv: (-int(kv[1]), int(kv[0])))
    ]
    score = float(fields) + float(type_hist.get(7, 0) * 3) + float(min(20, len(uniq_ids)) * 1.5)

    return {
        "start": int(start),
        "end": int(pos),
        "span": int(pos - start),
        "field_count": int(fields),
        "score": float(score),
        "types": type_names,
        "first_types": [int(x) for x in first_types],
        "id_like_strings": uniq_ids[:24],
    }


def _find_pc_style_token_candidates(
    data: bytes,
    max_results: int = 8,
    min_fields: int = 8,
) -> List[Dict[str, object]]:
    n = len(data)
    if n < 16:
        return []

    starts: List[int] = []
    # Probe all byte offsets, but only keep starts that have a plausible first+next token.
    for i in range(0, n - 8):
        t0 = int(data[i])
        if t0 not in _PC_BZN_FIELD_NAMES:
            continue
        sz0 = int(struct.unpack_from("<H", data, i + 2)[0])
        if sz0 > 1024:
            continue
        n1 = i + 4 + sz0
        if n1 + 4 > n:
            continue
        t1 = int(data[n1])
        if t1 not in _PC_BZN_FIELD_NAMES:
            continue
        starts.append(i)

    if not starts:
        return []

    # Keep density manageable for large files.
    if len(starts) > 4096:
        step = max(1, len(starts) // 4096)
        starts = starts[::step]

    recs: List[Dict[str, object]] = []
    for st in starts:
        rec = _parse_pc_style_token_stream(
            data,
            start=int(st),
            max_fields=512,
            max_payload_size=1024,
            min_fields=max(2, int(min_fields)),
        )
        if rec is None:
            continue
        recs.append(rec)

    if not recs:
        return []

    # Dedup near-identical spans.
    uniq: List[Dict[str, object]] = []
    seen_keys: set[Tuple[int, int]] = set()
    for r in sorted(recs, key=lambda x: (-float(x["score"]), -int(x["field_count"]), int(x["start"]))):
        key = (int(r["start"]) // 8, int(r["end"]) // 8)
        if key in seen_keys:
            continue
        seen_keys.add(key)
        uniq.append(r)
        if len(uniq) >= max(1, int(max_results)):
            break
    return uniq


def _default_id_exclude_values(scan_type: str) -> set[int]:
    common = {
        0,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        20,
        24,
        28,
        32,
        48,
        64,
        80,
        96,
        112,
        128,
        160,
        192,
        224,
        255,
        256,
        512,
        1024,
        2048,
        4096,
        8192,
        16384,
        32768,
        49152,
        65535,
    }
    if scan_type == "u16":
        common.update(
            {
                0x3F80,  # 1.0f high-word in BE
                0x4000,  # 2.0f high-word
                0x4040,
                0x4080,
                0x40A0,
                0x40C0,
                0x4100,
                0x4120,
                0x4140,
                0x4180,
                0x41A0,
                0x41C0,
                0x4200,
                0x4240,
                0x4280,
                0x42A0,
                0x42C0,
                0x4300,
                0x4340,
                0x4380,
                0x43A0,
                0x43C0,
                0x4400,
                0x4480,
                0x4500,
                0x4580,
                0x4600,
                0x4680,
                0x4700,
                0x4780,
                0x47C0,
            }
        )
    return common


def _scan_numeric_values(
    data: bytes,
    scan_type: str,
    byte_order: str,
    alignment: int,
    min_value: int,
    max_value: int,
    skip_bytes: int,
) -> Iterable[Tuple[int, int]]:
    n = len(data)
    if scan_type not in {"u16", "u32"}:
        raise ValueError(f"Unsupported scan_type: {scan_type}")
    if byte_order not in {"be", "le"}:
        raise ValueError(f"Unsupported byte_order: {byte_order}")

    item_size = 2 if scan_type == "u16" else 4
    step = max(1, int(alignment))
    start = max(0, int(skip_bytes))
    if step > 1:
        start = ((start + step - 1) // step) * step

    fmt = ">H" if (scan_type == "u16" and byte_order == "be") else "<H"
    if scan_type == "u32":
        fmt = ">I" if byte_order == "be" else "<I"

    for off in range(start, n - item_size + 1, step):
        v = int(struct.unpack_from(fmt, data, off)[0])
        if v < min_value or v > max_value:
            continue
        yield off, v


def _normalize_odf_name(raw: str) -> Optional[str]:
    s = raw.strip().strip(",").strip().strip("'").strip('"').strip()
    if not s:
        return None
    if s in {"[", "]"}:
        return None
    if s.startswith("["):
        s = s[1:].strip()
    if s.endswith("]"):
        s = s[:-1].strip()
    s = s.strip().strip(",").strip().strip("'").strip('"').strip()
    if not s:
        return None
    s = s.lower()
    if s.endswith(".odf"):
        base = s[:-4]
    else:
        base = s
    if not re.fullmatch(r"[a-z0-9_\-]{2,64}", base):
        return None
    return f"{base}.odf"


def _load_stock_odf_list(path: pathlib.Path) -> List[str]:
    rows: List[str] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        ln = raw_line.split("#", 1)[0].strip()
        if not ln:
            continue
        nm = _normalize_odf_name(ln)
        if nm:
            rows.append(nm)
    # Handle single-line Python-style lists if present.
    if not rows:
        text = path.read_text(encoding="utf-8", errors="ignore")
        for tok in re.findall(r"[A-Za-z0-9_\-]{2,64}(?:\.odf)?", text):
            nm = _normalize_odf_name(tok)
            if nm:
                rows.append(nm)
    return sorted(set(rows))


def _extract_stock_refs_from_blob(data: bytes, stock_set: set[str]) -> set[str]:
    if not stock_set:
        return set()
    refs: set[str] = set()
    strings = _extract_ascii_strings(data, min_len=3, max_strings=2000)
    for sr in strings:
        txt = str(sr.get("text", ""))
        for tok in re.findall(r"[A-Za-z][A-Za-z0-9_\-]{2,64}(?:\.odf)?", txt):
            nm = _normalize_odf_name(tok)
            if nm and nm in stock_set:
                refs.add(nm)
    return refs


def cmd_bzn_id_census(args: argparse.Namespace) -> int:
    in_dir = pathlib.Path(args.indir)
    all_files = sorted(in_dir.glob("*.bzn"))
    files = _filter_bzn_files(
        all_files,
        focus_patterns=list(getattr(args, "focus_name", []) or []),
        use_priority_set=bool(getattr(args, "focus_priority_set", False)),
    )
    if not files:
        raise FileNotFoundError(f"No matching .bzn files found in {in_dir}")

    scan_type = str(args.scan_type).lower()
    byte_order = str(args.byte_order).lower()
    alignment = int(args.alignment)
    skip_bytes = max(0, int(args.skip_bytes))
    min_value = int(args.min_value)
    max_value = int(args.max_value)
    min_global_count = max(1, int(args.min_global_count))
    min_file_count = max(1, int(args.min_file_count))
    top_values = max(1, int(args.top_values))
    per_file_top = max(1, int(args.per_file_top))
    sample_offsets_per_value = max(1, int(args.sample_offsets_per_value))

    exclude_values: set[int] = set()
    if bool(args.exclude_defaults):
        exclude_values.update(_default_id_exclude_values(scan_type))
    for raw in list(getattr(args, "exclude_value", []) or []):
        exclude_values.add(int(str(raw), 0))

    stock_odf_list_path = str(getattr(args, "stock_odf_list", "") or "").strip()
    stock_odf: List[str] = []
    stock_set: set[str] = set()
    if stock_odf_list_path:
        sp = pathlib.Path(stock_odf_list_path)
        if not sp.exists():
            raise FileNotFoundError(f"Stock ODF list not found: {sp}")
        stock_odf = _load_stock_odf_list(sp)
        stock_set = set(stock_odf)

    global_counts: Counter[int] = Counter()
    value_files: Dict[int, set[str]] = defaultdict(set)
    value_offsets: Dict[int, List[Dict[str, object]]] = defaultdict(list)
    stock_ref_files: Dict[str, set[str]] = defaultdict(set)
    per_file_records: List[Dict[str, object]] = []
    scanned_value_total = 0

    for p in files:
        data = p.read_bytes()
        local_counts: Counter[int] = Counter()
        local_seen: set[int] = set()

        for off, v in _scan_numeric_values(
            data=data,
            scan_type=scan_type,
            byte_order=byte_order,
            alignment=alignment,
            min_value=min_value,
            max_value=max_value,
            skip_bytes=skip_bytes,
        ):
            if v in exclude_values:
                continue
            scanned_value_total += 1
            local_counts[v] += 1
            global_counts[v] += 1
            local_seen.add(v)
            lst = value_offsets[v]
            if len(lst) < sample_offsets_per_value:
                lst.append({"file": p.name, "offset": int(off)})

        for v in local_seen:
            value_files[v].add(p.name)

        observed_stock_refs: List[str] = []
        if stock_set:
            srefs = _extract_stock_refs_from_blob(data, stock_set)
            observed_stock_refs = sorted(srefs)
            for nm in observed_stock_refs:
                stock_ref_files[nm].add(p.name)

        local_top = [
            {"value": int(v), "hex": f"0x{int(v):04X}" if scan_type == "u16" else f"0x{int(v):08X}", "count": int(c)}
            for v, c in local_counts.most_common(per_file_top)
        ]
        per_file_records.append(
            {
                "file": str(p),
                "stem": _base_bzn_stem(p),
                "size": len(data),
                "unique_values": len(local_counts),
                "observed_stock_odf_refs": observed_stock_refs,
                "top_values": local_top,
            }
        )

    ranked_values: List[Dict[str, object]] = []
    for v, c in global_counts.items():
        fc = len(value_files.get(v, set()))
        if c < min_global_count or fc < min_file_count:
            continue
        ranked_values.append(
            {
                "value": int(v),
                "hex": f"0x{int(v):04X}" if scan_type == "u16" else f"0x{int(v):08X}",
                "global_count": int(c),
                "file_count": int(fc),
                "sample_files": sorted(value_files.get(v, set()))[:10],
                "sample_offsets": value_offsets.get(v, []),
            }
        )
    ranked_values.sort(key=lambda r: (-int(r["global_count"]), -int(r["file_count"]), int(r["value"])))

    stock_refs_ranked: List[Dict[str, object]] = []
    for nm, fset in stock_ref_files.items():
        stock_refs_ranked.append(
            {
                "odf": nm,
                "file_count": len(fset),
                "sample_files": sorted(fset)[:12],
            }
        )
    stock_refs_ranked.sort(key=lambda r: (-int(r["file_count"]), str(r["odf"])))

    id_map_rows: List[Dict[str, object]] = []
    for rank, rec in enumerate(ranked_values[:top_values], start=1):
        id_map_rows.append(
            {
                "rank": rank,
                "id": int(rec["value"]),
                "hex": str(rec["hex"]),
                "global_count": int(rec["global_count"]),
                "file_count": int(rec["file_count"]),
                "mapped_odf": "",
                "status": "todo",
                "notes": "",
            }
        )

    id_map_csv = str(getattr(args, "id_map_csv", "") or "").strip()
    if id_map_csv:
        csv_path = pathlib.Path(id_map_csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="", encoding="utf-8") as fcsv:
            w = csv.DictWriter(
                fcsv,
                fieldnames=["rank", "id", "hex", "global_count", "file_count", "mapped_odf", "status", "notes"],
            )
            w.writeheader()
            w.writerows(id_map_rows)

    summary = {
        "input_dir": str(in_dir),
        "count_total": len(all_files),
        "count_selected": len(files),
        "scan_type": scan_type,
        "byte_order": byte_order,
        "alignment": alignment,
        "skip_bytes": skip_bytes,
        "value_range": [min_value, max_value],
        "exclude_defaults": bool(args.exclude_defaults),
        "exclude_values_count": len(exclude_values),
        "focus_patterns": list(getattr(args, "focus_name", []) or []),
        "focus_priority_set": bool(getattr(args, "focus_priority_set", False)),
        "stock_odf_list": (stock_odf_list_path if stock_odf_list_path else None),
        "stock_odf_loaded": len(stock_odf),
        "stock_odf_observed_in_selected": len(stock_refs_ranked),
        "id_map_csv": (id_map_csv if id_map_csv else None),
        "scanned_value_total": int(scanned_value_total),
        "unique_values_post_filter": len(global_counts),
        "candidate_values": len(ranked_values),
        "top_candidates": ranked_values[:top_values],
    }

    out_obj = {
        "summary": summary,
        "values": ranked_values[:top_values],
        "stock_odf_observed": stock_refs_ranked,
        "id_map_template": id_map_rows,
        "per_file": per_file_records,
    }
    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out_obj, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "out": str(out_path),
                "count_selected": summary["count_selected"],
                "candidate_values": summary["candidate_values"],
                "stock_odf_observed_in_selected": summary["stock_odf_observed_in_selected"],
                "unique_values_post_filter": summary["unique_values_post_filter"],
            },
            indent=2,
        )
    )
    return 0


def cmd_mission_clues(args: argparse.Namespace) -> int:
    in_dir = pathlib.Path(args.indir)
    all_files = sorted(in_dir.glob("*.bzn"))
    files = _filter_bzn_files(
        all_files,
        focus_patterns=list(getattr(args, "focus_name", []) or []),
        use_priority_set=bool(getattr(args, "focus_priority_set", False)),
    )
    if not files:
        raise FileNotFoundError(f"No matching .bzn files found in {in_dir}")

    token_re = re.compile(r"[A-Za-z][A-Za-z0-9_\-]{2,31}")
    token_files: Dict[str, set[str]] = {}
    u16_files: Dict[int, set[str]] = {}
    u16_total: Counter[int] = Counter()
    u16_single_file_count: Dict[str, Counter[int]] = {}

    records: List[Dict[str, object]] = []
    for p in files:
        d = p.read_bytes()
        file_name = p.name

        ref_rec: Optional[Dict[str, object]] = None
        try:
            rr = _bzn_ref_record(p)
            ref_rec = {
                "name": rr.get("name", ""),
                "header_id": rr.get("header_id", 0),
                "ref_a": rr.get("ref_a", 0),
                "ref_b": rr.get("ref_b", 0),
                "ref_c": rr.get("ref_c", 0),
                "ref_d": rr.get("ref_d", 0),
            }
        except Exception:
            ref_rec = None

        strings = _extract_ascii_strings(
            d,
            min_len=max(2, int(args.string_min_len)),
            max_strings=max(1, int(args.max_strings)),
        )
        longest_strings = sorted(strings, key=lambda r: (-int(r["length"]), int(r["offset"])))[:20]
        pointer_runs = _find_be_pointer_runs(
            d,
            min_run=max(2, int(args.min_pointer_run)),
            max_runs=max(1, int(args.max_pointer_runs)),
        )
        pc_token_runs = _find_pc_style_token_candidates(
            d,
            max_results=max(1, int(args.max_pc_token_runs)),
            min_fields=max(2, int(args.min_pc_token_fields)),
        )
        float_runs = _find_be_float_runs(
            d,
            min_run=max(2, int(args.min_float_run)),
            max_runs=max(1, int(args.max_float_runs)),
        )
        u16_top = _top_be_u16_counts(d, top_n=max(1, int(args.top_values)), min_count=max(2, int(args.min_u16_count)))
        u32_top = _top_be_u32_counts(d, top_n=max(1, int(args.top_values)), min_count=max(2, int(args.min_u32_count)))

        token_set: set[str] = set()
        for srec in strings:
            txt = str(srec.get("text", ""))
            for tok in token_re.findall(txt):
                t = tok.lower()
                if len(t) >= 3:
                    token_set.add(t)
        for tok in token_set:
            token_files.setdefault(tok, set()).add(file_name)

        u16_counts_local: Counter[int] = Counter()
        for i in range(0, len(d) - 1, 2):
            v = struct.unpack_from(">H", d, i)[0]
            u16_counts_local[v] += 1
        u16_single_file_count[file_name] = u16_counts_local
        for v, c in u16_counts_local.items():
            u16_total[int(v)] += int(c)
            u16_files.setdefault(int(v), set()).add(file_name)

        rec: Dict[str, object] = {
            "file": str(p),
            "size": len(d),
            "sha1": hashlib.sha1(d).hexdigest(),
            "entropy": float(_byte_entropy(d)),
            "ref_summary": ref_rec,
            "strings": strings,
            "longest_strings": longest_strings,
            "pointer_table_candidates": pointer_runs,
            "pc_token_candidates": pc_token_runs,
            "float_run_candidates_be": float_runs,
            "top_u16_be_values": u16_top,
            "top_u32_be_values": u32_top,
        }
        records.append(rec)

    common_tokens: List[Dict[str, object]] = []
    for tok, fset in token_files.items():
        if len(fset) < 2:
            continue
        common_tokens.append(
            {
                "token": tok,
                "file_count": len(fset),
                "sample_files": sorted(fset)[:6],
            }
        )
    common_tokens.sort(key=lambda r: (-int(r["file_count"]), str(r["token"])))

    common_u16: List[Dict[str, object]] = []
    for v, fset in u16_files.items():
        if len(fset) < 2:
            continue
        tv = int(u16_total.get(v, 0))
        if tv < max(2, int(args.min_u16_count)):
            continue
        common_u16.append(
            {
                "value": int(v),
                "hex": f"0x{int(v):04X}",
                "file_count": len(fset),
                "total_count": tv,
            }
        )
    common_u16.sort(key=lambda r: (-int(r["file_count"]), -int(r["total_count"]), int(r["value"])))

    mission_specific_u16: List[Dict[str, object]] = []
    min_specific = max(2, int(args.min_specific_u16_count))
    for v, fset in u16_files.items():
        if len(fset) != 1:
            continue
        if v in (0, 0xFFFF):
            continue
        only_file = next(iter(fset))
        local_count = int(u16_single_file_count.get(only_file, Counter()).get(v, 0))
        if local_count < min_specific:
            continue
        mission_specific_u16.append(
            {
                "value": int(v),
                "hex": f"0x{int(v):04X}",
                "file": only_file,
                "count": local_count,
            }
        )
    mission_specific_u16.sort(key=lambda r: (-int(r["count"]), str(r["file"]), int(r["value"])))

    summary = {
        "input_dir": str(in_dir),
        "count_total": len(all_files),
        "count_selected": len(records),
        "focus_patterns": list(getattr(args, "focus_name", []) or []),
        "focus_priority_set": bool(getattr(args, "focus_priority_set", False)),
        "files_with_pointer_tables": sum(1 for r in records if r["pointer_table_candidates"]),
        "files_with_pc_token_candidates": sum(1 for r in records if r["pc_token_candidates"]),
        "files_with_float_runs": sum(1 for r in records if r["float_run_candidates_be"]),
        "common_tokens": common_tokens[: max(1, int(args.top_values))],
        "common_u16_ids": common_u16[: max(1, int(args.top_values))],
        "mission_specific_u16_ids": mission_specific_u16[: max(1, int(args.top_values))],
    }

    out_obj = {
        "summary": summary,
        "records": records,
    }
    out_path = pathlib.Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out_obj, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "out": str(out_path),
                "count_selected": summary["count_selected"],
                "files_with_pointer_tables": summary["files_with_pointer_tables"],
                "files_with_pc_token_candidates": summary["files_with_pc_token_candidates"],
                "files_with_float_runs": summary["files_with_float_runs"],
            },
            indent=2,
        )
    )
    return 0


def cmd_terrain_cluster(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if Image is None:
        raise RuntimeError("Pillow is required for terrain PNG export: pip install pillow")

    headers = find_yay0_headers(rom)
    cluster = [off for off in headers if int(args.start, 0) <= off <= int(args.end, 0)]

    records: List[Dict[str, object]] = []
    u16_offs: List[int] = []
    ci4_offs: List[int] = []

    for off in cluster:
        dec, _ = yay0_decompress(rom, off)
        rec: Dict[str, object] = {"offset": off, "decompressed_size": len(dec)}
        bin_path = out_dir / f"{off:08X}.bin"
        bin_path.write_bytes(dec)
        rec["bin"] = str(bin_path)

        if len(dec) == 32768:
            # 128x128 x 16-bit terrain-like layer.
            n = 128 * 128
            hi = [dec[i * 2] for i in range(n)]
            lo = [dec[i * 2 + 1] for i in range(n)]
            be = [struct.unpack_from(">H", dec, i * 2)[0] for i in range(n)]
            le = [struct.unpack_from("<H", dec, i * 2)[0] for i in range(n)]
            
            # Normalize for u16norm output (consistent with before but now explicitly BE)
            mn, mx = min(be), max(be)
            rng = max(1, mx - mn)
            norm = [int((v - mn) * 255 / rng) for v in be]

            for suffix, data in [("hi", hi), ("lo", lo), ("be", hi), ("u16norm", norm)]:
                img = Image.new("L", (128, 128))
                img.putdata(data)
                p = out_dir / f"{off:08X}_{suffix}.png"
                img.save(p)
                rec[suffix] = str(p)

            rec["kind"] = "u16_128x128"
            u16_offs.append(off)

        elif len(dec) == 8192:
            # 128x128 x 4-bit index map.
            need = 128 * 128 // 2
            pix: List[int] = []
            for b in dec[:need]:
                pix.extend([((b >> 4) & 0xF) * 17, (b & 0xF) * 17])
            img = Image.new("L", (128, 128))
            img.putdata(pix)
            p = out_dir / f"{off:08X}_ci4.png"
            img.save(p)
            rec["ci4"] = str(p)
            rec["kind"] = "ci4_128x128"
            ci4_offs.append(off)
        elif len(dec) == 24576:
            # Observed near terrain bank: likely 3 packed 128x128 CI4 layers.
            part_paths: List[str] = []
            for part in range(3):
                chunk = dec[part * 8192 : (part + 1) * 8192]
                pix: List[int] = []
                for b in chunk:
                    pix.extend([((b >> 4) & 0xF) * 17, (b & 0xF) * 17])
                img = Image.new("L", (128, 128))
                img.putdata(pix)
                p = out_dir / f"{off:08X}_part{part}_ci4.png"
                img.save(p)
                part_paths.append(str(p))
            rec["ci4_parts"] = part_paths
            rec["kind"] = "ci4_3x128x128"
        else:
            rec["kind"] = "other"

        records.append(rec)

    # Simple nearest-next pairing: each u16 gets the first ci4 with higher offset.
    pairs: List[Dict[str, object]] = []
    sorted_ci4 = sorted(ci4_offs)
    for uoff in sorted(u16_offs):
        nxt = [c for c in sorted_ci4 if c > uoff]
        if not nxt:
            continue
        pairs.append(
            {
                "u16_offset": uoff,
                "ci4_offset": nxt[0],
                "u16_hi": str(out_dir / f"{uoff:08X}_hi.png"),
                "u16_lo": str(out_dir / f"{uoff:08X}_lo.png"),
                "u16_norm": str(out_dir / f"{uoff:08X}_u16norm.png"),
                "ci4": str(out_dir / f"{nxt[0]:08X}_ci4.png"),
            }
        )

    # Tag user-validated lightmaps if provided.
    validated = {int(x, 0) for x in args.valid_lightmap_offset}
    for rec in records:
        rec["validated_lightmap"] = bool(rec["offset"] in validated and rec.get("kind") == "u16_128x128")

    manifest = {
        "rom": args.rom,
        "range": {"start": int(args.start, 0), "end": int(args.end, 0)},
        "records": records,
        "pairs": pairs,
        "counts": {
            "total_chunks": len(records),
            "u16_chunks": len(u16_offs),
            "ci4_chunks": len(ci4_offs),
            "pairs": len(pairs),
            "validated_lightmaps": len([r for r in records if r.get("validated_lightmap")]),
        },
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"outdir": str(out_dir), "manifest": str(out_manifest), **manifest["counts"]}, indent=2))
    return 0


def _u16_be_grid(data: bytes, w: int, h: int) -> List[int]:
    n = w * h
    if len(data) < n * 2:
        raise ValueError("u16 grid decode out of range")
    return [struct.unpack_from(">H", data, i * 2)[0] for i in range(n)]


def _u16_le_grid(data: bytes, w: int, h: int) -> List[int]:
    n = w * h
    if len(data) < n * 2:
        raise ValueError("u16 grid decode out of range")
    return [struct.unpack_from("<H", data, i * 2)[0] for i in range(n)]


def _untile_linear(samples: Sequence[int], w: int, h: int, ts: int) -> List[int]:
    if ts <= 0 or (w % ts) != 0 or (h % ts) != 0:
        raise ValueError("invalid tile size for untile")
    out = [0] * (w * h)
    si = 0
    for ty in range(0, h, ts):
        for tx in range(0, w, ts):
            for y in range(ts):
                row = (ty + y) * w
                for x in range(ts):
                    out[row + tx + x] = samples[si]
                    si += 1
    return out


def _morton_decode_2d(n: int) -> Tuple[int, int]:
    x = 0
    y = 0
    bit = 0
    while n:
        x |= (n & 1) << bit
        n >>= 1
        y |= (n & 1) << bit
        n >>= 1
        bit += 1
    return x, y


def _untile_morton(samples: Sequence[int], w: int, h: int, ts: int) -> List[int]:
    if ts <= 0 or (ts & (ts - 1)) != 0 or (w % ts) != 0 or (h % ts) != 0:
        raise ValueError("invalid morton tile size")
    out = [0] * (w * h)
    si = 0
    for ty in range(0, h, ts):
        for tx in range(0, w, ts):
            for i in range(ts * ts):
                x, y = _morton_decode_2d(i)
                if x >= ts or y >= ts:
                    continue
                out[(ty + y) * w + (tx + x)] = samples[si]
                si += 1
    return out


def _u16_to_norm8(samples: Sequence[int]) -> List[int]:
    mn = min(samples)
    mx = max(samples)
    rng = max(1, mx - mn)
    return [int((v - mn) * 255 / rng) for v in samples]


def _ci4_to_l8(data: bytes, lohi: bool = False) -> List[int]:
    pix: List[int] = []
    for b in data:
        hi = ((b >> 4) & 0xF) * 17
        lo = (b & 0xF) * 17
        if lohi:
            pix.extend([lo, hi])
        else:
            pix.extend([hi, lo])
    return pix


def _be_hi_words_to_l8(data: bytes) -> List[int]:
    if (len(data) % 2) != 0:
        raise ValueError("u16 high-byte decode expects even byte length")
    return [int(data[i]) for i in range(0, len(data), 2)]


def _resize_nearest_l8(samples: Sequence[int], src_w: int, src_h: int, dst_w: int, dst_h: int) -> List[int]:
    if src_w <= 0 or src_h <= 0 or dst_w <= 0 or dst_h <= 0:
        raise ValueError("invalid resize dimensions")
    if len(samples) != (src_w * src_h):
        raise ValueError("resize input size does not match source dimensions")
    out = [0] * (dst_w * dst_h)
    for y in range(dst_h):
        sy = (y * src_h) // dst_h
        srow = sy * src_w
        drow = y * dst_w
        for x in range(dst_w):
            sx = (x * src_w) // dst_w
            out[drow + x] = int(samples[srow + sx])
    return out


def _l8_to_ci4_idx(samples: Sequence[int]) -> List[int]:
    out: List[int] = []
    for v in samples:
        i = int(round(float(v) / 17.0))
        if i < 0:
            i = 0
        elif i > 15:
            i = 15
        out.append(i)
    return out


def _ci4_idx_to_l8(indices: Sequence[int]) -> List[int]:
    out: List[int] = []
    for i in indices:
        ii = int(i)
        if ii < 0:
            ii = 0
        elif ii > 15:
            ii = 15
        out.append(ii * 17)
    return out


def _paint_mode_filter_indices(samples: Sequence[int], w: int, h: int, radius: int = 1, passes: int = 1) -> List[int]:
    if radius <= 0 or passes <= 0:
        return list(samples)
    cur = [int(v) for v in samples]
    for _ in range(passes):
        out = cur[:]
        for y in range(h):
            y0 = max(0, y - radius)
            y1 = min(h - 1, y + radius)
            for x in range(w):
                x0 = max(0, x - radius)
                x1 = min(w - 1, x + radius)
                bins = [0] * 16
                for yy in range(y0, y1 + 1):
                    row = yy * w
                    for xx in range(x0, x1 + 1):
                        v = cur[row + xx]
                        if v < 0:
                            v = 0
                        elif v > 15:
                            v = 15
                        bins[v] += 1
                best_i = 0
                best_c = -1
                for i, c in enumerate(bins):
                    if c > best_c:
                        best_c = c
                        best_i = i
                out[y * w + x] = best_i
        cur = out
    return cur


def _ci4_row_xor_bytes(data: bytes, w: int, h: int, xorv: int) -> bytes:
    row_bytes = w // 2
    if len(data) != row_bytes * h:
        raise ValueError("row-xor expects exact CI4 byte length")
    out = bytearray(len(data))
    for y in range(h):
        base = y * row_bytes
        for xb in range(row_bytes):
            src_xb = (xb ^ xorv) if (y & 1) else xb
            if src_xb >= row_bytes:
                src_xb = xb
            out[base + xb] = data[base + src_xb]
    return bytes(out)


def _xy_absdiff(samples: Sequence[int], w: int, h: int) -> Tuple[float, float]:
    sx = 0
    cx = 0
    sy = 0
    cy = 0
    for y in range(h):
        row = y * w
        for x in range(w):
            i = row + x
            if x + 1 < w:
                sx += abs(samples[i] - samples[i + 1])
                cx += 1
            if y + 1 < h:
                sy += abs(samples[i] - samples[i + w])
                cy += 1
    ax = (sx / cx) if cx else 0.0
    ay = (sy / cy) if cy else 0.0
    return ax, ay


def _grid_seam_penalty(samples: Sequence[int], w: int, h: int, periods: Sequence[int]) -> float:
    # Penalize strong seams at regular tile boundaries (common bad untile symptom).
    ax, ay = _xy_absdiff(samples, w, h)
    base = max(1.0, 0.5 * (ax + ay))
    worst = 0.0
    for p in periods:
        if p <= 1:
            continue
        sx = 0
        cx = 0
        sy = 0
        cy = 0
        for y in range(h):
            row = y * w
            for x in range(p - 1, w - 1, p):
                sx += abs(int(samples[row + x]) - int(samples[row + x + 1]))
                cx += 1
        for y in range(p - 1, h - 1, p):
            row = y * w
            for x in range(w):
                sy += abs(int(samples[row + x]) - int(samples[row + w + x]))
                cy += 1
        if cx == 0 and cy == 0:
            continue
        seam = 0.0
        n = 0
        if cx:
            seam += sx / cx
            n += 1
        if cy:
            seam += sy / cy
            n += 1
        seam /= max(1, n)
        # Ignore minor boundary contrast; penalize only clear seam amplification.
        excess = max(0.0, (seam / base) - 1.25)
        worst = max(worst, excess)
    return min(1.5, worst)


def _periodic_repeat_penalty(samples: Sequence[int], w: int, h: int, max_val: float, shifts: Sequence[int]) -> float:
    # Penalize layouts that repeat at fixed shifts (duplicate/scrambled tiles).
    best = 0.0
    denom = max(1.0, float(max_val))
    for shift in shifts:
        if shift >= w and shift >= h:
            continue
        total = 0
        cnt = 0
        if shift < w:
            for y in range(h):
                row = y * w
                for x in range(w - shift):
                    total += abs(int(samples[row + x]) - int(samples[row + x + shift]))
                    cnt += 1
        if shift < h:
            for y in range(h - shift):
                row = y * w
                row2 = (y + shift) * w
                for x in range(w):
                    total += abs(int(samples[row + x]) - int(samples[row2 + x]))
                    cnt += 1
        if cnt <= 0:
            continue
        d = (total / cnt) / denom
        # Very low shift-diff strongly indicates repeated blocks.
        p = max(0.0, (0.03 - d) / 0.03)
        best = max(best, p)
    return best


def _row_xor_words(samples: Sequence[int], w: int, h: int, xorv: int) -> List[int]:
    # Reorder odd rows by XORing x index; mirrors common N64 row swizzle artifacts.
    out = [0] * (w * h)
    for y in range(h):
        row = y * w
        for x in range(w):
            sx = (x ^ xorv) if (y & 1) else x
            if sx >= w:
                sx = x
            out[row + x] = int(samples[row + sx])
    return out


def _row_unzip_evenodd(samples: Sequence[int], w: int, h: int, odd_first: bool = False) -> List[int]:
    # Convert interlaced row order [e0,o0,e1,o1,...] into [e0,e1,...,o0,o1,...].
    if h <= 1:
        return list(samples)
    out = [0] * (w * h)
    half = h // 2
    if not odd_first:
        even_src = 0
        odd_src = 1
    else:
        even_src = 1
        odd_src = 0
    for y in range(half):
        src_e = (2 * y + even_src) * w
        src_o = (2 * y + odd_src) * w
        out[y * w : (y + 1) * w] = samples[src_e : src_e + w]
        out[(half + y) * w : (half + y + 1) * w] = samples[src_o : src_o + w]
    if h & 1:
        out[(h - 1) * w : h * w] = samples[(h - 1) * w : h * w]
    return out


def _row_zip_evenodd(samples: Sequence[int], w: int, h: int, odd_first: bool = False) -> List[int]:
    # Convert field-packed order [all even rows][all odd rows] into interlaced rows.
    if h <= 1:
        return list(samples)
    out = [0] * (w * h)
    half = h // 2
    for y in range(half):
        src_e = y * w
        src_o = (half + y) * w
        if not odd_first:
            dst_e = (2 * y) * w
            dst_o = (2 * y + 1) * w
        else:
            dst_e = (2 * y + 1) * w
            dst_o = (2 * y) * w
        out[dst_e : dst_e + w] = samples[src_e : src_e + w]
        out[dst_o : dst_o + w] = samples[src_o : src_o + w]
    if h & 1:
        out[(h - 1) * w : h * w] = samples[(h - 1) * w : h * w]
    return out


def _row_interlace_penalty(samples: Sequence[int], w: int, h: int, max_val: float) -> float:
    # Penalize comb-like horizontal striping: row(y) and row(y+2) too similar vs row(y+1).
    if h < 3:
        return 0.0
    d1 = 0.0
    c1 = 0
    d2 = 0.0
    c2 = 0
    for y in range(h - 1):
        r0 = y * w
        r1 = (y + 1) * w
        for x in range(w):
            d1 += abs(int(samples[r0 + x]) - int(samples[r1 + x]))
            c1 += 1
    for y in range(h - 2):
        r0 = y * w
        r2 = (y + 2) * w
        for x in range(w):
            d2 += abs(int(samples[r0 + x]) - int(samples[r2 + x]))
            c2 += 1
    if c1 == 0 or c2 == 0:
        return 0.0
    a1 = d1 / c1
    a2 = d2 / c2
    # If adjacent-row differences are much larger than 2-row differences, likely interlaced stripes.
    ratio = a1 / max(1.0, a2)
    return max(0.0, min(2.0, (ratio - 1.25) / 1.75))


def _u16_periodic_repeat_penalty(samples: Sequence[int], w: int, h: int) -> float:
    return _periodic_repeat_penalty(samples, w, h, 65535.0, shifts=[8, 16, 32, 64])


def _paint_artifact_score(samples: Sequence[int], w: int, h: int) -> float:
    # Lower is better. Penalize striping/checkerboard + isolated-pixel noise.
    ax, ay = _xy_absdiff(samples, w, h)
    aniso = abs(ax - ay) / max(1.0, ax + ay)
    variation = (ax + ay) / (2.0 * 255.0)
    parity_even = 0
    parity_odd = 0
    ce = 0
    co = 0
    isolated = 0
    isolated_n = 0
    for y in range(h):
        row = y * w
        for x in range(w):
            v = samples[row + x]
            if ((x + y) & 1) == 0:
                parity_even += int(v)
                ce += 1
            else:
                parity_odd += int(v)
                co += 1
            if 0 < x < (w - 1) and 0 < y < (h - 1):
                nbr = [samples[row + x - 1], samples[row + x + 1], samples[row - w + x], samples[row + w + x]]
                nbr.sort()
                med = 0.5 * (nbr[1] + nbr[2])
                isolated += abs(v - med)
                isolated_n += 1
    pmean_e = (parity_even / ce) if ce else 0.0
    pmean_o = (parity_odd / co) if co else 0.0
    parity_bias = abs(pmean_e - pmean_o) / 255.0
    isolated_noise = ((isolated / isolated_n) / 255.0) if isolated_n else 0.0
    seam = _grid_seam_penalty(samples, w, h, periods=[4, 8, 16, 32])
    repeat = _periodic_repeat_penalty(samples, w, h, 255.0, shifts=[8, 16, 32, 64])
    interlace = _row_interlace_penalty(samples, w, h, 255.0)
    grain = max(0.0, variation - 0.20) * max(0.0, isolated_noise - 0.18) * 6.0
    return (1.8 * aniso) + (0.7 * variation) + (1.3 * parity_bias) + (1.1 * isolated_noise) + (1.2 * seam) + (0.8 * repeat) + (1.1 * interlace) + grain


def _paint_variant_name_penalty(name: str) -> float:
    p = 0.0
    if name.startswith("raw_"):
        p += 0.030
    if name.endswith("_linear"):
        p += 0.008
    if "untile_linear_ts32" in name:
        p += 0.060
    elif "untile_linear_ts16" in name:
        p += 0.020
    elif "untile_morton_ts16" in name:
        p += 0.010
    m = re.search(r"rowxor(\d+)_", name)
    if m:
        xv = int(m.group(1))
        if xv != 1:
            p += 0.010 + (0.002 * min(15, abs(xv - 1)))
    return p


def _paint_entropy16_norm(samples: Sequence[int]) -> float:
    # CI4-style entropy over 16 bins, normalized to [0,1].
    bins = [0] * 16
    n = len(samples)
    if n <= 0:
        return 0.0
    for v in samples:
        i = int(round(float(v) / 17.0))
        if i < 0:
            i = 0
        elif i > 15:
            i = 15
        bins[i] += 1
    ent = 0.0
    for c in bins:
        if c <= 0:
            continue
        p = float(c) / float(n)
        ent -= p * math.log2(p)
    return ent / 4.0


def _paint_static_like(samples: Sequence[int], w: int, h: int) -> bool:
    # Heuristic for salt/pepper-like random index noise.
    ax, ay = _xy_absdiff(samples, w, h)
    variation = (ax + ay) / (2.0 * 255.0)
    aniso = abs(ax - ay) / max(1.0, ax + ay)
    ent = _paint_entropy16_norm(samples)
    hi_entropy_noise = (ent > 0.93 and variation > 0.24 and aniso < 0.22)
    lo_entropy_saltpepper = (ent < 0.70 and variation > 0.22 and aniso < 0.25)
    return bool(hi_entropy_noise or lo_entropy_saltpepper)


def _ci4_variant_images(data8192: bytes, w: int = 128, h: int = 128) -> List[Dict[str, object]]:
    if len(data8192) != (w * h) // 2:
        raise ValueError("CI4 variant decode expects exact 8192 bytes for 128x128")

    base_bytes: List[Tuple[str, bytes]] = [("raw", data8192)]
    for xv in [1, 2, 4, 8, 16, 32]:
        base_bytes.append((f"rowxor{xv}", _ci4_row_xor_bytes(data8192, w, h, xv)))

    bases: List[Tuple[str, List[int]]] = []
    for bname, bb in base_bytes:
        bases.append((f"{bname}_hilo", _ci4_to_l8(bb, lohi=False)))
        bases.append((f"{bname}_lohi", _ci4_to_l8(bb, lohi=True)))

    out: List[Dict[str, object]] = []
    for bname, base in bases:
        n = f"{bname}_linear"
        out.append({"name": n, "pixels": base, "score": _paint_artifact_score(base, w, h) + _paint_variant_name_penalty(n)})
        for ts in [2, 4, 8, 16, 32]:
            if (w % ts) == 0 and (h % ts) == 0:
                p = _untile_linear(base, w, h, ts)
                n = f"{bname}_untile_linear_ts{ts}"
                out.append(
                    {
                        "name": n,
                        "pixels": p,
                        "score": _paint_artifact_score(p, w, h) + _paint_variant_name_penalty(n),
                    }
                )
        for ts in [4, 8, 16]:
            if (w % ts) == 0 and (h % ts) == 0:
                p = _untile_morton(base, w, h, ts)
                n = f"{bname}_untile_morton_ts{ts}"
                out.append(
                    {
                        "name": n,
                        "pixels": p,
                        "score": _paint_artifact_score(p, w, h) + _paint_variant_name_penalty(n),
                    }
                )
    out.sort(key=lambda r: float(r["score"]))
    return out


def _u16_dup_score(samples: Sequence[int], w: int, h: int) -> float:
    # Penalize obvious split-duplicate layouts (common wrong width symptom).
    if w % 2 != 0:
        return 0.0
    hw = w // 2
    total = 0
    cnt = 0
    for y in range(h):
        row = y * w
        for x in range(hw):
            total += abs(samples[row + x] - samples[row + hw + x])
            cnt += 1
    if cnt == 0:
        return 0.0
    return (total / cnt) / 65535.0


def _u16_layout_score(samples: Sequence[int], w: int, h: int) -> float:
    # Use absolute neighbor difference to avoid range-normalization bias.
    # Heightmaps should be smooth relative to the 16-bit depth (65535).
    smooth = _neighbor_absdiff_score(samples, w, h)
    dup = _u16_dup_score(samples, w, h)
    repeat = _u16_periodic_repeat_penalty(samples, w, h)
    seam = _grid_seam_penalty(samples, w, h, periods=[8, 16, 32])
    interlace = _row_interlace_penalty(samples, w, h, 65535.0)
    # Prefer square-ish layouts.
    aspect_penalty = 0.08 * abs(math.log2(max(w, h) / max(1, min(w, h))))
    # Penalty for noise and suspiciously high duplication.
    # Lower is better.
    return (10.0 * smooth) + aspect_penalty + (0.5 * max(0.0, 0.1 - dup)) + (0.9 * repeat) + (0.7 * seam) + (1.0 * interlace)


def _parse_dim_list(spec: str, total_px: int) -> List[Tuple[int, int]]:
    dims: List[Tuple[int, int]] = []
    for tok in spec.split(","):
        t = tok.strip().lower()
        if not t or "x" not in t:
            continue
        a, b = t.split("x", 1)
        try:
            w = int(a)
            h = int(b)
        except Exception:
            continue
        if w > 0 and h > 0 and (w * h) == total_px:
            dims.append((w, h))
    # Keep order but dedupe.
    seen = set()
    out: List[Tuple[int, int]] = []
    for d in dims:
        if d in seen:
            continue
        seen.add(d)
        out.append(d)
    return out


def _neighbor_absdiff_score(samples: Sequence[int], w: int, h: int) -> float:
    # Heightmaps should be relatively smooth between adjacent samples.
    # Returns score normalized to 0.0-1.0 over the 16-bit range.
    total = 0
    cnt = 0
    for y in range(h):
        row = y * w
        for x in range(w):
            i = row + x
            v = samples[i]
            if x + 1 < w:
                total += abs(v - samples[i + 1])
                cnt += 1
            if y + 1 < h:
                total += abs(v - samples[i + w])
                cnt += 1
    if cnt <= 0:
        return 1.0
    # Normalize by the theoretical MAX range of depth (65535 for u16)
    # This prevents grainy/swapped data with high local volatility from
    # scoring well just because it ALSO has a large global range.
    return (total / cnt) / 65535.0


def _write_l8(path: pathlib.Path, w: int, h: int, data: Sequence[int]) -> None:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    img = Image.new("L", (w, h))
    img.putdata(list(data))
    img.save(path)


def _write_l8_resized(path: pathlib.Path, src_w: int, src_h: int, dst_w: int, dst_h: int, data: Sequence[int]) -> None:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    img = Image.new("L", (src_w, src_h))
    img.putdata(list(data))
    img = img.resize((dst_w, dst_h), resample=Image.NEAREST)
    img.save(path)


def _parse_offset_regex(items: Sequence[str]) -> Dict[int, re.Pattern[str]]:
    out: Dict[int, re.Pattern[str]] = {}
    for it in items:
        if ":" not in it:
            continue
        a, b = it.split(":", 1)
        try:
            off = int(a.strip(), 0)
            pat = re.compile(b.strip())
        except Exception:
            continue
        out[off] = pat
    return out


def _variant_bias(off: int, name: str, rules: Dict[int, re.Pattern[str]], bonus: float = 0.35) -> float:
    pat = rules.get(off)
    if pat is None:
        return 0.0
    return (-bonus) if pat.search(name) else 0.0


def _top_unique_variants(vars_in: Sequence[Dict[str, object]], topk: int) -> List[Dict[str, object]]:
    out: List[Dict[str, object]] = []
    seen: set[str] = set()
    for v in vars_in:
        pix = v.get("pixels")
        if not isinstance(pix, list):
            continue
        # Deduplicate by exported 8-bit appearance.
        k = hashlib.md5(bytes(_u16_to_norm8(pix))).hexdigest()
        if k in seen:
            continue
        seen.add(k)
        out.append(v)
        if len(out) >= topk:
            break
    return out


def _pick_preferred_variant(
    off: int,
    vars_in: Sequence[Dict[str, object]],
    rules: Dict[int, re.Pattern[str]],
    fallback: Dict[str, object],
    score_gap: float = 0.50,
) -> Dict[str, object]:
    pat = rules.get(off)
    if pat is None or not vars_in:
        return fallback
    base = float(vars_in[0].get("score", float("inf")))
    for v in vars_in:
        name = str(v.get("name", ""))
        if pat.search(name) and float(v.get("score", float("inf"))) <= (base + score_gap):
            return v
    return fallback


def cmd_height_candidates(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    w = int(args.width)
    h = int(args.height)
    if w * h * 2 != int(args.size):
        raise ValueError("--size must equal width*height*2 for u16 candidates")

    start = int(args.start, 0)
    end = int(args.end, 0)
    valid_light = {int(x, 0) for x in args.valid_lightmap_offset}
    valid_height_le = {int(x, 0) for x in args.valid_height_le_offset}
    valid_light_be = {int(x, 0) for x in args.valid_light_be_offset}

    headers = [o for o in find_yay0_headers(rom) if start <= o <= end]
    u16_offsets: List[int] = []
    blobs: Dict[int, bytes] = {}
    for yi, off in enumerate(headers):
        try:
            dec, _ = yay0_decompress(rom, off)
        except Exception:
            continue
        if len(dec) == int(args.size):
            u16_offsets.append(off)
            blobs[off] = dec

    rows: List[Dict[str, object]] = []
    top_n = int(args.keep_top)

    for off in sorted(u16_offsets):
        base = _u16_be_grid(blobs[off], w, h)
        le = _u16_le_grid(blobs[off], w, h)
        variants: Dict[str, Callable[[Sequence[int]], List[int]]] = {
            "be_linear": lambda a: list(a),
            "le_linear": lambda _a: list(le),
        }
        for ts in [2, 4, 8, 16, 32]:
            if w % ts == 0 and h % ts == 0:
                variants[f"untile_linear_ts{ts}"] = lambda a, t=ts: _untile_linear(a, w, h, t)
        for ts in [4, 8, 16]:
            if w % ts == 0 and h % ts == 0:
                variants[f"untile_morton_ts{ts}"] = lambda a, t=ts: _untile_morton(a, w, h, t)

        by_variant: List[Dict[str, object]] = []
        for name, fn in variants.items():
            try:
                arr = fn(base)
                score = _neighbor_absdiff_score(arr, w, h)
                by_variant.append({"variant": name, "score": score, "arr": arr})
            except Exception:
                continue
        by_variant.sort(key=lambda x: float(x["score"]))
        keep = by_variant[:top_n]

        chunk_dir = out_dir / f"{off:08X}"
        chunk_dir.mkdir(parents=True, exist_ok=True)

        saved: List[Dict[str, object]] = []
        for rec in keep:
            arr = rec["arr"]  # type: ignore[assignment]
            assert isinstance(arr, list)
            norm = _u16_to_norm8(arr)
            p = chunk_dir / f"{rec['variant']}.png"
            _write_l8(p, w, h, norm)
            saved.append(
                {
                    "variant": rec["variant"],
                    "score": rec["score"],
                    "png": str(p),
                }
            )

        rows.append(
            {
                "offset": off,
                "is_validated_lightmap": off in valid_light,
                "validated_light_be": off in valid_light_be,
                "validated_height_le": off in valid_height_le,
                "be_linear_png": str(chunk_dir / "be_linear.png"),
                "le_linear_png": str(chunk_dir / "le_linear.png"),
                "top": saved,
            }
        )

    # Rank by best non-lightmap smoothness to prioritize likely heightmaps.
    ranked = []
    for r in rows:
        top = r.get("top", [])
        if not isinstance(top, list) or not top:
            continue
        best = top[0]
        ranked.append(
            {
                "offset": r["offset"],
                "is_validated_lightmap": r["is_validated_lightmap"],
                "best_variant": best["variant"],
                "best_score": best["score"],
                "best_png": best["png"],
            }
        )
    ranked.sort(key=lambda x: (bool(x["is_validated_lightmap"]), float(x["best_score"])))

    resolved_pairs: List[Dict[str, object]] = []
    for r in rows:
        off = int(r["offset"])
        be_png = str(r["be_linear_png"])
        le_png = str(r["le_linear_png"])
        resolved_pairs.append(
            {
                "offset": off,
                "light_be_png": be_png,
                "height_le_png": le_png,
                "light_be_validated": bool(r["validated_light_be"] or r["is_validated_lightmap"]),
                "height_le_validated": bool(r["validated_height_le"]),
            }
        )

    manifest = {
        "rom": args.rom,
        "range": {"start": start, "end": end},
        "settings": {
            "width": w,
            "height": h,
            "u16_size": int(args.size),
            "keep_top": top_n,
        },
        "u16_chunks": len(u16_offsets),
        "records": rows,
        "ranked": ranked,
        "resolved_pairs": resolved_pairs,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "outdir": str(out_dir),
                "manifest": str(out_manifest),
                "u16_chunks": len(u16_offsets),
                "ranked": len(ranked),
            },
            indent=2,
        )
    )
    return 0


def _md5_hex(b: bytes) -> str:
    return hashlib.md5(b).hexdigest()


def cmd_terrain_all(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if Image is None:
        raise RuntimeError("Pillow is required for terrain PNG export: pip install pillow")

    headers = find_yay0_headers(rom)
    start = int(args.start, 0) if args.start is not None else 0
    end = int(args.end, 0) if args.end is not None else (len(rom) - 1)
    headers = [o for o in headers if start <= o <= end]

    u16_records: List[Dict[str, object]] = []
    paint_records: List[Dict[str, object]] = []
    other_records: List[Dict[str, object]] = []
    topk = max(1, int(args.paint_variants_top))
    u16_topk = max(1, int(args.u16_variants_top))
    light_pref = _parse_offset_regex(args.prefer_light_variant)
    height_pref = _parse_offset_regex(args.prefer_height_variant)
    paint_pref = _parse_offset_regex(args.prefer_paint_variant)
    default_paint_pref: Dict[int, re.Pattern[str]] = {}
    if 0x951BE4 not in paint_pref:
        default_paint_pref[0x951BE4] = re.compile(r"behi64_nearest")

    for yi, off in enumerate(headers):
        try:
            dec, _ = yay0_decompress(rom, off)
        except Exception as ex:
            other_records.append({"offset": off, "error": str(ex)})
            continue

        size = len(dec)
        u16_dims_local = _parse_dim_list(args.u16_dims, (size // 2)) if (size % 2) == 0 else []
        if u16_dims_local:
            n = size // 2
            be = [struct.unpack_from(">H", dec, i * 2)[0] for i in range(n)]
            le = [struct.unpack_from("<H", dec, i * 2)[0] for i in range(n)]

            hi = [dec[i * 2] for i in range(n)]
            lo = [dec[i * 2 + 1] for i in range(n)]

            def _u16_variants(vals: Sequence[int], tag: str) -> List[Dict[str, object]]:
                rows: List[Dict[str, object]] = []
                # If interpreting as 8-bit bytes (hi/lo), we scale to 16-bit range for consistent scoring.
                is_8bit = tag in ("hi", "lo")
                scale = 256 if is_8bit else 1

                for (w, h) in u16_dims_local:
                    base = [v * scale for v in vals]
                    rows.append({
                        "name": f"{tag}_linear_{w}x{h}",
                        "w": w, "h": h,
                        "pixels": base,
                        "score": _u16_layout_score(base, w, h),
                        "is_8bit": is_8bit,
                    })
                    rows.append({
                        "name": f"{tag}_linear_zipeo_{w}x{h}",
                        "w": w, "h": h,
                        "pixels": _row_zip_evenodd(base, w, h, odd_first=False),
                        "score": _u16_layout_score(_row_zip_evenodd(base, w, h, odd_first=False), w, h),
                        "is_8bit": is_8bit,
                    })
                    rows.append({
                        "name": f"{tag}_linear_unzipoe_{w}x{h}",
                        "w": w, "h": h,
                        "pixels": _row_unzip_evenodd(base, w, h, odd_first=True),
                        "score": _u16_layout_score(_row_unzip_evenodd(base, w, h, odd_first=True), w, h),
                        "is_8bit": is_8bit,
                    })
                    if tag in ("be", "le"):
                        for xv in [1, 2, 4, 8, 16, 32]:
                            p = _row_xor_words(base, w, h, xv)
                            rows.append({
                                "name": f"{tag}_rowxor{xv}_{w}x{h}",
                                "w": w, "h": h,
                                "pixels": p,
                                "score": _u16_layout_score(p, w, h),
                                "is_8bit": is_8bit,
                            })
                    for ts in [2, 4, 8, 16, 32]:
                        if (w % ts) == 0 and (h % ts) == 0:
                            p = _untile_linear(base, w, h, ts)
                            rows.append({
                                "name": f"{tag}_untile_linear_ts{ts}_{w}x{h}",
                                "w": w, "h": h,
                                "pixels": p,
                                "score": _u16_layout_score(p, w, h),
                                "is_8bit": is_8bit,
                            })
                            for rn, rp in [
                                ("zipeo", _row_zip_evenodd(p, w, h, odd_first=False)),
                                ("zipoe", _row_zip_evenodd(p, w, h, odd_first=True)),
                                ("unzipeo", _row_unzip_evenodd(p, w, h, odd_first=False)),
                                ("unzipoe", _row_unzip_evenodd(p, w, h, odd_first=True)),
                            ]:
                                rows.append({
                                    "name": f"{tag}_untile_linear_ts{ts}_{rn}_{w}x{h}",
                                    "w": w, "h": h,
                                    "pixels": rp,
                                    "score": _u16_layout_score(rp, w, h),
                                    "is_8bit": is_8bit,
                                })
                    for ts in [4, 8, 16]:
                        if (w % ts) == 0 and (h % ts) == 0:
                            p = _untile_morton(base, w, h, ts)
                            rows.append({
                                "name": f"{tag}_untile_morton_ts{ts}_{w}x{h}",
                                "w": w, "h": h,
                                "pixels": p,
                                "score": _u16_layout_score(p, w, h),
                                "is_8bit": is_8bit,
                            })
                            for rn, rp in [
                                ("zipeo", _row_zip_evenodd(p, w, h, odd_first=False)),
                                ("zipoe", _row_zip_evenodd(p, w, h, odd_first=True)),
                                ("unzipeo", _row_unzip_evenodd(p, w, h, odd_first=False)),
                                ("unzipoe", _row_unzip_evenodd(p, w, h, odd_first=True)),
                            ]:
                                rows.append({
                                    "name": f"{tag}_untile_morton_ts{ts}_{rn}_{w}x{h}",
                                    "w": w, "h": h,
                                    "pixels": rp,
                                    "score": _u16_layout_score(rp, w, h),
                                    "is_8bit": is_8bit,
                                })
                rows.sort(key=lambda r: float(r["score"]))
                return rows

            be_vars = _u16_variants(be, "be")
            le_vars = _u16_variants(le, "le")
            hi_vars = _u16_variants(hi, "hi")
            lo_vars = _u16_variants(lo, "lo")

            for lst in (be_vars, le_vars, hi_vars, lo_vars):
                for v in lst:
                    name = str(v.get("name", ""))
                    s = float(v.get("score", 0.0))
                    # Global preference against very noisy large untile variants.
                    if "_untile_linear_ts32_" in name:
                        s += 0.080
                    elif "_untile_linear_ts16_" in name:
                        s += 0.020
                    # Optional per-offset user calibration preferences.
                    if name.startswith(("be_", "hi_")):
                        s += _variant_bias(off, name, light_pref)
                    if name.startswith(("le_", "lo_")):
                        s += _variant_bias(off, name, height_pref)
                    v["score"] = s
                lst.sort(key=lambda r: float(r["score"]))

            # Collate all and pick the top two distinct "roles"
            all_v = sorted(be_vars + le_vars + hi_vars + lo_vars, key=lambda x: float(x["score"]))

            # Export best overall and best runner-up from a different tag group if possible
            best_v1 = all_v[0]
            tag1 = best_v1["name"].split("_")[0]

            # Find best v2 that isn't the same prefix (e.g. if BE is best, find best LE/HI/LO)
            best_v2 = next((v for v in all_v if not v["name"].startswith(tag1)), all_v[min(1, len(all_v)-1)])

            layer1_png = out_dir / f"{off:08X}_layer1_{best_v1['name']}.png"
            layer2_png = out_dir / f"{off:08X}_layer2_{best_v2['name']}.png"

            _write_l8(layer1_png, int(best_v1["w"]), int(best_v1["h"]), _u16_to_norm8(best_v1["pixels"]))
            _write_l8(layer2_png, int(best_v2["w"]), int(best_v2["h"]), _u16_to_norm8(best_v2["pixels"]))

            # Keep explicit BE/LE exports for light/height compatibility.
            be_best = be_vars[0]
            le_best = le_vars[0]
            be_png = out_dir / f"{off:08X}_light_{be_best['name']}.png"
            le_png = out_dir / f"{off:08X}_height_{le_best['name']}.png"
            _write_l8(be_png, int(be_best["w"]), int(be_best["h"]), _u16_to_norm8(be_best["pixels"]))
            _write_l8(le_png, int(le_best["w"]), int(le_best["h"]), _u16_to_norm8(le_best["pixels"]))

            light_best = min([be_vars[0], hi_vars[0]], key=lambda v: float(v["score"]))
            # Prefer LE as height interpretation when scores are close; LO is often less stable.
            height_best_base = min(
                [le_vars[0], lo_vars[0]],
                key=lambda v: float(v["score"]) + (0.020 if str(v["name"]).startswith("lo_") else 0.0),
            )
            height_pool = sorted(le_vars + lo_vars, key=lambda v: float(v["score"]))
            height_best = _pick_preferred_variant(off, height_pool, height_pref, height_best_base, score_gap=0.65)
            light_best_png = out_dir / f"{off:08X}_light_best_{light_best['name']}.png"
            height_best_png = out_dir / f"{off:08X}_height_best_{height_best['name']}.png"
            _write_l8(
                light_best_png,
                int(light_best["w"]),
                int(light_best["h"]),
                _u16_to_norm8(light_best["pixels"]),
            )
            _write_l8(
                height_best_png,
                int(height_best["w"]),
                int(height_best["h"]),
                _u16_to_norm8(height_best["pixels"]),
            )

            be_saved: List[Dict[str, object]] = []
            for v in _top_unique_variants(be_vars, u16_topk):
                vp = out_dir / f"{off:08X}_{v['name']}.png"
                _write_l8(vp, int(v["w"]), int(v["h"]), _u16_to_norm8(v["pixels"]))
                be_saved.append({"name": v["name"], "score": v["score"], "w": v["w"], "h": v["h"], "png": str(vp)})

            le_saved: List[Dict[str, object]] = []
            for v in _top_unique_variants(le_vars, u16_topk):
                vp = out_dir / f"{off:08X}_{v['name']}.png"
                _write_l8(vp, int(v["w"]), int(v["h"]), _u16_to_norm8(v["pixels"]))
                le_saved.append({"name": v["name"], "score": v["score"], "w": v["w"], "h": v["h"], "png": str(vp)})

            # Heuristic: very low unique value count implies indexed/paint-like data.
            be_unique = len(set(be))
            le_unique = len(set(le))
            # Use the best scores for classification
            if be_unique <= 96 and le_unique <= 96:
                u16_class = "indexed_u16_paintlike"
            # After seam/repeat penalties, good terrain-like maps usually sit below
            # ~0.85 in this score family; higher values are typically noisy/wrong decode.
            elif float(all_v[0]["score"]) > 0.70:
                u16_class = "noisy_or_unknown_u16"
            else:
                u16_class = "terrain_u16_candidate"
                
            u16_records.append(
                {
                    "offset": off,
                    "yay0_offset": off,
                    "yay0_index": yi,
                    "decompressed_size": size,
                    "raw_md5": _md5_hex(dec),
                    "light_be_png": str(be_png),
                    "height_le_png": str(le_png),
                    "score_be": float(be_vars[0]["score"]),
                    "score_le": float(le_vars[0]["score"]),
                    "score_hi": float(hi_vars[0]["score"]),
                    "score_lo": float(lo_vars[0]["score"]),
                    "best_variant": best_v1["name"],
                    "best_score": best_v1["score"],
                    "u16_confidence": 1.0 / (1.0 + float(best_v1["score"])),
                    "light_best_png": str(light_best_png),
                    "light_best_variant": str(light_best["name"]),
                    "light_best_score": float(light_best["score"]),
                    "height_best_png": str(height_best_png),
                    "height_best_variant": str(height_best["name"]),
                    "height_best_score": float(height_best["score"]),
                    "layer1_png": str(layer1_png),
                    "layer2_png": str(layer2_png),
                    "layer1_variant": best_v1["name"],
                    "layer2_variant": best_v2["name"],
                    "score_layer1": float(best_v1["score"]),
                    "score_layer2": float(best_v2["score"]),
                    "layer1_dims": [int(best_v1["w"]), int(best_v1["h"])],
                    "layer2_dims": [int(best_v2["w"]), int(best_v2["h"])],
                    "be_variants": be_saved,
                    "le_variants": le_saved,
                    "be_unique_values": be_unique,
                    "le_unique_values": le_unique,
                    "u16_class": u16_class,
                    "paint_like_u16": bool(u16_class != "terrain_u16_candidate"),
                }
            )
            continue

        paint_dims_full = _parse_dim_list(args.paint_dims, size * 2)
        if paint_dims_full:
            use_denoise_score = int(args.paint_denoise_passes) > 0 and int(args.paint_denoise_radius) > 0
            dim_variants: List[Dict[str, object]] = []
            for (pw, ph) in paint_dims_full:
                variants = _ci4_variant_images(dec, w=pw, h=ph)
                for v in variants:
                    n = str(v["name"])
                    n2 = f"{n}_{pw}x{ph}"
                    raw_pixels = v["pixels"]
                    raw_score = float(v["score"])
                    clean_pixels = None
                    score_clean = raw_score
                    if use_denoise_score:
                        idx = _l8_to_ci4_idx(raw_pixels)  # type: ignore[arg-type]
                        idx_clean = _paint_mode_filter_indices(
                            idx,
                            int(pw),
                            int(ph),
                            radius=int(args.paint_denoise_radius),
                            passes=int(args.paint_denoise_passes),
                        )
                        clean_pixels = _ci4_idx_to_l8(idx_clean)
                        score_clean = _paint_artifact_score(clean_pixels, int(pw), int(ph)) + _paint_variant_name_penalty(n)
                    s = (score_clean if use_denoise_score else raw_score)
                    s += _variant_bias(off, n, paint_pref)
                    s += _variant_bias(off, n2, paint_pref)
                    s += _variant_bias(off, n, default_paint_pref, bonus=1.0)
                    s += _variant_bias(off, n2, default_paint_pref, bonus=1.0)
                    dim_variants.append(
                        {
                            "name": n,
                            "score": s,
                            "raw_score": raw_score,
                            "score_clean": score_clean,
                            "pixels": raw_pixels,
                            "clean_pixels": clean_pixels,
                            "w": int(pw),
                            "h": int(ph),
                            "name_with_dims": n2,
                        }
                    )
            # Legacy-adaptive path: treat payload as BE u16 words and keep high bytes.
            # For 8192-byte paint chunks this yields 64x64 and can produce cleaner class maps.
            if not bool(getattr(args, "paint_no_behi64", False)) and (len(dec) % 2) == 0:
                hi = _be_hi_words_to_l8(dec)
                hp = len(hi)
                side = int(round(math.sqrt(float(hp))))
                if side * side == hp:
                    src_w = side
                    src_h = side
                    for (pw, ph) in paint_dims_full:
                        p_hi = _resize_nearest_l8(hi, src_w, src_h, int(pw), int(ph))
                        n = f"behi{src_w}_nearest"
                        n2 = f"{n}_{pw}x{ph}"
                        raw_score = _paint_artifact_score(p_hi, int(pw), int(ph)) + _paint_variant_name_penalty(n)
                        s = raw_score
                        s += _variant_bias(off, n, paint_pref)
                        s += _variant_bias(off, n2, paint_pref)
                        s += _variant_bias(off, n, default_paint_pref, bonus=1.0)
                        s += _variant_bias(off, n2, default_paint_pref, bonus=1.0)
                        dim_variants.append(
                            {
                                "name": n,
                                "score": s,
                                "raw_score": raw_score,
                                "score_clean": raw_score,
                                "pixels": p_hi,
                                "clean_pixels": None,
                                "w": int(pw),
                                "h": int(ph),
                                "name_with_dims": n2,
                            }
                        )
            dim_variants.sort(key=lambda r: float(r["score"]))
            best = dim_variants[0]
            pw = int(best["w"])
            ph = int(best["h"])
            p = out_dir / f"{off:08X}_paint_{best['name_with_dims']}.png"
            _write_l8(p, pw, ph, best["pixels"])  # type: ignore[arg-type]
            paint_square_png: Optional[pathlib.Path] = None
            paint_clean_square_png: Optional[pathlib.Path] = None
            side = int(round(math.sqrt(float(pw * ph))))
            if pw != ph and (side * side) == (pw * ph):
                p_square = out_dir / f"{off:08X}_paint_square_{best['name_with_dims']}_{side}x{side}.png"
                _write_l8_resized(p_square, pw, ph, side, side, best["pixels"])  # type: ignore[arg-type]
                paint_square_png = p_square
            paint_clean_png: Optional[pathlib.Path] = None
            if int(args.paint_denoise_passes) > 0 and int(args.paint_denoise_radius) > 0:
                p_clean = out_dir / f"{off:08X}_paint_clean_{best['name_with_dims']}_r{int(args.paint_denoise_radius)}_p{int(args.paint_denoise_passes)}.png"
                clean_l8 = best.get("clean_pixels")
                if clean_l8 is None:
                    idx = _l8_to_ci4_idx(best["pixels"])  # type: ignore[arg-type]
                    idx_clean = _paint_mode_filter_indices(
                        idx,
                        pw,
                        ph,
                        radius=int(args.paint_denoise_radius),
                        passes=int(args.paint_denoise_passes),
                    )
                    clean_l8 = _ci4_idx_to_l8(idx_clean)
                _write_l8(p_clean, pw, ph, clean_l8)
                paint_clean_png = p_clean
                if pw != ph and (side * side) == (pw * ph):
                    p_clean_square = out_dir / f"{off:08X}_paint_clean_square_{best['name_with_dims']}_r{int(args.paint_denoise_radius)}_p{int(args.paint_denoise_passes)}_{side}x{side}.png"
                    _write_l8_resized(p_clean_square, pw, ph, side, side, clean_l8)
                    paint_clean_square_png = p_clean_square
            variant_files: List[Dict[str, object]] = []
            for v in _top_unique_variants(dim_variants, topk):
                vp = out_dir / f"{off:08X}_paint_{v['name_with_dims']}.png"
                if str(vp) != str(p):
                    _write_l8(vp, int(v["w"]), int(v["h"]), v["pixels"])  # type: ignore[arg-type]
                variant_files.append(
                    {
                        "name": v["name"],
                        "name_with_dims": v["name_with_dims"],
                        "score": v["score"],
                        "dims": [int(v["w"]), int(v["h"])],
                        "png": str(vp),
                    }
                )
            paint_records.append(
                {
                    "offset": off,
                    "yay0_offset": off,
                    "yay0_index": yi,
                    "kind": f"ci4_{pw}x{ph}",
                    "raw_md5": _md5_hex(dec),
                    "paint_dims": [int(pw), int(ph)],
                    "paint_png": str(p),
                    "paint_clean_png": (str(paint_clean_png) if paint_clean_png is not None else None),
                    "paint_square_png": (str(paint_square_png) if paint_square_png is not None else None),
                    "paint_clean_square_png": (str(paint_clean_square_png) if paint_clean_square_png is not None else None),
                    "paint_variant": best["name"],
                    "paint_variant_with_dims": best["name_with_dims"],
                    "paint_score": best["score"],
                    "paint_entropy16": _paint_entropy16_norm(best["pixels"]),  # type: ignore[arg-type]
                    "paint_static_like": _paint_static_like(best["pixels"], pw, ph),  # type: ignore[arg-type]
                    "paint_class": (
                        "terrain_paint_candidate"
                        if (float(best["score"]) <= 0.37 and not _paint_static_like(best["pixels"], pw, ph))  # type: ignore[arg-type]
                        else "noisy_or_unknown_paint"
                    ),
                    "paint_confidence": 1.0 / (1.0 + float(best["score"])),
                    "paint_variants": variant_files,
                }
            )
            continue

        packed_done = False
        for (pw, ph) in _parse_dim_list(args.paint_dims, 128 * 128) + _parse_dim_list(args.paint_dims, 256 * 256) + _parse_dim_list(args.paint_dims, 512 * 512):
            part_bytes = (pw * ph) // 2
            if part_bytes <= 0 or (size % part_bytes) != 0:
                continue
            parts = size // part_bytes
            if parts <= 1:
                continue
            packed_done = True
            for part in range(parts):
                chunk = dec[part * part_bytes : (part + 1) * part_bytes]
                variants = _ci4_variant_images(chunk, w=pw, h=ph)
                best = variants[0]
                p = out_dir / f"{off:08X}_part{part}_paint_{best['name']}.png"
                _write_l8(p, pw, ph, best["pixels"])  # type: ignore[arg-type]
                paint_square_png: Optional[pathlib.Path] = None
                paint_clean_square_png: Optional[pathlib.Path] = None
                side = int(round(math.sqrt(float(pw * ph))))
                if pw != ph and (side * side) == (pw * ph):
                    p_square = out_dir / f"{off:08X}_part{part}_paint_square_{best['name']}_{side}x{side}.png"
                    _write_l8_resized(p_square, pw, ph, side, side, best["pixels"])  # type: ignore[arg-type]
                    paint_square_png = p_square
                paint_clean_png: Optional[pathlib.Path] = None
                if int(args.paint_denoise_passes) > 0 and int(args.paint_denoise_radius) > 0:
                    idx = _l8_to_ci4_idx(best["pixels"])  # type: ignore[arg-type]
                    idx_clean = _paint_mode_filter_indices(
                        idx,
                        pw,
                        ph,
                        radius=int(args.paint_denoise_radius),
                        passes=int(args.paint_denoise_passes),
                    )
                    p_clean = out_dir / f"{off:08X}_part{part}_paint_clean_{best['name']}_r{int(args.paint_denoise_radius)}_p{int(args.paint_denoise_passes)}.png"
                    clean_l8 = _ci4_idx_to_l8(idx_clean)
                    _write_l8(p_clean, pw, ph, clean_l8)
                    paint_clean_png = p_clean
                    if pw != ph and (side * side) == (pw * ph):
                        p_clean_square = out_dir / f"{off:08X}_part{part}_paint_clean_square_{best['name']}_r{int(args.paint_denoise_radius)}_p{int(args.paint_denoise_passes)}_{side}x{side}.png"
                        _write_l8_resized(p_clean_square, pw, ph, side, side, clean_l8)
                        paint_clean_square_png = p_clean_square
                variant_files: List[Dict[str, object]] = []
                for v in variants[:topk]:
                    vp = out_dir / f"{off:08X}_part{part}_paint_{v['name']}.png"
                    if str(vp) != str(p):
                        _write_l8(vp, pw, ph, v["pixels"])  # type: ignore[arg-type]
                    variant_files.append({"name": v["name"], "score": v["score"], "png": str(vp)})
                paint_records.append(
                    {
                        "offset": off,
                        "yay0_offset": off,
                        "yay0_index": yi,
                        "subpart": part,
                        "approx_offset": off + (part * part_bytes),
                        "kind": f"ci4_{parts}x{pw}x{ph}_part",
                        "raw_md5": _md5_hex(chunk),
                        "paint_dims": [int(pw), int(ph)],
                        "paint_png": str(p),
                        "paint_clean_png": (str(paint_clean_png) if paint_clean_png is not None else None),
                        "paint_square_png": (str(paint_square_png) if paint_square_png is not None else None),
                        "paint_clean_square_png": (str(paint_clean_square_png) if paint_clean_square_png is not None else None),
                        "paint_variant": best["name"],
                        "paint_score": best["score"],
                        "paint_entropy16": _paint_entropy16_norm(best["pixels"]),  # type: ignore[arg-type]
                        "paint_static_like": _paint_static_like(best["pixels"], pw, ph),  # type: ignore[arg-type]
                        "paint_class": (
                            "terrain_paint_candidate"
                            if (float(best["score"]) <= 0.37 and not _paint_static_like(best["pixels"], pw, ph))  # type: ignore[arg-type]
                            else "noisy_or_unknown_paint"
                        ),
                        "paint_confidence": 1.0 / (1.0 + float(best["score"])),
                        "paint_variants": variant_files,
                    }
                )
            break
        if packed_done:
            continue

        other_records.append({"offset": off, "decompressed_size": size})

    # Pair each u16 chunk with nearest-next paint chunk by offset (non-exclusive).
    paints_sorted = sorted(
        paint_records,
        key=lambda r: int(r.get("approx_offset", r.get("offset", 0))),  # type: ignore[arg-type]
    )
    paints_good = [p for p in paints_sorted if str(p.get("paint_class", "")) == "terrain_paint_candidate"]
    paints_by_yay0: Dict[int, List[Dict[str, object]]] = {}
    for p in paints_sorted:
        key = int(p.get("yay0_offset", p.get("offset", 0)))
        paints_by_yay0.setdefault(key, []).append(p)
    pairs: List[Dict[str, object]] = []
    for u in sorted(u16_records, key=lambda r: int(r["offset"])):
        uoff = int(u["offset"])
        local = paints_by_yay0.get(int(u.get("yay0_offset", uoff)), [])
        picked: Optional[Dict[str, object]] = None
        method = None
        if local:
            nxt = [p for p in local if int(p.get("approx_offset", p.get("offset", 0))) >= uoff]
            if nxt:
                picked = nxt[0]
                method = "same_yay0_next"
            else:
                picked = min(local, key=lambda p: abs(int(p.get("approx_offset", p.get("offset", 0))) - uoff))
                method = "same_yay0_nearest"
        if picked is None:
            base = paints_good if paints_good else paints_sorted
            nxt = [p for p in base if int(p.get("approx_offset", p.get("offset", 0))) >= uoff]
            if nxt:
                picked = nxt[0]
                method = "global_next_filtered" if paints_good else "global_next"
            elif base:
                picked = min(base, key=lambda p: abs(int(p.get("approx_offset", p.get("offset", 0))) - uoff))
                method = "global_nearest_filtered" if paints_good else "global_nearest"
        if picked is not None:
            pairs.append(
                {
                    "u16_offset": uoff,
                    "paint_offset": int(picked.get("offset", 0)),
                    "paint_subpart": picked.get("subpart"),
                    "light_be_png": u["light_be_png"],
                    "height_le_png": u["height_le_png"],
                    "light_png": u.get("light_best_png", u["light_be_png"]),
                    "height_png": u.get("height_best_png", u["height_le_png"]),
                    "paint_png": picked["paint_png"],
                    "paint_display_png": picked.get("paint_clean_square_png")
                    or picked.get("paint_square_png")
                    or picked.get("paint_clean_png")
                    or picked["paint_png"],
                    "pair_method": method,
                    "pair_score": float(u.get("best_score", 0.0)) + float(picked.get("paint_score", 0.0)),
                }
            )

    # Also provide nearest paint candidates for every u16 chunk (non-exclusive).
    nearest_pairs: List[Dict[str, object]] = []
    for u in sorted(u16_records, key=lambda r: int(r["offset"])):
        uoff = int(u["offset"])
        local = paints_by_yay0.get(int(u.get("yay0_offset", uoff)), [])
        ranked = []
        method = "global_nearest"
        if local:
            ranked = sorted(
                local,
                key=lambda p: abs(int(p.get("approx_offset", p.get("offset", 0))) - uoff),
            )
            method = "same_yay0_nearest"
        elif paints_good or paints_sorted:
            base = paints_good if paints_good else paints_sorted
            ranked = sorted(
                base,
                key=lambda p: abs(int(p.get("approx_offset", p.get("offset", 0))) - uoff),
            )
        if not ranked:
            break
        top = ranked[:3]
        nearest_pairs.append(
            {
                "u16_offset": uoff,
                "light_be_png": u["light_be_png"],
                "height_le_png": u["height_le_png"],
                "light_png": u.get("light_best_png", u["light_be_png"]),
                "height_png": u.get("height_best_png", u["height_le_png"]),
                "pair_method": method,
                "paint_candidates": [
                    {
                        "paint_offset": int(p.get("offset", 0)),
                        "paint_subpart": p.get("subpart"),
                        "paint_png": p["paint_png"],
                        "paint_display_png": p.get("paint_clean_square_png")
                        or p.get("paint_square_png")
                        or p.get("paint_clean_png")
                        or p["paint_png"],
                        "distance": abs(int(p.get("approx_offset", p.get("offset", 0))) - uoff),
                    }
                    for p in top
                ],
            }
        )

    # Unique terrain candidates are unique u16 raw payloads.
    by_raw: Dict[str, List[int]] = {}
    for r in u16_records:
        by_raw.setdefault(str(r["raw_md5"]), []).append(int(r["offset"]))
    unique_u16 = [{"raw_md5": h, "offsets": offs, "count": len(offs)} for h, offs in by_raw.items()]
    unique_u16.sort(key=lambda x: int(x["offsets"][0]))

    # Unique paints by raw payload.
    by_paint: Dict[str, List[Dict[str, object]]] = {}
    for r in paint_records:
        by_paint.setdefault(str(r["raw_md5"]), []).append(r)
    unique_paint: List[Dict[str, object]] = []
    for h, items in by_paint.items():
        unique_paint.append(
            {
                "raw_md5": h,
                "count": len(items),
                "refs": [
                    {
                        "offset": int(x["offset"]),
                        "subpart": x.get("subpart"),
                        "paint_png": x["paint_png"],
                    }
                    for x in items
                ],
            }
        )
    unique_paint.sort(key=lambda x: int(x["refs"][0]["offset"]))

    manifest = {
        "rom": args.rom,
        "range": {"start": start, "end": end},
        "counts": {
            "yay0_headers_in_range": len(headers),
            "u16_32768": len(u16_records),
            "paint_chunks": len(paint_records),
            "pairs": len(pairs),
            "nearest_pairs": len(nearest_pairs),
            "unique_u16": len(unique_u16),
            "unique_paint": len(unique_paint),
            "other_chunks": len(other_records),
        },
        "u16_records": u16_records,
        "paint_records": paint_records,
        "pairs": pairs,
        "nearest_pairs": nearest_pairs,
        "unique_u16": unique_u16,
        "unique_paint": unique_paint,
        "other_records": other_records,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"outdir": str(out_dir), "manifest": str(out_manifest), **manifest["counts"]}, indent=2))
    return 0


def cmd_dump_textures_yay0(args: argparse.Namespace) -> int:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)
    headers = find_yay0_headers(rom)
    if args.limit:
        headers = headers[: int(args.limit)]

    all_entries: List[Dict[str, object]] = []
    per_chunk: List[Dict[str, object]] = []

    for i, off in enumerate(headers):
        try:
            dec, comp = yay0_decompress(rom, off)
        except Exception as e:
            per_chunk.append(
                {
                    "yay0_index": i,
                    "yay0_offset": off,
                    "error": str(e),
                    "decompressed_size": None,
                    "compressed_size_est": None,
                    "textures": 0,
                }
            )
            continue

        chunk_label = f"y{i:04d}_{off:08X}"
        model_out_dir = out_dir / "model_dl"
        gen_out_dir = out_dir / "generic"
        model_entries = _extract_textures_from_model_chunk(
            data=dec,
            chunk_label=chunk_label,
            out_dir=model_out_dir,
            min_tri_hits=max(1, int(args.min_tri_hits)),
        )
        generic_entries: List[Dict[str, object]] = []
        if args.generic:
            generic_entries = _extract_generic_texture_candidates(
                data=dec,
                chunk_label=chunk_label,
                out_dir=gen_out_dir,
                top_per_chunk=max(1, int(args.generic_top_per_chunk)),
            )
        entries = model_entries + generic_entries
        all_entries.extend(entries)
        per_chunk.append(
            {
                "yay0_index": i,
                "yay0_offset": off,
                "error": None,
                "decompressed_size": len(dec),
                "compressed_size_est": comp,
                "textures": len(entries),
                "model_dl_textures": len(model_entries),
                "generic_textures": len(generic_entries),
            }
        )

    # De-dup by raw PNG hash to collapse repeated tiny tiles.
    by_hash: Dict[str, List[Dict[str, object]]] = {}
    for e in all_entries:
        p = pathlib.Path(str(e["png"]))
        try:
            h = hashlib.md5(p.read_bytes()).hexdigest()
        except Exception:
            continue
        by_hash.setdefault(h, []).append(e)
    unique_entries: List[Dict[str, object]] = []
    for h, items in by_hash.items():
        best = sorted(items, key=lambda x: float(x.get("score", 999.0)))[0]
        unique_entries.append(
            {
                "png_md5": h,
                "representative": best,
                "ref_count": len(items),
                "refs": [
                    {
                        "source": it.get("source"),
                        "chunk": it.get("chunk"),
                        "png": it.get("png"),
                        "score": it.get("score"),
                        "texture_state": it.get("texture_state"),
                    }
                    for it in items[:20]
                ],
            }
        )
    unique_entries.sort(key=lambda x: int(x.get("ref_count", 0)), reverse=True)

    manifest = {
        "rom": args.rom,
        "yay0_processed": len(headers),
        "counts": {
            "textures_dumped": len(all_entries),
            "unique_png_md5": len(unique_entries),
            "model_dl": sum(1 for e in all_entries if e.get("source") == "model_dl"),
            "generic": sum(1 for e in all_entries if str(e.get("source", "")).startswith("generic_")),
        },
        "chunks": per_chunk,
        "entries": all_entries,
        "unique": unique_entries,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "outdir": str(out_dir),
                "manifest": str(out_manifest),
                **manifest["counts"],
            },
            indent=2,
        )
    )
    return 0


def _parse_scan_dims(spec: str) -> List[Tuple[int, int]]:
    dims: List[Tuple[int, int]] = []
    for tok in str(spec).replace(";", ",").split(","):
        t = tok.strip().lower()
        if not t or "x" not in t:
            continue
        a, b = t.split("x", 1)
        try:
            w = int(a, 0)
            h = int(b, 0)
        except Exception:
            continue
        if w > 0 and h > 0:
            dims.append((w, h))
    out: List[Tuple[int, int]] = []
    seen: set[Tuple[int, int]] = set()
    for d in dims:
        if d in seen:
            continue
        seen.add(d)
        out.append(d)
    return out


def _parse_scan_nums(spec: str) -> List[int]:
    vals: List[int] = []
    for tok in re.split(r"[,\s;]+", str(spec).strip()):
        t = tok.strip()
        if not t:
            continue
        try:
            vals.append(int(_parse_num_auto(t)))
        except Exception:
            continue
    return vals


def _parse_offset_seed_list(items: Sequence[str]) -> List[int]:
    offs: List[int] = []
    for raw in items:
        for tok in re.split(r"[,\s;]+", str(raw).strip()):
            t = tok.strip()
            if not t:
                continue
            # Accept forms like AD73FE_2 by taking the leading numeric token.
            m = re.match(r"^[+-]?(?:0x)?[0-9A-Fa-f]+", t)
            if not m:
                continue
            n = m.group(0)
            try:
                offs.append(int(_parse_num_auto(n)))
            except Exception:
                continue
    out: List[int] = []
    seen: set[int] = set()
    for o in offs:
        if o in seen:
            continue
        seen.add(o)
        out.append(o)
    return out


def _pal16_likely(data: bytes, pal_off: int, min_unique: int = 6) -> bool:
    if pal_off < 0 or (pal_off + 32) > len(data):
        return False
    cols: List[Tuple[int, int, int, int]] = []
    try:
        for i in range(16):
            v = (data[pal_off + (i * 2)] << 8) | data[pal_off + (i * 2) + 1]
            cols.append(rgba5551_to_rgba8888(v))
    except Exception:
        return False
    uniq = len(set(cols))
    if uniq < max(2, int(min_unique)):
        return False
    alphas = {c[3] for c in cols}
    if len(alphas) == 1 and (0 in alphas):
        return False
    return True


def _pal256_likely(data: bytes, pal_off: int, min_unique: int = 48) -> bool:
    if pal_off < 0 or (pal_off + 512) > len(data):
        return False
    cols: List[Tuple[int, int, int, int]] = []
    try:
        for i in range(256):
            v = (data[pal_off + (i * 2)] << 8) | data[pal_off + (i * 2) + 1]
            cols.append(rgba5551_to_rgba8888(v))
    except Exception:
        return False
    uniq = len(set(cols))
    if uniq < max(16, int(min_unique)):
        return False
    alpha0 = sum(1 for c in cols if c[3] == 0) / 256.0
    if alpha0 > 0.98:
        return False
    # Reject extremely flat TLUTs that are usually false positives.
    lum = [0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2] for c in cols]
    mean = sum(lum) / max(1, len(lum))
    var = sum((v - mean) ** 2 for v in lum) / max(1, len(lum))
    if (var ** 0.5) < 6.0:
        return False
    return True


def _rgba_pixels_md5(pixels: Sequence[Tuple[int, int, int, int]]) -> str:
    raw = bytearray()
    for r, g, b, a in pixels:
        raw.extend((int(r) & 0xFF, int(g) & 0xFF, int(b) & 0xFF, int(a) & 0xFF))
    return hashlib.md5(bytes(raw)).hexdigest()


def _scan_texture_best_at(
    data: bytes,
    off: int,
    recipes: Sequence[Tuple[str, int, int, int, int, Sequence[Optional[int]]]],
    min_pal_unique: int,
    min_pal256_unique: int,
    allow_pal_overlap: bool,
    max_score: float,
) -> Optional[Dict[str, object]]:
    best: Optional[Dict[str, object]] = None
    best_score = float("inf")
    n = len(data)
    pal16_ok_cache: Dict[int, bool] = {}
    pal256_ok_cache: Dict[int, bool] = {}

    # Reuse palette plausibility checks across recipes/dimensions for this offset.
    pal_delta_set: set[int] = set()
    for _rn, _fmt, _siz, _w, _h, pds in recipes:
        for pd in pds:
            if pd is not None:
                pal_delta_set.add(int(pd))
    for pd in pal_delta_set:
        pal_off = int(off + pd)
        pal16_ok_cache[pd] = _pal16_likely(data, pal_off, min_unique=min_pal_unique)
        pal256_ok_cache[pd] = _pal256_likely(data, pal_off, min_unique=min_pal256_unique)

    for recipe_name, fmt, siz, w, h, pal_deltas in recipes:
        # Quick bounds reject before decoding.
        bpt = 4 << int(siz)
        need = ((int(w) * int(h) * int(bpt)) + 7) // 8
        if off < 0 or (off + need) > n:
            continue
        for pd in pal_deltas:
            pal_off: Optional[int] = None
            if pd is not None:
                pal_off = int(off + int(pd))
                if pal_off < 0 or (pal_off + 32) > len(data):
                    continue
                if int(fmt) == 2 and int(siz) == 0:
                    if not pal16_ok_cache.get(int(pd), False):
                        continue
                elif int(fmt) == 2 and int(siz) == 1:
                    if (pal_off + 512) > n:
                        continue
                    if not pal256_ok_cache.get(int(pd), False):
                        continue
                else:
                    continue

                if not allow_pal_overlap:
                    # TLUT and indexed image should be separate regions; overlapping is
                    # almost always a bad pairing in raw ROM scans.
                    pal_end = pal_off + (32 if int(siz) == 0 else 512)
                    img_end = int(off + need)
                    if not (pal_end <= off or pal_off >= img_end):
                        continue
            try:
                pixels = _decode_texture_pixels(
                    data=data,
                    fmt=int(fmt),
                    siz=int(siz),
                    img_off=int(off),
                    width=int(w),
                    height=int(h),
                    pal_off=pal_off,
                    pal_bank=0,
                )
            except Exception:
                continue
            s = float(_texture_visual_score(pixels, int(w), int(h)))
            if s > max_score:
                continue
            if s < best_score:
                best_score = s
                best = {
                    "offset": int(off),
                    "score": s,
                    "recipe": str(recipe_name),
                    "fmt": int(fmt),
                    "siz": int(siz),
                    "w": int(w),
                    "h": int(h),
                    "pal_offset": (None if pal_off is None else int(pal_off)),
                    "pal_delta": (None if pd is None else int(pd)),
                    "pixels": pixels,
                }
                # Strong visual confidence: no need to test all remaining recipes.
                if s <= 0.78:
                    return best
    return best


def cmd_scan_textures_rom(args: argparse.Namespace) -> int:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")

    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    start = int(args.start, 0) if args.start is not None else 0
    end = int(args.end, 0) if args.end is not None else (len(rom) - 1)
    if start < 0:
        start = 0
    if end >= len(rom):
        end = len(rom) - 1
    if end <= start:
        raise ValueError("Invalid scan range: --end must be greater than --start")

    scan_step = max(2, int(args.step))
    if (scan_step % 2) != 0:
        scan_step += 1
    refine_step = max(2, int(args.refine_step))
    if (refine_step % 2) != 0:
        refine_step += 1

    refine_radius = max(0, int(args.refine_radius, 0))
    coarse_keep = max(0, int(args.coarse_keep))
    coarse_dedupe_radius = max(0, int(args.coarse_dedupe_radius, 0))
    keep_n = max(1, int(args.keep))
    min_pal_unique = max(2, int(args.min_pal_unique))
    min_pal256_unique = max(16, int(args.min_pal256_unique))
    allow_pal_overlap = bool(args.allow_pal_overlap)

    dims = _parse_scan_dims(args.dims)
    if not dims:
        dims = [(16, 16), (32, 32)]
    pal_deltas = _parse_scan_nums(args.pal_deltas)
    if not pal_deltas:
        pal_deltas = [-0x200, -0x80, -0x40, -0x20, 0x20, 0x40]
    ci8_max_pixels = max(0, int(args.ci8_max_pixels))

    # Coarse probes are intentionally narrow/cheap, then refined around hits.
    coarse_recipes: List[Tuple[str, int, int, int, int, Sequence[Optional[int]]]] = [
        ("ci4_probe_16", 2, 0, 16, 16, [-0x200, -0x80, -0x20, 0x20]),
        ("ci4_probe_32", 2, 0, 32, 32, [-0x80, -0x20, 0x20]),
        ("ci8_probe_16", 2, 1, 16, 16, [-0x40, -0x20, 0x20]),
        ("ci8_probe_32", 2, 1, 32, 32, [-0x40, -0x20, 0x20]),
        ("i8_probe_16", 4, 1, 16, 16, [None]),
    ]
    coarse_threshold = float(args.coarse_threshold)

    fine_recipes: List[Tuple[str, int, int, int, int, Sequence[Optional[int]]]] = []
    for w, h in dims:
        fine_recipes.append(("ci4", 2, 0, int(w), int(h), list(pal_deltas)))
        if ci8_max_pixels <= 0 or (int(w) * int(h)) <= ci8_max_pixels:
            fine_recipes.append(("ci8", 2, 1, int(w), int(h), list(pal_deltas)))
        if not bool(args.ci_only):
            fine_recipes.append(("i4", 4, 0, int(w), int(h), [None]))
            fine_recipes.append(("i8", 4, 1, int(w), int(h), [None]))
            fine_recipes.append(("ia4", 3, 0, int(w), int(h), [None]))
            fine_recipes.append(("ia8", 3, 1, int(w), int(h), [None]))
            fine_recipes.append(("ia16", 3, 2, int(w), int(h), [None]))
            fine_recipes.append(("rgba16", 0, 2, int(w), int(h), [None]))
    fine_threshold = float(args.threshold)

    # 1) Coarse scan.
    coarse_hits: List[Dict[str, object]] = []
    coarse_offsets_total = 0
    if coarse_keep > 0:
        for off in range(start, end + 1, scan_step):
            coarse_offsets_total += 1
            best = _scan_texture_best_at(
                data=rom,
                off=int(off),
                recipes=coarse_recipes,
                min_pal_unique=min_pal_unique,
                min_pal256_unique=min_pal256_unique,
                allow_pal_overlap=allow_pal_overlap,
                max_score=coarse_threshold,
            )
            if best is None:
                continue
            coarse_hits.append(
                {
                    "offset": int(best["offset"]),
                    "score": float(best["score"]),
                    "recipe": str(best["recipe"]),
                }
            )

    coarse_hits.sort(key=lambda r: float(r["score"]))
    coarse_seed_offsets: List[int] = []
    if coarse_keep > 0:
        for h in coarse_hits:
            o = int(h["offset"])
            if coarse_dedupe_radius > 0:
                if any(abs(o - s) <= coarse_dedupe_radius for s in coarse_seed_offsets):
                    continue
            coarse_seed_offsets.append(o)
            if len(coarse_seed_offsets) >= coarse_keep:
                break

    # 2) Explicit seeds from user hints (including tokens like AD73FE_2).
    seed_offsets_raw = _parse_offset_seed_list(args.seed_offset or [])
    seed_offsets = [o for o in seed_offsets_raw if start <= o <= end]

    # 3) Fine scan offsets: around coarse seeds + explicit seeds.
    fine_offsets: set[int] = set()
    for s in coarse_seed_offsets + seed_offsets:
        lo = max(start, int(s) - refine_radius)
        hi = min(end, int(s) + refine_radius)
        for o in range(lo, hi + 1, refine_step):
            fine_offsets.add(int(o))

    # Ensure explicit seeds are always probed exactly.
    for s in seed_offsets:
        fine_offsets.add(int(s))

    fine_hits_by_hash: Dict[str, Dict[str, object]] = {}
    fine_scanned = 0
    accepted = 0
    for off in sorted(fine_offsets):
        fine_scanned += 1
        best = _scan_texture_best_at(
            data=rom,
            off=int(off),
            recipes=fine_recipes,
            min_pal_unique=min_pal_unique,
            min_pal256_unique=min_pal256_unique,
            allow_pal_overlap=allow_pal_overlap,
            max_score=fine_threshold,
        )
        if best is None:
            continue
        pixels = best.pop("pixels")
        if not isinstance(pixels, list):
            continue
        accepted += 1
        rgba_md5 = _rgba_pixels_md5(pixels)
        rec = fine_hits_by_hash.get(rgba_md5)
        ref = {
            "offset": int(best["offset"]),
            "score": float(best["score"]),
            "recipe": str(best["recipe"]),
            "fmt": int(best["fmt"]),
            "siz": int(best["siz"]),
            "w": int(best["w"]),
            "h": int(best["h"]),
            "pal_offset": best["pal_offset"],
            "pal_delta": best["pal_delta"],
        }
        if rec is None:
            fine_hits_by_hash[rgba_md5] = {
                "png_md5": rgba_md5,
                "best": dict(ref),
                "pixels": pixels,
                "refs": [ref],
            }
            continue
        rec_refs = rec.get("refs")
        if isinstance(rec_refs, list) and len(rec_refs) < 24:
            rec_refs.append(ref)
        best_old = rec.get("best")
        if isinstance(best_old, dict) and float(ref["score"]) < float(best_old.get("score", 999.0)):
            rec["best"] = dict(ref)
            rec["pixels"] = pixels

    unique_rows = sorted(
        fine_hits_by_hash.values(),
        key=lambda r: float(cast(Dict[str, object], r.get("best", {})).get("score", 999.0)),
    )

    entries: List[Dict[str, object]] = []
    png_dir = out_dir / "png"
    png_dir.mkdir(parents=True, exist_ok=True)
    for rank, row in enumerate(unique_rows[:keep_n], start=1):
        best = cast(Dict[str, object], row["best"])
        pixels = cast(List[Tuple[int, int, int, int]], row["pixels"])
        w = int(best["w"])
        h = int(best["h"])
        off = int(best["offset"])
        score_m = int(round(float(best["score"]) * 1000.0))
        png_name = (
            f"r{rank:04d}_{off:08X}_{str(best['recipe'])}_"
            f"f{int(best['fmt'])}z{int(best['siz'])}_{w}x{h}_s{score_m:04d}_{str(row['png_md5'])[:8]}.png"
        )
        png_path = png_dir / png_name
        _save_texture_png(png_path, pixels, w, h)
        entries.append(
            {
                "rank": rank,
                "png_md5": row["png_md5"],
                "png": str(png_path),
                "best": best,
                "ref_count": len(cast(List[Dict[str, object]], row.get("refs", []))),
                "refs": cast(List[Dict[str, object]], row.get("refs", [])),
            }
        )

    manifest = {
        "rom": args.rom,
        "range": {"start": int(start), "end": int(end)},
        "settings": {
            "step": int(scan_step),
            "refine_step": int(refine_step),
            "refine_radius": int(refine_radius),
            "dims": [f"{int(w)}x{int(h)}" for (w, h) in dims],
            "pal_deltas": [int(x) for x in pal_deltas],
            "ci8_max_pixels": int(ci8_max_pixels),
            "coarse_threshold": float(coarse_threshold),
            "threshold": float(fine_threshold),
            "min_pal_unique": int(min_pal_unique),
            "min_pal256_unique": int(min_pal256_unique),
            "allow_pal_overlap": bool(allow_pal_overlap),
            "ci_only": bool(args.ci_only),
            "coarse_keep": int(coarse_keep),
            "coarse_dedupe_radius": int(coarse_dedupe_radius),
            "keep": int(keep_n),
        },
        "counts": {
            "coarse_offsets_scanned": int(coarse_offsets_total),
            "coarse_hits": len(coarse_hits),
            "coarse_seed_offsets": len(coarse_seed_offsets),
            "seed_offsets_from_user": len(seed_offsets),
            "fine_offsets_scanned": int(fine_scanned),
            "accepted_fine_hits": int(accepted),
            "unique_png_md5": len(unique_rows),
            "written_png": len(entries),
        },
        "seed_offsets": [int(o) for o in seed_offsets],
        "coarse_hits_top": coarse_hits[: min(256, len(coarse_hits))],
        "entries": entries,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "outdir": str(out_dir),
                "manifest": str(out_manifest),
                **manifest["counts"],
            },
            indent=2,
        )
    )
    return 0


def cmd_trace_seg4_context(args: argparse.Namespace) -> int:
    rom = pathlib.Path(args.rom).read_bytes()
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    yoff = int(args.yay0_offset, 0)
    model_off = int(args.model_offset, 0)
    dl_off = int(args.dl_offset, 0) if args.dl_offset is not None else (model_off + 8)
    tri_div2 = bool(not args.tri_raw)

    dec, comp_len = yay0_decompress(rom, yoff)
    v0, t0, s0, ts0, _uv0 = parse_model_with_texstates(dec, dl_off=dl_off, tri_div2=tri_div2)
    local_score = _tri_state_decode_score(dec, ts0)
    unresolved = _collect_unresolved_setimg_seg_offsets(dec, dl_off=dl_off, seg_id=0x04)
    max_need = (max(unresolved) + int(args.pad_bytes)) if unresolved else 0

    headers = find_yay0_headers(rom)
    if args.limit:
        headers = headers[: int(args.limit)]

    ranked: List[Dict[str, object]] = []
    for cand_off in headers:
        cand_off = int(cand_off)
        if cand_off == yoff:
            continue
        try:
            cdec, ccomp = yay0_decompress(rom, cand_off)
        except Exception:
            continue
        if max_need > 0 and len(cdec) < max_need:
            continue

        comp = dec + cdec
        hint = {0x04: len(dec)}
        try:
            v2, t2, s2, ts2, _uv2 = parse_model_with_texstates(
                comp,
                dl_off=dl_off,
                tri_div2=tri_div2,
                seg_bases_init=hint,
            )
        except Exception:
            continue
        if len(v2) != len(v0) or len(t2) != len(t0):
            continue
        score = _tri_state_decode_score(comp, ts2)
        ranked.append(
            {
                "cand_yay0_offset": cand_off,
                "cand_yay0_hex": f"0x{cand_off:08X}",
                "cand_decompressed_size": len(cdec),
                "cand_compressed_size_est": int(ccomp),
                "score": float(score),
                "score_delta_vs_local": float(score - local_score),
                "unresolved_setimg": int(s2.get("unresolved_setimg", -1)),
                "texture_states": len({st for st in ts2 if st is not None}),
            }
        )
    ranked.sort(key=lambda r: float(r.get("score", 1e9)))

    previews: List[Dict[str, object]] = []
    top_n = max(0, int(args.preview_top))
    if top_n > 0 and Image is not None:
        local_best = _best_texture_from_tri_states(dec, ts0)
        if local_best is not None:
            px, w, h = local_best
            p = out_dir / "local_best_tex00.png"
            _save_texture_png(p, px, w, h)
            previews.append({"kind": "local", "png": str(p), "score": float(local_score)})
        for i, r in enumerate(ranked[:top_n]):
            c_off = int(r["cand_yay0_offset"])
            try:
                cdec, _ = yay0_decompress(rom, c_off)
            except Exception:
                continue
            comp = dec + cdec
            hint = {0x04: len(dec)}
            try:
                _v, _t, _s, ts2, _uv = parse_model_with_texstates(
                    comp,
                    dl_off=dl_off,
                    tri_div2=tri_div2,
                    seg_bases_init=hint,
                )
            except Exception:
                continue
            best = _best_texture_from_tri_states(comp, ts2)
            if best is None:
                continue
            px, w, h = best
            p = out_dir / f"cand_{i:02d}_{c_off:08X}_best_tex00.png"
            _save_texture_png(p, px, w, h)
            previews.append(
                {
                    "kind": "candidate",
                    "rank": i,
                    "cand_yay0_offset": c_off,
                    "score": float(r["score"]),
                    "png": str(p),
                }
            )

    manifest = {
        "rom": args.rom,
        "target": {
            "yay0_offset": yoff,
            "yay0_hex": f"0x{yoff:08X}",
            "model_offset": model_off,
            "model_hex": f"0x{model_off:08X}",
            "dl_offset": dl_off,
            "tri_div2": bool(tri_div2),
            "target_decompressed_size": len(dec),
            "target_compressed_size_est": int(comp_len),
            "local_score": float(local_score),
            "local_unresolved_setimg": int(s0.get("unresolved_setimg", -1)),
            "unresolved_seg4_offsets": unresolved,
        },
        "search": {
            "candidates_considered": max(0, len(headers) - 1),
            "candidates_ranked": len(ranked),
            "pad_bytes": int(args.pad_bytes),
        },
        "top": ranked[: max(1, int(args.top))],
        "all": ranked,
        "previews": previews,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "outdir": str(out_dir),
                "manifest": str(out_manifest),
                "local_score": local_score,
                "ranked": len(ranked),
                "best_score": (ranked[0]["score"] if ranked else None),
            },
            indent=2,
        )
    )
    return 0


def _rol32(v: int, s: int) -> int:
    s &= 31
    return ((v << s) | (v >> (32 - s))) & 0xFFFFFFFF


def _detect_rom_order(raw: bytes) -> str:
    if len(raw) < 4:
        return "unknown"
    b0, b1, b2, b3 = raw[0], raw[1], raw[2], raw[3]
    if (b0, b1, b2, b3) == (0x80, 0x37, 0x12, 0x40):
        return "z64"
    if (b0, b1, b2, b3) == (0x37, 0x80, 0x40, 0x12):
        return "v64"
    if (b0, b1, b2, b3) == (0x40, 0x12, 0x37, 0x80):
        return "n64"
    return "unknown"


def _normalize_rom_be(raw: bytes) -> Tuple[bytes, str]:
    order = _detect_rom_order(raw)
    if order == "z64":
        return raw, order
    if order == "v64":
        out = bytearray(raw)
        for i in range(0, len(out) - 1, 2):
            out[i], out[i + 1] = out[i + 1], out[i]
        return bytes(out), order
    if order == "n64":
        out = bytearray(raw)
        for i in range(0, len(out) - 3, 4):
            out[i + 0], out[i + 1], out[i + 2], out[i + 3] = (
                out[i + 3],
                out[i + 2],
                out[i + 1],
                out[i + 0],
            )
        return bytes(out), order
    return raw, order


def _bootcode_crc32(rom_be: bytes) -> int:
    if len(rom_be) < 0x1000:
        return 0
    return int(binascii.crc32(rom_be[0x40:0x1000]) & 0xFFFFFFFF)


def _detect_cic(rom_be: bytes) -> Optional[int]:
    c = _bootcode_crc32(rom_be)
    by_crc = {
        0x6170A4A1: 6101,
        0x90BB6CB5: 6102,
        0x0B050EE0: 6103,
        0x98BC2C86: 6105,
        0xACC8580A: 6106,
    }
    return by_crc.get(c)


def _n64_crc_compute(rom_be: bytes, cic: int) -> Tuple[int, int]:
    if len(rom_be) < 0x101000:
        raise ValueError("ROM is too small for N64 CRC algorithm")

    if cic in (6101, 6102):
        seed = 0xF8CA4DDC
    elif cic == 6103:
        seed = 0xA3886759
    elif cic == 6105:
        seed = 0xDF26F436
    elif cic == 6106:
        seed = 0x1FEA617A
    else:
        raise ValueError(f"Unsupported CIC: {cic}")

    t1 = seed
    t2 = seed
    t3 = seed
    t4 = seed
    t5 = seed
    t6 = seed

    for i in range(0x1000, 0x101000, 4):
        d = be32(rom_be, i)
        if ((t6 + d) & 0xFFFFFFFF) < t6:
            t4 = (t4 + 1) & 0xFFFFFFFF
        t6 = (t6 + d) & 0xFFFFFFFF
        t3 ^= d
        r = _rol32(d, d & 0x1F)
        t5 = (t5 + r) & 0xFFFFFFFF
        if t2 > d:
            t2 ^= r
        else:
            t2 ^= (t6 ^ d) & 0xFFFFFFFF
        if cic == 6105:
            idx = 0x40 + (i & 0xFF)
            t1 = (t1 + (be32(rom_be, idx) ^ d)) & 0xFFFFFFFF
        else:
            t1 = (t1 + (t5 ^ d)) & 0xFFFFFFFF

    if cic == 6103:
        crc1 = ((t6 ^ t4) + t3) & 0xFFFFFFFF
        crc2 = ((t5 ^ t2) + t1) & 0xFFFFFFFF
    elif cic == 6106:
        crc1 = ((t6 * t4) + t3) & 0xFFFFFFFF
        crc2 = ((t5 * t2) + t1) & 0xFFFFFFFF
    else:
        crc1 = (t6 ^ t4 ^ t3) & 0xFFFFFFFF
        crc2 = (t5 ^ t2 ^ t1) & 0xFFFFFFFF
    return crc1, crc2


def _rom_name(rom_be: bytes) -> str:
    if len(rom_be) < 0x34:
        return ""
    raw = rom_be[0x20:0x34]
    return raw.decode("ascii", errors="ignore").rstrip(" \x00")


def _write_u32_be(buf: bytearray, off: int, value: int) -> None:
    struct.pack_into(">I", buf, off, value & 0xFFFFFFFF)


def find_mio0_headers(rom: bytes) -> List[int]:
    offs: List[int] = []
    pos = 0
    n = len(rom)
    while True:
        i = rom.find(b"MIO0", pos)
        if i < 0:
            break
        if i + 0x10 <= n:
            dec_size = be32(rom, i + 4)
            comp_off = be32(rom, i + 8)
            raw_off = be32(rom, i + 12)
            if 0 < dec_size <= 0x4000000 and 0x10 <= comp_off < raw_off < 0x1000000:
                offs.append(i)
        pos = i + 1
    return offs


def mio0_decompress(rom: bytes, off: int) -> Tuple[bytes, int]:
    if rom[off : off + 4] != b"MIO0":
        raise ValueError("Not an MIO0 stream")
    dec_size = be32(rom, off + 4)
    comp_off = be32(rom, off + 8)
    raw_off = be32(rom, off + 12)

    bit_pos = off + 0x10
    cpos = off + comp_off
    rpos = off + raw_off
    out = bytearray()

    n = len(rom)
    mask = 0
    bits_left = 0

    while len(out) < dec_size:
        if bits_left == 0:
            if bit_pos >= n:
                raise ValueError("MIO0 bitstream out of range")
            mask = rom[bit_pos]
            bit_pos += 1
            bits_left = 8

        raw = (mask & 0x80) != 0
        mask = ((mask << 1) & 0xFF)
        bits_left -= 1

        if raw:
            if rpos >= n:
                raise ValueError("MIO0 raw stream out of range")
            out.append(rom[rpos])
            rpos += 1
        else:
            if cpos + 1 >= n:
                raise ValueError("MIO0 comp stream out of range")
            code0 = rom[cpos]
            code1 = rom[cpos + 1]
            cpos += 2
            length = (code0 >> 4) + 3
            dist = (((code0 & 0x0F) << 8) | code1) + 1
            src = len(out) - dist
            if src < 0:
                raise ValueError("MIO0 invalid back-reference")
            for _ in range(length):
                out.append(out[src])
                src += 1
                if len(out) >= dec_size:
                    break
    return bytes(out), max(bit_pos, cpos, rpos) - off


def find_yaz0_headers(rom: bytes) -> List[int]:
    offs: List[int] = []
    pos = 0
    n = len(rom)
    while True:
        i = rom.find(b"Yaz0", pos)
        if i < 0:
            break
        if i + 0x10 <= n:
            dec_size = be32(rom, i + 4)
            if 0 < dec_size <= 0x4000000:
                offs.append(i)
        pos = i + 1
    return offs


def yaz0_decompress(rom: bytes, off: int) -> Tuple[bytes, int]:
    if rom[off : off + 4] != b"Yaz0":
        raise ValueError("Not a Yaz0 stream")
    dec_size = be32(rom, off + 4)
    src = off + 0x10
    out = bytearray()
    n = len(rom)
    valid = 0
    code = 0
    while len(out) < dec_size:
        if valid == 0:
            if src >= n:
                raise ValueError("Yaz0 code stream out of range")
            code = rom[src]
            src += 1
            valid = 8
        if (code & 0x80) != 0:
            if src >= n:
                raise ValueError("Yaz0 raw out of range")
            out.append(rom[src])
            src += 1
        else:
            if src + 1 >= n:
                raise ValueError("Yaz0 backref out of range")
            b1 = rom[src]
            b2 = rom[src + 1]
            src += 2
            dist = ((b1 & 0x0F) << 8) | b2
            copy_len = b1 >> 4
            if copy_len == 0:
                if src >= n:
                    raise ValueError("Yaz0 count out of range")
                copy_len = rom[src] + 0x12
                src += 1
            else:
                copy_len += 2
            back = len(out) - (dist + 1)
            if back < 0:
                raise ValueError("Yaz0 invalid back-reference")
            for _ in range(copy_len):
                out.append(out[back])
                back += 1
                if len(out) >= dec_size:
                    break
        code = (code << 1) & 0xFF
        valid -= 1
    return bytes(out), src - off


def _load_config(path: pathlib.Path) -> Dict[str, Any]:
    ext = path.suffix.lower()
    text = path.read_text(encoding="utf-8")
    if ext == ".json":
        data = json.loads(text)
    else:
        if yaml is None:
            raise RuntimeError("PyYAML is required for YAML configs: pip install pyyaml")
        data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError("Config root must be a mapping/object")
    return data


def _to_int(v: Any) -> int:
    if isinstance(v, int):
        return int(v)
    if isinstance(v, str):
        return int(v, 0)
    raise ValueError(f"Expected int-like value, got: {type(v).__name__}")


def _export_split_segment(
    rom_be: bytes,
    seg: Dict[str, Any],
    out_dir: pathlib.Path,
    parent_name: str = "",
) -> Dict[str, Any]:
    name = str(seg.get("name", "segment"))
    full_name = f"{parent_name}.{name}" if parent_name else name
    stype = str(seg.get("type", "raw")).lower()
    start = _to_int(seg.get("start", 0))
    end = _to_int(seg.get("end", start))
    if end < start:
        raise ValueError(f"Segment '{full_name}' has end < start")

    src = rom_be[start:end]
    out_rec: Dict[str, Any] = {
        "name": full_name,
        "type": stype,
        "start": start,
        "end": end,
        "size": max(0, end - start),
        "children": [],
    }

    seg_path = out_dir / f"{full_name}.bin"
    seg_path.parent.mkdir(parents=True, exist_ok=True)
    seg_path.write_bytes(src)
    out_rec["bin"] = str(seg_path)

    decomp: Optional[bytes] = None
    if stype == "yay0":
        decomp, _ = yay0_decompress(rom_be, start)
    elif stype == "mio0":
        decomp, _ = mio0_decompress(rom_be, start)
    elif stype == "yaz0":
        decomp, _ = yaz0_decompress(rom_be, start)

    if decomp is not None:
        dpath = out_dir / f"{full_name}.dec.bin"
        dpath.write_bytes(decomp)
        out_rec["dec_bin"] = str(dpath)
        out_rec["decompressed_size"] = len(decomp)

    children = seg.get("children") or []
    if isinstance(children, list) and children:
        base = decomp if decomp is not None else src
        for child in children:
            if not isinstance(child, dict):
                continue
            cstart = _to_int(child.get("start", 0))
            cend = _to_int(child.get("end", cstart))
            cname = str(child.get("name", "child"))
            ctype = str(child.get("type", "raw")).lower()
            if cend < cstart or cstart < 0 or cend > len(base):
                continue
            cbytes = base[cstart:cend]
            cfull = f"{full_name}.{cname}"
            cpath = out_dir / f"{cfull}.bin"
            cpath.write_bytes(cbytes)
            out_rec["children"].append(
                {
                    "name": cfull,
                    "type": ctype,
                    "start": cstart,
                    "end": cend,
                    "size": cend - cstart,
                    "bin": str(cpath),
                }
            )
    return out_rec


def cmd_rom_info(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, order = _normalize_rom_be(raw)
    header_crc1 = be32(rom_be, 0x10) if len(rom_be) >= 0x18 else 0
    header_crc2 = be32(rom_be, 0x14) if len(rom_be) >= 0x18 else 0
    cic = _detect_cic(rom_be)
    by_cic: Dict[str, Dict[str, object]] = {}
    for c in (6101, 6102, 6103, 6105, 6106):
        try:
            c1, c2 = _n64_crc_compute(rom_be, c)
            by_cic[str(c)] = {
                "crc1": f"0x{c1:08X}",
                "crc2": f"0x{c2:08X}",
                "matches_header": (c1 == header_crc1 and c2 == header_crc2),
            }
        except Exception as e:
            by_cic[str(c)] = {"error": str(e)}
    report = {
        "rom": args.rom,
        "rom_order": order,
        "size": len(raw),
        "name": _rom_name(rom_be),
        "header_crc1": f"0x{header_crc1:08X}",
        "header_crc2": f"0x{header_crc2:08X}",
        "bootcode_crc32": f"0x{_bootcode_crc32(rom_be):08X}",
        "detected_cic": cic,
        "crc_by_cic": by_cic,
    }
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


def cmd_rom_crc_verify(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, _ = _normalize_rom_be(raw)
    cic = int(args.cic) if args.cic else (_detect_cic(rom_be) or 6102)
    c1, c2 = _n64_crc_compute(rom_be, cic)
    h1 = be32(rom_be, 0x10)
    h2 = be32(rom_be, 0x14)
    ok = (c1 == h1 and c2 == h2)
    print(
        json.dumps(
            {
                "rom": args.rom,
                "cic": cic,
                "header_crc1": f"0x{h1:08X}",
                "header_crc2": f"0x{h2:08X}",
                "computed_crc1": f"0x{c1:08X}",
                "computed_crc2": f"0x{c2:08X}",
                "valid": ok,
            },
            indent=2,
        )
    )
    return 0 if ok else 2


def cmd_rom_crc_fix(args: argparse.Namespace) -> int:
    src = pathlib.Path(args.rom)
    raw = src.read_bytes()
    rom_be, order = _normalize_rom_be(raw)
    cic = int(args.cic) if args.cic else (_detect_cic(rom_be) or 6102)
    c1, c2 = _n64_crc_compute(rom_be, cic)
    patched = bytearray(rom_be)
    _write_u32_be(patched, 0x10, c1)
    _write_u32_be(patched, 0x14, c2)

    # Preserve original ROM byte order when writing back.
    out_bytes = bytes(patched)
    if order == "v64":
        out = bytearray(out_bytes)
        for i in range(0, len(out) - 1, 2):
            out[i], out[i + 1] = out[i + 1], out[i]
        out_bytes = bytes(out)
    elif order == "n64":
        out = bytearray(out_bytes)
        for i in range(0, len(out) - 3, 4):
            out[i + 0], out[i + 1], out[i + 2], out[i + 3] = (
                out[i + 3],
                out[i + 2],
                out[i + 1],
                out[i + 0],
            )
        out_bytes = bytes(out)

    dst = pathlib.Path(args.out) if args.out else src
    dst.write_bytes(out_bytes)
    print(
        json.dumps(
            {
                "input": args.rom,
                "output": str(dst),
                "cic": cic,
                "crc1": f"0x{c1:08X}",
                "crc2": f"0x{c2:08X}",
            },
            indent=2,
        )
    )
    return 0


def cmd_scan_containers(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, _ = _normalize_rom_be(raw)
    report = {
        "rom": args.rom,
        "yay0": find_yay0_headers(rom_be),
        "mio0": find_mio0_headers(rom_be),
        "yaz0": find_yaz0_headers(rom_be),
    }
    report["counts"] = {k: len(v) for k, v in report.items() if isinstance(v, list)}
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


def cmd_decompress_stream(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, _ = _normalize_rom_be(raw)
    off = int(args.offset, 0)
    stype = str(args.type).lower()
    if stype == "auto":
        sig = rom_be[off : off + 4]
        if sig == b"Yay0":
            stype = "yay0"
        elif sig == b"MIO0":
            stype = "mio0"
        elif sig == b"Yaz0":
            stype = "yaz0"
        else:
            raise ValueError(f"Unknown stream signature at 0x{off:08X}: {sig!r}")
    if stype == "yay0":
        dec, comp = yay0_decompress(rom_be, off)
    elif stype == "mio0":
        dec, comp = mio0_decompress(rom_be, off)
    elif stype == "yaz0":
        dec, comp = yaz0_decompress(rom_be, off)
    else:
        raise ValueError("type must be one of: auto,yay0,mio0,yaz0")
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(dec)
    print(
        json.dumps(
            {
                "rom": args.rom,
                "offset": f"0x{off:08X}",
                "type": stype,
                "out": str(out),
                "decompressed_size": len(dec),
                "compressed_size_est": comp,
            },
            indent=2,
        )
    )
    return 0


def cmd_split_config(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, order = _normalize_rom_be(raw)
    cfg = _load_config(pathlib.Path(args.config))
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    segs = cfg.get("segments", [])
    if not isinstance(segs, list):
        raise ValueError("Config 'segments' must be a list")

    recs: List[Dict[str, Any]] = []
    for seg in segs:
        if not isinstance(seg, dict):
            continue
        recs.append(_export_split_segment(rom_be, seg, out_dir))

    report = {
        "rom": args.rom,
        "rom_order": order,
        "config": args.config,
        "outdir": str(out_dir),
        "segments": recs,
        "count": len(recs),
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"manifest": str(out_manifest), "segments": len(recs)}, indent=2))
    return 0


def _parse_tex_fmt_siz(fmt: str, siz: int) -> Tuple[int, int]:
    f = fmt.strip().lower()
    if f == "rgba":
        ff = 0
    elif f == "yuv":
        ff = 1
    elif f == "ci":
        ff = 2
    elif f == "ia":
        ff = 3
    elif f == "i":
        ff = 4
    else:
        raise ValueError("fmt must be one of: rgba,yuv,ci,ia,i")
    if siz not in (4, 8, 16, 32):
        raise ValueError("siz must be one of: 4,8,16,32")
    ss = {4: 0, 8: 1, 16: 2, 32: 3}[siz]
    return ff, ss


def _to_rgba16(r: int, g: int, b: int, a: int) -> int:
    rv = (r * 31 + 127) // 255
    gv = (g * 31 + 127) // 255
    bv = (b * 31 + 127) // 255
    av = 1 if a >= 128 else 0
    return ((rv & 31) << 11) | ((gv & 31) << 6) | ((bv & 31) << 1) | av


def _encode_i4(values: Sequence[int]) -> bytes:
    out = bytearray((len(values) + 1) // 2)
    j = 0
    for i in range(0, len(values), 2):
        hi = max(0, min(15, values[i]))
        lo = max(0, min(15, values[i + 1] if (i + 1) < len(values) else 0))
        out[j] = ((hi & 0xF) << 4) | (lo & 0xF)
        j += 1
    return bytes(out)


def _encode_ci_from_image(img_rgba: "Image.Image", bits: int) -> Tuple[bytes, bytes]:
    if Image is None:
        raise RuntimeError("Pillow is required")
    colors = 16 if bits == 4 else 256
    pal_img = img_rgba.convert("P", palette=Image.ADAPTIVE, colors=colors)
    idx = list(pal_img.getdata())
    pal = pal_img.getpalette() or []
    tlut = bytearray()
    for i in range(colors):
        base = i * 3
        if (base + 2) < len(pal):
            r, g, b = pal[base], pal[base + 1], pal[base + 2]
        else:
            r, g, b = 0, 0, 0
        v = _to_rgba16(r, g, b, 255)
        tlut.extend(struct.pack(">H", v))
    if bits == 8:
        return bytes(idx), bytes(tlut)
    return _encode_i4(idx), bytes(tlut)


def cmd_texture_decode_raw(args: argparse.Namespace) -> int:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    w = int(args.width)
    h = int(args.height)
    fmt, siz = _parse_tex_fmt_siz(args.fmt, int(args.siz))
    img_data = pathlib.Path(args.input).read_bytes()
    tlut_data = b""
    if args.tlut:
        tlut_data = pathlib.Path(args.tlut).read_bytes()
    data = tlut_data + img_data
    img_off = len(tlut_data)
    pal_off = 0
    texels = w * h
    px = _decode_texture_pixels(
        data=data,
        fmt=fmt,
        siz=siz,
        img_off=img_off,
        texels=texels,
        pal_off=pal_off if fmt == 2 else None,
        pal_count=256,
        pal_bank=0,
    )
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    _save_texture_png(out, px, w, h)
    print(json.dumps({"out": str(out), "width": w, "height": h, "fmt": args.fmt, "siz": int(args.siz)}, indent=2))
    return 0


def cmd_texture_encode_raw(args: argparse.Namespace) -> int:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    fmt = str(args.fmt).lower()
    siz = int(args.siz)
    img = Image.open(args.input).convert("RGBA")
    w, h = img.size
    rgba = list(img.getdata())

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    raw = bytearray()
    tlut = None

    if fmt == "rgba" and siz == 16:
        for r, g, b, a in rgba:
            raw.extend(struct.pack(">H", _to_rgba16(r, g, b, a)))
    elif fmt == "ia" and siz == 16:
        for r, g, b, a in rgba:
            i8 = int(round((0.299 * r) + (0.587 * g) + (0.114 * b)))
            raw.append(max(0, min(255, i8)))
            raw.append(a)
    elif fmt == "ia" and siz == 8:
        for r, g, b, a in rgba:
            i4 = int(round((((0.299 * r) + (0.587 * g) + (0.114 * b)) / 255.0) * 15.0))
            a4 = int(round((a / 255.0) * 15.0))
            raw.append(((i4 & 0xF) << 4) | (a4 & 0xF))
    elif fmt == "i" and siz == 8:
        for r, g, b, _a in rgba:
            i8 = int(round((0.299 * r) + (0.587 * g) + (0.114 * b)))
            raw.append(max(0, min(255, i8)))
    elif fmt == "i" and siz == 4:
        vals: List[int] = []
        for r, g, b, _a in rgba:
            i4 = int(round((((0.299 * r) + (0.587 * g) + (0.114 * b)) / 255.0) * 15.0))
            vals.append(i4)
        raw.extend(_encode_i4(vals))
    elif fmt == "ci" and siz in (4, 8):
        ci_raw, ci_tlut = _encode_ci_from_image(img, siz)
        raw.extend(ci_raw)
        tlut = ci_tlut
    else:
        raise ValueError("Supported encode targets: rgba16, ia16, ia8, i8, i4, ci4, ci8")

    out.write_bytes(bytes(raw))
    tlut_path = None
    if tlut is not None:
        tlut_path = pathlib.Path(args.tlut_out) if args.tlut_out else out.with_suffix(out.suffix + ".tlut.bin")
        tlut_path.write_bytes(tlut)
    print(
        json.dumps(
            {
                "input": args.input,
                "out_raw": str(out),
                "out_tlut": (str(tlut_path) if tlut_path is not None else None),
                "width": w,
                "height": h,
                "fmt": fmt,
                "siz": siz,
            },
            indent=2,
        )
    )
    return 0


AL_ADPCM_WAVE = 0
AL_RAW16_WAVE = 1
AL_WAV = 7

BZ64_DEFAULT_AUDIO_BANKS: Tuple[Tuple[int, int], ...] = (
    (0x00C217B0, 0x00C06F08),
    (0x00E05718, 0x00C22140),
)

BZ64_DEFAULT_MIDI_RANGE: Tuple[int, int] = (
    0x00BE4118,
    0x00C054A8,
)
BZ64_DEFAULT_MIDI_GAME_NAME = "Battlezone - Rise of the Black Dogs (U)"


@dataclasses.dataclass
class N64AdpcmBook:
    order: int
    npredictors: int
    predictors: List[int]


@dataclasses.dataclass
class N64Loop:
    start: int
    end: int
    count: int
    state: List[int]


@dataclasses.dataclass
class N64WaveEntry:
    base: int
    length: int
    wave_type: int
    wave_flags: int
    data: bytes
    loop: Optional[N64Loop]
    book: Optional[N64AdpcmBook]
    decode8_only: bool


@dataclasses.dataclass
class N64SoundEntry:
    keybase: int
    wave: N64WaveEntry


@dataclasses.dataclass
class N64InstEntry:
    samplerate: int
    sounds: List[N64SoundEntry]


@dataclasses.dataclass
class N64BankEntry:
    samplerate: int
    instruments: List[N64InstEntry]
    percussion: List[N64SoundEntry]


@dataclasses.dataclass
class GeTrackEvent:
    absolute_time: int
    delta_time: int
    event_type: int
    data: bytes
    duration_time: int = 0


@dataclasses.dataclass
class GeRepeatState:
    pattern: Optional[bytes] = None
    offset: int = 0


def _be_u16(data: bytes, off: int) -> int:
    if off < 0 or (off + 2) > len(data):
        raise ValueError(f"Out-of-range u16 read @0x{off:X}")
    return struct.unpack_from(">H", data, off)[0]


def _be_s16(data: bytes, off: int) -> int:
    if off < 0 or (off + 2) > len(data):
        raise ValueError(f"Out-of-range s16 read @0x{off:X}")
    return struct.unpack_from(">h", data, off)[0]


def _be_u32(data: bytes, off: int) -> int:
    if off < 0 or (off + 4) > len(data):
        raise ValueError(f"Out-of-range u32 read @0x{off:X}")
    return struct.unpack_from(">I", data, off)[0]


def _parse_num_auto(text: str) -> int:
    t = text.strip()
    if not t:
        raise ValueError("Empty numeric token")
    if t.lower().startswith(("+0x", "-0x", "0x")):
        return int(t, 0)
    if re.fullmatch(r"[0-9A-Fa-f]+", t):
        return int(t, 16)
    return int(t, 0)


def _parse_bank_pair(pair: str) -> Tuple[int, int]:
    s = pair.strip()
    if s.startswith("[") and s.endswith("]"):
        s = s[1:-1].strip()
    if s.startswith("(") and s.endswith(")"):
        s = s[1:-1].strip()
    if "," in s:
        a, b = [p.strip() for p in s.split(",", 1)]
    elif ":" in s:
        a, b = [p.strip() for p in s.split(":", 1)]
    else:
        raise ValueError(f"Invalid bank pair '{pair}', expected CTL,TBL")
    a = a.strip("[](){} ")
    b = b.strip("[](){} ")
    return _parse_num_auto(a), _parse_num_auto(b)


def _load_soundtool_ini_banks(ini_path: pathlib.Path, game_name: str) -> Tuple[str, List[Tuple[int, int]]]:
    game_target = game_name.strip().lower()
    cur_game = ""
    cur_type = "Uncompressed"
    found_type = "Uncompressed"
    banks: List[Tuple[int, int]] = []

    for raw_line in ini_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith(("/", ";", "#")):
            continue
        if line.startswith("["):
            close = line.find("]")
            if close < 0:
                cur_game = ""
                cur_type = "Uncompressed"
                continue
            header = line[1:close]
            cur_type = "Uncompressed"
            if ":type=" in header:
                left, right = header.split(":type=", 1)
                cur_game = left.strip()
                cur_type = right.strip()
            else:
                cur_game = header.strip()
            if cur_game.lower() == game_target:
                found_type = cur_type
            continue
        if cur_game.lower() != game_target:
            continue
        if line.startswith(("{", "|")):
            continue
        if "," not in line:
            continue
        first_two = [p.strip() for p in line.split(",", 2)[:2]]
        if len(first_two) != 2:
            continue
        try:
            banks.append((_parse_num_auto(first_two[0]), _parse_num_auto(first_two[1])))
        except Exception:
            continue
    return found_type, banks


def _resolve_ctl_ptr(ptr: int, flags: int, ctl_offset: int, ctl_flagged_offset: int, mask: int) -> int:
    if flags == 0x0100:
        return int(ctl_flagged_offset + (ptr - mask))
    return int(ctl_offset + ptr)


def _load_wave_data(rom_be: bytes, tbl_view: bytes, base: int, length: int, top_flag: int) -> bytes:
    if length <= 0:
        return b""
    if top_flag == 0x40:
        end = base + length
        if base < 0 or end > len(rom_be):
            raise ValueError(f"Wave data out-of-range in ROM: 0x{base:X}+0x{length:X}")
        return rom_be[base:end]
    end = base + length
    if base < 0 or end > len(tbl_view):
        raise ValueError(f"Wave data out-of-range in TBL: 0x{base:X}+0x{length:X}")
    return tbl_view[base:end]


def _parse_adpcm_loop(ctl: bytes, loop_off: int) -> N64Loop:
    state = [_be_s16(ctl, loop_off + 0x0C + (i * 2)) for i in range(16)]
    return N64Loop(
        start=_be_u32(ctl, loop_off + 0x00),
        end=_be_u32(ctl, loop_off + 0x04),
        count=_be_u32(ctl, loop_off + 0x08),
        state=state,
    )


def _parse_adpcm_book(ctl: bytes, book_off: int) -> N64AdpcmBook:
    order = _be_u32(ctl, book_off + 0x00)
    npredictors = _be_u32(ctl, book_off + 0x04)
    pred_count = int(order) * int(npredictors) * 8
    if order <= 0 or npredictors <= 0 or pred_count <= 0 or pred_count > 0x20000:
        raise ValueError(f"Unsupported ADPCM book shape order={order} npred={npredictors}")
    predictors = [_be_s16(ctl, book_off + 0x08 + (i * 2)) for i in range(pred_count)]
    return N64AdpcmBook(order=int(order), npredictors=int(npredictors), predictors=predictors)


def _parse_wave_table(
    rom_be: bytes,
    ctl: bytes,
    tbl_view: bytes,
    wave_off: int,
    sound_flags: int,
    ctl_offset: int,
    ctl_flagged_offset: int,
    mask: int,
) -> N64WaveEntry:
    raw_base = _be_u32(ctl, wave_off + 0x00)
    top_flag = (raw_base >> 24) & 0xFF
    if top_flag == 0x40:
        base = (raw_base & 0x00FFFFFF) + 0x140000
    else:
        base = raw_base
    length = _be_u32(ctl, wave_off + 0x04)
    wave_type = int(ctl[wave_off + 0x08])
    wave_flags = int(ctl[wave_off + 0x09])
    decode8_only = (wave_flags & 0x30) == 0x30

    wav_data = _load_wave_data(rom_be, tbl_view, base, length, top_flag)

    loop_ptr = _be_u32(ctl, wave_off + 0x0C)
    book_ptr = _be_u32(ctl, wave_off + 0x10)
    loop: Optional[N64Loop] = None
    book: Optional[N64AdpcmBook] = None

    if loop_ptr != 0:
        loop_off = _resolve_ctl_ptr(loop_ptr, sound_flags, ctl_offset, ctl_flagged_offset, mask)
        loop = _parse_adpcm_loop(ctl, loop_off)

    if wave_type == AL_ADPCM_WAVE and book_ptr != 0:
        book_off = _resolve_ctl_ptr(book_ptr, sound_flags, ctl_offset, ctl_flagged_offset, mask)
        book = _parse_adpcm_book(ctl, book_off)

    return N64WaveEntry(
        base=int(base),
        length=int(length),
        wave_type=wave_type,
        wave_flags=wave_flags,
        data=wav_data,
        loop=loop,
        book=book,
        decode8_only=decode8_only,
    )


def _parse_sound_entry(
    rom_be: bytes,
    ctl: bytes,
    tbl_view: bytes,
    sound_off: int,
    sound_flags: int,
    ctl_offset: int,
    ctl_flagged_offset: int,
    mask: int,
) -> N64SoundEntry:
    key_ptr = _be_u32(ctl, sound_off + 0x04)
    key_off = _resolve_ctl_ptr(key_ptr, sound_flags, ctl_offset, ctl_flagged_offset, mask)
    keybase = int(ctl[key_off + 0x04]) if 0 <= (key_off + 0x04) < len(ctl) else 0x3C

    wave_ptr = _be_u32(ctl, sound_off + 0x08)
    wave_off = _resolve_ctl_ptr(wave_ptr, sound_flags, ctl_offset, ctl_flagged_offset, mask)
    wave = _parse_wave_table(rom_be, ctl, tbl_view, wave_off, sound_flags, ctl_offset, ctl_flagged_offset, mask)
    return N64SoundEntry(keybase=keybase, wave=wave)


def _parse_inst_sounds(
    rom_be: bytes,
    ctl: bytes,
    tbl_view: bytes,
    inst_off: int,
    inst_flags: int,
    sound_count: int,
    ctl_offset: int,
    ctl_flagged_offset: int,
    mask: int,
) -> List[N64SoundEntry]:
    out: List[N64SoundEntry] = []
    if sound_count <= 0:
        return out
    for i in range(sound_count):
        snd_ptr = _be_u32(ctl, inst_off + 0x10 + (i * 4))
        if snd_ptr == 0:
            continue
        snd_off = _resolve_ctl_ptr(snd_ptr, inst_flags, ctl_offset, ctl_flagged_offset, mask)
        snd_flags = int(ctl[snd_off + 0x0E])
        if snd_flags not in (0x00, 0x01):
            continue
        try:
            out.append(
                _parse_sound_entry(
                    rom_be=rom_be,
                    ctl=ctl,
                    tbl_view=tbl_view,
                    sound_off=snd_off,
                    sound_flags=snd_flags,
                    ctl_offset=ctl_offset,
                    ctl_flagged_offset=ctl_flagged_offset,
                    mask=mask,
                )
            )
        except Exception:
            continue
    return out


def _parse_standard_audio_bank(
    rom_be: bytes,
    ctl_offset: int,
    tbl_offset: int,
    bank_number: int = 0,
    ctl_flagged_offset: int = 0,
    mask: int = 0xFFFFFFFF,
) -> N64BankEntry:
    ctl = rom_be
    tbl_view = rom_be[tbl_offset:]
    bank_ptr = _be_u32(ctl, ctl_offset + 0x04 + (bank_number * 4))
    if ((bank_ptr >> 16) & 0xF0) > 0:
        bank_off = ctl_flagged_offset + (bank_ptr - mask)
    else:
        bank_off = ctl_offset + bank_ptr
    if bank_off < 0 or (bank_off + 0x0C) > len(ctl):
        raise ValueError(f"Invalid bank header offset 0x{bank_off:X}")

    bank_count = _be_u16(ctl, bank_off + 0x00)
    bank_flags = _be_u16(ctl, bank_off + 0x02)
    bank_samplerate = _be_u16(ctl, bank_off + 0x06)
    percussion_ptr = _be_u32(ctl, bank_off + 0x08)

    if bank_flags not in (0x0000, 0x0100):
        raise ValueError(f"Unsupported bank flags 0x{bank_flags:04X}")
    if bank_count > 4096:
        raise ValueError(f"Suspicious instrument count: {bank_count}")

    instruments: List[N64InstEntry] = []
    percussion: List[N64SoundEntry] = []

    if percussion_ptr != 0:
        per_off = _resolve_ctl_ptr(percussion_ptr, bank_flags, ctl_offset, ctl_flagged_offset, mask)
        per_flags = int(ctl[per_off + 0x03])
        per_count = _be_u16(ctl, per_off + 0x0E)
        if per_flags in (0x00, 0x01) and per_count <= 4096:
            percussion = _parse_inst_sounds(
                rom_be=rom_be,
                ctl=ctl,
                tbl_view=tbl_view,
                inst_off=per_off,
                inst_flags=per_flags,
                sound_count=per_count,
                ctl_offset=ctl_offset,
                ctl_flagged_offset=ctl_flagged_offset,
                mask=mask,
            )

    for i in range(bank_count):
        inst_ptr = _be_u32(ctl, bank_off + 0x0C + (i * 4))
        if inst_ptr == 0:
            instruments.append(N64InstEntry(samplerate=0, sounds=[]))
            continue
        inst_off = _resolve_ctl_ptr(inst_ptr, bank_flags, ctl_offset, ctl_flagged_offset, mask)
        inst_flags = int(ctl[inst_off + 0x03])
        sound_count = _be_u16(ctl, inst_off + 0x0E)
        if inst_flags not in (0x00, 0x01) or sound_count > 4096:
            instruments.append(N64InstEntry(samplerate=0, sounds=[]))
            continue
        sounds = _parse_inst_sounds(
            rom_be=rom_be,
            ctl=ctl,
            tbl_view=tbl_view,
            inst_off=inst_off,
            inst_flags=inst_flags,
            sound_count=sound_count,
            ctl_offset=ctl_offset,
            ctl_flagged_offset=ctl_flagged_offset,
            mask=mask,
        )
        instruments.append(N64InstEntry(samplerate=0, sounds=sounds))

    return N64BankEntry(samplerate=int(bank_samplerate), instruments=instruments, percussion=percussion)


ITABLE: Tuple[int, ...] = (0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1)
ITABLE_HALF: Tuple[int, ...] = (0, 1, -2, -1)


def _sign_extend(bits: int, value: int) -> int:
    m = 1 << (bits - 1)
    value = value & ((1 << bits) - 1)
    return (value ^ m) - m


def _clamp_s16(value: int) -> int:
    if value > 32767:
        return 32767
    if value < -32768:
        return -32768
    return int(value)


def _decode_8_block(data4: bytes, index: int, pred1: Sequence[int], lastsmp: List[int]) -> List[int]:
    if len(data4) < 4:
        raise ValueError("decode_8 requires 4 data bytes")
    if len(pred1) < 16:
        raise ValueError("decode_8 requires 16 predictor entries")
    pred2 = pred1[8:16]
    tmp: List[int] = [0] * 8
    byte_pos = 0
    for i in range(8):
        if i & 1:
            nib = data4[byte_pos] & 0x0F
            byte_pos += 1
        else:
            nib = (data4[byte_pos] >> 4) & 0x0F
        tmp[i] = _sign_extend(index + 4, ITABLE[nib] << index)
    out: List[int] = [0] * 8
    for i in range(8):
        total = (pred1[i] * lastsmp[6]) + (pred2[i] * lastsmp[7])
        if i > 0:
            for x in range(i - 1, -1, -1):
                total += tmp[(i - 1) - x] * pred2[x]
        out[i] = _clamp_s16(((tmp[i] << 11) + total) >> 11)
    lastsmp[:] = out
    return out


def _decode_8_half_block(data2: bytes, index: int, pred1: Sequence[int], lastsmp: List[int]) -> List[int]:
    if len(data2) < 2:
        raise ValueError("decode_8_half requires 2 data bytes")
    if len(pred1) < 16:
        raise ValueError("decode_8_half requires 16 predictor entries")
    pred2 = pred1[8:16]
    b0 = data2[0]
    b1 = data2[1]
    codes = [
        (b0 >> 6) & 0x03,
        (b0 >> 4) & 0x03,
        (b0 >> 2) & 0x03,
        b0 & 0x03,
        (b1 >> 6) & 0x03,
        (b1 >> 4) & 0x03,
        (b1 >> 2) & 0x03,
        b1 & 0x03,
    ]
    tmp = [_sign_extend(index + 2, ITABLE_HALF[c] << index) for c in codes]
    out: List[int] = [0] * 8
    for i in range(8):
        total = (pred1[i] * lastsmp[6]) + (pred2[i] * lastsmp[7])
        if i > 0:
            for x in range(i - 1, -1, -1):
                total += tmp[(i - 1) - x] * pred2[x]
        out[i] = _clamp_s16(((tmp[i] << 11) + total) >> 11)
    lastsmp[:] = out
    return out


def _decode_n64_adpcm(data: bytes, book: N64AdpcmBook, decode8_only: bool = False) -> List[int]:
    if book.npredictors <= 0:
        return []
    lastsmp = [0] * 8
    preds = list(book.predictors)
    out: List[int] = []
    if not decode8_only:
        usable = (len(data) // 9) * 9
        pos = 0
        while pos < usable:
            cmd = data[pos]
            pos += 1
            index = (cmd >> 4) & 0x0F
            pred = (cmd & 0x0F) % book.npredictors
            base = pred * 16
            if (base + 16) > len(preds):
                break
            pred1 = preds[base : base + 16]
            out.extend(_decode_8_block(data[pos : pos + 4], index, pred1, lastsmp))
            pos += 4
            out.extend(_decode_8_block(data[pos : pos + 4], index, pred1, lastsmp))
            pos += 4
    else:
        usable = (len(data) // 5) * 5
        pos = 0
        while pos < usable:
            cmd = data[pos]
            pos += 1
            index = (cmd >> 4) & 0x0F
            pred = (cmd & 0x0F) % book.npredictors
            base = pred * 16
            if (base + 16) > len(preds):
                break
            pred1 = preds[base : base + 16]
            out.extend(_decode_8_half_block(data[pos : pos + 2], index, pred1, lastsmp))
            pos += 2
            out.extend(_decode_8_half_block(data[pos : pos + 2], index, pred1, lastsmp))
            pos += 2
    return out


def _raw16_n64_to_pcm_le(raw: bytes) -> bytes:
    if len(raw) <= 2:
        return b""
    payload = raw[1:-1]
    if len(payload) & 1:
        payload = payload[:-1]
    out = bytearray(len(payload))
    for i in range(0, len(payload), 2):
        out[i] = payload[i + 1]
        out[i + 1] = payload[i]
    return bytes(out)


def _pcm16le_from_samples(samples: Sequence[int]) -> bytes:
    if not samples:
        return b""
    clamped = [_clamp_s16(int(s)) for s in samples]
    return struct.pack(f"<{len(clamped)}h", *clamped)


def _write_pcm16_mono_wav(path: pathlib.Path, pcm16le: bytes, sample_rate: int) -> None:
    with wave.open(str(path), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(max(1, int(sample_rate)))
        wf.writeframes(pcm16le)


def _export_sound_wav(out_path: pathlib.Path, wave_entry: N64WaveEntry, sample_rate: int) -> Dict[str, object]:
    rec: Dict[str, object] = {
        "path": str(out_path),
        "type": int(wave_entry.wave_type),
        "base": int(wave_entry.base),
        "length": int(wave_entry.length),
        "sample_rate": int(sample_rate),
        "written": False,
    }
    if wave_entry.wave_type == AL_ADPCM_WAVE:
        if wave_entry.book is None:
            rec["reason"] = "missing_adpcm_book"
            return rec
        samples = _decode_n64_adpcm(wave_entry.data, wave_entry.book, decode8_only=wave_entry.decode8_only)
        pcm16le = _pcm16le_from_samples(samples)
        rec["sample_count"] = len(samples)
    elif wave_entry.wave_type == AL_RAW16_WAVE:
        pcm16le = _raw16_n64_to_pcm_le(wave_entry.data)
        rec["sample_count"] = len(pcm16le) // 2
    elif wave_entry.wave_type == AL_WAV and wave_entry.data.startswith(b"RIFF"):
        out_path.write_bytes(wave_entry.data)
        rec["written"] = True
        rec["sample_count"] = None
        return rec
    else:
        rec["reason"] = f"unsupported_wave_type_{wave_entry.wave_type}"
        return rec

    if not pcm16le:
        rec["reason"] = "empty_pcm"
        return rec

    _write_pcm16_mono_wav(out_path, pcm16le, sample_rate)
    rec["written"] = True
    if wave_entry.loop is not None:
        rec["loop"] = {
            "start": int(wave_entry.loop.start),
            "end": int(wave_entry.loop.end),
            "count": int(wave_entry.loop.count),
        }
    return rec


def cmd_audio_extract_wav(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, _ = _normalize_rom_be(raw)
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    bank_pairs: List[Tuple[int, int]] = []
    if args.bank:
        bank_pairs = [_parse_bank_pair(s) for s in args.bank]
    elif args.config_ini:
        ini_path = pathlib.Path(args.config_ini)
        if not ini_path.exists():
            raise FileNotFoundError(f"Config INI not found: {ini_path}")
        game_type, ini_pairs = _load_soundtool_ini_banks(ini_path, args.game)
        if game_type.strip().lower() not in ("", "uncompressed"):
            raise ValueError(f"Unsupported game type '{game_type}' for this command (supports Uncompressed only)")
        if ini_pairs:
            bank_pairs = ini_pairs
    if not bank_pairs:
        bank_pairs = list(BZ64_DEFAULT_AUDIO_BANKS)

    target_rate = max(1, int(args.sample_rate))
    if args.half_sampling_rate:
        target_rate = max(1, target_rate // 2)

    written = 0
    skipped = 0
    max_files = int(args.limit) if args.limit is not None else None
    manifest_banks: List[Dict[str, object]] = []

    for bank_index, (ctl_off, tbl_off) in enumerate(bank_pairs):
        if max_files is not None and written >= max_files:
            break
        bank_rec: Dict[str, object] = {
            "bank_index": bank_index,
            "ctl_offset": int(ctl_off),
            "tbl_offset": int(tbl_off),
            "files": [],
        }
        try:
            bank = _parse_standard_audio_bank(rom_be, ctl_off, tbl_off)
        except Exception as exc:
            bank_rec["error"] = str(exc)
            manifest_banks.append(bank_rec)
            continue

        bank_rec["parsed_samplerate"] = int(bank.samplerate)
        bank_rec["output_samplerate"] = target_rate
        bank_rec["instrument_count"] = len(bank.instruments)
        bank_rec["percussion_count"] = len(bank.percussion)
        files: List[Dict[str, object]] = []

        for pi, snd in enumerate(bank.percussion):
            if max_files is not None and written >= max_files:
                break
            out_path = out_dir / f"BANK_{bank_index:02d}_PERC_{pi:04d}.wav"
            rec = _export_sound_wav(out_path, snd.wave, target_rate)
            rec["kind"] = "percussion"
            rec["percussion_index"] = pi
            files.append(rec)
            if bool(rec.get("written")):
                written += 1
            else:
                skipped += 1

        for ii, inst in enumerate(bank.instruments):
            if max_files is not None and written >= max_files:
                break
            for si, snd in enumerate(inst.sounds):
                if max_files is not None and written >= max_files:
                    break
                out_path = out_dir / f"BANK_{bank_index:02d}_INSTR_{ii:04d}_SND_{si:04d}.wav"
                rec = _export_sound_wav(out_path, snd.wave, target_rate)
                rec["kind"] = "instrument"
                rec["instrument_index"] = ii
                rec["sound_index"] = si
                rec["keybase"] = int(snd.keybase)
                files.append(rec)
                if bool(rec.get("written")):
                    written += 1
                else:
                    skipped += 1

        bank_rec["files"] = files
        manifest_banks.append(bank_rec)

    manifest = {
        "rom": args.rom,
        "outdir": str(out_dir),
        "bank_pairs": [{"ctl": int(c), "tbl": int(t)} for c, t in bank_pairs],
        "counts": {"written_wavs": written, "skipped": skipped},
        "banks": manifest_banks,
    }
    manifest_path = out_dir / "manifest_audio_wav.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), **manifest["counts"]}, indent=2))
    return 0


def _ge_read_byte(data: bytes, pos: int, repeat: GeRepeatState, include_fe_repeats: bool = True) -> Tuple[int, int]:
    if repeat.pattern is not None:
        if repeat.offset >= len(repeat.pattern):
            repeat.pattern = None
            repeat.offset = 0
        else:
            out = int(repeat.pattern[repeat.offset])
            repeat.offset += 1
            if repeat.offset >= len(repeat.pattern):
                repeat.pattern = None
                repeat.offset = 0
            return out, pos

    if pos >= len(data):
        raise EOFError("Unexpected end of GE sequence stream")
    out = int(data[pos])
    pos += 1

    if include_fe_repeats and out == 0xFE and pos < len(data):
        if data[pos] != 0xFE:
            if (pos + 2) >= len(data):
                raise EOFError("Truncated FE repeat marker")
            repeat_first = int(data[pos])
            pos += 1
            repeat_distance = ((repeat_first << 8) | int(data[pos])) & 0xFFFF
            pos += 1
            repeat_count = int(data[pos])
            pos += 1

            start = (pos - 4) - repeat_distance
            end = start + repeat_count
            if repeat_count <= 0 or start < 0 or end > len(data):
                raise ValueError("Invalid FE repeat pattern range")
            repeat.pattern = bytes(data[start:end])
            repeat.offset = 0
            out = int(repeat.pattern[0])
            repeat.offset = 1
            if repeat.offset >= len(repeat.pattern):
                repeat.pattern = None
                repeat.offset = 0
        else:
            # Skip duplicated FE marker.
            pos += 1

    return out, pos


def _ge_read_vl(data: bytes, pos: int, repeat: GeRepeatState, include_fe_repeats: bool = True) -> Tuple[int, int]:
    vl_val = 0
    for _ in range(4):
        b, pos = _ge_read_byte(data, pos, repeat, include_fe_repeats=include_fe_repeats)
        vl_val = ((vl_val << 8) | b) & 0xFFFFFFFF
        if (b >> 7) == 0:
            break
    out = 0
    for c, a in zip((0, 8, 16, 24), (0, 7, 14, 21)):
        out += ((vl_val >> c) & 0x7F) << a
    return int(out), pos


def _midi_vlq_encode(value: int) -> bytes:
    v = max(0, int(value))
    out = [v & 0x7F]
    v >>= 7
    while v > 0:
        out.append(0x80 | (v & 0x7F))
        v >>= 7
    out.reverse()
    return bytes(out)


def _scan_bz64_uncompressed_sequences(rom_be: bytes, start: int, end: int) -> List[Tuple[int, int]]:
    out: List[Tuple[int, int]] = []
    current = max(0, int(start))
    end_spot = min(len(rom_be), int(end))
    if current >= end_spot:
        return out

    while current < end_spot:
        start_current = current
        invalid = False
        highest_end = 0x44

        for test in range(current, min(current + 0x40, end_spot - 3), 4):
            off = be32(rom_be, test)
            if ((off >> 8) & 0x0FFFFFFF) == 0x00FF2F:
                highest_end = off + 4
                invalid = True
                break
            if off == 0:
                continue

            byte_check = 0
            while True:
                p = current + off + byte_check
                if p < 0 or (p + 1) >= len(rom_be):
                    break
                if rom_be[p] == 0xFF and rom_be[p + 1] == 0x2F:
                    byte_check += 2
                    break
                if (p + 3) < len(rom_be) and rom_be[p : p + 4] in (b"\x00\x00\x00\x44", b"\x42\x31\x00\x01"):
                    break
                byte_check += 1
                if byte_check > end_spot:
                    break

            highest_end = max(highest_end, off + byte_check)

        if (highest_end % 4) != 0:
            highest_end += 4 - (highest_end % 4)
        highest_end = max(4, highest_end)
        current += highest_end

        while (current + 4) < end_spot:
            if rom_be[current : current + 5] == b"\x00\x00\x00\x44\x00":
                break
            current += 4

        if not invalid and current > start_current:
            out.append((start_current, current - start_current))
        if current <= start_current:
            current = start_current + 4

    return out


def _parse_ge_track_events(track_data: bytes, track_offset: int, channel: int) -> Tuple[List[GeTrackEvent], int, Optional[int], Optional[int]]:
    events: List[GeTrackEvent] = []
    instruments = 0
    loop_start: Optional[int] = None
    loop_end: Optional[int] = None

    pos = int(track_offset)
    abs_time = 0
    prev_event = 0
    repeat = GeRepeatState()
    loop_ends_with_count: Dict[int, int] = {}

    safety = 0
    while pos < len(track_data) and safety < 0x80000:
        safety += 1
        delta, pos = _ge_read_vl(track_data, pos, repeat, include_fe_repeats=True)
        abs_time += delta

        event_val, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
        status_bit = event_val < 0x80

        if event_val == 0xFF or (status_bit and prev_event == 0xFF):
            if status_bit:
                subtype = event_val
            else:
                subtype, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)

            if subtype == 0x51:
                b1, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                b2, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                b3, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                events.append(
                    GeTrackEvent(
                        absolute_time=abs_time,
                        delta_time=delta,
                        event_type=0xFF,
                        data=bytes((0x51, 0x03, b1, b2, b3)),
                    )
                )
            elif subtype == 0x2D:
                loop_count, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                _cur_loop, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                b0, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=False)
                b1, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=False)
                b2, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=False)
                b3, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=False)
                offset_to_loop = ((b0 << 24) | (b1 << 16) | (b2 << 8) | b3) & 0xFFFFFFFF
                if loop_count in (0x00, 0xFF):
                    loop_end = abs_time
                elif repeat.pattern is None:
                    key = pos
                    if key in loop_ends_with_count:
                        count_left = loop_ends_with_count[key]
                        if count_left == 0:
                            del loop_ends_with_count[key]
                        else:
                            loop_ends_with_count[key] = count_left - 1
                            pos = max(0, pos - int(offset_to_loop))
                    else:
                        loop_ends_with_count[key] = max(0, int(loop_count) - 1)
                        pos = max(0, pos - int(offset_to_loop))
            elif subtype == 0x2E:
                loop_number, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                _end_loop_ff, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
                loop_start = abs_time
                events.append(
                    GeTrackEvent(
                        absolute_time=abs_time,
                        delta_time=delta,
                        event_type=(0xB0 | (channel & 0x0F)),
                        data=bytes((102, loop_number)),
                    )
                )
            elif subtype == 0x2F:
                events.append(
                    GeTrackEvent(
                        absolute_time=abs_time,
                        delta_time=delta,
                        event_type=0xFF,
                        data=b"\x2F\x00",
                    )
                )
                break

            if not status_bit:
                prev_event = event_val
            continue

        if status_bit and prev_event == 0:
            break

        status = prev_event if status_bit else event_val
        if (status & 0xF0) == 0x80:
            if status_bit:
                note = event_val
            else:
                note, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            vel, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((note, vel))))
        elif (status & 0xF0) == 0x90:
            if status_bit:
                note = event_val
            else:
                note, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            vel, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            dur, pos = _ge_read_vl(track_data, pos, repeat, include_fe_repeats=True)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((note, vel)), duration_time=int(dur)))
        elif (status & 0xF0) == 0xA0:
            if status_bit:
                p1 = event_val
            else:
                p1, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            p2, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((p1, p2))))
        elif (status & 0xF0) == 0xB0:
            if status_bit:
                ctype = event_val
            else:
                ctype, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            cval, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((ctype, cval))))
        elif (status & 0xF0) == 0xC0:
            if status_bit:
                prog = event_val
            else:
                prog, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            instruments = max(instruments, int(prog) + 1)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((prog,))))
        elif (status & 0xF0) == 0xD0:
            if status_bit:
                val = event_val
            else:
                val, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((val,))))
        elif (status & 0xF0) == 0xE0:
            if status_bit:
                lsb = event_val
            else:
                lsb, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            msb, pos = _ge_read_byte(track_data, pos, repeat, include_fe_repeats=True)
            events.append(GeTrackEvent(abs_time, delta, status, bytes((lsb, msb))))
        elif status == 0xFE:
            # Repeat marker should be consumed in _ge_read_byte.
            pass
        else:
            break

        if not status_bit:
            prev_event = event_val

    return events, instruments, loop_start, loop_end


def _expand_ge_note_durations(events: Sequence[GeTrackEvent]) -> List[GeTrackEvent]:
    expanded: List[Tuple[int, int, int, int, bytes]] = []
    end_time: Optional[int] = None

    for idx, ev in enumerate(events):
        expanded.append((int(ev.absolute_time), 1, idx, int(ev.event_type), bytes(ev.data)))
        if ev.event_type == 0xFF and ev.data.startswith(b"\x2F"):
            end_time = ev.absolute_time

    for idx, ev in enumerate(events):
        if not (0x90 <= ev.event_type < 0xA0):
            continue
        if ev.duration_time <= 0 or len(ev.data) < 2:
            continue
        note_off_time = ev.absolute_time + ev.duration_time
        if end_time is not None and note_off_time > end_time:
            note_off_time = end_time
        expanded.append((int(note_off_time), 0, idx, int(ev.event_type), bytes((ev.data[0], 0))))

    expanded.sort(key=lambda t: (t[0], t[1], t[2]))
    out: List[GeTrackEvent] = []
    prev_abs = 0
    for abs_time, _priority, _idx, event_type, data in expanded:
        delta = max(0, int(abs_time) - int(prev_abs))
        out.append(
            GeTrackEvent(
                absolute_time=int(abs_time),
                delta_time=delta,
                event_type=int(event_type),
                data=bytes(data),
            )
        )
        prev_abs = int(abs_time)
    return out


def _encode_ge_track_events(events: Sequence[GeTrackEvent]) -> bytes:
    out = bytearray()
    prev_event: Optional[int] = None
    for ev in events:
        out.extend(_midi_vlq_encode(ev.delta_time))
        if prev_event != ev.event_type or ev.event_type == 0xFF:
            out.append(ev.event_type & 0xFF)
        out.extend(ev.data)
        prev_event = ev.event_type
    return bytes(out)


def _ge_sequence_to_midi(seq_data: bytes) -> Tuple[bytes, Dict[str, object]]:
    if len(seq_data) < 0x44:
        raise ValueError("GE sequence chunk is too small (expected >= 0x44 bytes)")

    track_offsets: List[int] = [be32(seq_data, i) for i in range(0, 0x40, 4)]
    used_tracks: List[Tuple[int, int]] = [(ti, off) for ti, off in enumerate(track_offsets) if off != 0]
    if not used_tracks:
        raise ValueError("No non-zero track offsets found in GE sequence header")

    division = be32(seq_data, 0x40) & 0xFFFF
    if division <= 0:
        division = 120

    tracks_payload: List[bytes] = []
    instruments = 0
    loop_start: Optional[int] = None
    loop_end: Optional[int] = None

    for ti, off in used_tracks:
        if off >= len(seq_data):
            raise ValueError(f"Track {ti} offset 0x{off:X} is out of range")
        events, inst_count, lstart, lend = _parse_ge_track_events(seq_data, off, channel=ti)
        instruments = max(instruments, inst_count)
        if lstart is not None and loop_start is None:
            loop_start = lstart
        if lend is not None and loop_end is None:
            loop_end = lend
        expanded = _expand_ge_note_durations(events)
        payload = _encode_ge_track_events(expanded)
        tracks_payload.append(payload)

    midi = bytearray()
    midi.extend(b"MThd")
    midi.extend(struct.pack(">I", 6))
    midi.extend(struct.pack(">H", 1))
    midi.extend(struct.pack(">H", len(tracks_payload)))
    midi.extend(struct.pack(">H", int(division)))
    for payload in tracks_payload:
        midi.extend(b"MTrk")
        midi.extend(struct.pack(">I", len(payload)))
        midi.extend(payload)

    info: Dict[str, object] = {
        "track_count": len(tracks_payload),
        "division": int(division),
        "instrument_count": int(instruments),
        "loop_start": (int(loop_start) if loop_start is not None else None),
        "loop_end": (int(loop_end) if loop_end is not None else None),
    }
    return bytes(midi), info


def cmd_audio_export_midi_bz64(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, _ = _normalize_rom_be(raw)

    start = _parse_num_auto(str(args.start))
    end = _parse_num_auto(str(args.end))
    if end <= start:
        raise ValueError(f"Invalid range: end (0x{end:X}) must be greater than start (0x{start:X})")

    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    seq_ranges = _scan_bz64_uncompressed_sequences(rom_be, start, end)
    if args.limit is not None:
        seq_ranges = seq_ranges[: max(0, int(args.limit))]

    game_name = str(args.game_name).strip() or BZ64_DEFAULT_MIDI_GAME_NAME
    files: List[Dict[str, object]] = []
    written_mid = 0
    written_bin = 0
    failed = 0

    for idx, (off, size) in enumerate(seq_ranges):
        base_name = f"{game_name} {idx:08X} {off:08X}"
        rec: Dict[str, object] = {
            "index": idx,
            "offset": int(off),
            "size": int(size),
            "base_name": base_name,
        }

        seq = rom_be[off : off + size]
        if not args.no_bin:
            bin_path = out_dir / f"{base_name}.bin"
            bin_path.write_bytes(seq)
            rec["bin_path"] = str(bin_path)
            written_bin += 1

        try:
            midi_bytes, meta = _ge_sequence_to_midi(seq)
            mid_path = out_dir / f"{base_name}.mid"
            mid_path.write_bytes(midi_bytes)
            rec["mid_path"] = str(mid_path)
            rec["midi_size"] = len(midi_bytes)
            rec.update(meta)
            written_mid += 1
        except Exception as exc:
            rec["error"] = str(exc)
            failed += 1

        files.append(rec)

    manifest = {
        "rom": args.rom,
        "outdir": str(out_dir),
        "game_name": game_name,
        "range": {"start": int(start), "end": int(end)},
        "counts": {
            "sequence_chunks": len(seq_ranges),
            "written_midis": written_mid,
            "written_bins": written_bin,
            "failed": failed,
        },
        "files": files,
    }
    manifest_path = out_dir / "manifest_midi.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"manifest": str(manifest_path), **manifest["counts"]}, indent=2))
    return 0


def _likely_seq_table(rom: bytes, off: int) -> Optional[Dict[str, object]]:
    if off + 12 >= len(rom):
        return None
    rev = struct.unpack_from(">H", rom, off)[0]
    cnt = struct.unpack_from(">H", rom, off + 2)[0]
    if cnt < 2 or cnt > 512:
        return None
    end = off + 4 + (cnt * 8)
    if end > len(rom):
        return None
    good = 0
    total_len = 0
    for i in range(cnt):
        s = be32(rom, off + 4 + (i * 8) + 0)
        l = be32(rom, off + 4 + (i * 8) + 4)
        if l == 0 or l > 0x200000:
            continue
        if s + l <= 0x4000000:
            good += 1
            total_len += l
    if good < max(2, cnt // 2):
        return None
    return {"offset": off, "revision": rev, "count": cnt, "good_entries": good, "sum_len": total_len}


def _parse_midi_length(data: bytes, off: int) -> Optional[int]:
    if off + 14 > len(data) or data[off : off + 4] != b"MThd":
        return None
    hd_len = be32(data, off + 4)
    pos = off + 8 + hd_len
    if pos > len(data):
        return None
    tracks = struct.unpack_from(">H", data, off + 10)[0]
    for _ in range(tracks):
        if pos + 8 > len(data) or data[pos : pos + 4] != b"MTrk":
            return None
        tlen = be32(data, pos + 4)
        pos += 8 + tlen
        if pos > len(data):
            return None
    return pos - off


def cmd_audio_scan(args: argparse.Namespace) -> int:
    raw = pathlib.Path(args.rom).read_bytes()
    rom_be, _ = _normalize_rom_be(raw)
    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    midi_hits: List[Dict[str, object]] = []
    pos = 0
    while True:
        i = rom_be.find(b"MThd", pos)
        if i < 0:
            break
        mlen = _parse_midi_length(rom_be, i)
        if mlen is not None and mlen > 0:
            out = out_dir / f"midi_{i:08X}.mid"
            out.write_bytes(rom_be[i : i + mlen])
            midi_hits.append({"offset": i, "size": mlen, "path": str(out)})
        pos = i + 1

    seq_tables: List[Dict[str, object]] = []
    step = max(1, int(args.step))
    for off in range(0, max(0, len(rom_be) - 16), step):
        rec = _likely_seq_table(rom_be, off)
        if rec is not None:
            seq_tables.append(rec)
    # Deduplicate nearby hits
    dedup: List[Dict[str, object]] = []
    last = -0x10000
    for r in sorted(seq_tables, key=lambda x: int(x["offset"])):
        o = int(r["offset"])
        if o - last < 0x20:
            continue
        dedup.append(r)
        last = o
    seq_tables = dedup[: int(args.max_tables)]

    report = {
        "rom": args.rom,
        "counts": {
            "midi_headers": len(midi_hits),
            "sequence_table_candidates": len(seq_tables),
        },
        "midi": midi_hits,
        "sequence_tables": seq_tables,
    }
    out_manifest = out_dir / "manifest.json"
    out_manifest.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({"manifest": str(out_manifest), **report["counts"]}, indent=2))
    return 0


def _u64hex_le(b: bytes) -> str:
    return f"0x{int.from_bytes(b, 'little'):016X}"


def _sanitize_scene_token(name: str) -> str:
    token = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip())
    token = token.strip("._-")
    return token if token else "scene"


def _import_scene_rip_one(
    input_path: pathlib.Path,
    out_dir: pathlib.Path,
    include_zero_crc: bool = False,
) -> Tuple[Dict[str, object], Counter[str], Counter[str]]:
    data = input_path.read_bytes()
    if len(data) < 36:
        raise ValueError("Input is too small for .glr header")
    if data[0:6] != b"GL64R\x00":
        raise ValueError("Invalid scene-rip magic, expected GL64R\\0")

    version = struct.unpack_from("<H", data, 6)[0]
    rom_name = data[8:28].decode("ascii", errors="ignore").rstrip(" \x00")
    tri_count = struct.unpack_from("<I", data, 28)[0]
    microcode = struct.unpack_from("<I", data, 32)[0]
    payload = data[36:]
    if tri_count <= 0:
        raise ValueError("Scene-rip file contains zero triangles")
    if len(payload) % tri_count != 0:
        raise ValueError("Scene-rip payload size is not divisible by triangle count")

    tri_sz = len(payload) // tri_count
    if tri_sz not in (216,):
        raise ValueError(f"Unsupported triangle record size: {tri_sz}")

    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    obj_path = out_dir / "scene.obj"
    csv_path = out_dir / "triangles.csv"
    manifest_path = out_dir / "manifest.json"

    tri_rows: List[Dict[str, object]] = []
    tex_counter: Counter[str] = Counter()
    tex_slot_counter: Counter[str] = Counter()
    triangles_with_tex_crc = 0
    v_count = 0
    vt_count = 0
    min_xyz = [float("inf"), float("inf"), float("inf")]
    max_xyz = [float("-inf"), float("-inf"), float("-inf")]
    zero_crc = "0x0000000000000000"

    with obj_path.open("w", encoding="utf-8") as fobj:
        fobj.write("# GLideN64 scene rip import\n")
        fobj.write(f"# rom={rom_name} version={version} microcode={microcode}\n")
        for i in range(tri_count):
            base = i * tri_sz
            rec = payload[base : base + tri_sz]
            verts: List[Tuple[float, float, float, float, float]] = []
            o = 0
            for _vi in range(3):
                sx, sy, sz, _r, _g, _b, _a, s0, t0, _s1, _t1 = struct.unpack_from("<11f", rec, o)
                o += 44
                verts.append((sx, sy, sz, s0, t0))
                min_xyz[0] = min(min_xyz[0], sx)
                min_xyz[1] = min(min_xyz[1], sy)
                min_xyz[2] = min(min_xyz[2], sz)
                max_xyz[0] = max(max_xyz[0], sx)
                max_xyz[1] = max(max_xyz[1], sy)
                max_xyz[2] = max(max_xyz[2], sz)

            # Skip non-geometry color/mode fields to tex info region.
            tex_off = 192
            crc0 = _u64hex_le(rec[tex_off + 0 : tex_off + 8])
            crc1 = _u64hex_le(rec[tex_off + 12 : tex_off + 20])
            tri_has_tex = False
            if include_zero_crc or crc0 != zero_crc:
                tex_counter[crc0] += 1
                tex_slot_counter["tex0"] += 1
                tri_has_tex = True
            if include_zero_crc or crc1 != zero_crc:
                tex_counter[crc1] += 1
                tex_slot_counter["tex1"] += 1
                tri_has_tex = True
            if tri_has_tex:
                triangles_with_tex_crc += 1

            tri_rows.append({"index": i, "tex0_crc": crc0, "tex1_crc": crc1})

            fobj.write(f"g tri_{i:06d}\n")
            for sx, sy, sz, _s, _t in verts:
                fobj.write(f"v {sx:.6f} {sy:.6f} {sz:.6f}\n")
            for _sx, _sy, _sz, s, t in verts:
                fobj.write(f"vt {s:.6f} {t:.6f}\n")
            fobj.write(
                f"f {v_count+1}/{vt_count+1} {v_count+2}/{vt_count+2} {v_count+3}/{vt_count+3}\n"
            )
            v_count += 3
            vt_count += 3

    with csv_path.open("w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=["index", "tex0_crc", "tex1_crc"])
        w.writeheader()
        w.writerows(tri_rows)

    manifest = {
        "input": str(input_path),
        "rom_name": rom_name,
        "version": version,
        "microcode": microcode,
        "triangles": tri_count,
        "triangle_record_size": tri_sz,
        "obj": str(obj_path),
        "csv": str(csv_path),
        "bbox_min": min_xyz,
        "bbox_max": max_xyz,
        "triangles_with_texture_crc": triangles_with_tex_crc,
        "texture_crc_hits": int(sum(tex_counter.values())),
        "unique_texture_crc": len(tex_counter),
        "texture_crc_slot_hits": {
            "tex0": int(tex_slot_counter.get("tex0", 0)),
            "tex1": int(tex_slot_counter.get("tex1", 0)),
        },
        "texture_crc_top": [
            {"crc": crc, "triangle_slot_hits": hits}
            for crc, hits in tex_counter.most_common(32)
        ],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest, tex_counter, tex_slot_counter


def cmd_import_scene_rip(args: argparse.Namespace) -> int:
    manifest, _tex_counter, _tex_slot_counter = _import_scene_rip_one(
        input_path=pathlib.Path(args.input),
        out_dir=pathlib.Path(args.outdir),
        include_zero_crc=bool(args.include_zero_crc),
    )
    print(json.dumps(manifest, indent=2))
    return 0


def cmd_import_scene_rip_batch(args: argparse.Namespace) -> int:
    in_dir = pathlib.Path(args.indir)
    if not in_dir.exists() or not in_dir.is_dir():
        raise ValueError(f"Input directory not found: {in_dir}")

    out_dir = pathlib.Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pattern = str(args.glob)
    if bool(args.recursive):
        cand = sorted(in_dir.rglob(pattern))
    else:
        cand = sorted(in_dir.glob(pattern))
    inputs = [p for p in cand if p.is_file()]
    if args.limit is not None and int(args.limit) > 0:
        inputs = inputs[: int(args.limit)]
    if not inputs:
        raise ValueError(f"No files matched pattern '{pattern}' in {in_dir}")

    include_zero_crc = bool(args.include_zero_crc)
    global_tex_counter: Counter[str] = Counter()
    global_scene_hits: Counter[str] = Counter()
    global_slot_hits: Counter[str] = Counter()
    scene_rows: List[Dict[str, object]] = []
    failures: List[Dict[str, object]] = []
    triangles_total = 0

    for i, src in enumerate(inputs):
        scene_key = f"s{i:04d}_{_sanitize_scene_token(src.stem)}"
        scene_out = out_dir / scene_key
        try:
            manifest, tex_counter, tex_slot_counter = _import_scene_rip_one(
                input_path=src,
                out_dir=scene_out,
                include_zero_crc=include_zero_crc,
            )
        except Exception as exc:
            failures.append({"input": str(src), "error": str(exc)})
            if bool(args.stop_on_error):
                raise
            continue

        triangles_total += int(manifest.get("triangles", 0))
        global_tex_counter.update(tex_counter)
        global_slot_hits.update(tex_slot_counter)
        for crc in tex_counter.keys():
            global_scene_hits[crc] += 1

        try:
            rel_path = src.relative_to(in_dir)
        except ValueError:
            rel_path = src
        scene_rows.append(
            {
                "input": str(src),
                "relative_input": str(rel_path),
                "outdir": str(scene_out),
                "manifest": str(scene_out / "manifest.json"),
                "triangles": int(manifest.get("triangles", 0)),
                "unique_texture_crc": int(manifest.get("unique_texture_crc", 0)),
                "rom_name": str(manifest.get("rom_name", "")),
                "microcode": int(manifest.get("microcode", 0)),
            }
        )

    texture_rows = [
        {
            "crc": crc,
            "triangle_slot_hits": int(hits),
            "scene_hits": int(global_scene_hits.get(crc, 0)),
        }
        for crc, hits in global_tex_counter.most_common()
    ]
    texture_manifest = {
        "indir": str(in_dir),
        "outdir": str(out_dir),
        "glob": pattern,
        "recursive": bool(args.recursive),
        "include_zero_crc": include_zero_crc,
        "counts": {
            "unique_texture_crc": len(global_tex_counter),
            "texture_crc_hits": int(sum(global_tex_counter.values())),
        },
        "slot_hits": {
            "tex0": int(global_slot_hits.get("tex0", 0)),
            "tex1": int(global_slot_hits.get("tex1", 0)),
        },
        "textures": texture_rows,
    }
    texture_manifest_path = out_dir / "texture_crc_manifest.json"
    texture_manifest_path.write_text(json.dumps(texture_manifest, indent=2), encoding="utf-8")

    batch_manifest = {
        "indir": str(in_dir),
        "outdir": str(out_dir),
        "glob": pattern,
        "recursive": bool(args.recursive),
        "include_zero_crc": include_zero_crc,
        "counts": {
            "matched_files": len(inputs),
            "imported": len(scene_rows),
            "failed": len(failures),
            "triangles_total": triangles_total,
            "unique_texture_crc": len(global_tex_counter),
            "texture_crc_hits": int(sum(global_tex_counter.values())),
        },
        "slot_hits": {
            "tex0": int(global_slot_hits.get("tex0", 0)),
            "tex1": int(global_slot_hits.get("tex1", 0)),
        },
        "texture_crc_manifest": str(texture_manifest_path),
        "scenes": scene_rows,
        "failures": failures,
    }
    batch_manifest_path = out_dir / "manifest.json"
    batch_manifest_path.write_text(json.dumps(batch_manifest, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "manifest": str(batch_manifest_path),
                "texture_crc_manifest": str(texture_manifest_path),
                **batch_manifest["counts"],
            },
            indent=2,
        )
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Battlezone 64 extraction helper")
    sub = p.add_subparsers(dest="cmd", required=True)

    pri = sub.add_parser("rom-info", help="Report ROM byte order, header, bootcode CRC, and computed CRCs")
    pri.add_argument("--rom", required=True, help="Path to ROM (.z64/.v64/.n64)")
    pri.add_argument("--json", help="Optional output JSON path")
    pri.set_defaults(func=cmd_rom_info)

    prv = sub.add_parser("rom-crc-verify", help="Verify ROM header CRCs against computed values")
    prv.add_argument("--rom", required=True, help="Path to ROM")
    prv.add_argument("--cic", type=int, choices=[6101, 6102, 6103, 6105, 6106], help="Override CIC variant")
    prv.set_defaults(func=cmd_rom_crc_verify)

    prf = sub.add_parser("rom-crc-fix", help="Recompute and patch ROM header CRC1/CRC2")
    prf.add_argument("--rom", required=True, help="Path to ROM")
    prf.add_argument("--out", help="Optional output path (default: in-place)")
    prf.add_argument("--cic", type=int, choices=[6101, 6102, 6103, 6105, 6106], help="Override CIC variant")
    prf.set_defaults(func=cmd_rom_crc_fix)

    psc = sub.add_parser("scan-containers", help="Scan ROM for Yay0/MIO0/Yaz0 stream headers")
    psc.add_argument("--rom", required=True, help="Path to ROM")
    psc.add_argument("--json", help="Optional output JSON path")
    psc.set_defaults(func=cmd_scan_containers)

    pds = sub.add_parser("decompress-stream", help="Decompress a single Yay0/MIO0/Yaz0 stream by ROM offset")
    pds.add_argument("--rom", required=True, help="Path to ROM")
    pds.add_argument("--offset", required=True, help="Stream offset (hex or int)")
    pds.add_argument("--type", default="auto", choices=["auto", "yay0", "mio0", "yaz0"], help="Stream type")
    pds.add_argument("--out", required=True, help="Output decompressed binary path")
    pds.set_defaults(func=cmd_decompress_stream)

    psp = sub.add_parser("split-config", help="Config-driven splitter (JSON/YAML) with optional stream decompression")
    psp.add_argument("--rom", required=True, help="Path to ROM")
    psp.add_argument("--config", required=True, help="Config file (.json/.yaml/.yml)")
    psp.add_argument("--outdir", required=True, help="Output folder")
    psp.set_defaults(func=cmd_split_config)

    ptd = sub.add_parser("texture-decode-raw", help="Decode raw N64 texture bytes to PNG")
    ptd.add_argument("--input", required=True, help="Raw texture binary path")
    ptd.add_argument("--out", required=True, help="Output PNG path")
    ptd.add_argument("--width", required=True, type=int, help="Texture width")
    ptd.add_argument("--height", required=True, type=int, help="Texture height")
    ptd.add_argument("--fmt", required=True, help="Format: rgba|ia|i|ci|yuv")
    ptd.add_argument("--siz", required=True, type=int, help="Bit size: 4|8|16|32")
    ptd.add_argument("--tlut", help="Optional TLUT binary path for CI formats")
    ptd.set_defaults(func=cmd_texture_decode_raw)

    pte = sub.add_parser("texture-encode-raw", help="Encode PNG to raw N64 texture bytes")
    pte.add_argument("--input", required=True, help="Input image path")
    pte.add_argument("--out", required=True, help="Output raw binary path")
    pte.add_argument("--fmt", required=True, help="Target format: rgba|ia|i|ci")
    pte.add_argument("--siz", required=True, type=int, help="Target bit size: 4|8|16")
    pte.add_argument("--tlut-out", help="Optional TLUT output path for CI formats")
    pte.set_defaults(func=cmd_texture_encode_raw)

    pas = sub.add_parser("audio-scan", help="Scan ROM for MIDI and sequence-table-like audio structures")
    pas.add_argument("--rom", required=True, help="Path to ROM")
    pas.add_argument("--outdir", required=True, help="Output directory for carved files + manifest")
    pas.add_argument("--step", type=int, default=4, help="Scan step in bytes for table heuristics (default: 4)")
    pas.add_argument("--max-tables", type=int, default=256, help="Max sequence-table candidates kept (default: 256)")
    pas.set_defaults(func=cmd_audio_scan)

    paw = sub.add_parser("audio-extract-wav", help="Extract all bank audio samples to WAV (N64SoundTools-style standard banks)")
    paw.add_argument("--rom", required=True, help="Path to ROM")
    paw.add_argument("--outdir", required=True, help="Output folder for WAV files + manifest")
    paw.add_argument(
        "--bank",
        action="append",
        default=[],
        help="Explicit CTL,TBL pair (hex/int), repeatable. Example: --bank C217B0,C06F08",
    )
    paw.add_argument("--config-ini", help="Optional gameconfigsound.ini path from N64SoundTools")
    paw.add_argument(
        "--game",
        default="Battlezone - Rise of the Black Dogs (U)",
        help="Game section name when using --config-ini (default: Battlezone section)",
    )
    paw.add_argument("--sample-rate", type=int, default=8000, help="Output WAV sample rate (default: 8000)")
    paw.add_argument("--half-sampling-rate", action="store_true", help="Write WAV files at half the parsed/fallback sample rate")
    paw.add_argument("--limit", type=int, help="Optional maximum number of WAV files to write")
    paw.set_defaults(func=cmd_audio_extract_wav)

    pam = sub.add_parser(
        "audio-export-midi-bz64",
        help="Battlezone-specific GE MIDI export (split sequence bank and convert to .mid)",
    )
    pam.add_argument("--rom", required=True, help="Path to ROM")
    pam.add_argument("--outdir", required=True, help="Output folder for MIDI files + manifest")
    pam.add_argument(
        "--start",
        default=f"0x{BZ64_DEFAULT_MIDI_RANGE[0]:08X}",
        help=f"Start offset for Battlezone sequence bank (default: 0x{BZ64_DEFAULT_MIDI_RANGE[0]:08X})",
    )
    pam.add_argument(
        "--end",
        default=f"0x{BZ64_DEFAULT_MIDI_RANGE[1]:08X}",
        help=f"End offset (exclusive) for Battlezone sequence bank (default: 0x{BZ64_DEFAULT_MIDI_RANGE[1]:08X})",
    )
    pam.add_argument(
        "--game-name",
        default=BZ64_DEFAULT_MIDI_GAME_NAME,
        help="Filename prefix for exported files (default: Battlezone section name)",
    )
    pam.add_argument("--limit", type=int, help="Optional maximum sequence chunks to convert")
    pam.add_argument("--no-bin", action="store_true", help="Skip writing raw .bin sequence chunks")
    pam.set_defaults(func=cmd_audio_export_midi_bz64)

    psr = sub.add_parser("import-scene-rip", help="Import GLideN64 .glr scene-rip file to OBJ/CSV/JSON")
    psr.add_argument("--input", required=True, help="Input .glr file path")
    psr.add_argument("--outdir", required=True, help="Output directory")
    psr.add_argument("--include-zero-crc", action="store_true", help="Include 0 CRC values in texture summary stats")
    psr.set_defaults(func=cmd_import_scene_rip)

    psrb = sub.add_parser("import-scene-rip-batch", help="Batch import GLideN64 .glr files and build texture CRC manifest")
    psrb.add_argument("--indir", required=True, help="Input directory containing scene-rip files")
    psrb.add_argument("--outdir", required=True, help="Output directory")
    psrb.add_argument("--glob", default="*.glr", help="Filename pattern to import (default: *.glr)")
    psrb.add_argument("--recursive", action="store_true", help="Recursively search subdirectories of --indir")
    psrb.add_argument("--limit", type=int, help="Optional maximum files to import")
    psrb.add_argument("--include-zero-crc", action="store_true", help="Include 0 CRC values in texture index")
    psrb.add_argument("--stop-on-error", action="store_true", help="Abort batch on first failed import")
    psrb.set_defaults(func=cmd_import_scene_rip_batch)

    ps = sub.add_parser("scan", help="Scan ROM for model signatures and mission-like filenames")
    ps.add_argument("--rom", required=True, help="Path to .z64 ROM")
    ps.add_argument("--json", help="Optional output JSON report path")
    ps.set_defaults(func=cmd_scan)

    pm = sub.add_parser("export-model", help="Export BZn64Model blob to OBJ/MTL/PNG")
    pm.add_argument("--input", required=True, help="Input model blob (e.g. A1E6E4.bin)")
    pm.add_argument("--out", required=True, help="Output base path, without extension")
    pm.add_argument("--texture", action="store_true", help="Export CI4 texture PNG (requires Pillow)")
    pm.add_argument("--strict-mesh", action="store_true", help="Apply strict mesh cleanup/checks during single export")
    pm.set_defaults(func=cmd_export_model)

    pb = sub.add_parser("batch-models", help="Batch export candidate model blobs from ROM")
    pb.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pb.add_argument("--outdir", required=True, help="Output folder for batch model exports")
    pb.add_argument("--limit", type=int, help="Optional max candidates to process")
    pb.add_argument("--window", type=lambda x: int(x, 0), default=0x6000, help="Bytes to slice per candidate offset (default: 0x6000)")
    pb.add_argument("--min-tris", type=int, default=8, help="Skip exports with fewer triangles (default: 8)")
    pb.add_argument("--texture", action="store_true", help="Try CI4 texture extraction using current known layout")
    pb.add_argument("--strict-mesh", action="store_true", help="Apply stricter anti-spaghetti mesh filtering")
    pb.add_argument("--standalone-only", action="store_true", help="Exclude models that reference non-local matrix/DL segments")
    pb.set_defaults(func=cmd_batch_models)

    py = sub.add_parser("batch-models-yay0", help="Decompress Yay0 chunks and batch export embedded models")
    py.add_argument("--rom", required=True, help="Path to .z64 ROM")
    py.add_argument("--outdir", required=True, help="Output folder for extracted models")
    py.add_argument("--limit", type=int, help="Optional max Yay0 chunks to process")
    py.add_argument("--window", type=lambda x: int(x, 0), default=0x6000, help="Bytes to slice from each decompressed model offset")
    py.add_argument("--min-tris", type=int, default=8, help="Skip exports with fewer triangles (default: 8)")
    py.add_argument("--texture", action="store_true", help="Try CI4 texture extraction using current known layout")
    py.add_argument("--all-signatures", action="store_true", help="Export every model signature in each chunk (default: only root offsets 0x08/0x10)")
    py.add_argument("--strict-mesh", action="store_true", help="Apply stricter anti-spaghetti mesh filtering")
    py.add_argument("--standalone-only", action="store_true", help="Exclude models that reference non-local matrix/DL segments")
    py.add_argument("--merge-chunk-models", action="store_true", help="Also write one merged OBJ/MTL per Yay0 chunk from exported signatures")
    py.set_defaults(func=cmd_batch_models_yay0)

    pz = sub.add_parser("extract-bzn", help="Extract decompressed .bzn chunks from Yay0 streams")
    pz.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pz.add_argument("--outdir", required=True, help="Output folder for .bzn files")
    pz.set_defaults(func=cmd_extract_bzn)

    pr = sub.add_parser("bzn-refs", help="Extract stable reference fields from extracted .bzn containers")
    pr.add_argument("--indir", required=True, help="Directory containing extracted .bzn files")
    pr.add_argument("--out", required=True, help="Output JSON path")
    pr.set_defaults(func=cmd_bzn_refs)

    pic = sub.add_parser("bzn-id-census", help="Census candidate numeric IDs in extracted .bzn containers")
    pic.add_argument("--indir", required=True, help="Directory containing extracted .bzn files")
    pic.add_argument("--out", required=True, help="Output JSON report path")
    pic.add_argument("--focus-name", action="append", default=[], help="Optional glob pattern for BZN stems, repeatable (e.g. n64dm*).")
    pic.add_argument("--focus-priority-set", action="store_true", help="Analyze default high-value set: arcd*, arcdb*, arcds*, n64dm*, race*, misn00, misn20..misn27.")
    pic.add_argument("--scan-type", choices=["u16", "u32"], default="u32", help="Numeric width to scan (default: u32)")
    pic.add_argument("--byte-order", choices=["be", "le"], default="be", help="Byte order for numeric decode (default: be)")
    pic.add_argument("--alignment", type=int, default=4, help="Step size in bytes between scans (default: 4)")
    pic.add_argument("--skip-bytes", type=int, default=0, help="Skip this many initial bytes in each file before scanning (default: 0)")
    pic.add_argument("--min-value", type=int, default=1, help="Minimum numeric value to include (default: 1)")
    pic.add_argument("--max-value", type=int, default=65535, help="Maximum numeric value to include (default: 65535)")
    pic.add_argument("--stock-odf-list", help="Optional text list of stock ODF filenames (one per line or quoted list syntax)")
    pic.add_argument("--id-map-csv", help="Optional CSV template output for manual ID->ODF mapping")
    pic.add_argument("--no-exclude-defaults", action="store_false", dest="exclude_defaults", help="Do not exclude common constants/noise values")
    pic.add_argument("--exclude-value", action="append", default=[], help="Extra value to exclude (hex or int), repeatable")
    pic.add_argument("--min-global-count", type=int, default=8, help="Minimum total occurrences across selected files (default: 8)")
    pic.add_argument("--min-file-count", type=int, default=2, help="Minimum number of files containing the value (default: 2)")
    pic.add_argument("--top-values", type=int, default=256, help="Maximum candidate values written to output (default: 256)")
    pic.add_argument("--per-file-top", type=int, default=48, help="Top values retained per file in output (default: 48)")
    pic.add_argument("--sample-offsets-per-value", type=int, default=8, help="How many sample file/offset hits to keep per value (default: 8)")
    pic.set_defaults(func=cmd_bzn_id_census)

    pmc = sub.add_parser("mission-clues", help="Heuristic clue report for extracted mission .bzn containers")
    pmc.add_argument("--indir", required=True, help="Directory containing extracted .bzn files")
    pmc.add_argument("--out", required=True, help="Output JSON report path")
    pmc.add_argument("--focus-name", action="append", default=[], help="Optional glob pattern for BZN stems, repeatable (e.g. n64dm*).")
    pmc.add_argument("--focus-priority-set", action="store_true", help="Analyze default high-value set: arcd*, arcdb*, arcds*, n64dm*, race*, misn00, misn20..misn27.")
    pmc.add_argument("--string-min-len", type=int, default=4, help="Minimum printable ASCII string length (default: 4)")
    pmc.add_argument("--max-strings", type=int, default=120, help="Max strings kept per file (default: 120)")
    pmc.add_argument("--top-values", type=int, default=32, help="Top-N values/tokens kept in summaries (default: 32)")
    pmc.add_argument("--min-u16-count", type=int, default=3, help="Minimum count for BE u16 frequency tables (default: 3)")
    pmc.add_argument("--min-u32-count", type=int, default=2, help="Minimum count for BE u32 frequency tables (default: 2)")
    pmc.add_argument("--min-specific-u16-count", type=int, default=6, help="Minimum per-file count for mission-specific u16 IDs (default: 6)")
    pmc.add_argument("--min-pointer-run", type=int, default=4, help="Minimum entries for pointer-table candidate runs (default: 4)")
    pmc.add_argument("--max-pointer-runs", type=int, default=24, help="Maximum pointer-table candidate runs per file (default: 24)")
    pmc.add_argument("--min-pc-token-fields", type=int, default=8, help="Minimum fields for PC-style token-stream candidates (default: 8)")
    pmc.add_argument("--max-pc-token-runs", type=int, default=8, help="Maximum PC-style token-stream candidates per file (default: 8)")
    pmc.add_argument("--min-float-run", type=int, default=3, help="Minimum BE-float run length considered a clue (default: 3)")
    pmc.add_argument("--max-float-runs", type=int, default=24, help="Maximum BE-float run candidates per file (default: 24)")
    pmc.set_defaults(func=cmd_mission_clues)

    pt = sub.add_parser("terrain-cluster", help="Extract/label terrain-like chunk cluster as PNG layers")
    pt.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pt.add_argument("--outdir", required=True, help="Output folder for terrain layers and manifest")
    pt.add_argument("--start", default="0x928128", help="Start ROM offset (hex or int)")
    pt.add_argument("--end", default="0x97056A", help="End ROM offset (hex or int)")
    pt.add_argument(
        "--valid-lightmap-offset",
        action="append",
        default=[],
        help="Mark an offset as user-validated lightmap (repeatable, e.g. --valid-lightmap-offset 0x95BDE4)",
    )
    pt.set_defaults(func=cmd_terrain_cluster)

    ph = sub.add_parser("height-candidates", help="Rank likely heightmaps from u16 Yay0 terrain chunks")
    ph.add_argument("--rom", required=True, help="Path to .z64 ROM")
    ph.add_argument("--outdir", required=True, help="Output folder for candidate PNGs and manifest")
    ph.add_argument("--start", default="0x928128", help="Start ROM offset (hex or int)")
    ph.add_argument("--end", default="0x97056A", help="End ROM offset (hex or int)")
    ph.add_argument("--width", type=int, default=128, help="Grid width (default: 128)")
    ph.add_argument("--height", type=int, default=128, help="Grid height (default: 128)")
    ph.add_argument("--size", type=int, default=32768, help="Decompressed chunk size to treat as u16 grid (default: 32768)")
    ph.add_argument("--keep-top", type=int, default=4, help="How many best layout variants to save per chunk (default: 4)")
    ph.add_argument(
        "--valid-lightmap-offset",
        action="append",
        default=[],
        help="Known lightmap offset to demote in ranking (repeatable, e.g. --valid-lightmap-offset 0x95BDE4)",
    )
    ph.add_argument(
        "--valid-light-be-offset",
        action="append",
        default=[],
        help="Offset confirmed where be_linear is lightmap (repeatable)",
    )
    ph.add_argument(
        "--valid-height-le-offset",
        action="append",
        default=[],
        help="Offset confirmed where le_linear is heightmap (repeatable)",
    )
    ph.set_defaults(func=cmd_height_candidates)

    pa = sub.add_parser("terrain-all", help="Full ROM terrain-like extraction (u16 layers + CI4 paints)")
    pa.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pa.add_argument("--outdir", required=True, help="Output folder for exported terrain images and manifest")
    pa.add_argument("--start", help="Optional start ROM offset (hex or int)")
    pa.add_argument("--end", help="Optional end ROM offset (hex or int)")
    pa.add_argument(
        "--u16-dims",
        default="128x128,256x256,512x512",
        help="Comma-separated u16 layout dimensions to evaluate when size matches width*height*2",
    )
    pa.add_argument(
        "--paint-dims",
        default="128x128,64x256,256x64,32x512,512x32,256x256,512x512",
        help="Comma-separated CI4 layout dimensions to evaluate for paint chunks",
    )
    pa.add_argument("--u16-variants-top", type=int, default=3, help="Number of best u16 variants to save per endianness (default: 3)")
    pa.add_argument("--paint-variants-top", type=int, default=3, help="Number of best paint decode variants to save per paint chunk (default: 3)")
    pa.add_argument("--paint-denoise-passes", type=int, default=0, help="Optional paint cleanup passes (3x3 mode filter on CI4 indices)")
    pa.add_argument("--paint-denoise-radius", type=int, default=1, help="Neighborhood radius for paint cleanup filter (default: 1)")
    pa.add_argument(
        "--paint-no-behi64",
        action="store_true",
        help="Disable BE-u16-highbyte 64x64 adaptive paint candidate (enabled by default when size allows)",
    )
    pa.add_argument(
        "--prefer-light-variant",
        action="append",
        default=[],
        help="Offset-specific regex preference for light variants, format OFFSET:REGEX (repeatable)",
    )
    pa.add_argument(
        "--prefer-height-variant",
        action="append",
        default=[],
        help="Offset-specific regex preference for height variants, format OFFSET:REGEX (repeatable)",
    )
    pa.add_argument(
        "--prefer-paint-variant",
        action="append",
        default=[],
        help="Offset-specific regex preference for paint variants, format OFFSET:REGEX (repeatable)",
    )
    pa.set_defaults(func=cmd_terrain_all)

    pd = sub.add_parser("dump-textures-yay0", help="Dump texture/sprite-like PNGs from Yay0 chunks")
    pd.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pd.add_argument("--outdir", required=True, help="Output folder for dumped texture PNGs")
    pd.add_argument("--limit", type=int, help="Optional max Yay0 chunks to process")
    pd.add_argument("--min-tri-hits", type=int, default=2, help="Min triangle hits per model DL texture state (default: 2)")
    pd.add_argument("--generic", action="store_true", help="Enable extra generic sprite/texture probing (noisier)")
    pd.add_argument("--generic-top-per-chunk", type=int, default=8, help="Keep top-N generic candidates per chunk (default: 8)")
    pd.set_defaults(func=cmd_dump_textures_yay0)

    psr2 = sub.add_parser("scan-textures-rom", help="Scan raw ROM ranges for standalone texture/image candidates")
    psr2.add_argument("--rom", required=True, help="Path to .z64 ROM")
    psr2.add_argument("--outdir", required=True, help="Output folder for candidate PNGs and manifest")
    psr2.add_argument("--start", help="Optional start ROM offset (hex or int, default: 0)")
    psr2.add_argument("--end", help="Optional end ROM offset (hex or int, default: end of ROM)")
    psr2.add_argument("--step", type=lambda x: int(x, 0), default=0x20, help="Coarse scan step in bytes (default: 0x20)")
    psr2.add_argument("--refine-step", type=lambda x: int(x, 0), default=2, help="Fine scan step in bytes (default: 2)")
    psr2.add_argument("--refine-radius", default="0x40", help="Fine scan radius around each seed offset (default: 0x40)")
    psr2.add_argument("--dims", default="16x16,32x32", help="Candidate dimensions list (comma/semicolon separated)")
    psr2.add_argument("--ci8-max-pixels", type=int, default=1024, help="Limit CI8 scans to dimensions with <= this many pixels (0 disables limit)")
    psr2.add_argument("--pal-deltas", default="-0x200,-0x80,-0x40,-0x20,0x20,0x40", help="Palette offsets relative to image offset for CI scans")
    psr2.add_argument("--coarse-threshold", type=float, default=1.40, help="Max visual score for coarse seed probes (lower is stricter)")
    psr2.add_argument("--threshold", type=float, default=1.45, help="Max visual score for refined candidate acceptance")
    psr2.add_argument("--min-pal-unique", type=int, default=6, help="Min unique colors in 16-entry palette probe (default: 6)")
    psr2.add_argument("--min-pal256-unique", type=int, default=48, help="Min unique colors in 256-entry CI8 palette probe (default: 48)")
    psr2.add_argument("--allow-pal-overlap", action="store_true", help="Allow palette bytes to overlap image bytes (off by default)")
    psr2.add_argument("--coarse-keep", type=int, default=256, help="Maximum coarse seed offsets kept for refine pass (0 disables coarse seeds)")
    psr2.add_argument("--coarse-dedupe-radius", default="0x80", help="Deduplicate coarse seeds within this byte radius (default: 0x80)")
    psr2.add_argument("--seed-offset", action="append", default=[], help="Extra offset hints to refine (hex/int tokens; supports forms like AD73FE_2)")
    psr2.add_argument("--keep", type=int, default=512, help="Max unique PNG candidates written (default: 512)")
    psr2.add_argument("--ci-only", action="store_true", help="Only evaluate CI4/CI8 recipes (skip I/IA/RGBA probes)")
    psr2.set_defaults(func=cmd_scan_textures_rom)

    pc = sub.add_parser("trace-seg4-context", help="Trace external seg4 texture-context candidates for one model")
    pc.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pc.add_argument("--outdir", required=True, help="Output folder for report/previews")
    pc.add_argument("--yay0-offset", required=True, help="Target Yay0 offset (hex or int)")
    pc.add_argument("--model-offset", required=True, help="Model signature offset in decompressed blob (hex or int)")
    pc.add_argument("--dl-offset", help="Optional explicit DL offset (default: model_offset + 8)")
    pc.add_argument("--tri-raw", action="store_true", help="Use raw TRI indices (default: div2)")
    pc.add_argument("--pad-bytes", type=lambda x: int(x, 0), default=0x800, help="Required bytes beyond max unresolved seg4 offset (default: 0x800)")
    pc.add_argument("--limit", type=int, help="Optional max Yay0 chunks to test")
    pc.add_argument("--top", type=int, default=20, help="Top candidates kept in manifest (default: 20)")
    pc.add_argument("--preview-top", type=int, default=6, help="Write best-texture previews for top-N candidates (default: 6)")
    pc.set_defaults(func=cmd_trace_seg4_context)

    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
