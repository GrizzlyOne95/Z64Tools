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
import dataclasses
import math
import json
import hashlib
import pathlib
import re
import struct
import statistics
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from PIL import Image  # type: ignore
except Exception:  # pragma: no cover
    Image = None


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


def _iter_display_list_exec(data: bytes, dl_off: int = 8, max_cmds: int = 200000) -> Iterable[Tuple[int, int, int, int]]:
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

        if op == 0xDE:  # G_DL
            target = _resolve_segment_offset(data, w1)
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


def _resolve_segment_offset(data: bytes, seg_addr: int) -> Optional[int]:
    seg = (seg_addr >> 24) & 0xFF
    off = seg_addr & 0x00FFFFFF
    # In current captures, segment ids 0x08+ often point into the same
    # decompressed blob; treat them as local when in range.
    if seg in (0x08, 0x09, 0x0A, 0x0B) and 0 <= off < len(data):
        return off
    return None


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
        if 0xDF in ops and 0x01 in ops and 0x05 in ops:
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
    cmds = list(_iter_display_list_exec(data, dl_off=dl_off))
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
            mtx_off = _resolve_segment_offset(data, w1)
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
            if _resolve_segment_offset(data, w1) is None:
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

            voff = _resolve_segment_offset(data, w1)
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


def write_obj_mtl(
    out_base: pathlib.Path,
    verts: Sequence[Vtx],
    tris: Sequence[Tri],
    with_texture: bool,
) -> None:
    obj_path = out_base.with_suffix(".obj")
    mtl_path = out_base.with_suffix(".mtl")
    tex_name = out_base.with_suffix(".png").name

    with obj_path.open("w", encoding="utf-8", newline="\n") as f:
        if with_texture:
            f.write(f"mtllib {mtl_path.name}\n")
            f.write("usemtl mat0\n")
        for v in verts:
            f.write(f"v {v.x} {v.y} {v.z}\n")
        # UV scaling observed in sample: 10.2 fixed-ish style with 4096 tile range.
        for v in verts:
            u = v.s / 4096.0
            vv = 1.0 - (v.t / 4096.0)
            f.write(f"vt {u:.6f} {vv:.6f}\n")
        for t in tris:
            # OBJ is 1-based.
            a = t.i0 + 1
            b = t.i1 + 1
            c = t.i2 + 1
            f.write(f"f {a}/{a} {b}/{b} {c}/{c}\n")

    if with_texture:
        with mtl_path.open("w", encoding="utf-8", newline="\n") as f:
            f.write("newmtl mat0\n")
            f.write("Ka 1.0 1.0 1.0\n")
            f.write("Kd 1.0 1.0 1.0\n")
            f.write("Ks 0.0 0.0 0.0\n")
            f.write("d 1.0\n")
            f.write("illum 1\n")
            f.write(f"map_Kd {tex_name}\n")


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

    verts, tris, stats = parse_model(blob)
    pruned_tris = 0
    if args.strict_mesh:
        verts, tris, pruned_tris = _prune_outlier_tris(verts, tris)
        if not mesh_quality_strict_ok(verts, tris):
            raise ValueError("strict mesh checks failed for this model")
    wrote_texture = False
    if args.texture:
        if Image is None:
            raise RuntimeError("Pillow is required for PNG texture export: pip install pillow")
        pixels, w, h = extract_ci4_texture(blob)
        img = Image.new("RGBA", (w, h))
        img.putdata(pixels)
        img.save(out_base.with_suffix(".png"))
        wrote_texture = True

    write_obj_mtl(out_base, verts, tris, with_texture=wrote_texture)
    print(json.dumps({"out": str(out_base), **stats, "tri_count": len(tris), "vert_count": len(verts), "pruned_outlier_tris": pruned_tris, "texture": wrote_texture}, indent=2))
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
) -> Optional[Dict[str, object]]:
    starts: List[int] = []
    for s in [rom_off, max(0, rom_off - 8), max(0, rom_off - 16)]:
        if s in starts:
            continue
        starts.append(s)

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
    if stats["tri_count"] <= 0 or stats["vert_count"] <= 0:
        return None

    pruned_tris = 0
    if strict_mesh:
        if not disable_prune:
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

    wrote_texture = False
    if texture and Image is not None:
        try:
            pixels, w, h = extract_ci4_texture(chunk)
            img = Image.new("RGBA", (w, h))
            img.putdata(pixels)
            img.save(out_base.with_suffix(".png"))
            wrote_texture = True
        except Exception:
            wrote_texture = False

    write_obj_mtl(out_base, verts, tris, with_texture=wrote_texture)
    return {
        "rom_offset": rom_off,
        "name": out_base.name,
        "obj": str(out_base.with_suffix(".obj")),
        "mtl": str(out_base.with_suffix(".mtl")) if wrote_texture else None,
        "png": str(out_base.with_suffix(".png")) if wrote_texture else None,
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
            vals = [struct.unpack_from(">H", dec, i * 2)[0] for i in range(n)]
            mn, mx = min(vals), max(vals)
            rng = max(1, mx - mn)
            norm = [int((v - mn) * 255 / rng) for v in vals]

            for suffix, data in [("hi", hi), ("lo", lo), ("u16norm", norm)]:
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
    return (1.8 * aniso) + (0.7 * variation) + (1.3 * parity_bias) + (1.1 * isolated_noise)


def _paint_variant_name_penalty(name: str) -> float:
    p = 0.0
    if name.startswith("raw_"):
        p += 0.030
    if name.endswith("_linear"):
        p += 0.008
    m = re.search(r"rowxor(\d+)_", name)
    if m:
        xv = int(m.group(1))
        if xv != 1:
            p += 0.010 + (0.002 * min(15, abs(xv - 1)))
    return p


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
    smooth = _neighbor_absdiff_score(samples, w, h)
    dup = _u16_dup_score(samples, w, h)
    # Prefer square-ish layouts unless another shape is significantly better.
    aspect_penalty = 0.08 * abs(math.log2(max(w, h) / max(1, min(w, h))))
    # Lower smoothness is good for terrain; too-similar halves are suspicious.
    return smooth + aspect_penalty + (0.5 * max(0.0, 0.1 - dup))


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
        return 1e9
    mn = min(samples)
    mx = max(samples)
    rng = max(1, mx - mn)
    return (total / cnt) / rng


def _write_l8(path: pathlib.Path, w: int, h: int, data: Sequence[int]) -> None:
    if Image is None:
        raise RuntimeError("Pillow is required for PNG export: pip install pillow")
    img = Image.new("L", (w, h))
    img.putdata(list(data))
    img.save(path)


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
    for off in headers:
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
    u16_dims = _parse_dim_list(args.u16_dims, 128 * 128)
    if not u16_dims:
        u16_dims = [(128, 128)]

    for off in headers:
        try:
            dec, _ = yay0_decompress(rom, off)
        except Exception as ex:
            other_records.append({"offset": off, "error": str(ex)})
            continue

        size = len(dec)
        if size == 32768:
            n = 128 * 128
            be = [struct.unpack_from(">H", dec, i * 2)[0] for i in range(n)]
            le = [struct.unpack_from("<H", dec, i * 2)[0] for i in range(n)]

            def _u16_variants(vals: Sequence[int], tag: str) -> List[Dict[str, object]]:
                rows: List[Dict[str, object]] = []
                for (w, h) in u16_dims:
                    base = list(vals)
                    rows.append({"name": f"{tag}_linear_{w}x{h}", "w": w, "h": h, "pixels": base, "score": _u16_layout_score(base, w, h)})
                    for ts in [2, 4, 8, 16, 32]:
                        if (w % ts) == 0 and (h % ts) == 0:
                            p = _untile_linear(base, w, h, ts)
                            rows.append(
                                {
                                    "name": f"{tag}_untile_linear_ts{ts}_{w}x{h}",
                                    "w": w,
                                    "h": h,
                                    "pixels": p,
                                    "score": _u16_layout_score(p, w, h),
                                }
                            )
                    for ts in [4, 8, 16]:
                        if (w % ts) == 0 and (h % ts) == 0:
                            p = _untile_morton(base, w, h, ts)
                            rows.append(
                                {
                                    "name": f"{tag}_untile_morton_ts{ts}_{w}x{h}",
                                    "w": w,
                                    "h": h,
                                    "pixels": p,
                                    "score": _u16_layout_score(p, w, h),
                                }
                            )
                rows.sort(key=lambda r: float(r["score"]))
                return rows

            be_vars = _u16_variants(be, "be")
            le_vars = _u16_variants(le, "le")
            be_best = be_vars[0]
            le_best = le_vars[0]

            be_png = out_dir / f"{off:08X}_light_be.png"
            le_png = out_dir / f"{off:08X}_height_le.png"
            _write_l8(be_png, int(be_best["w"]), int(be_best["h"]), _u16_to_norm8(be_best["pixels"]))  # type: ignore[arg-type]
            _write_l8(le_png, int(le_best["w"]), int(le_best["h"]), _u16_to_norm8(le_best["pixels"]))  # type: ignore[arg-type]

            be_saved: List[Dict[str, object]] = []
            for v in be_vars[:u16_topk]:
                vp = out_dir / f"{off:08X}_{v['name']}.png"
                _write_l8(vp, int(v["w"]), int(v["h"]), _u16_to_norm8(v["pixels"]))  # type: ignore[arg-type]
                be_saved.append({"name": v["name"], "score": v["score"], "w": v["w"], "h": v["h"], "png": str(vp)})

            le_saved: List[Dict[str, object]] = []
            for v in le_vars[:u16_topk]:
                vp = out_dir / f"{off:08X}_{v['name']}.png"
                _write_l8(vp, int(v["w"]), int(v["h"]), _u16_to_norm8(v["pixels"]))  # type: ignore[arg-type]
                le_saved.append({"name": v["name"], "score": v["score"], "w": v["w"], "h": v["h"], "png": str(vp)})

            # Heuristic: very low unique value count implies indexed/paint-like data.
            be_unique = len(set(be))
            le_unique = len(set(le))
            if be_unique <= 96 and le_unique <= 96:
                u16_class = "indexed_u16_paintlike"
            elif float(le_best["score"]) > 0.22 and float(be_best["score"]) < 0.10:
                u16_class = "lightmap_or_indexed_noisy_height"
            else:
                u16_class = "terrain_u16_candidate"
            u16_records.append(
                {
                    "offset": off,
                    "decompressed_size": size,
                    "raw_md5": _md5_hex(dec),
                    "light_be_png": str(be_png),
                    "height_le_png": str(le_png),
                    "score_be": float(be_best["score"]),
                    "score_le": float(le_best["score"]),
                    "light_variant": be_best["name"],
                    "height_variant": le_best["name"],
                    "light_dims": [int(be_best["w"]), int(be_best["h"])],
                    "height_dims": [int(le_best["w"]), int(le_best["h"])],
                    "light_variants": be_saved,
                    "height_variants": le_saved,
                    "be_unique_values": be_unique,
                    "le_unique_values": le_unique,
                    "u16_class": u16_class,
                    "paint_like_u16": bool(u16_class != "terrain_u16_candidate"),
                }
            )
            continue

        if size == 8192:
            variants = _ci4_variant_images(dec)
            best = variants[0]
            p = out_dir / f"{off:08X}_paint_{best['name']}.png"
            _write_l8(p, 128, 128, best["pixels"])  # type: ignore[arg-type]
            variant_files: List[Dict[str, object]] = []
            for v in variants[:topk]:
                vp = out_dir / f"{off:08X}_paint_{v['name']}.png"
                if str(vp) != str(p):
                    _write_l8(vp, 128, 128, v["pixels"])  # type: ignore[arg-type]
                variant_files.append({"name": v["name"], "score": v["score"], "png": str(vp)})
            paint_records.append(
                {
                    "offset": off,
                    "kind": "ci4_128x128",
                    "raw_md5": _md5_hex(dec),
                    "paint_png": str(p),
                    "paint_variant": best["name"],
                    "paint_score": best["score"],
                    "paint_variants": variant_files,
                }
            )
            continue

        if size == 24576:
            for part in range(3):
                chunk = dec[part * 8192 : (part + 1) * 8192]
                variants = _ci4_variant_images(chunk)
                best = variants[0]
                p = out_dir / f"{off:08X}_part{part}_paint_{best['name']}.png"
                _write_l8(p, 128, 128, best["pixels"])  # type: ignore[arg-type]
                variant_files: List[Dict[str, object]] = []
                for v in variants[:topk]:
                    vp = out_dir / f"{off:08X}_part{part}_paint_{v['name']}.png"
                    if str(vp) != str(p):
                        _write_l8(vp, 128, 128, v["pixels"])  # type: ignore[arg-type]
                    variant_files.append({"name": v["name"], "score": v["score"], "png": str(vp)})
                paint_records.append(
                    {
                        "offset": off,
                        "subpart": part,
                        "approx_offset": off + (part * 8192),
                        "kind": "ci4_3x128x128_part",
                        "raw_md5": _md5_hex(chunk),
                        "paint_png": str(p),
                        "paint_variant": best["name"],
                        "paint_score": best["score"],
                        "paint_variants": variant_files,
                    }
                )
            continue

        other_records.append({"offset": off, "decompressed_size": size})

    # Pair each u16 chunk with nearest-next paint chunk by offset (non-exclusive).
    paints_sorted = sorted(
        paint_records,
        key=lambda r: int(r.get("approx_offset", r.get("offset", 0))),  # type: ignore[arg-type]
    )
    pairs: List[Dict[str, object]] = []
    for u in sorted(u16_records, key=lambda r: int(r["offset"])):
        uoff = int(u["offset"])
        nxt = [p for p in paints_sorted if int(p.get("approx_offset", p.get("offset", 0))) >= uoff]
        if nxt:
            p = nxt[0]
            pairs.append(
                {
                    "u16_offset": uoff,
                    "paint_offset": int(p.get("offset", 0)),
                    "paint_subpart": p.get("subpart"),
                    "light_be_png": u["light_be_png"],
                    "height_le_png": u["height_le_png"],
                    "paint_png": p["paint_png"],
                }
            )

    # Also provide nearest paint candidates for every u16 chunk (non-exclusive).
    nearest_pairs: List[Dict[str, object]] = []
    for u in sorted(u16_records, key=lambda r: int(r["offset"])):
        uoff = int(u["offset"])
        if not paints_sorted:
            break
        ranked = sorted(
            paints_sorted,
            key=lambda p: abs(int(p.get("approx_offset", p.get("offset", 0))) - uoff),
        )
        top = ranked[:3]
        nearest_pairs.append(
            {
                "u16_offset": uoff,
                "light_be_png": u["light_be_png"],
                "height_le_png": u["height_le_png"],
                "paint_candidates": [
                    {
                        "paint_offset": int(p.get("offset", 0)),
                        "paint_subpart": p.get("subpart"),
                        "paint_png": p["paint_png"],
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


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Battlezone 64 extraction helper")
    sub = p.add_subparsers(dest="cmd", required=True)

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
    py.set_defaults(func=cmd_batch_models_yay0)

    pz = sub.add_parser("extract-bzn", help="Extract decompressed .bzn chunks from Yay0 streams")
    pz.add_argument("--rom", required=True, help="Path to .z64 ROM")
    pz.add_argument("--outdir", required=True, help="Output folder for .bzn files")
    pz.set_defaults(func=cmd_extract_bzn)

    pr = sub.add_parser("bzn-refs", help="Extract stable reference fields from extracted .bzn containers")
    pr.add_argument("--indir", required=True, help="Directory containing extracted .bzn files")
    pr.add_argument("--out", required=True, help="Output JSON path")
    pr.set_defaults(func=cmd_bzn_refs)

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
        default="128x128",
        help="Comma-separated u16 layout dimensions to evaluate for 32768-byte chunks",
    )
    pa.add_argument("--u16-variants-top", type=int, default=3, help="Number of best u16 variants to save per endianness (default: 3)")
    pa.add_argument("--paint-variants-top", type=int, default=3, help="Number of best paint decode variants to save per paint chunk (default: 3)")
    pa.set_defaults(func=cmd_terrain_all)

    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
