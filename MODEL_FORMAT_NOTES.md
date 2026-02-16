# Battlezone 64 Binary Model Format Notes

Last updated: 2026-02-16

This file captures the current reverse-engineered model format behavior used by `tools/bz64_extract.py`.

## Core Signature
- Model stream signature: `D700000280008000`
- Typical model blob starts at display-list offset `0x08`.

## Display List Semantics Used
- Command words are 8-byte pairs (`w0`, `w1`) in big-endian.
- Implemented opcodes:
1. `0x01` (`VTX`)
2. `0x05` (`TRI1`)
3. `0xDF` (`EndDL`)

### `VTX` decode (F3DEX2-like)
- `n = (w0 >> 12) & 0xFF`
- `end = (w0 & 0x0FFF) >> 1`
- `v0 = end - n`
- Vertex source address comes from `w1`, and segment `0x08` is currently accepted.
- Parsed vertex stride is 16 bytes:
1. `x,y,z` signed 16-bit
2. `s,t` signed 16-bit
3. `rgba` as 8-bit channels

### `TRI1` decode
- Vertex indices are taken from low bytes of `w0`, divided by 2 (`//2`) to map to cache slots.

## Vertex Cache / Mesh Build
- Parser uses an RSP-like vertex cache (64 slots for safety).
- Faces are emitted from resolved cache slots.
- Final mesh compacts to only referenced vertices.

## Current Quality Checks
- Mesh must have at least 1 triangle and 3 vertices.
- Degenerate-triangle ratio must be <= 5%.
- Bounding extents must be non-collapsed and not absurdly large.
- Optional strict mode (`--strict-mesh`) adds anti-spaghetti checks:
  - rejects meshes with excessive very-long edge ratio
  - rejects extreme axis-stretch outliers

## Texture Extraction (Current Assumption)
- Known working CI4 layout:
1. TLUT at `0x0BA0`
2. CI4 pixels at `0x0BC0`
3. Size `32x32`
- This works for many assets but not all.
- Small model blobs often fail texture extraction only because this fixed offset/layout is not present.

## Reference Ground Truth Binaries
- `9F41DA.bin`: `tri=360`, `vert=352`
- `9F347C.bin`: `tri=224`, `vert=240`
- `9F64EA.bin`: `tri=360`, `vert=391`
- Reference mapping artifact:
  - `extract_out/models_yay0_focus_v1/reference_matches.json`

## Validation Across Curated `models/` Set
- Input set: `models/*.bin` (`206` files)
- Parse success: `206/206`
- Mesh quality pass: `166/206`
- Texture decode pass (fixed CI4 assumption): `178/206`
- Report artifacts:
  - `extract_out/models_reference_check.json`
  - `extract_out/models_reference_check.md`

## ROM Extraction Results Using Current Knowledge
- Raw ROM signature extraction:
  - Command: `python tools\bz64_extract.py batch-models --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_romscan_v1 --texture`
  - Exported: `182`
- Yay0 extraction (all signatures, low tri threshold):
  - Command: `python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0_focus_v2 --texture --all-signatures --min-tris 1`
  - Exported: `299`
- Blender-ready extraction profile:
  - Command: `python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0_blender_v2 --texture --min-tris 24 --strict-mesh`
  - Exported: `44`
- Combined summary artifact:
  - `extract_out/model_extraction_summary_v2.json`

## Known Gaps
- Texture decoding is still sample-layout-driven and needs generalized layout detection.
- Some parsed blobs with very small triangle counts are likely effects/helpers, not gameplay-visible meshes.
- Some larger model-like binaries still produce weak geometry and may use variant DL conventions not yet decoded.
