# Battlezone 64 Extraction Notes (Agent Handoff)

This repository contains active reverse-engineering tooling and outputs for:
- **Battlezone - Rise of the Black Dogs (N64)**
- Goal: extract assets to modern, usable formats (OBJ/PNG/JSON)

Primary script:
- `tools/bz64_extract.py`
- Detailed model format notes:
  - `MODEL_FORMAT_NOTES.md`

Working ROM path used during analysis:
- `Battlezone - Rise of the Black Dogs (USA).z64`

## Current Priorities
1. 3D models + textures
2. Missions/containers (`.bzn`) and script-related references
3. Terrain data (heightmaps, lightmaps, paintmaps) [paused after v2 cleanup]
4. Audio (lower priority; separate tools already available)

## Quick Status
- **Yay0 decompression is implemented and stable**.
- **Model parsing is working for known format samples** (RSP vertex cache semantics fixed).
- **Model texture extraction now uses DL-driven CI4 state** (palette/image pointers + tile size inferred from commands when present).
- **Terrain extraction has a stable v2 baseline** (square-first layouts + improved paint scoring).
- **BZN containers are extracted** and reference fields are indexed.

## Tools / Commands
All commands are run from repo root.

### 1) Scan ROM for candidates
```powershell
python tools\bz64_extract.py scan --rom "Battlezone - Rise of the Black Dogs (USA).z64"
```

### 2) Export a known model blob
```powershell
python tools\bz64_extract.py export-model --input A1E6E4.bin --out extract_out\A1E6E4_tex --texture
```

### 3) Batch models from Yay0 chunks
```powershell
python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0 --texture --all-signatures
```

### 3b) Broad model recovery (low tri threshold)
```powershell
python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0_focus_v2 --texture --all-signatures --min-tris 1
```

### 3c) Blender-ready model recovery (recommended)
```powershell
python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0_blender_v2 --texture --min-tris 24 --strict-mesh
```

### 3d) Raw ROM signature extraction
```powershell
python tools\bz64_extract.py batch-models --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_romscan_v1 --texture
```

### 4) Extract `.bzn` chunks
```powershell
python tools\bz64_extract.py extract-bzn --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\bzn
```

### 5) Index stable BZN reference fields
```powershell
python tools\bz64_extract.py bzn-refs --indir extract_out\bzn --out extract_out\bzn_refs.json
```

### 6) Terrain cluster extraction (focused range)
```powershell
python tools\bz64_extract.py terrain-cluster --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\terrain_cluster_92_97_plus --start 0x921994 --end 0x97056A
```

### 7) Height candidate ranking (focused range)
```powershell
python tools\bz64_extract.py height-candidates --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\height_candidates_92_97_labeled --start 0x928128 --end 0x97056A --valid-light-be-offset 0x928128 --valid-light-be-offset 0x938B84 --valid-light-be-offset 0x95BDE4 --valid-light-be-offset 0x96CE6C --valid-height-le-offset 0x928128 --valid-height-le-offset 0x96CE6C
```

### 8) Full-ROM terrain extraction (recommended)
```powershell
python tools\bz64_extract.py terrain-all --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\terrain_all_tuned_v2 --u16-variants-top 2 --paint-variants-top 3
```

## Implemented Format Knowledge

## Yay0
- Header: `Yay0`
- Decompressor in `tools/bz64_extract.py` (`yay0_decompress`).

## Model stream (known sample)
- Signature: `D700000280008000`
- Display list parsing supports:
  - `0x01` VTX
  - `0x05` TRI1
  - `0xDF` EndDL
- Vertex cache behavior uses F3DEX2-like semantics (`end`, `n`, `v0`).

### Model reference binaries (validated)
- `9F41DA.bin` -> `extract_out/models_yay0_focus_v1/y0281_009F41DA_m00_000008.obj` (`tri=360`, `vert=352`)
- `9F347C.bin` -> `extract_out/models_yay0_focus_v1/y0280_009F347C_m00_000008.obj` (`tri=224`, `vert=240`)
- `9F64EA.bin` -> `extract_out/models_yay0_focus_v1/y0282_009F64EA_m00_000008.obj` (`tri=360`, `vert=391`)
- Mapping artifact: `extract_out/models_yay0_focus_v1/reference_matches.json`

## Terrain-like chunk families (decompressed sizes)
- `32768` bytes: `u16` grid (candidate light/height layers)
- `8192` bytes: `CI4 128x128` paint/index layer
- `24576` bytes: interpreted as **3 packed CI4 128x128** parts

## Terrain decode model (current best)
- For many validated cases:
  - `be_linear` (big-endian) behaves as lightmap-like layer
  - `le_linear` (little-endian reinterpretation) can reveal heightmap-like layer
- Not universally true across all offsets; dynamic variant scoring is used.

## Recent Terrain Improvements

### U16 (`32768`) improvements
- Default decode is now square-first (`128x128`) to avoid vertically stacked layouts.
- Optional non-square layouts are still available via `--u16-dims` when needed.
- Plus untile variants:
  - linear tile untile (`ts2/4/8/16/32`)
  - morton untile (`ts4/8/16`)
- Best-scored variant is exported as:
  - `<offset>_light_be.png`
  - `<offset>_height_le.png`

### CI4 paint improvements
- Variant search includes:
  - nibble order (`hilo`/`lohi`)
  - row deinterleave (`rowxor1/2/4/8/16/32` on odd rows)
  - untile (linear + morton)
- Artifact scoring now penalizes checkerboard/parity noise and isolated-pixel noise.
- Best variant exported as `paint_png` in manifests.
- Top-N candidates are also saved for manual validation.

## Important Validations from Manual Review
Confirmed examples:
- `00938B84 be_linear` looks like valid lightmap.
- `0095BDE4 be_linear` looks like valid lightmap.
- `00928128 be_linear` lightmap, and `00928128 le_linear` matching heightmap.
- `0096CE6C be_linear` lightmap, and `0096CE6C le_linear` matching heightmap.

## Known Caveats / Misclassification Risks
- Some `32768` chunks are **not** true terrain height+light pairs.
- Some outputs look paint/index-like even in u16 family.
- Misalignment/duplication can occur when wrong dimensions are assumed.
- `terrain-all` now records per-chunk variant metadata, but classification is still heuristic.

## Key Outputs

### Terrain (latest full run)
- `extract_out/terrain_all_tuned_v2/manifest.json`
- Contains:
  - `u16_records`
  - `paint_records`
  - `pairs`
  - `nearest_pairs`
  - `unique_u16`
  - `unique_paint`

Latest observed counts (from v2 full run):
- `yay0_headers_in_range`: 639
- `u16_32768`: 98
- `paint_chunks`: 50
- `pairs`: 98
- `nearest_pairs`: 98
- `unique_u16`: 94
- `unique_paint`: 50
- `height_dims`: `128x128` for all 98 u16 records

### BZN
- `extract_out/bzn/`
- `extract_out/bzn/manifest.json`
- `extract_out/bzn_refs.json`

### Models
- Sample success:
  - `extract_out/A1E6E4.obj`
  - `extract_out/A1E6E4_tex.obj`
  - `extract_out/A1E6E4_tex.png`
- Current model focus run:
  - `extract_out/models_yay0_focus_v1/manifest.json`
  - `extract_out/models_yay0_focus_v1/model_quality_report.md`
  - `extract_out/models_yay0_focus_v1/model_quality_report.json`
- Broad extraction runs:
  - `extract_out/models_yay0_focus_v2/manifest.json` (`299` exported)
  - `extract_out/models_yay0_blender_v2/manifest.json` (`44` exported, Blender-oriented)
  - `extract_out/models_romscan_v1/manifest.json` (`182` exported, `132` failed)
  - `extract_out/model_extraction_summary_v2.json` (combined run stats)
  - Note: `models_yay0_focus_v2` intentionally includes many low-tri/internal signatures and is not Blender-ready as-is.

## Model Texture Notes (Current)
- CI4 texture decode now prefers palette/image pairs that are active during triangle draw commands in the executed DL stream.
- Palette and image addresses are resolved from segmented pointers first, then low-24 local offsets as fallback.
- Tile width/height is inferred from `G_SETTILESIZE` when available; fallback size candidates are tried if needed.
- This improves color correctness versus fixed-offset extraction, especially where multiple `FD` texture loads exist in one model stream.
- Current OBJ export still writes one material/PNG per mesh (dominant inferred tile). Many game models use multiple tiny tiled textures across sub-meshes, so full per-material texture reconstruction is still pending.

## N64-Exclusive Map Focus
Known high-priority extracted BZN containers:
- `extract_out/bzn/dn64dm01__00A9CFB4.bzn`
- `extract_out/bzn/dn64dm02__00A9D1B4.bzn`
- `extract_out/bzn/dn64dm03__00A9DAEC.bzn`
- `extract_out/bzn/dn64dm04__00A9E428.bzn`
- `extract_out/bzn/dn64dm05__00A9EE2C.bzn`
- `extract_out/bzn/dn64dm06__00A9FF28.bzn`
- `extract_out/bzn/dn64dm07__00AA1030.bzn`

## Suggested Next Agent Tasks
1. Model extraction expansion (primary focus):
- Cluster/de-dup model exports from `batch-models-yay0`.
- Validate texture layouts beyond known sample and tag per-signature confidence.

2. Build deterministic map linkage:
- Resolve each `dn64dm*` BZN to exact terrain offset triplets (light/height/paint).

3. Export convenience layer:
- Add command to emit per-map packaged folders (`map_name/height.png`, `light.png`, `paint.png`, metadata JSON).

4. Terrain follow-up (deferred):
- Only revisit terrain decode if model/BZN linkage requires targeted fixes.

## Notes
- This repo may contain experimental outputs; prefer manifests over ad-hoc filenames.
- If rerunning extraction, keep output path versioned (e.g. `terrain_all_tuned_v2`) to preserve comparisons.
