# Agent Quickstart (Battlezone 64 / Z64Tools)

Fast onboarding for new agents working this repo.

## Workspace
- Repo: `c:\Users\istuart\Documents\GIT\Z64Tools`
- Main script: `tools\bz64_extract.py`
- ROM used: `Battlezone - Rise of the Black Dogs (USA).z64`

## 0) Preflight
```powershell
python -m py_compile tools\bz64_extract.py
```

## 1) Model-First Run (Recommended First)
Known sample export:
```powershell
python tools\bz64_extract.py export-model --input A1E6E4.bin --out extract_out\A1E6E4_tex --texture
```
Batch from Yay0:
```powershell
python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0 --texture --all-signatures
```
Broad recovery pass (captures many small meshes):
```powershell
python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0_focus_v2 --texture --all-signatures --min-tris 1
```
Blender-ready pass (recommended for viewing/import):
```powershell
python tools\bz64_extract.py batch-models-yay0 --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_yay0_blender_v7 --texture --min-tris 24 --strict-mesh --standalone-only
```
Raw ROM signature sweep:
```powershell
python tools\bz64_extract.py batch-models --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\models_romscan_v1 --texture
```
Expected outputs:
- `extract_out\A1E6E4_tex.obj`
- `extract_out\A1E6E4_tex.png`
- `extract_out\models_yay0\manifest.json`
- `extract_out\models_yay0_focus_v2\manifest.json`
- `extract_out\models_yay0_blender_v7\manifest.json`
- `extract_out\models_romscan_v1\manifest.json`

## 1.1) Latest Model Findings (2026-02-24)
- `y0515_00B22068` is a key hardening reference chunk:
  - `m00 @ 0x10`, `m01 @ 0x1A0`, `m02 @ 0x2B0`
  - all three should force `tri_div2` for best face recovery.
- CI4-vs-I/IA decode ambiguity is real for this title:
  - some CI4 states decode as noisy/mostly-transparent junk;
  - stronger alpha-hole penalties now bias decode selection toward usable opaque outputs.
- `map_d` emission behavior:
  - MTL now writes `map_d` only when PNG alpha actually exists (`A.min < 255`);
  - this removed major false transparency artifacts in Blender.
- Known model-specific UV/material fix is now automated:
  - for `y0515_00B22068_m00_000010`, right-side triangles are remapped/mirrored to the matching left-side texture family;
  - this fixes the non-symmetric right wing artifact seen in Blender.
- Merged OBJ/MTL per Yay0 chunk is supported and recommended for inspection:
  - use `--merge-chunk-models` on `batch-models-yay0`.

Reference outputs from the validated run:
- `extract_out\models_y0515_harden_probe_v7_autofix\y0515_00B22068_merged.obj`
- `extract_out\models_y0515_harden_probe_v7_autofix\y0515_00B22068_merged.mtl`
- `extract_out\models_y0515_harden_probe_v7_autofix\manifest.json`

### 2.1) Validated clean extraction for `0x949B3C` / `0x951BE4` (2026-02-24)
Use this method for one clean light/height/paint set:
```powershell
python tools\bz64_extract.py terrain-all --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\terrain_probe_949B3C_951BE4_match_user --start 0x949B3C --end 0x951BE4 --u16-variants-top 2 --paint-variants-top 5 --paint-denoise-passes 1 --paint-denoise-radius 1 --prefer-paint-variant "0x951BE4:rowxor1_hilo_linear(_256x64)?$"
```

For the legacy-adaptive paint style that matches `tools\ADAPTIVE_64_asset_00951BE4.bin.png`:
```powershell
python tools\bz64_extract.py terrain-all --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\terrain_probe_949B3C_951BE4_adaptive64 --start 0x949B3C --end 0x951BE4 --u16-variants-top 2 --paint-variants-top 8
```
`0x951BE4` now defaults to `behi64_nearest` unless explicitly overridden with `--prefer-paint-variant`.

Expected best files from this run:
- `light`: `extract_out\terrain_probe_949B3C_951BE4_match_user\00949B3C_be_linear_128x128.png`
- `height`: `extract_out\terrain_probe_949B3C_951BE4_match_user\00949B3C_height_best_le_linear_128x128.png`
- `paint` (overlay/display-ready): `extract_out\terrain_probe_949B3C_951BE4_match_user\00951BE4_paint_clean_square_rowxor1_hilo_linear_256x64_r1_p1_128x128.png`

Single-best export bundle:
- `extract_out\terrain_probe_949B3C_951BE4_single_best\light_00949B3C_128x128.png`
- `extract_out\terrain_probe_949B3C_951BE4_single_best\height_00949B3C_128x128.png`
- `extract_out\terrain_probe_949B3C_951BE4_single_best\paint_00951BE4_128x128.png`

## 3) High-Value Focus Range (N64 exclusive terrain cluster)
```powershell
python tools\bz64_extract.py terrain-cluster --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\terrain_cluster_92_97_plus --start 0x921994 --end 0x97056A
```

## 4) Height/Light Validation Pass (known good offsets)
```powershell
python tools\bz64_extract.py height-candidates --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\height_candidates_92_97_labeled --start 0x928128 --end 0x97056A --valid-light-be-offset 0x928128 --valid-light-be-offset 0x938B84 --valid-light-be-offset 0x95BDE4 --valid-light-be-offset 0x96CE6C --valid-height-le-offset 0x928128 --valid-height-le-offset 0x96CE6C
```
Expected output:
- `extract_out\height_candidates_92_97_labeled\manifest.json`

## 5) BZN Containers / References
Extract BZN chunks:
```powershell
python tools\bz64_extract.py extract-bzn --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\bzn
```
Build ref index:
```powershell
python tools\bz64_extract.py bzn-refs --indir extract_out\bzn --out extract_out\bzn_refs.json
```

## Known Ground Truth
- `00938B84 be_linear` = valid lightmap
- `0095BDE4 be_linear` = valid lightmap
- `00928128 be_linear` = lightmap, `le_linear` = matching heightmap
- `0096CE6C be_linear` = lightmap, `le_linear` = matching heightmap

## Notes for Agents
- Current active focus is model extraction and model texture correctness.
- Terrain extraction now has a validated single-best workflow for specific offset pairs (see section 2.1).
- Model format behavior and current assumptions are documented in `MODEL_FORMAT_NOTES.md`.
- Texture decode path is DL-driven for CI4 (active palette/image + tile size), with fixed-offset behavior only as fallback.
- Models often bind many small tiled textures; OBJ export now supports multi-material texture emission and chunk-level merged OBJ output.
- Do not assume every `32768` chunk is a true terrain height/light pair.
- Use manifest data (not filename guesses) for pipeline decisions.
- Keep outputs versioned (`terrain_all_tuned_v2`, etc.) to preserve comparisons.
- Full background is in `README.md`.

## Priority List For Extraction: 
1. Models with correct UVs, materials, textures
2. Terrain height maps, light maps, and paint maps
3. BZN reconstruction for object placement, paths, etc on the N64 mission files
4. All sprites or random textures not used on Models
5. All audio, USF or MIDI files for music
6. [Unlikely - Requires ASM] Original mission scripts
