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

## 2) Terrain Baseline (Paused; use as reference only)
```powershell
python tools\bz64_extract.py terrain-all --rom "Battlezone - Rise of the Black Dogs (USA).z64" --outdir extract_out\terrain_all_tuned_v2 --u16-variants-top 2 --paint-variants-top 3
```
Expected key output:
- `extract_out\terrain_all_tuned_v2\manifest.json`

Current baseline counts (latest known):
- `u16_32768`: ~98
- `paint_chunks`: ~50
- `pairs`: ~98
- `unique_u16`: ~94
- `height_dims`: `128x128` across u16 records

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

N64-exclusive BZN files to prioritize:
- `extract_out\bzn\dn64dm01__00A9CFB4.bzn`
- `extract_out\bzn\dn64dm02__00A9D1B4.bzn`
- `extract_out\bzn\dn64dm03__00A9DAEC.bzn`
- `extract_out\bzn\dn64dm04__00A9E428.bzn`
- `extract_out\bzn\dn64dm05__00A9EE2C.bzn`
- `extract_out\bzn\dn64dm06__00A9FF28.bzn`
- `extract_out\bzn\dn64dm07__00AA1030.bzn`

## Known Ground Truth
- `00938B84 be_linear` = valid lightmap
- `0095BDE4 be_linear` = valid lightmap
- `00928128 be_linear` = lightmap, `le_linear` = matching heightmap
- `0096CE6C be_linear` = lightmap, `le_linear` = matching heightmap

## Notes for Agents
- Current active focus is model extraction and model texture correctness.
- Terrain work is paused at `extract_out\terrain_all_tuned_v2` unless explicitly reopened.
- Model format behavior and current assumptions are documented in `MODEL_FORMAT_NOTES.md`.
- `models_yay0_focus_v2` is a broad research dump (includes noisy helper/internal signatures); use `models_yay0_blender_v2` for Blender import first.
- Do not assume every `32768` chunk is a true terrain height/light pair.
- Use manifest data (not filename guesses) for pipeline decisions.
- Keep outputs versioned (`terrain_all_tuned_v2`, etc.) to preserve comparisons.
- Full background is in `README.md`.
