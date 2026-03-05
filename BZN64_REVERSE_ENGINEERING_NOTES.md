# BZN64 Reverse-Engineering Notes (2026-03-05)

This document consolidates findings from:
- This repo's extraction/analyzer tooling (`tools/bz64_extract.py`)
- Local extracted N64 BZN files in `bzn/`
- Cross-validation against `Nielk1/BZNTools` parser logic

## 1) Confirmed N64 BZN Binary Grammar

From `BZNTools` (`BZNStreamReader` + `BZNFileBattlezone` + `EntityDescriptor`), N64 BZN files are:
- Big-endian
- Start in binary mode (`StartBinary == true`)
- No type byte in token headers (`TypeSize = 0`)
- 2-byte size prefix (`SizeSize = 2`)
- 2-byte alignment

Token record layout:
1. `u16 size_be`
2. `size` bytes payload
3. optional 1-byte pad if `size` is odd

So this is effectively a typed stream *without explicit type tags*; type is inferred from parser context.

## 2) Confirmed High-Level File Layout

For N64 (`BattlezoneN64` mode), the leading structure is consistently:
1. `seq_count` (payload size 4, `u32`)
2. `missionSave` (payload size 1, bool-ish)
3. `TerrainName` (payload size 100, C-string + zero padding)
4. `size` (payload size 4, object count)

Then `size` Entity records follow.

### Entity Descriptor (N64)
The fixed entity preamble (before class-specific payload) is:
1. `PrjID` (`u16` enum ID)
2. `seqno` (`u16`)
3. `pos` (`vec3f`, 12 bytes)
4. `team` (`u32`)
5. `label` (`u16`, not text on N64)
6. `isUser` (`u32` expected 0/1)
7. `obj_addr` (`u32` pointer-like)
8. `transform` (`MAT3DOLD`, 48 bytes / 12 floats)

This matches direct hex inspection of:
- `bzn/dn64dm01__00A9CFB4.bzn`
- `bzn/dn64dm07__00AA1030.bzn`
- `bzn/dmisn00__00AA7518.bzn`

## 3) Local Dataset Parse Results (BZNTools-backed)

Generated artifact:
- `extract_out/bzn_analysis_20260305/bzntools_parse_summary_with_hints.json`

Summary:
- Files parsed: 70/70
- Format: all `BattlezoneN64`
- Parse failures: 0
- Total entities parsed: 1616

Top PrjID names observed:
- `apammo` (287)
- `aprepa` (228)
- `svfigh` (103)
- `nparr` (69)
- `svturr` (67)
- `svtank` (59)
- `eggeizr1` (56)

Top resolved class labels observed:
- `wingman` (477)
- `ammopack` (287)
- `repairkit` (228)
- `turrettank` (125)
- `scrap` (63)
- `geyser` (56)

Notes:
- Some records still resolve to ambiguous class sets (MultiClass-style outcomes).
- `Mission` string is not currently trustworthy in N64 mode in `BZNTools` (`BZNFileBattlezone` stores `tok.GetString()` while logging a synthetic `BZn64Mission_xxxx`).

## 4) Repo-Native BZN Analysis Outputs

Generated artifacts:
- `extract_out/bzn_analysis_20260305/bzn_refs.json`
- `extract_out/bzn_analysis_20260305/mission_clues.json`
- `extract_out/bzn_analysis_20260305/bzn_id_census_u16be.json`
- `extract_out/bzn_analysis_20260305/bzn_id_census_u32be.json`
- `extract_out/bzn_analysis_20260305/bzn_id_map_template.csv`

Key signals:
- `bzn_refs`: 70 files grouped into 46 reference tuples.
- Largest shared tuple: `298:2:1:12` (14 files).
- Priority-set clue scan (`mission-clues`) selected 35 files and found:
  - `files_with_pc_token_candidates = 35`
  - `files_with_float_runs = 35`

## 5) N64DM Map Snapshot (first entity)

From `bzntools_parse_summary_with_hints.json`:
- `dn64dm01`: `prjid=player`, `class=wingman`, pos `(4339.24, 77.44, 97779.3)`
- `dn64dm02`: `prjid=player`, `class=wingman`, pos `(1845.54, 98.88, 100507.0)`
- `dn64dm03`: `prjid=avtank`, `class=wingman`, pos `(1936.77, 19.67, 99066.8)`
- `dn64dm04`: `prjid=avtank`, `class=wingman`, pos `(3030.71, 31.87, 99206.6)`
- `dn64dm05`: `prjid=avtank`, `class=wingman`, pos `(2205.43, 1.0, 99891.2)`
- `dn64dm06`: `prjid=avtank`, `class=wingman`, pos `(2603.9, 6.0, 99471.5)`
- `dn64dm07`: `prjid=avtank`, `class=wingman`, pos `(1672.88, 180.916, 99332.9)`

## 6) Immediate Next Reverse-Engineering Targets

1. Build a stable N64 entity serializer/deserializer around the confirmed fixed preamble and token grammar.
2. Resolve ambiguous class parses by adding stronger N64-specific PrjID->Class hints (derived from mission context and known ODF/class pairings).
3. Correlate `ref_a/ref_b/ref_c/ref_d` groups (`bzn_refs.json`) with terrain chunk offsets and map packages.
4. Patch `BZNTools` N64 tail mission handling so mission IDs are preserved as numeric fields rather than `GetString()`.
5. Export a per-entity table (`file`, `seqno`, `prjid`, `class`, `team`, `pos`, `obj_addr`) for map-level diffs.

## 7) Repro Commands Used

```powershell
python tools\bz64_extract.py bzn-refs --indir bzn --out extract_out\bzn_analysis_20260305\bzn_refs.json

python tools\bz64_extract.py mission-clues --indir bzn --out extract_out\bzn_analysis_20260305\mission_clues.json --focus-priority-set --top-values 80 --max-strings 1200 --max-pc-token-runs 10 --min-pc-token-fields 6

python tools\bz64_extract.py bzn-id-census --indir bzn --out extract_out\bzn_analysis_20260305\bzn_id_census_u16be.json --scan-type u16 --byte-order be --alignment 2 --skip-bytes 0 --min-value 0 --max-value 65535 --min-global-count 8 --min-file-count 4 --top-values 300 --per-file-top 40 --focus-priority-set --stock-odf-list stockodfs.txt --id-map-csv extract_out\bzn_analysis_20260305\bzn_id_map_template.csv

python tools\bz64_extract.py bzn-id-census --indir bzn --out extract_out\bzn_analysis_20260305\bzn_id_census_u32be.json --scan-type u32 --byte-order be --alignment 4 --skip-bytes 0 --min-value 0 --max-value 4294967295 --min-global-count 4 --min-file-count 3 --top-values 250 --per-file-top 30 --focus-priority-set --id-map-csv extract_out\bzn_analysis_20260305\bzn_id_map_template_u32.csv
```

Parser cross-check command (from `BZNTools` context):

```powershell
dotnet run -c Release --project "c:\Users\istuart\Documents\GIT\Battlezone_Z64Tools\.tmp_bzn_probe_20260305" -- "c:\Users\istuart\Documents\GIT\Battlezone_Z64Tools\bzn" "c:\Users\istuart\Documents\GIT\Battlezone_Z64Tools\extract_out\bzn_analysis_20260305\bzntools_parse_summary_with_hints.json"
```
