# Region Classifier Pilot — Verdict

**Source**: `results/results-phase1-gemma-20260506-175330.json`  
**Generated**: 2026-05-06T17:53:53  
**Vocabulary**: office, kitchen, meeting_room, bedroom, bathroom, corridor, storage, unknown  
**Thresholds**: obvious≥80%, ambiguous≥60%, empty≥60%, bias≤35%, off_vocab≤5%

## Summary

| Model | Obvious | Ambiguous | Empty | Bias | Off-vocab | Lat p95 | Verdict |
|---|---|---|---|---|---|---|---|
| gemma-4-E2B | 20/30 (67%) | 8/10 (80%) | 5/10 (50%) | office 58% | 0% | 0.02s | **NO-GO** |

## gemma-4-E2B

### Gates
- `obvious_acc` = 0.667 (threshold 0.800) — **FAIL**
- `ambiguous_acc` = 0.800 (threshold 0.600) — **PASS**
- `empty_acc` = 0.500 (threshold 0.600) — **FAIL**
- `no_systematic_bias` = 0.580 (threshold 0.350) — **FAIL**
- `off_vocab_rate` = 0.000 (threshold 0.050) — **PASS**

### Failures
- `M01` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M02` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M03` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M04` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M05` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M06` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M07` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M08` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M09` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M10` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `A05` (ambiguous) gt=['meeting_room', 'kitchen'] raw='office' parsed=office
- `A07` (ambiguous) gt=['office', 'bathroom', 'unknown'] raw='kitchen' parsed=kitchen
- `E02` (empty) gt=['corridor', 'unknown'] raw='office' parsed=office
- `E03` (empty) gt=['corridor', 'unknown'] raw='office' parsed=office
- `E04` (empty) gt=['corridor', 'unknown', 'storage'] raw='office' parsed=office
- `E05` (empty) gt=['corridor', 'unknown'] raw='office' parsed=office
- `E07` (empty) gt=['corridor', 'unknown'] raw='kitchen' parsed=kitchen

### Confusion (parsed label -> count)
- `office`: 29
- `kitchen`: 15
- `unknown`: 5
- `meeting_room`: 1

