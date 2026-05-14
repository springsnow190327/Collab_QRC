# Region Classifier Pilot — Verdict

**Source**: `results/results-phase1-v2_examples-20260506-175453.json`  
**Generated**: 2026-05-06T17:54:59  
**Vocabulary**: office, kitchen, meeting_room, bedroom, bathroom, corridor, storage, unknown  
**Thresholds**: obvious≥80%, ambiguous≥60%, empty≥60%, bias≤35%, off_vocab≤5%

## Summary

| Model | Obvious | Ambiguous | Empty | Bias | Off-vocab | Lat p95 | Verdict |
|---|---|---|---|---|---|---|---|
| gemma-4-E2B | 22/30 (73%) | 9/10 (90%) | 8/10 (80%) | office 44% | 0% | 0.02s | **NO-GO** |

## gemma-4-E2B

### Gates
- `obvious_acc` = 0.733 (threshold 0.800) — **FAIL**
- `ambiguous_acc` = 0.900 (threshold 0.600) — **PASS**
- `empty_acc` = 0.800 (threshold 0.600) — **PASS**
- `no_systematic_bias` = 0.440 (threshold 0.350) — **FAIL**
- `off_vocab_rate` = 0.000 (threshold 0.050) — **PASS**

### Failures
- `M02` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M03` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M04` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M05` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M06` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M07` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M08` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M10` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `A07` (ambiguous) gt=['office', 'bathroom', 'unknown'] raw='kitchen' parsed=kitchen
- `E02` (empty) gt=['corridor', 'unknown'] raw='bedroom' parsed=bedroom
- `E07` (empty) gt=['corridor', 'unknown'] raw='storage' parsed=storage

### Confusion (parsed label -> count)
- `office`: 22
- `kitchen`: 15
- `unknown`: 4
- `corridor`: 3
- `meeting_room`: 2
- `storage`: 2
- `bathroom`: 1
- `bedroom`: 1

