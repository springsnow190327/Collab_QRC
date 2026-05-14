# Region Classifier Pilot — Verdict

**Source**: `results/results-phase1-v3_defs_examples-20260506-175454.json`  
**Generated**: 2026-05-06T17:55:09  
**Vocabulary**: office, kitchen, meeting_room, bedroom, bathroom, corridor, storage, unknown  
**Thresholds**: obvious≥80%, ambiguous≥60%, empty≥60%, bias≤35%, off_vocab≤5%

## Summary

| Model | Obvious | Ambiguous | Empty | Bias | Off-vocab | Lat p95 | Verdict |
|---|---|---|---|---|---|---|---|
| gemma-4-E2B | 22/30 (73%) | 8/10 (80%) | 10/10 (100%) | office 46% | 0% | 0.03s | **NO-GO** |

## gemma-4-E2B

### Gates
- `obvious_acc` = 0.733 (threshold 0.800) — **FAIL**
- `ambiguous_acc` = 0.800 (threshold 0.600) — **PASS**
- `empty_acc` = 1.000 (threshold 0.600) — **PASS**
- `no_systematic_bias` = 0.460 (threshold 0.350) — **FAIL**
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
- `A05` (ambiguous) gt=['meeting_room', 'kitchen'] raw='office' parsed=office
- `A07` (ambiguous) gt=['office', 'bathroom', 'unknown'] raw='kitchen' parsed=kitchen

### Confusion (parsed label -> count)
- `office`: 23
- `kitchen`: 13
- `corridor`: 7
- `meeting_room`: 3
- `unknown`: 2
- `bathroom`: 1
- `storage`: 1

