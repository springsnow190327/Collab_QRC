# Region Classifier Pilot — Verdict

**Source**: `results/results-phase1-v1_defs-20260506-175452.json`  
**Generated**: 2026-05-06T17:54:59  
**Vocabulary**: office, kitchen, meeting_room, bedroom, bathroom, corridor, storage, unknown  
**Thresholds**: obvious≥80%, ambiguous≥60%, empty≥60%, bias≤35%, off_vocab≤5%

## Summary

| Model | Obvious | Ambiguous | Empty | Bias | Off-vocab | Lat p95 | Verdict |
|---|---|---|---|---|---|---|---|
| gemma-4-E2B | 21/30 (70%) | 9/10 (90%) | 10/10 (100%) | office 50% | 0% | 0.03s | **NO-GO** |

## gemma-4-E2B

### Gates
- `obvious_acc` = 0.700 (threshold 0.800) — **FAIL**
- `ambiguous_acc` = 0.900 (threshold 0.600) — **PASS**
- `empty_acc` = 1.000 (threshold 0.600) — **PASS**
- `no_systematic_bias` = 0.500 (threshold 0.350) — **FAIL**
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
- `A05` (ambiguous) gt=['meeting_room', 'kitchen'] raw='office' parsed=office

### Confusion (parsed label -> count)
- `office`: 25
- `kitchen`: 12
- `corridor`: 7
- `meeting_room`: 3
- `unknown`: 2
- `storage`: 1

