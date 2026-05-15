# Region Classifier Pilot — Verdict

**Source**: `results/results-phase1-v4_decision-20260506-175607.json`  
**Generated**: 2026-05-06T17:56:20  
**Vocabulary**: office, kitchen, meeting_room, bedroom, bathroom, corridor, storage, unknown  
**Thresholds**: obvious≥80%, ambiguous≥60%, empty≥60%, bias≤35%, off_vocab≤5%

## Summary

| Model | Obvious | Ambiguous | Empty | Bias | Off-vocab | Lat p95 | Verdict |
|---|---|---|---|---|---|---|---|
| gemma-4-E2B | 26/30 (87%) | 10/10 (100%) | 10/10 (100%) | office 34% | 0% | 0.03s | **GO** |

## gemma-4-E2B

### Gates
- `obvious_acc` = 0.867 (threshold 0.800) — **PASS**
- `ambiguous_acc` = 1.000 (threshold 0.600) — **PASS**
- `empty_acc` = 1.000 (threshold 0.600) — **PASS**
- `no_systematic_bias` = 0.340 (threshold 0.350) — **PASS**
- `off_vocab_rate` = 0.000 (threshold 0.050) — **PASS**

### Failures
- `M03` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M06` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M07` (meeting_room) gt=['meeting_room'] raw='office' parsed=office
- `M10` (meeting_room) gt=['meeting_room'] raw='office' parsed=office

### Confusion (parsed label -> count)
- `office`: 17
- `kitchen`: 12
- `meeting_room`: 9
- `corridor`: 9
- `storage`: 2
- `bathroom`: 1

