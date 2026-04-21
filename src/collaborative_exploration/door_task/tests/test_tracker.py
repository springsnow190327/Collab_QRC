from door_task.perception.detector import Detection
from door_task.perception.tracker import IouTracker, iou


def _det(x0, y0, x1, y1, conf=0.9, cls="thing"):
    return Detection(bbox_xyxy=(x0, y0, x1, y1), conf=conf, yolo_class=cls)


def test_iou_basic():
    a = (0, 0, 10, 10)
    b = (5, 5, 15, 15)
    v = iou(a, b)
    assert 0.0 < v < 1.0


def test_tracker_assigns_new_ids_for_unrelated_dets():
    t = IouTracker(iou_threshold=0.3)
    out = t.step([_det(0, 0, 10, 10), _det(100, 100, 120, 120)])
    assert len(out) == 2


def test_tracker_persists_id_across_overlapping_frames():
    t = IouTracker(iou_threshold=0.3)
    out1 = t.step([_det(0, 0, 10, 10)])
    tid1 = next(iter(out1))
    out2 = t.step([_det(1, 1, 11, 11)])
    tid2 = next(iter(out2))
    assert tid1 == tid2


def test_tracker_drops_after_misses():
    t = IouTracker(iou_threshold=0.3, max_misses=2)
    t.step([_det(0, 0, 10, 10)])
    t.step([])
    t.step([])
    t.step([])  # third miss should drop
    assert len(t.tracks) == 0
