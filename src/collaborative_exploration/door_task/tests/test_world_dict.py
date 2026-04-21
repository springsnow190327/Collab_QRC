from door_task.perception.world_dict import WorldDict


def test_observe_creates_entry():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    e = wd.observe((1.0, 2.0), "red", "fire hydrant", (200, 30, 30), now=0.0)
    assert e.entry_id == 1
    assert e.color_label == "red"
    snap = wd.snapshot(now=0.0)
    assert len(snap["entries"]) == 1


def test_nearby_observation_merges():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    wd.observe((1.0, 2.0), "red", "x", (200, 0, 0), now=0.0)
    wd.observe((1.1, 2.05), "red", "x", (200, 0, 0), now=0.5)
    snap = wd.snapshot(now=0.5)
    assert len(snap["entries"]) == 1
    assert snap["entries"][0]["hits"] == 2


def test_far_observation_creates_new_entry():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    wd.observe((1.0, 2.0), "red", "x", (200, 0, 0), now=0.0)
    wd.observe((5.0, 5.0), "red", "x", (200, 0, 0), now=0.1)
    snap = wd.snapshot(now=0.1)
    assert len(snap["entries"]) == 2


def test_decay_drops_old_entries():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=2.0)
    wd.observe((1.0, 2.0), "red", "x", (200, 0, 0), now=0.0)
    snap = wd.snapshot(now=10.0)
    assert len(snap["entries"]) == 0


def test_query_by_color_returns_best_confidence():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    # "noise" red, 1 hit
    wd.observe((1.0, 1.0), "red", "x", (200, 0, 0), now=0.0)
    # "real" red, 5 hits at the same spot
    for _ in range(5):
        wd.observe((5.0, 5.0), "red", "x", (200, 0, 0), now=0.5)
    best = wd.query_by_color("red", now=0.6)
    assert best is not None
    assert best.world_xy[0] > 4.0  # the high-hit cluster wins
