from door_task.perception.world_dict import WorldDict


def test_semantic_label_stored_on_observe():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    wd.observe(
        (1.0, 2.0), "red", "fire hydrant", (200, 30, 30), now=0.0,
        semantic_label="red button", semantic_conf=0.7,
    )
    snap = wd.snapshot(now=0.0)
    assert snap["entries"][0]["semantic_label"] == "red button"
    assert snap["entries"][0]["semantic_conf"] == 0.7


def test_query_by_semantic_returns_match():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    wd.observe((1.0, 2.0), "red", "x", (200, 0, 0), now=0.0,
               semantic_label="door", semantic_conf=0.55)
    wd.observe((5.0, 5.0), "red", "x", (200, 0, 0), now=0.1,
               semantic_label="red button", semantic_conf=0.72)
    hit = wd.query_by_semantic("red button", now=0.1)
    assert hit is not None
    assert hit.world_xy[0] > 4.0


def test_query_by_semantic_no_match_returns_none():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    wd.observe((1.0, 2.0), "red", "x", (200, 0, 0), now=0.0,
               semantic_label="door", semantic_conf=0.55)
    assert wd.query_by_semantic("red button", now=0.0) is None


def test_semantic_upgrade_keeps_best():
    wd = WorldDict(merge_radius_m=0.5, decay_sec=10.0)
    wd.observe((1.0, 2.0), "red", "x", (200, 0, 0), now=0.0,
               semantic_label="unknown object", semantic_conf=0.40)
    wd.observe((1.05, 2.05), "red", "x", (200, 0, 0), now=0.2,
               semantic_label="red button", semantic_conf=0.80)
    snap = wd.snapshot(now=0.2)
    assert snap["entries"][0]["semantic_label"] == "red button"
    assert snap["entries"][0]["hits"] == 2
