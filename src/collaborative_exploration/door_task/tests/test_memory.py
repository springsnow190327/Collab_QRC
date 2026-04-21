from door_task.core.memory import default_world_memory


def test_default_memory_shape():
    m = default_world_memory()
    assert set(m) >= {"pillar", "door", "rooms", "notes"}
    assert m["pillar"]["known"] is False
    assert m["pillar"]["world_xy"] is None
    assert m["door"]["known"] is False


def test_default_memory_is_independent():
    a = default_world_memory()
    b = default_world_memory()
    a["pillar"]["known"] = True
    assert b["pillar"]["known"] is False
