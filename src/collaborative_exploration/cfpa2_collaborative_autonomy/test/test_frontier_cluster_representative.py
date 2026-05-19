import math

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


def test_cluster_representative_is_a_raw_clearance_checked_frontier():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    raw_frontiers = [(0.0, 0.6), (0.0, -0.6)]

    reps = node._cluster_representatives(raw_frontiers, 2.0)

    assert len(reps) == 1
    # ctypes float boundary in the C++ accelerator means the returned tuple
    # may differ from `raw_frontiers` by single-precision quantum (~1e-7).
    # Match with tolerance instead of `in`.
    rx, ry = reps[0]
    assert any(
        math.isclose(rx, f[0], abs_tol=1e-5) and math.isclose(ry, f[1], abs_tol=1e-5)
        for f in raw_frontiers
    )
