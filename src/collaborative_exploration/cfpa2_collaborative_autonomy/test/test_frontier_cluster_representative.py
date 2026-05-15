from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


def test_cluster_representative_is_a_raw_clearance_checked_frontier():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    raw_frontiers = [(0.0, 0.6), (0.0, -0.6)]

    reps = node._cluster_representatives(raw_frontiers, 2.0)

    assert len(reps) == 1
    assert reps[0] in raw_frontiers
