from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import CFPA2Coordinator


def test_frontier_raw_capacity_overfetches_large_elevation_maps():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.frontier_raw_overfetch_factor = 20
    node.frontier_raw_overfetch_min = 4096

    assert node._frontier_raw_capacity(300, 300, 180) == 4096


def test_frontier_raw_capacity_respects_small_maps():
    node = CFPA2Coordinator.__new__(CFPA2Coordinator)
    node.frontier_raw_overfetch_factor = 20
    node.frontier_raw_overfetch_min = 4096

    assert node._frontier_raw_capacity(40, 40, 180) == 1600
