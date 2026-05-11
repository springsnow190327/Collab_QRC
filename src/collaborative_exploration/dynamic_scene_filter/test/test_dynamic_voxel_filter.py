from dynamic_scene_filter.temporal_voxel_filter import (
    DynamicFilterParams,
    TemporalVoxelFilter,
)


def test_repeated_stationary_voxel_becomes_static() -> None:
    filt = TemporalVoxelFilter(
        DynamicFilterParams(
            voxel_size=0.5,
            static_min_observations=3,
            static_min_lifetime_sec=1.0,
            max_static_velocity=0.15,
            min_dynamic_velocity=0.35,
        )
    )

    labels = []
    for i, t in enumerate((0.0, 0.6, 1.2)):
        result = filt.classify_points([(1.0, 0.0, 0.2)], stamp_sec=t)
        labels.append(result.labels[0])

    assert labels[-1] == "static"
    assert result.static_points == [(1.0, 0.0, 0.2)]
    assert result.dynamic_points == []


def test_fast_moving_voxel_is_dynamic_and_decays() -> None:
    filt = TemporalVoxelFilter(
        DynamicFilterParams(
            voxel_size=0.25,
            static_min_observations=3,
            static_min_lifetime_sec=2.0,
            dynamic_obstacle_ttl_sec=1.0,
            max_static_velocity=0.15,
            min_dynamic_velocity=0.35,
        )
    )

    filt.classify_points([(0.0, 0.0, 0.1)], stamp_sec=0.0)
    result = filt.classify_points([(1.0, 0.0, 0.1)], stamp_sec=0.5)

    assert result.labels[0] == "dynamic"
    assert result.dynamic_points == [(1.0, 0.0, 0.1)]

    filt.prune(stamp_sec=2.0)
    assert filt.dynamic_voxel_count == 0
