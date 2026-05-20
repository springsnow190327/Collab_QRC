#
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#

from typing import Callable, Any

import numpy as np
import torch

from nvblox_torch.mapper import Mapper, QueryType
from nvblox_torch import indexing
from nvblox_torch.constants import constants
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
from nvblox_torch.sensor import Sensor
from .helpers.camera_utils import make_camera_intrinsics_matrix
from .helpers.scene_utils import generate_random_pose

IMAGE_HEIGHT = 240
IMAGE_WIDTH = 320
NUM_FRAMES = 10
VOXEL_SIZE_M = 0.1
MAX_DEPTH = 4.0

FEATURE_ARRAY_NUM_ELEMENTS = constants.feature_array_num_elements()


# Increased resource usage (num images and image size) in this test would typically
# lead to an out-of-memory error, but has occasionally resulted in a segfault in
# BlocksToUpdateTracker::addBlocksToUpdate. TODO(dtingdahl) investigate why this happens.
def test_add_depth_frame() -> None:
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT)

    for _ in range(NUM_FRAMES):
        camera_pose = generate_random_pose(device='cpu')
        depth_frame = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
        assert depth_frame.is_cuda
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)


def _get_weight_sum(mapper: Mapper, mapper_id: int) -> float:
    blocks = mapper.tsdf_layer_view(mapper_id=mapper_id).get_all_blocks()[0]
    weights = torch.stack([b[..., 1] for b in blocks]).flatten()
    return torch.sum(weights)


def test_decay() -> None:
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT)
    for _ in range(NUM_FRAMES):
        camera_pose = generate_random_pose(device='cpu')
        depth_frame = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)

    weigth_sum_before = _get_weight_sum(mapper, 0)
    mapper.decay()
    weigth_sum_after = _get_weight_sum(mapper, 0)

    assert weigth_sum_after < weigth_sum_before


def test_add_color_frame() -> None:
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT)

    for _ in range(NUM_FRAMES):
        camera_pose = generate_random_pose(device='cpu')
        depth_frame = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)
        color_frame = torch.randint(0,
                                    256, (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                                    dtype=torch.uint8,
                                    device='cuda')
        mapper.add_color_frame(color_frame=color_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mapper_id=0)


def test_add_feature_frame() -> None:
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT)

    for _ in range(NUM_FRAMES):
        camera_pose = generate_random_pose(device='cpu')
        depth_frame = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)
        feature_frame = torch.rand(IMAGE_HEIGHT,
                                   IMAGE_WIDTH,
                                   FEATURE_ARRAY_NUM_ELEMENTS,
                                   dtype=torch.float16,
                                   device='cuda')
        mapper.add_feature_frame(feature_frame=feature_frame,
                                 t_w_c=camera_pose,
                                 sensor=sensor,
                                 mapper_id=0)

        _, indices = mapper.feature_layer_view().get_all_blocks()
        voxel_centers_grids = indexing.get_voxel_center_grids(indices, VOXEL_SIZE_M)
        # Flatten to Nx3
        voxel_centers = torch.cat([t.view(-1, 3) for t in voxel_centers_grids], dim=0)
        features_and_weight = mapper.query_layer(QueryType.FEATURE, voxel_centers, mapper_id=0)

        # We should not get any nan values
        assert not torch.isnan(features_and_weight).any().item()


def test_wrong_floating_point_type_depth_frame() -> None:
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    camera_pose = generate_random_pose(device='cpu')
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT)
    # Try to add some bad depth frames and check we fail.
    bad_depth_frames = [
        MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda', dtype=torch.float64),
        MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, 1, device='cuda', dtype=torch.float32),
        MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cpu', dtype=torch.float32),
    # TODO(Vik) Add this back in after enabling the nan check in the mapper
    # torch.full((IMAGE_HEIGHT, IMAGE_WIDTH), float('nan'), device='cuda', dtype=torch.float32),
    ]
    for depth_frame in bad_depth_frames:
        call_expect_throw(mapper.add_depth_frame, depth_frame, camera_pose, sensor, 0)
    # Try to add some bad color frames and check we fail.
    bad_color_frames = [
        torch.randint(0, 256, (IMAGE_HEIGHT, IMAGE_WIDTH, 4), device='cuda', dtype=torch.float32),
        torch.randint(0, 256, (IMAGE_HEIGHT, IMAGE_WIDTH, 5), device='cuda', dtype=torch.uint8),
        torch.randint(0, 256, (IMAGE_HEIGHT, IMAGE_WIDTH), device='cuda', dtype=torch.uint8),
        torch.randint(0, 256, (IMAGE_HEIGHT, IMAGE_WIDTH, 4), device='cpu', dtype=torch.uint8),
    # TODO(Vik) Add this back in after enabling the nan check in the mapper
    # torch.full((IMAGE_HEIGHT, IMAGE_WIDTH, 3), float('nan'), device='cuda',
    #            dtype=torch.float32),
    ]
    for color_frame in bad_color_frames:
        call_expect_throw(mapper.add_color_frame, color_frame, camera_pose, sensor, 0)
    # Try to add some bad feature frames and check we fail.
    bad_feature_frames = [
        torch.rand(IMAGE_HEIGHT,
                   IMAGE_WIDTH,
                   FEATURE_ARRAY_NUM_ELEMENTS,
                   device='cuda',
                   dtype=torch.float32),
        torch.rand(IMAGE_HEIGHT,
                   IMAGE_WIDTH,
                   FEATURE_ARRAY_NUM_ELEMENTS + 1,
                   device='cuda',
                   dtype=torch.float16),
        torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda', dtype=torch.float16),
        torch.rand(IMAGE_HEIGHT,
                   IMAGE_WIDTH,
                   FEATURE_ARRAY_NUM_ELEMENTS,
                   device='cpu',
                   dtype=torch.float16),
    # TODO(Vik) Add this back in after enabling the nan check in the mapper
    # torch.full((IMAGE_HEIGHT, IMAGE_WIDTH, FEATURE_ARRAY_NUM_ELEMENTS),
    #            float('nan'),
    #            device='cuda',
    #            dtype=torch.float16),
    ]
    for feature_frame in bad_feature_frames:
        call_expect_throw(mapper.add_feature_frame, feature_frame, camera_pose, sensor, 0)


def test_add_depth_frame_with_distorted_camera() -> None:
    """Test depth integration with a distorted camera sensor."""
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')

    # Create distorted camera sensor with radial and tangential distortion
    radial_dist = np.array([-0.2, 0.1, -0.01, 0.0, 0.0, 0.0],
                           dtype=np.float32)    # k1, k2, k3, k4, k5, k6
    tangential_dist = np.array([0.001, -0.001], dtype=np.float32)    # p1, p2
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT, radial_dist,
                                       tangential_dist)

    # Verify sensor was created correctly
    assert sensor.modality == 'camera'
    assert sensor.width == IMAGE_WIDTH
    assert sensor.height == IMAGE_HEIGHT

    for _ in range(NUM_FRAMES):
        camera_pose = generate_random_pose(device='cpu')
        depth_frame = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
        assert depth_frame.is_cuda
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)


def test_add_color_frame_with_distorted_camera() -> None:
    """Test color integration with a distorted camera sensor."""
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])
    intrinsics = make_camera_intrinsics_matrix(h_fov=90,
                                               height=IMAGE_HEIGHT,
                                               width=IMAGE_WIDTH,
                                               device='cpu')

    # Create distorted camera sensor with only radial distortion (3 coefficients)
    radial_dist = np.array([-0.15, 0.08, -0.005, 0, 0, 0], dtype=np.float32)    # k1, k2, k3
    sensor = Sensor.from_camera_matrix(intrinsics, IMAGE_WIDTH, IMAGE_HEIGHT, radial_dist)

    for _ in range(NUM_FRAMES):
        camera_pose = generate_random_pose(device='cpu')
        depth_frame = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)
        color_frame = torch.randint(0,
                                    256, (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                                    dtype=torch.uint8,
                                    device='cuda')
        mapper.add_color_frame(color_frame=color_frame,
                               t_w_c=camera_pose,
                               sensor=sensor,
                               mapper_id=0)


def test_add_depth_frame_with_lidar() -> None:
    """Test depth integration with a lidar sensor."""
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])

    # Create a lidar sensor (spinning lidar with 16 beams, 360 degree coverage)
    num_azimuth_divisions = 1800    # 0.2 degree resolution
    num_elevation_divisions = 16    # 16 beams
    vertical_fov_rad = 0.524    # ~30 degrees
    min_valid_range_m = 0.5

    sensor = Sensor.from_lidar(num_azimuth_divisions, num_elevation_divisions, vertical_fov_rad,
                               min_valid_range_m)

    # Verify sensor was created correctly
    assert sensor.modality == 'lidar'
    assert sensor.width == num_azimuth_divisions
    assert sensor.height == num_elevation_divisions

    for _ in range(NUM_FRAMES):
        lidar_pose = generate_random_pose(device='cpu')
        # Lidar depth frame has different dimensions than camera
        depth_frame = MAX_DEPTH * torch.rand(
            num_elevation_divisions, num_azimuth_divisions, device='cuda', dtype=torch.float32)
        assert depth_frame.is_cuda
        mapper.add_depth_frame(depth_frame=depth_frame,
                               t_w_c=lidar_pose,
                               sensor=sensor,
                               mask_frame=None,
                               mapper_id=0)


def test_mixed_sensors() -> None:
    """Test using multiple different sensor types in the same mapper."""
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE_M], integrator_types=[ProjectiveIntegratorType.TSDF])

    # Create a pinhole camera sensor
    intrinsics_camera = make_camera_intrinsics_matrix(h_fov=90,
                                                      height=IMAGE_HEIGHT,
                                                      width=IMAGE_WIDTH,
                                                      device='cpu')
    camera_sensor = Sensor.from_camera_matrix(intrinsics_camera, IMAGE_WIDTH, IMAGE_HEIGHT)

    # Create a distorted camera sensor with full radial distortion
    radial_dist = np.array([-0.1, 0.05, -0.01, 0.0, 0.0, 0.0], dtype=np.float32)    # k1-k6
    distorted_camera_sensor = Sensor.from_camera_matrix(intrinsics_camera, IMAGE_WIDTH,
                                                        IMAGE_HEIGHT, radial_dist)

    # Create a lidar sensor
    lidar_sensor = Sensor.from_lidar(1800, 16, 0.524, 0.5)

    # Integrate frames from each sensor type
    pose = generate_random_pose(device='cpu')

    # Camera depth
    camera_depth = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
    mapper.add_depth_frame(camera_depth, pose, camera_sensor, mapper_id=0)

    # Distorted camera depth and color
    distorted_depth = MAX_DEPTH * torch.rand(IMAGE_HEIGHT, IMAGE_WIDTH, device='cuda')
    mapper.add_depth_frame(distorted_depth, pose, distorted_camera_sensor, mapper_id=0)

    color_frame = torch.randint(0,
                                256, (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
                                dtype=torch.uint8,
                                device='cuda')
    mapper.add_color_frame(color_frame, pose, distorted_camera_sensor, mapper_id=0)

    # Lidar depth
    lidar_depth = MAX_DEPTH * torch.rand(16, 1800, device='cuda', dtype=torch.float32)
    mapper.add_depth_frame(lidar_depth, pose, lidar_sensor, mapper_id=0)


def call_expect_throw(fn: Callable, *args: Any) -> None:
    threw_exception = False
    try:
        fn(*args)
    except AssertionError as e:
        print(f'Correctly threw: {e}')
        threw_exception = True
        pass
    assert threw_exception, "Didn't throw exception for wrong inputs"
