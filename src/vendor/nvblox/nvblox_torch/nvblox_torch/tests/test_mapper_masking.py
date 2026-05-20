#
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
from typing import Tuple

import torch

from nvblox_torch.constants import constants
from nvblox_torch.mapper import Mapper
from nvblox_torch.mesh import Mesh
from nvblox_torch.tests.helpers.camera_utils import make_camera_intrinsics_matrix
from nvblox_torch.sensor import Sensor

DEPTH = 2.0
HEIGHT = 480
WIDTH = 640
CAMERA_FOV = 90


def get_pose_and_sensor() -> Tuple[torch.Tensor, Sensor]:
    pose = torch.eye(4, device='cpu', dtype=torch.float32)
    intrinsics = make_camera_intrinsics_matrix(h_fov=CAMERA_FOV,
                                               height=HEIGHT,
                                               width=WIDTH,
                                               device='cpu')
    sensor = Sensor.from_camera_matrix(intrinsics, WIDTH, HEIGHT)
    return pose, sensor


def map_plane_with_mask(mask: torch.Tensor) -> Mapper:
    """Returns a mapper object containing a mapped plane."""
    pose, sensor = get_pose_and_sensor()
    depth_image = DEPTH * torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.float32)

    mapper = Mapper(voxel_sizes_m=[0.05])
    mapper.add_depth_frame(depth_frame=depth_image, t_w_c=pose, sensor=sensor, mask_frame=mask)
    mapper.update_color_mesh()
    return mapper


def test_mapper_depth_masking() -> None:
    # Map a scene containing a single plane with 3 different masks
    # - include all pixels
    # - include no pixels
    # - include half the pixels
    mask_include_all = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_none = torch.zeros(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_half = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_half[HEIGHT // 2:] = 0

    mapper_include_all = map_plane_with_mask(mask_include_all)
    mapper_include_none = map_plane_with_mask(mask_include_none)
    mapper_include_half = map_plane_with_mask(mask_include_half)

    mesh_include_all = mapper_include_all.get_color_mesh()
    mesh_include_none = mapper_include_none.get_color_mesh()
    mesh_include_half = mapper_include_half.get_color_mesh()

    # Check that include all generated vertices.
    assert mesh_include_all.vertices().shape[0] > 0

    # Check that include none generated no vertices.
    assert mesh_include_none.vertices().shape[0] == 0

    # Check that include half generated half the vertices of include all.
    proportion_half_vertices = mesh_include_half.vertices().shape[0] / mesh_include_all.vertices(
    ).shape[0]
    assert abs(proportion_half_vertices - 0.5) < 0.01

    # Check that we only generated vertices for the top half the image.
    vertex_y_values = mesh_include_half.vertices()[:, 1]
    assert torch.all(vertex_y_values <= 0.0)


def test_mapper_depth_no_mask() -> None:
    mask_include_all = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_all_with_none = None

    mapper_include_all = map_plane_with_mask(mask_include_all)
    mapper_include_all_with_none = map_plane_with_mask(mask_include_all_with_none)

    mesh_include_all = mapper_include_all.get_color_mesh()
    mesh_include_all_with_none = mapper_include_all_with_none.get_color_mesh()

    # Check that include all generated vertices.
    assert mesh_include_all.vertices().shape[0] > 0

    # Check that we get the same result with no mask as the allow-all mask.
    assert mesh_include_all_with_none.vertices().shape[0] == mesh_include_all.vertices().shape[0]


def add_red_frame_to_mapper(mapper: Mapper, mask: torch.Tensor) -> None:
    pose, sensor = get_pose_and_sensor()
    red_color_frame = torch.zeros(HEIGHT, WIDTH, 3, device='cuda', dtype=torch.uint8)
    red_color_frame[:, :, 0] = 255
    mapper.add_color_frame(color_frame=red_color_frame, t_w_c=pose, sensor=sensor, mask_frame=mask)
    mapper.update_color_mesh()


def add_one_feature_frame_to_mapper(mapper: Mapper, mask: torch.Tensor) -> None:
    pose, sensor = get_pose_and_sensor()
    one_feature_frame = torch.ones(HEIGHT,
                                   WIDTH,
                                   constants.feature_array_num_elements(),
                                   device='cuda',
                                   dtype=torch.float16)
    mapper.add_feature_frame(feature_frame=one_feature_frame,
                             t_w_c=pose,
                             sensor=sensor,
                             mask_frame=mask)
    mapper.update_feature_mesh()


def get_proportion_red_vertices(mesh: Mesh) -> float:
    vertex_colors = mesh.vertex_appearances()
    num_red_vertices = torch.sum(vertex_colors[:, 0] == 255)
    proportion_red_vertices = num_red_vertices / mesh.vertices().shape[0]
    return proportion_red_vertices


def test_mapper_color_masking() -> None:
    mask_include_all = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_none = torch.zeros(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_half = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_half[HEIGHT // 2:] = 0

    # No masking on the geometry to get the whole plane.
    mapper_full = map_plane_with_mask(mask=None)
    mapper_none = map_plane_with_mask(mask=None)
    mapper_half = map_plane_with_mask(mask=None)

    add_red_frame_to_mapper(mapper_full, mask=mask_include_all)
    add_red_frame_to_mapper(mapper_none, mask=mask_include_none)
    add_red_frame_to_mapper(mapper_half, mask=mask_include_half)

    mesh_full = mapper_full.get_color_mesh()
    mesh_none = mapper_none.get_color_mesh()
    mesh_half = mapper_half.get_color_mesh()

    proportion_full = get_proportion_red_vertices(mesh_full)
    proportion_none = get_proportion_red_vertices(mesh_none)
    proportion_half = get_proportion_red_vertices(mesh_half)

    # NOTE(alexmillane, 2025.05.05): The all-in mask only produces 88% red vertices,
    # because of grey vertices at the border. Probably we could get more turning down
    # the subsampling factor in raycasting, but this proves to point for the test.
    assert proportion_full > 0.85
    assert proportion_none == 0.0
    assert abs(proportion_half - 0.5) < 0.05


def get_proportion_one_vertices(mesh: Mesh) -> float:
    vertex_features = mesh.vertex_appearances()
    num_one_vertices = torch.sum(torch.all(vertex_features == 1.0, dim=1))
    proportion_one_vertices = num_one_vertices / mesh.vertices().shape[0]
    return proportion_one_vertices


def test_mapper_feature_masking() -> None:
    mask_include_all = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_none = torch.zeros(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_half = torch.ones(HEIGHT, WIDTH, device='cuda', dtype=torch.uint8)
    mask_include_half[HEIGHT // 2:] = 0

    # No masking on the geometry to get the whole plane.
    mapper_full = map_plane_with_mask(mask=None)
    mapper_none = map_plane_with_mask(mask=None)
    mapper_half = map_plane_with_mask(mask=None)

    add_one_feature_frame_to_mapper(mapper_full, mask=mask_include_all)
    add_one_feature_frame_to_mapper(mapper_none, mask=mask_include_none)
    add_one_feature_frame_to_mapper(mapper_half, mask=mask_include_half)

    mesh_full = mapper_full.get_feature_mesh()
    mesh_none = mapper_none.get_feature_mesh()
    mesh_half = mapper_half.get_feature_mesh()

    proportion_full = get_proportion_one_vertices(mesh_full)
    proportion_none = get_proportion_one_vertices(mesh_none)
    proportion_half = get_proportion_one_vertices(mesh_half)

    # NOTE(alexmillane, 2025.05.05): The all-in mask only produces 88% red vertices,
    # because of grey vertices at the border. Probably we could get more turning down
    # the subsampling factor in raycasting, but this proves to point for the test.
    assert proportion_full > 0.85
    assert proportion_none == 0.0
    assert abs(proportion_half - 0.5) < 0.05
