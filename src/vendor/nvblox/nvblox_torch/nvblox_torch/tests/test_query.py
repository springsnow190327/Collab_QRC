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

from typing import List

import torch
import math
from nvblox_torch.scene import Scene
from nvblox_torch.mapper import Mapper, QueryType
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType

from .helpers import scene_utils, tsdf_test_helpers

# Mapper params
VOXEL_SIZE_M = 0.05
TRUNCATION_DISTANCE_VOX = 4.0
TRUNCATION_DISTANCE_M = TRUNCATION_DISTANCE_VOX * VOXEL_SIZE_M

# Scene params
SPHERE_RADIUS = 1.0
AABB_SIZE = 2.0
CENTER_0 = [0.0, 0.0, 0.0]
CENTER_1 = [1.0, 0.0, 0.0]


def get_two_sphere_scene(integrator_type: ProjectiveIntegratorType) -> Mapper:
    # The edge of the AABB +/-
    box_half_length = AABB_SIZE / 2

    # Create a scene containing a sphere at [0,0,0]
    scene_0 = Scene()
    scene_0.set_aabb([-box_half_length, -box_half_length, -box_half_length],
                     [box_half_length, box_half_length, box_half_length])
    scene_0.add_primitive('sphere', CENTER_0 + [SPHERE_RADIUS])

    # Create a scene containing a sphere at [1,0,0]
    scene_1 = Scene()
    scene_1.set_aabb([-box_half_length, -box_half_length, -box_half_length],
                     [box_half_length, box_half_length, box_half_length])
    scene_1.add_primitive('sphere', CENTER_1 + [SPHERE_RADIUS])

    # 2 x Mappers
    mapper = scene_0.to_mapper([VOXEL_SIZE_M, VOXEL_SIZE_M],
                               integrator_types=[integrator_type, integrator_type])
    scene_0.append_to_mapper(mapper, mapper_id=0)
    scene_1.append_to_mapper(mapper, mapper_id=1)

    assert mapper.num_mappers() == 2
    return mapper


def get_tsdf_from_both_spheres(points: torch.Tensor, centers: List[List[float]],
                               sphere_radius: float) -> torch.Tensor:
    assert len(centers) == 2
    distances_0 = scene_utils.get_distances_from_sphere(points, centers[0], sphere_radius)
    distances_1 = scene_utils.get_distances_from_sphere(points, centers[1], sphere_radius)
    distances = torch.min(distances_0, distances_1)
    truncated_distances = tsdf_test_helpers.truncate_distances(distances, TRUNCATION_DISTANCE_M)
    return truncated_distances


def test_tsdf_query_single_map() -> None:
    # Load scene
    mapper = get_two_sphere_scene(ProjectiveIntegratorType.TSDF)
    # Get some test points.
    num_points = 1000
    points = scene_utils.get_random_points_in_box(num_samples=num_points,
                                                  box_size_length_m=AABB_SIZE)

    # Query the TSDF
    output_tensor = torch.zeros((num_points, 2), device='cuda')
    tsdf_values = mapper.query_layer(QueryType.TSDF, points, output_tensor, mapper_id=0)
    distances = tsdf_values[:, 0]

    # Get the expected (untruncated) values
    expected_distances = scene_utils.get_distances_from_sphere(points, CENTER_0, SPHERE_RADIUS)

    # Truncate
    truncated_expected_distances = tsdf_test_helpers.truncate_distances(
        expected_distances, TRUNCATION_DISTANCE_M)

    # Test
    error = distances - truncated_expected_distances
    max_error = torch.max(error).item()
    print(f'The maximum observed error in scene 0 was: {max_error}')
    assert torch.all(torch.abs(error) < VOXEL_SIZE_M)


def test_tsdf_query_multi_map() -> None:
    # Load scene
    mapper = get_two_sphere_scene(ProjectiveIntegratorType.TSDF)

    # Get some test points.
    num_points = 1000
    points = scene_utils.get_random_points_in_box(num_samples=num_points,
                                                  box_size_length_m=AABB_SIZE)

    # Query both scenes
    output_tensor = torch.zeros((num_points, 2), device='cuda')
    tsdf_values = mapper.query_layer(QueryType.TSDF, points, output_tensor, mapper_id=-1)
    distances = tsdf_values[:, 0]

    # Expected distances
    truncated_expected_distances = get_tsdf_from_both_spheres(points, [CENTER_0, CENTER_1],
                                                              SPHERE_RADIUS)

    # Test
    error = distances - truncated_expected_distances
    max_error = torch.max(error).item()
    print(f'The maximum observed error in scene 0+1 was: {max_error}')
    assert torch.all(torch.abs(error) < VOXEL_SIZE_M)


def test_occupancy_query_multi_map() -> None:
    # Make test deterministic
    torch.manual_seed(1)

    mapper = get_two_sphere_scene(ProjectiveIntegratorType.OCCUPANCY)

    # Get some test points.
    num_points = 1000
    points = scene_utils.get_random_points_in_box(num_samples=num_points,
                                                  box_size_length_m=AABB_SIZE)

    # Query the occupancy
    output_tensor = torch.zeros((num_points, 1), device='cuda')
    occupancy_values = mapper.query_layer(QueryType.OCCUPANCY, points, output_tensor, mapper_id=-1)

    assert not torch.all(occupancy_values == 0.0)

    # Expected occupanies
    truncated_expected_distances = get_tsdf_from_both_spheres(points, [CENTER_0, CENTER_1],
                                                              SPHERE_RADIUS)

    # Occupancy threshold corresponds to the voxels generatedby the Scene class
    distance_thresh = 0.5 * VOXEL_SIZE_M * math.sqrt(3)

    num_inside_gt = torch.sum(truncated_expected_distances < distance_thresh)
    num_inside = torch.sum(occupancy_values > 0.0)
    assert abs(num_inside - num_inside_gt) / num_inside_gt < 0.03

    num_outside_gt = torch.sum(truncated_expected_distances > distance_thresh)
    num_outside = torch.sum(occupancy_values < 0.0)
    assert abs(num_outside - num_outside_gt) / num_outside_gt < 0.03
