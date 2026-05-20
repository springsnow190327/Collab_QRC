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

from nvblox_torch.examples.utils.scenes import get_single_sphere_scene_mapper
from nvblox_torch.scene import Scene
from nvblox_torch.mapper import Mapper, QueryType
from nvblox_torch.mapper_params import EsdfIntegratorParams, MapperParams
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
from .helpers import scene_utils
# NOTE(alexmillane): We get larger errors in ESDF inside objects
# because we treat a thickness of voxels as sites.
# TODO(alexmillane): Right now the max site distance is hardcoded
# in py_mapper.h, and not retrievable by querying. So I've harded
# it here to be the same. Remove this hardcoding when we come up
# with a way of interacting with the parameters.
VOXEL_SIZE = 0.05
MAX_SITE_DISTANCE = 1.73
MAX_ERROR_OUTSIDE_M = 1.5 * VOXEL_SIZE
MAX_ERROR_INSIDE_M = VOXEL_SIZE * (round(MAX_SITE_DISTANCE) + 1)
GRADIENT_PASSING_INSIDE_RANGE_RATIO = 0.90
GRADIENT_ALLOWABLE_ERROR_VECTOR_NORM = VOXEL_SIZE


def points_to_4d_points(points: torch.tensor) -> torch.tensor:
    assert points.dim() == 2
    assert points.shape[1] == 3
    num_samples = points.shape[0]
    points_4d = torch.cat(
        (points, torch.zeros(num_samples, 1, device='cuda')),
        dim=1,
    )
    return points_4d


def assert_distances_in_bounds(distances: torch.Tensor, expected_distances: torch.Tensor) -> None:
    # Asserts ESDF queried distances match our expectations. These are:
    # - Query point outside surfaces: close to 1 voxel error.
    # - Inside: 1 voxel greater than max site distance.
    error = torch.abs(distances - expected_distances)
    outside_flags = expected_distances > 0.0
    inside_flags = expected_distances < 0.0
    assert torch.all(error[outside_flags] < MAX_ERROR_OUTSIDE_M)
    assert torch.all(error[inside_flags] < MAX_ERROR_INSIDE_M)


def assert_gradients_in_bounds(gradient_directions: torch.Tensor,
                               expected_gradient_directions: torch.Tensor) -> None:
    # Some queries return a zero vector. Get rid of these
    rows_with_zero_norm = torch.norm(gradient_directions, dim=1) == 0.0
    num_queried_zero = torch.sum(rows_with_zero_norm)
    ratio_with_zero_norm = num_queried_zero / gradient_directions.shape[0]
    print(f'Ratio of queries returning zero: {ratio_with_zero_norm}')
    assert ratio_with_zero_norm < 0.01
    # Calculate the error vectors
    error_vectors = torch.norm(gradient_directions[~rows_with_zero_norm] -
                               expected_gradient_directions[~rows_with_zero_norm],
                               dim=1)
    # Check that the error vectors are within the allowed bounds
    ratio_in_bounds = torch.sum(
        error_vectors < GRADIENT_ALLOWABLE_ERROR_VECTOR_NORM) / error_vectors.shape[0]
    print(f'Ratio in bounds: {ratio_in_bounds}')
    assert ratio_in_bounds > GRADIENT_PASSING_INSIDE_RANGE_RATIO


def get_distances_from_both_spheres(points: torch.Tensor, centers: List[List[float]],
                                    radius: float) -> torch.Tensor:
    assert len(centers) == 2
    true_distances_0 = scene_utils.get_distances_from_sphere(points, centers[0], radius)
    true_distances_1 = scene_utils.get_distances_from_sphere(points, centers[1], radius)
    # NOTE(alexmillane): The query_esdf kernel takes the most negative distance.
    # Note sure if this makes sense, but that's what it does right now.
    distances = torch.min(true_distances_0, true_distances_1)
    return distances


def normalize_vectors(vectors: torch.Tensor) -> torch.Tensor:
    assert vectors.ndim == 2
    assert vectors.shape[1] == 3
    return vectors / torch.norm(vectors, dim=1).unsqueeze(-1)


def test_query_esdf() -> None:

    # Make test deterministic
    torch.manual_seed(1)

    # Create a scene containing a sphere at [0,0,0]
    scene_0 = Scene()
    scene_0.set_aabb([-5.5, -5.5, -5.5], [5.5, 5.5, 5.5])
    center_0 = [0.0, 0.0, 0.0]
    radius = 1.0
    scene_0.add_primitive('sphere', center_0 + [radius])

    # Create a scene containing a sphere at [1,0,0]
    scene_1 = Scene()
    scene_1.set_aabb([-5.5, -5.5, -5.5], [5.5, 5.5, 5.5])
    center_1 = [1.0, 0.0, 0.0]
    radius = 1.0
    scene_1.add_primitive('sphere', center_1 + [radius])

    # Need to set the max ESDF distance high (5m) to match GT distances later
    esdf_integrator_params = EsdfIntegratorParams()
    esdf_integrator_params.esdf_integrator_max_distance_m = 10.0
    mapper_params = MapperParams()
    mapper_params.set_esdf_integrator_params(esdf_integrator_params)

    # 2 x Mappers
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE, VOXEL_SIZE],
                    integrator_types=[ProjectiveIntegratorType.TSDF, ProjectiveIntegratorType.TSDF],
                    mapper_parameters=mapper_params)
    scene_0.append_to_mapper(mapper, mapper_id=0)
    scene_1.append_to_mapper(mapper, mapper_id=1)
    assert mapper.num_mappers() == 2

    # Generate a bunch of points inside an AABB and query their distance
    # NOTE(alexmillane): Right now max distance is hardcoded for the Scene to 5m
    # so we generate points in a 3mx3mx3m cube.
    num_samples = 1000
    random_sample_cube_side_length = 10.0
    random_points_in_scene = scene_utils.get_random_points_in_box(num_samples,
                                                                  random_sample_cube_side_length)
    random_points_in_scene_4d = points_to_4d_points(random_points_in_scene)

    # Query the first map
    out_spheres_0 = torch.zeros_like(random_points_in_scene_4d)
    distances_0 = mapper.query_differentiable_layer(QueryType.ESDF,
                                                    random_points_in_scene_4d,
                                                    out_spheres_0,
                                                    mapper_id=0)

    # Query the second map
    out_spheres_1 = torch.zeros_like(random_points_in_scene_4d)
    distances_1 = mapper.query_differentiable_layer(QueryType.ESDF,
                                                    random_points_in_scene_4d,
                                                    out_spheres_1,
                                                    mapper_id=1)

    # Query both maps
    out_spheres_both = torch.zeros_like(random_points_in_scene_4d)
    distances_both = mapper.query_differentiable_layer(QueryType.ESDF, random_points_in_scene_4d,
                                                       out_spheres_both)

    # Check distances from spheres
    expected_distances_0 = scene_utils.get_distances_from_sphere(random_points_in_scene, center_0,
                                                                 radius)
    assert_distances_in_bounds(distances_0, expected_distances_0)

    expected_distances_1 = scene_utils.get_distances_from_sphere(random_points_in_scene, center_1,
                                                                 radius)
    assert_distances_in_bounds(distances_1, expected_distances_1)

    # Get the gradient directions
    gradient_directions_0 = out_spheres_0[:, 0:3]
    gradient_directions_1 = out_spheres_1[:, 0:3]

    # Check that the gradient directions are pointing away from the center of the spheres.
    expected_gradient_directions_0 = random_points_in_scene - torch.tensor(center_0, device='cuda')
    expected_gradient_directions_0 = normalize_vectors(expected_gradient_directions_0)
    expected_gradient_directions_1 = random_points_in_scene - torch.tensor(center_1, device='cuda')
    expected_gradient_directions_1 = normalize_vectors(expected_gradient_directions_1)

    assert_gradients_in_bounds(gradient_directions_0, expected_gradient_directions_0)
    assert_gradients_in_bounds(gradient_directions_1, expected_gradient_directions_1)

    expected_distances_both = get_distances_from_both_spheres(random_points_in_scene,
                                                              [center_0, center_1], radius)
    assert_distances_in_bounds(distances_both, expected_distances_both)


def test_query_esdf_distances() -> None:

    mapper = get_single_sphere_scene_mapper(radius_m=1.0)
    query_points = torch.stack(torch.meshgrid(torch.linspace(0, 1, steps=10, device='cuda'),
                                              torch.linspace(0, 1, steps=10, device='cuda'),
                                              torch.linspace(0, 1, steps=10, device='cuda')),
                               dim=-1).view(-1, 3)
    query_spheres = points_to_4d_points(query_points)

    num_queries = query_points.shape[0]

    # Query differentiable layer
    output_differential_query = torch.zeros((num_queries, 4), device='cuda')
    distances_differential_query = mapper.query_differentiable_layer(QueryType.ESDF, query_spheres,
                                                                     output_differential_query)

    # Query with gradients
    output_gradient_query = torch.zeros((num_queries, 4), device='cuda')
    distances_gradient_query = mapper.query_layer(QueryType.ESDF_GRAD, query_spheres,
                                                  output_gradient_query)

    # Query without gradients
    output_nograd_query = torch.zeros((num_queries, 1), device='cuda')
    distances_nograd_query = mapper.query_layer(QueryType.ESDF, query_spheres, output_nograd_query)

    # Retrieved distancdes should be identical for the queries
    assert len(distances_nograd_query) > 0
    assert torch.all(distances_nograd_query.flatten() == distances_gradient_query[:, 3])
    assert torch.all(distances_nograd_query.flatten() == distances_differential_query, )


def get_single_sphere_test_scene() -> Mapper:
    # Scene
    scene_0 = Scene()
    scene_0.set_aabb([-5.5, -5.5, -5.5], [5.5, 5.5, 5.5])
    center_0 = [0.0, 0.0, 0.0]
    radius = 1.0
    scene_0.add_primitive('sphere', center_0 + [radius])

    # Load into mapper
    return scene_0.to_mapper([VOXEL_SIZE], mapper_id=0)


def test_query_esdf_with_cpu_vectors() -> None:
    # Create a scene containing a sphere at [0,0,0]
    mapper = get_single_sphere_test_scene()

    query_spheres = torch.zeros((2, 4), device='cpu')
    # NOTE(alexmillane): I noticed, when there was a bug in this function
    # that we only get a CUDA error after the second call. So I query twice.
    query_went_through = False
    for _ in range(2):
        try:
            mapper.query_differentiable_layer(
                QueryType.ESDF,
                query_spheres,
                mapper_id=0,
            )
            query_went_through = True
        except ValueError as _:
            print('Correctly threw an exception.')
            continue
    # We expect that the query function throws and the query doesn't go through.
    assert not query_went_through
