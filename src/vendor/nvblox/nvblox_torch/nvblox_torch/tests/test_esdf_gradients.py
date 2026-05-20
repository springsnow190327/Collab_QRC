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
import torch
import matplotlib.pyplot as plt

from nvblox_torch.scene import Scene
from nvblox_torch.mapper import Mapper, QueryType
from nvblox_torch.mapper_params import EsdfIntegratorParams, MapperParams

SCENE_SIZE = 10.0
VOXEL_SIZE = 0.05
GRID_NUM_POINTS_PER_SIDE = 5
CENTER = [0.0, 0.0, 0.0]
RADIUS = 1.0

ERROR_BOUND_DISTANCE = VOXEL_SIZE
ERROR_BOUND_VECTOR_NORM = 0.05


def get_torch_grid_for_scene(scene_size: float, num_points_per_side: int) -> torch.Tensor:
    # Define the 1D arrays for x, y, z
    x = torch.linspace(-scene_size / 2, scene_size / 2, steps=num_points_per_side)
    y = torch.linspace(-scene_size / 2, scene_size / 2, steps=num_points_per_side)
    z = torch.linspace(-scene_size / 2, scene_size / 2, steps=num_points_per_side)
    # Create the grid
    grid_x, grid_y, grid_z = torch.meshgrid(x, y, z, indexing='ij')
    # Stack them into a (N, 3) tensor
    grid_points = torch.stack((grid_x, grid_y, grid_z), dim=-1).reshape(-1, 3)
    return grid_points.to('cuda')


def normalize_vectors(vectors: torch.Tensor) -> torch.Tensor:
    assert vectors.ndim == 2
    assert vectors.shape[1] == 3
    return vectors / torch.norm(vectors, dim=1).unsqueeze(-1)


def get_sphere_scene_mapper() -> Mapper:
    scene = Scene()
    scene.set_aabb([-SCENE_SIZE / 2, -SCENE_SIZE / 2, -SCENE_SIZE / 2],
                   [SCENE_SIZE / 2, SCENE_SIZE / 2, SCENE_SIZE / 2])
    scene.add_primitive('sphere', CENTER + [RADIUS])

    # Need to set the max ESDF distance high (10m) to match GT distances later
    esdf_integrator_params = EsdfIntegratorParams()
    esdf_integrator_params.esdf_integrator_max_distance_m = 10.0
    mapper_params = MapperParams()
    mapper_params.set_esdf_integrator_params(esdf_integrator_params)

    # Mapper with the scene inside
    return scene.to_mapper([VOXEL_SIZE], mapper_parameters=mapper_params)


def check_distances_and_gradients_for_sphere_scene(
    query_grid: torch.Tensor,
    distances: torch.Tensor,
    gradients: torch.Tensor,
) -> None:
    assert query_grid.ndim == 2
    assert distances.ndim == 1
    assert gradients.ndim == 2
    assert query_grid.shape[0] == distances.shape[0]
    assert query_grid.shape[0] == gradients.shape[0]
    assert query_grid.shape[1] == 3
    # Expected distances
    expected_distance = torch.norm(query_grid - torch.tensor(CENTER, device='cuda'),
                                   dim=-1) - RADIUS
    # Check Distances
    errors = distances - expected_distance
    percentage_errors_in_bounds = torch.sum(errors < ERROR_BOUND_DISTANCE) / errors.numel()
    print(f'Percentage of errors less than voxel size: {percentage_errors_in_bounds}')
    assert percentage_errors_in_bounds > 0.99
    # Expected gradient directions
    expected_gradient_directions = query_grid - torch.tensor(CENTER, device='cuda')
    expected_gradient_directions = normalize_vectors(expected_gradient_directions)
    rows_containing_nans = torch.isnan(expected_gradient_directions).any(dim=1)
    expected_gradient_directions[rows_containing_nans, :] = 0
    # Check for nans
    assert torch.sum(torch.isnan(gradients)) == 0
    # Check gradient directions
    error_vectors = torch.norm(gradients - expected_gradient_directions, dim=1)
    percentage_error_vectors_in_bounds = torch.sum(
        error_vectors < ERROR_BOUND_VECTOR_NORM) / error_vectors.numel()
    print(f'Percentage of error vectors in bounds: {percentage_error_vectors_in_bounds}')
    assert percentage_error_vectors_in_bounds > 0.99


def vectors_to_zero_radius_spheres(vectors: torch.Tensor) -> torch.Tensor:
    sphere_radii = torch.zeros((vectors.shape[0], 1), device='cuda')
    query_spheres = torch.cat((vectors, sphere_radii), dim=1)
    return query_spheres


def plot_gradients(points: torch.Tensor, gradients: torch.Tensor) -> None:
    assert points.ndim == 2
    assert gradients.ndim == 2
    assert points.shape[1] == 3
    assert gradients.shape[1] == 3
    assert points.shape[0] == gradients.shape[0]
    fig = plt.figure()
    points_np = points.cpu().numpy()
    gradients_np = gradients.cpu().numpy()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points_np[:, 0], points_np[:, 1], points_np[:, 2])
    ax.quiver(points_np[:, 0], points_np[:, 1], points_np[:, 2], gradients_np[:, 0],
              gradients_np[:, 1], gradients_np[:, 2])
    plt.show()


def test_esdf_gradients_sphere_scene_query_output() -> None:
    # Test scene inside the mapper
    mapper = get_sphere_scene_mapper()

    # Get a grid of points to query
    grid_points = get_torch_grid_for_scene(scene_size=SCENE_SIZE,
                                           num_points_per_side=GRID_NUM_POINTS_PER_SIDE)

    # Query
    query_spheres = vectors_to_zero_radius_spheres(grid_points)
    query_output = torch.zeros(query_spheres.shape[0], 4, device='cuda')
    distance = mapper.query_differentiable_layer(
        QueryType.ESDF,
        query=query_spheres,
        output=query_output,
    )

    # Normalized estimated gradient directions
    estimated_gradient_directions = query_output[:, :3]

    # Test!
    check_distances_and_gradients_for_sphere_scene(
        query_grid=grid_points,
        distances=distance,
        gradients=estimated_gradient_directions,
    )

    # Debug plot
    plot = False
    if plot:
        plot_gradients(grid_points, estimated_gradient_directions)


def test_esdf_gradients_sphere_scene_backprop() -> None:
    # Test scene inside the mapper
    mapper = get_sphere_scene_mapper()

    # Get a grid of points to query
    grid_points = get_torch_grid_for_scene(scene_size=SCENE_SIZE,
                                           num_points_per_side=GRID_NUM_POINTS_PER_SIDE)
    grid_points.requires_grad = True

    # Query
    query_spheres = vectors_to_zero_radius_spheres(grid_points)
    # query_spheres.requires_grad = True
    distance = mapper.query_differentiable_layer(QueryType.ESDF, query=query_spheres)

    # Get a scalar to backprop
    # NOTE(alexmillane): The gradient of a summation with respect to the summands
    # is the gradient of the summands themselves. So the gradients with respect to
    # the query points are the same as the gradients with respect to the distance
    # field.
    total_distance = torch.sum(distance)
    total_distance.backward()

    # Test!
    with torch.no_grad():
        check_distances_and_gradients_for_sphere_scene(
            query_grid=grid_points,
            distances=distance,
            gradients=grid_points.grad,
        )

    # Debug plot
    plot = False
    if plot:
        plot_gradients(grid_points.detach(), grid_points.grad.detach())


def test_esdf_gradients_points_on_surface() -> None:
    # Scene with a sphere
    mapper = get_sphere_scene_mapper()
    # Points on the surface of the sphere
    points_on_sphere = torch.tensor([
        [RADIUS, 0.0, 0.0],
        [-RADIUS, 0.0, 0.0],
        [0.0, RADIUS, 0.0],
        [0.0, 0.0, RADIUS],
    ],
                                    device='cuda')
    # Query
    query_output = torch.zeros((points_on_sphere.shape[0], 4), device='cuda')
    distance = mapper.query_differentiable_layer(
        QueryType.ESDF,
        query=points_on_sphere,
        output=query_output,
    )
    # Check that the results are all zero
    assert torch.allclose(distance, torch.zeros_like(distance))
    assert torch.allclose(query_output, torch.zeros_like(distance))
