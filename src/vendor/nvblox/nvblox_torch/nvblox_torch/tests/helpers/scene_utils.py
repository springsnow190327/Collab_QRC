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
import random
from transforms3d import affines, euler
from nvblox_torch.scene import Scene
from nvblox_torch.mapper import Mapper


def get_random_points_in_box(num_samples: int, box_size_length_m: float) -> torch.Tensor:
    random_points_in_scene = box_size_length_m * torch.rand(num_samples, 3, device='cuda')
    random_points_in_scene -= torch.tensor([
        box_size_length_m / 2.0,
        box_size_length_m / 2.0,
        box_size_length_m / 2.0,
    ],
                                           device='cuda')
    return random_points_in_scene


def get_distances_from_sphere(points: torch.Tensor, center: List[float],
                              radius: float) -> torch.Tensor:
    assert points.shape[1] == 3
    vec = points - torch.tensor(center, device='cuda')
    distance = torch.norm(vec, dim=1) - radius
    return distance


def generate_random_pose(device: str) -> torch.Tensor:
    """
    Generate a random pose matrix.

    Args:
        device (str): The device on which to create the tensor (e.g., 'cuda:0' or 'cpu').

    Returns:
        torch.Tensor: A 4x4 transformation matrix representing the pose matrix.
    """
    randomized_euler_angles_deg = [random.uniform(-180, 180) for _ in range(3)]
    randomized_translation_vector = [random.uniform(-10, 10) for _ in range(3)]

    euler_angles_rad = [math.radians(angle) for angle in randomized_euler_angles_deg]
    rotation_matrix = euler.euler2mat(*euler_angles_rad)
    zooms = [1, 1, 1]
    transformation_matrix = affines.compose(randomized_translation_vector, rotation_matrix, zooms)

    return torch.Tensor(transformation_matrix).to(device)


def get_single_sphere_scene(center: List[float], radius: float, aabb_dim: float) -> Mapper:
    # Create a scene containing a single sphere
    scene = Scene()
    scene.set_aabb([-aabb_dim, -aabb_dim, -aabb_dim], [aabb_dim, aabb_dim, aabb_dim])
    scene.add_primitive('sphere', center + [radius])

    return scene.to_mapper(voxel_sizes_m=[0.05])


def are_vertices_on_sphere(points: torch.Tensor, radius: float, eps: float = 1e-4) -> bool:
    assert points.shape[0] > 0
    assert points.shape[1] == 3
    radii = torch.norm(points, dim=1)
    distance_off_sphere = radii - torch.tensor(radius)
    print(torch.min(distance_off_sphere))
    return torch.all(distance_off_sphere < eps)
