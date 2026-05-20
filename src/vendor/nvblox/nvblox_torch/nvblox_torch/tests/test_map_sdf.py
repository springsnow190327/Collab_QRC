#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
import torch

from nvblox_torch.scene import Scene
from nvblox_torch.mapper import QueryType


def create_dummy_map() -> Scene:
    scene = Scene()

    scene.set_aabb([-5.5, -5.5, -0.5], [5.5, 5.5, 5.5])
    scene.add_plane_boundaries(-5.0, 5.0, -5.0, 5.0)
    scene.add_ground_level(0.0)
    scene.add_ceiling(5.0)
    scene.add_primitive('cube', [0.0, 0.0, 2.0, 2.0, 2.0, 2.0])
    scene.add_primitive('sphere', [0.0, 0.0, 2.0, 2.0])
    return scene


def test_py_dummy_map() -> None:
    scene = create_dummy_map()
    mapper = scene.to_mapper([0.02])
    batch_size = 10
    tensor_args = {'device': 'cuda', 'dtype': torch.float32}
    query_spheres = torch.zeros((batch_size, 4), **tensor_args)
    query_spheres[:, 3] = 0.001
    out_points = torch.zeros((batch_size, 4), **tensor_args)

    mapper.update_esdf()

    mapper.query_differentiable_layer(QueryType.ESDF, query_spheres, out_points, 0)
