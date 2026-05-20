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

import numpy as np

from nvblox_torch.scene import Scene


def elements_equal(list_1: List[float], list_2: List[float]) -> bool:
    return np.all(np.array(list_1) == np.array(list_2))


def test_aabb() -> None:
    scene = Scene()
    input_min = [-1.0, -1.0, -1.0]
    input_max = [1.0, 1.0, 1.0]
    scene.set_aabb(input_min, input_max)
    output_min, output_max = scene.get_aabb()
    assert elements_equal(output_min, input_min)
    assert elements_equal(output_max, input_max)


def test_plane_boundaries() -> None:
    scene = Scene()
    scene.add_plane_boundaries(x_min=0, x_max=1, y_min=2, y_max=3)
    primitives_type_list = scene.get_primitives_type_list()
    assert len(primitives_type_list) == 4
    for primitive_type in primitives_type_list:
        assert primitive_type == 'kPlane'


def test_ground_and_ceiling() -> None:
    scene = Scene()
    scene.add_ground_level(0.0)
    scene.add_ceiling(1.0)
    primitives_type_list = scene.get_primitives_type_list()
    assert len(primitives_type_list) == 2
    assert primitives_type_list[0] == 'kPlane'
    assert primitives_type_list[1] == 'kPlane'


def test_cube_and_sphere() -> None:
    scene = Scene()
    center = [0.0, 0.0, 0.0]
    size = [1.0, 2.0, 3.0]
    scene.add_primitive('cube', center + size)
    center = [0.0, 0.0, 0.0]
    radius = [1.0]
    scene.add_primitive('sphere', center + radius)
    primitives_type_list = scene.get_primitives_type_list()
    assert len(primitives_type_list) == 2
    assert primitives_type_list[0] == 'kCube'
    assert primitives_type_list[1] == 'kSphere'


def test_dummy_scene() -> None:
    # The dummy scene is a sphere inside as box.
    # There are also 6 planes which bound the scene
    scene = Scene()
    scene.create_dummy_map()
    primitives_type_list = scene.get_primitives_type_list()
    assert len(primitives_type_list) == 8
    assert primitives_type_list[0] == 'kPlane'
    assert primitives_type_list[1] == 'kPlane'
    assert primitives_type_list[2] == 'kPlane'
    assert primitives_type_list[3] == 'kPlane'
    assert primitives_type_list[4] == 'kPlane'
    assert primitives_type_list[5] == 'kPlane'
    assert primitives_type_list[6] == 'kCube'
    assert primitives_type_list[7] == 'kSphere'


def test_plane() -> None:
    scene = Scene()
    scene.add_primitive('plane', [0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    primitives_type_list = scene.get_primitives_type_list()
    assert len(primitives_type_list) == 1
    assert primitives_type_list[0] == 'kPlane'


def test_to_mapper() -> None:
    scene = Scene()
    scene.create_dummy_map()
    mapper = scene.to_mapper(voxel_sizes_m=[0.1])
    assert mapper.tsdf_layer_view().num_blocks() > 0
