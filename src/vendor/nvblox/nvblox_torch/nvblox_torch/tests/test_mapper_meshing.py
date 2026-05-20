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

from .helpers.scene_utils import get_single_sphere_scene, are_vertices_on_sphere


def test_mapper_mesh_retrieval() -> None:
    # Get test scene
    aabb_dim = 5.5
    center = [0.0, 0.0, 0.0]
    radius = 1.0
    mapper = get_single_sphere_scene(center=center, radius=radius, aabb_dim=aabb_dim)

    # Generate the mesh
    mapper.update_color_mesh(0)

    # Get the vertices and check
    mesh = mapper.get_color_mesh()

    # Check that all mesh vertices are very close to lying on a sphere.
    assert are_vertices_on_sphere(mesh.vertices(), radius)
