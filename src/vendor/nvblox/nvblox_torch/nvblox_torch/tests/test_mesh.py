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
import torch
import pathlib
import numpy as np
import open3d as o3d

from .helpers.scene_utils import get_single_sphere_scene, are_vertices_on_sphere

from nvblox_torch.mesh import ColorMesh, FeatureMesh


def test_empty_color_mesh() -> None:
    mesh = ColorMesh()
    assert mesh.vertices().shape == torch.Size([0, 3])
    assert mesh.triangles().shape == torch.Size([0, 3])
    assert mesh.vertex_colors().shape == torch.Size([0, 3])


def test_empty_feature_mesh() -> None:
    mesh = FeatureMesh()
    assert mesh.vertices().shape == torch.Size([0, 3])
    assert mesh.triangles().shape == torch.Size([0, 3])
    assert mesh.vertex_features().shape[0] == 0
    assert mesh.vertex_features().shape[1] >= 0


def test_mesh_saving(tmp_path: pathlib.Path) -> None:

    # Create a scene containing a single sphere
    aabb_dim = 5.5
    center = [0.0, 0.0, 0.0]
    radius = 1.0
    mapper = get_single_sphere_scene(center=center, radius=radius, aabb_dim=aabb_dim)

    # Generate the mesh
    mapper.update_color_mesh(0)
    color_mesh = mapper.get_color_mesh()

    # Save the mesh
    mesh_path = tmp_path / 'mesh.ply'
    color_mesh.save(str(mesh_path))

    # Load the mesh back
    mesh = o3d.io.read_point_cloud(str(mesh_path))

    assert len(mesh.points) > 0
    assert len(mesh.points) == len(color_mesh.vertices())
    assert len(mesh.colors) == len(color_mesh.vertex_colors())

    # Check that all mesh vertices are very close to lying on a sphere.
    vertices = torch.tensor(np.asarray(mesh.points))
    assert are_vertices_on_sphere(vertices, radius)
