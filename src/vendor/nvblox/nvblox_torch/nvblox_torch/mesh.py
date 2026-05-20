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
import open3d as o3d
from typing import Any
from abc import ABC, abstractmethod

from nvblox_torch.lib.utils import get_nvblox_torch_class


class Mesh(ABC):
    """Abstract Mesh class for PyTorch."""

    def __init__(self, c_mesh: Any = None) -> None:
        """Construct from a serialized C++ mesh."""
        if c_mesh is None:
            self._c_mesh = self._create_empty_mesh()
        else:
            self._c_mesh = c_mesh

    @abstractmethod
    def _create_empty_mesh(self) -> Any:
        """Create a default layer for the mesh."""
        pass

    def vertices(self) -> torch.Tensor:
        """Get vertices of the mesh.

        Returns
            Vertices (N, 3)
        """
        return self._c_mesh.vertices()

    def triangles(self) -> torch.Tensor:
        """Get triangle indices of the mesh.

        Returns
            Index triplets: (M, 3)
        """
        return self._c_mesh.triangles()

    def vertex_appearances(self) -> torch.Tensor:
        """Get vertex appearances of the mesh.

        Returns
            Vertex appearances: (N, F)
        """
        return self._c_mesh.vertex_appearances()

    def __str__(self) -> str:
        """String representation of the mesh contents."""
        return (f'Mesh('
                f'vertices={self.vertices().shape}, '
                f'triangles={self.triangles().shape}, '
                f'vertex_appearances={self.vertex_appearances().shape}, ')


class ColorMesh(Mesh):
    """ColorMesh class for PyTorch, inheriting from Mesh.

    A mesh representation that includes RGB color information for each vertex.
    The vertex_appearances() method returns a uint8 tensor of shape (N,3) containing
    the RGB colors for each of the N vertices.
    """

    def vertex_colors(self) -> torch.Tensor:
        """Get vertex colors."""
        return self.vertex_appearances()

    def _create_empty_mesh(self) -> Any:
        """Create an empty color mesh."""
        return get_nvblox_torch_class('ColorMesh')()

    def to_open3d(self) -> o3d.geometry.TriangleMesh:
        """Convert the mesh to an Open3D TriangleMesh.

        Returns
            An Open3D mesh.

        """
        mesh_o3d = o3d.geometry.TriangleMesh()
        # To tensor
        vertices = self.vertices()
        vertex_colors = self.vertex_colors().to(torch.float64) / 255.0
        triangles = self.triangles()
        # To numpy
        vertices_np = vertices.cpu().numpy()
        vertex_colors_np = vertex_colors.cpu().numpy()
        triangles_np = triangles.cpu().numpy()
        # To open3d
        mesh_o3d.vertices = o3d.utility.Vector3dVector(vertices_np)
        mesh_o3d.vertex_colors = o3d.utility.Vector3dVector(vertex_colors_np)
        mesh_o3d.triangles = o3d.utility.Vector3iVector(triangles_np)

        return mesh_o3d

    def save(self, mesh_fname: str) -> None:
        """Save the mesh to a file."""
        o3d_mesh = self.to_open3d()
        o3d.io.write_triangle_mesh(mesh_fname, o3d_mesh)


class FeatureMesh(Mesh):
    """FeatureMesh class for PyTorch, inheriting from Mesh.

    A mesh representation that includes feature vectors for each vertex.
    The vertex_appearances() method returns a float tensor of shape (N,F) containing
    the F-dimensional feature vectors for each of the N vertices.
    """

    def vertex_features(self) -> torch.Tensor:
        """Get vertex features."""
        return self.vertex_appearances()

    def _create_empty_mesh(self) -> Any:
        """Create an empty feature mesh."""
        return get_nvblox_torch_class('FeatureMesh')()
