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

# pylint: disable=invalid-name

import copy
from typing import Optional

import open3d as o3d
import torch
import numpy as np
import numpy.typing as npt
import matplotlib

from nvblox_torch.transforms import look_at_to_transformation_matrix

# pylint: disable=invalid-name


def to_open3d_pointcloud(pointcloud: npt.NDArray,
                         colors: npt.NDArray,
                         max_distance: Optional[float] = None,
                         compute_normals: bool = True) -> o3d.geometry.PointCloud:
    """Convert numpy arrays to Open3D point cloud.

    Args:
        pointcloud: Array of shape (N, 3) containing 3D points
        colors: Array of shape (N, 3) or (1, 3) containing RGB colors (0-255)
        max_distance: Optional maximum distance to filter points
        compute_normals: Whether to estimate point normals

    Returns:
        Open3D PointCloud object
    """
    assert pointcloud.shape[1] == 3
    # Visualizing the 3D point cloud using Open3D
    pcd_o3d = o3d.geometry.PointCloud()

    if max_distance is not None:
        mask = np.linalg.norm(pointcloud, axis=1) < max_distance
    else:
        mask = np.ones(pointcloud.shape[0], dtype=bool)
    pointcloud_filtered = pointcloud[mask]
    pcd_o3d.points = o3d.utility.Vector3dVector(pointcloud_filtered)
    # Add color to pointcloud
    if colors.shape[0] == 1:
        colors = np.array([colors[0] for _ in range(len(pointcloud_filtered))])
    assert pointcloud.shape[0] == colors.shape[0]
    pcd_o3d.colors = o3d.utility.Vector3dVector(colors[mask] / 255.0)
    if compute_normals:
        pcd_o3d.estimate_normals()
    return pcd_o3d


def to_open3d_voxel_grid(pointcloud: npt.NDArray,
                         colors: npt.NDArray,
                         voxel_size: float,
                         max_distance: Optional[float] = None) -> o3d.geometry.VoxelGrid:
    """Visualize points as a voxel grid in Open3D.

    Args:
        pointcloud: Tensor of shape (N, 3) containing 3D points
        colors: Array of shape (N, 3) containing RGB colors (0-255)
        voxel_size: Size of voxels for visualization
        max_distance: Optional maximum distance to filter points
    """
    pcd_o3d = to_open3d_pointcloud(pointcloud, colors, max_distance)
    voxel_grid_o3d = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_o3d, voxel_size=voxel_size)
    return voxel_grid_o3d


def get_open3d_coordinate_frame(T_W_C: Optional[torch.Tensor] = None,
                                size: float = 1.0) -> o3d.geometry.TriangleMesh:
    """Get a coordinate frame as a mesh for visualization.

    Args:
        T_W_C (npt.NDArray): Transform from the Camera coordinate frame
        (C) to the world coordinate frame (W).

    Returns
        Mesh representing the axis.

    """
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    if T_W_C is not None:
        axis.transform(T_W_C.cpu().numpy())
    return axis


def get_voxel_mesh(centers: torch.Tensor,
                   voxel_size_m: float,
                   colors: Optional[torch.Tensor] = None) -> o3d.geometry.TriangleMesh:
    """Gets an Open3d mesh representing a voxel grid.

    Args:
        centers: Nx3 voxels centers
        voxel_size_m: The size of the voxels.
        colors: Optional Nx3 colors. Defaults to None.

    Returns
        Mesh of the grid.
    """
    assert centers.dim() == 2
    assert centers.shape[-1] == 3
    if colors is not None:
        assert colors.shape[-1] == 3
        assert colors.dim() == 2
        assert centers.shape[0] == colors.shape[0]
    # Visualize
    cube_size_m = 0.9 * voxel_size_m
    voxel_mesh = o3d.geometry.TriangleMesh()
    cube_prototype = o3d.geometry.TriangleMesh.create_box(cube_size_m, cube_size_m, cube_size_m)
    cube_prototype.compute_vertex_normals()
    for idx, center in enumerate(centers):
        cube = copy.deepcopy(cube_prototype)
        cube.translate(center.cpu().numpy())
        if colors is not None:
            cube.paint_uniform_color(colors[idx].cpu().numpy())
        voxel_mesh += cube
    return voxel_mesh


def get_tsdf_colors(tsdfs: torch.Tensor) -> torch.Tensor:
    """Get colors from TSDF values.

    Args:
        tsdfs: An Nx1 tensor of TSDF values.

    Returns
        A tensor of colors
    """
    assert tsdfs.ndim == 1
    max_tsdf = torch.max(tsdfs)
    min_tsdf = torch.min(tsdfs)
    tsdfs_normalized = (tsdfs - min_tsdf) / (max_tsdf - min_tsdf)
    cmap = matplotlib.colormaps.get_cmap('plasma')
    colors = cmap(tsdfs_normalized.cpu().numpy())
    # Remove alpha
    colors = np.squeeze(colors[..., 0:-1])
    assert colors.shape[-1] == 3
    return torch.tensor(colors, device=tsdfs.device)


def get_tsdf_visualization_o3d(tsdfs: torch.Tensor, voxel_centers_m: torch.Tensor,
                               voxel_size_m: float) -> o3d.geometry.TriangleMesh:
    """Get a coordinate frame as a mesh for visualization.

    Args:
        tsdfs: Nx2 tensor containing the TSDF voxel values.
        voxel_centers_m: Nx3 tensor containing the voxel center positions.
        voxel_size_m: Voxel size.

    Returns
        A single mesh containing the voxel cubes.

    """
    assert tsdfs.shape[-1] == 2
    assert voxel_centers_m.shape[-1] == 3
    assert tsdfs.shape[0] == voxel_centers_m.shape[0]
    tsdf_colors = get_tsdf_colors(tsdfs[:, 0])
    return get_voxel_mesh(voxel_centers_m, voxel_size_m, tsdf_colors)


def get_segment_mesh(position_start: torch.Tensor, position_end: torch.Tensor,
                     radius: float) -> o3d.geometry.TriangleMesh:
    """Gets a mesh of a segment joining two positions.

    Args:
        position_start: First end-point
        position_end: Second end-point
        radius: Radius of the cylinder mesh.

    Returns:
        A mesh of the connection.
    """
    center = (position_end - position_start) / 2.0 + position_start
    length = torch.norm(position_end - position_start)
    device = position_start.device
    T_W_C = look_at_to_transformation_matrix(center_W=center,
                                             look_at_point_W=position_start,
                                             camera_up_W=torch.tensor([0.0, 0.0, 1.0],
                                                                      device=device))
    segment = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius,
        height=length,
    )
    segment.compute_vertex_normals()
    segment.transform(T_W_C.cpu().numpy())
    return segment


def get_sphere_mesh(position: torch.Tensor,
                    radius: float,
                    color: Optional[npt.NDArray] = None) -> o3d.geometry.TriangleMesh:
    """Get a spere mesh at a position.

    Args:
        position: 3D position tensor
        radius: radius of the sphere

    Returns:
        A mesh of the sphere.
    """
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.compute_vertex_normals()
    sphere.translate(position.cpu().numpy())
    if color is not None:
        sphere.paint_uniform_color(color)
    return sphere
