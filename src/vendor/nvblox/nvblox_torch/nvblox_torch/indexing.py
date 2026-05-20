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

NUM_VOXELS_PER_SIDE = 8


def get_voxel_index_grid(device: torch.device = 'cuda') -> torch.Tensor:
    """Generate an 8x8x8X3 grid of 3D voxel indices within a VoxelBlock.

    Returns
        A 8x8x8X3 tensor on device of type int32.

    """
    xyz_linspaces = []
    for _ in range(3):
        xyz_linspaces.append(
            torch.linspace(0,
                           NUM_VOXELS_PER_SIDE - 1,
                           NUM_VOXELS_PER_SIDE,
                           device=device,
                           dtype=torch.int32))
        xyz_grids = torch.meshgrid(xyz_linspaces, indexing='ij')
        indices_grid = torch.stack(xyz_grids, dim=-1)
    return indices_grid


def get_local_voxel_center_grid(voxel_size: float, device: torch.device = 'cuda') -> torch.Tensor:
    """Generate an 8x8x8x3 grid of 3D voxel center positions wrt VoxelBlock Origin.

    i.e. not global position.

    Returns
        A 8x8x8X3 tensor on device of type float32.
    """
    indices_grid = get_voxel_index_grid(device=device)
    center_grid = (indices_grid.type(torch.float32) + 0.5) * voxel_size
    return center_grid


def get_voxel_center_grids(block_indices: List[torch.Tensor],
                           voxel_size: float,
                           device: torch.device = 'cuda') -> List[torch.Tensor]:
    """Generate a list of 8x8x8X3 grids of 3D voxel center positions wrt the world.

    Returns
        A 8x8x8X3 tensor on device of type float32.

    """
    voxel_block_size = NUM_VOXELS_PER_SIDE * voxel_size
    local_voxel_center_grid = get_local_voxel_center_grid(voxel_size, device=device)
    voxel_centers_list = []
    for block_index in block_indices:
        block_origin = block_index.type(torch.float32).to(device) * voxel_block_size
        voxel_center_grid = block_origin + local_voxel_center_grid
        voxel_centers_list.append(voxel_center_grid)
    return voxel_centers_list
