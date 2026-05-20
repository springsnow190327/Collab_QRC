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

from nvblox_torch import indexing


def test_local_index_grid() -> None:
    indices_grid = indexing.get_voxel_index_grid()
    # Index grid test
    assert indices_grid[0, 0, 0, 0] == 0
    assert indices_grid[0, 0, 0, 1] == 0
    assert indices_grid[0, 0, 0, 2] == 0

    assert indices_grid[1, 2, 3, 0] == 1
    assert indices_grid[1, 2, 3, 1] == 2
    assert indices_grid[1, 2, 3, 2] == 3


def test_local_centers_grid() -> None:
    voxel_size = 0.1
    center_grid = indexing.get_local_voxel_center_grid(voxel_size)

    assert center_grid[0, 0, 0, 0] == 0.5 * voxel_size
    assert center_grid[0, 0, 0, 1] == 0.5 * voxel_size
    assert center_grid[0, 0, 0, 2] == 0.5 * voxel_size

    assert center_grid[1, 2, 3, 0] == 1.5 * voxel_size
    assert center_grid[1, 2, 3, 1] == 2.5 * voxel_size
    assert center_grid[1, 2, 3, 2] == 3.5 * voxel_size


def test_global_centers_grid() -> None:
    voxel_size = 0.1
    block_indices = [
        torch.tensor([0, 0, 0], dtype=torch.int32),
        torch.tensor([1, 2, 3], dtype=torch.int32)
    ]
    voxel_center_grids = indexing.get_voxel_center_grids(block_indices, voxel_size)

    assert len(voxel_center_grids) == 2

    assert voxel_center_grids[0][0, 0, 0, 0] == 0.5 * voxel_size
    assert voxel_center_grids[0][1, 2, 3, 0] == 1.5 * voxel_size
    assert voxel_center_grids[0][1, 2, 3, 1] == 2.5 * voxel_size
    assert voxel_center_grids[0][1, 2, 3, 2] == 3.5 * voxel_size

    # Note: Floating point error started creeping in here so I stopped using ==
    eps = 1e-5
    voxel_block_size = indexing.NUM_VOXELS_PER_SIDE * voxel_size
    assert voxel_center_grids[1][0, 0, 0, 0] - (0.5 * voxel_size + 1.0 * voxel_block_size) < eps
    assert voxel_center_grids[1][0, 0, 0, 1] - (0.5 * voxel_size + 2.0 * voxel_block_size) < eps
    assert voxel_center_grids[1][0, 0, 0, 2] - (0.5 * voxel_size + 3.0 * voxel_block_size) < eps
    assert voxel_center_grids[1][1, 0, 0, 0] - (1.5 * voxel_size + 1.0 * voxel_block_size) < eps
