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

from nvblox_torch.mapper import Mapper
from nvblox_torch.scene import Scene
from nvblox_torch import indexing
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType

from .helpers import scene_utils, tsdf_test_helpers

VOXEL_SIZE = 0.1


def test_tsdf_from_sphere_scene() -> None:
    # Create a scene containing a single sphere
    scene = Scene()
    scene.set_aabb([-5.5, -5.5, -5.5], [5.5, 5.5, 5.5])
    center = [0.0, 0.0, 0.0]
    radius = 1.0
    scene.add_primitive('sphere', center + [radius])

    # Mapper
    mapper = scene.to_mapper([VOXEL_SIZE])

    # Get the TSDF layer to test.
    tsdf_layer = mapper.tsdf_layer_view()
    assert tsdf_layer.num_blocks() > 0

    # Extract the voxel data as tensors.
    voxel_blocks, indices = tsdf_layer.get_all_blocks()

    # Max allowable error in the TSDF
    eps = 1e-2

    # Loop over all the blocks and test their TSDF distances.
    voxel_centers_grids = indexing.get_voxel_center_grids(indices, VOXEL_SIZE)
    for voxel_centers_grid, voxel_block in zip(voxel_centers_grids, voxel_blocks):
        # Get what we expect from the sphere
        expected_distances = scene_utils.get_distances_from_sphere(voxel_centers_grid.view((-1, 3)),
                                                                   center, radius)
        expected_tsdf = tsdf_test_helpers.truncate_distances(expected_distances,
                                                             clip_value=4.0 * VOXEL_SIZE)
        # Extract what we have in the TSDF layer
        measured_tsdf = torch.flatten(voxel_block[..., 0])
        weights = torch.flatten(voxel_block[..., 1])
        # Check that the error is low.
        errors = torch.abs(expected_tsdf - measured_tsdf)
        valid_errors = errors[weights > 0.0]
        max_error = torch.max(valid_errors)
        assert max_error < eps


def test_modification_of_layer_inside_mapper() -> None:
    mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE], integrator_types=[ProjectiveIntegratorType.TSDF])
    # Get the TSDF layer out
    tsdf_layer = mapper.tsdf_layer_view()

    # Make some modifications
    index = torch.IntTensor([0, 0, 0])
    tsdf_layer.allocate_block_at_index(index)
    tsdf_block = tsdf_layer.get_block_at_index(index)
    tsdf_block[0, 0, 0, 0] = 1.0

    # Get a second reference to the layer
    tsdf_layer_2 = mapper.tsdf_layer_view()

    # Check that the modifications still hold
    tsdf_block_2 = tsdf_layer_2.get_block_at_index(index)
    assert tsdf_block_2[0, 0, 0, 0] == 1.0
