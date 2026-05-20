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

from nvblox_torch.mapper import Mapper, QueryType
from nvblox_torch.layer import TsdfLayer, FeatureLayer, convert_layer_to_dense_tensor
from nvblox_torch.constants import constants
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType

import torch
import pytest
from typing import Type, Union

VOXEL_SIZE = 0.1
BLOCK_SIZE = 8 * VOXEL_SIZE
LAYER_TYPES = [FeatureLayer, TsdfLayer]

LayerTypes = Type[Union[TsdfLayer, FeatureLayer]]


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_voxel_size(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    assert layer.voxel_size() == pytest.approx(VOXEL_SIZE)


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_allocate_block_at_index(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    layer.allocate_block_at_index(torch.IntTensor([0, 0, 0]))
    assert layer.num_blocks() == 1


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_num_blocks(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    layer.allocate_block_at_index(torch.IntTensor([0, 0, 0]))
    layer.allocate_block_at_index(torch.IntTensor([1, 1, 1]))
    assert layer.num_blocks() == 2


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_num_allocated_bytes(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)

    # Num preallocated blocks is defined in nvblox_lib. We hardcode it for now.
    # (TODO) dtingdahl, make this a parameter.
    num_preallocated = 2048

    # Feature layer use float16
    field_size = 2 if issubclass(layer_type, FeatureLayer) else 4

    voxel_size_bytes = layer.num_elements_per_voxel() * field_size
    assert layer.num_allocated_bytes() == num_preallocated * 8 * 8 * 8 * voxel_size_bytes


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_num_allocated_blocks(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)

    # Num preallocated blocks is defined in nvblox_lib. We hardcode it for now.
    # (TODO) dtingdahl, make this a parameter.
    num_preallocated = 2048
    assert layer.num_allocated_blocks() == num_preallocated


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_clear(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    layer.allocate_block_at_index(torch.IntTensor([0, 0, 0]))
    layer.clear()
    assert layer.num_blocks() == 0


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_get_block_at_index(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    index = torch.IntTensor([0, 0, 0])
    layer.allocate_block_at_index(index)
    block = layer.get_block_at_index(index)

    assert block.dim() == 4
    assert block.shape == torch.Size([8, 8, 8, layer.num_elements_per_voxel()])


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_get_block_at_nonexisting_index(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    block = layer.get_block_at_index(torch.IntTensor([0, 0, 0]))
    assert block is None


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_get_all_block_indices(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)

    indices = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
    for index in indices:
        layer.allocate_block_at_index(torch.IntTensor(index))

    block_indices = layer.get_all_block_indices()

    indices.reverse()    # Indices are retrieved in reverse order
    assert torch.all(block_indices == torch.IntTensor(indices))


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_get_all_blocks(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)

    indices = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
    for i, index in enumerate(indices):
        layer.allocate_block_at_index(torch.IntTensor(index))
        block = layer.get_block_at_index(torch.IntTensor(index))
        block[:, :, :] = float(i)

    blocks_and_indices = layer.get_all_blocks()

    assert len(blocks_and_indices) == 2
    assert len(blocks_and_indices[0]) == len(indices)
    assert len(blocks_and_indices[1]) == len(indices)

    blocks = blocks_and_indices[0]
    blocks.reverse()    # Indices are retrieved in reverse order

    dtype = torch.float16 if issubclass(layer_type, FeatureLayer) else torch.float32

    for i, index in enumerate(indices):
        torch.all(
            torch.isclose(blocks[i].cpu(),
                          torch.ones([8, 8, 8, layer.num_elements_per_voxel()], dtype=dtype) * i))


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_constructors(layer_type: LayerTypes) -> None:
    layer = layer_type(voxel_size_m=0.05)
    assert layer.voxel_size() == pytest.approx(0.05)

    # Check we can allocate a single block
    test_index_tensor = torch.tensor([0, 0, 0], dtype=torch.int32)
    layer.allocate_block_at_index(test_index_tensor)
    assert layer.num_blocks() == 1

    # Check the block is where we allocated it to be
    block_indices_tensor = layer.get_all_block_indices()
    assert torch.all(block_indices_tensor == test_index_tensor).item()


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_modification(layer_type: LayerTypes) -> None:
    # Allocate a block
    layer = layer_type(voxel_size_m=0.05)
    test_index_tensor = torch.tensor([0, 0, 0], dtype=torch.int32)
    layer.allocate_block_at_index(test_index_tensor)

    # Check that the start value is zero
    block_tensor = layer.get_block_at_index(test_index_tensor)
    assert block_tensor.size()[0] == layer.block_dim_in_voxels
    assert len(block_tensor.size()) == 4
    assert torch.max(block_tensor - torch.zeros_like(block_tensor)).item() == pytest.approx(0.0)

    # Modify a single value
    block_tensor[0, 0, 0, 0] = 1.0
    assert block_tensor[0, 0, 0, 0] == 1.0

    # Check it worked, by getting the block again
    block_tensor_2 = layer.get_block_at_index(test_index_tensor)
    assert block_tensor_2[0, 0, 0, 0] == 1.0

    # Modify the entire block
    new_block_value = 2.0
    new_block_values = new_block_value * torch.ones_like(block_tensor_2)
    block_tensor_2.copy_(new_block_values)
    assert torch.all(block_tensor_2 == new_block_value).item()

    # Check that it affected the block
    block_tensor_3 = layer.get_block_at_index(test_index_tensor)
    assert torch.all(block_tensor_3 == new_block_value).item()


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_access_unallocated(layer_type: LayerTypes) -> None:
    # Allocate a block
    layer = layer_type(voxel_size_m=0.05)
    test_index_tensor = torch.tensor([0, 0, 0], dtype=torch.int32)
    layer.allocate_block_at_index(test_index_tensor)

    # Check that we get an empty tensor if we request the wrong block
    wrong_test_index_tensor = torch.tensor([0, 0, 1], dtype=torch.int32)
    wrong_block_tensor = layer.get_block_at_index(wrong_test_index_tensor)
    assert wrong_block_tensor is None


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_get_block_limits(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    indices = [[0, 1, 0], [2, 3, 2], [1, -1, 4]]
    for index in indices:
        layer.allocate_block_at_index(torch.IntTensor(index))

    calculated_aabb_min_indices, calculated_aabb_max_indices = layer.get_block_limits()

    expected_aabb_min_indices = torch.IntTensor([0, -1, 0])
    expected_aabb_max_indices = torch.IntTensor([2, 3, 4])

    assert torch.equal(calculated_aabb_min_indices, expected_aabb_min_indices)
    assert torch.equal(calculated_aabb_max_indices, expected_aabb_max_indices)


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_convert_layer_to_dense_tensor(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    test_block_indices = [[0, 0, 0], [1, 1, 1]]
    for test_block_index in test_block_indices:
        layer.allocate_block_at_index(torch.IntTensor(test_block_index))

    block_tensor_val_one = 1.0
    block_tensor_one = layer.get_block_at_index(torch.IntTensor([0, 0, 0]))
    block_tensor_one[:, :, :] = block_tensor_val_one

    block_tensor_val_two = 2.0
    block_tensor_two = layer.get_block_at_index(torch.IntTensor([1, 1, 1]))
    block_tensor_two[:, :, :] = block_tensor_val_two

    actual_dense_tensor, _ = convert_layer_to_dense_tensor(layer)

    if isinstance(layer, TsdfLayer):
        expected_dense_tensor_shape = (16, 16, 16, 1)
        # One block allocated for value 0 & value 1 respectively
        expected_num_values_val_one = 8 * 8 * 8
        expected_num_values_val_two = 8 * 8 * 8
    elif isinstance(layer, FeatureLayer):
        feature_size = constants.feature_array_num_elements()
        expected_dense_tensor_shape = (16, 16, 16, feature_size)
        # One block allocated for value 0 & value 1 respectively
        expected_num_values_val_one = 8 * 8 * 8 * feature_size
        expected_num_values_val_two = 8 * 8 * 8 * feature_size
    else:
        raise TypeError(f'Unsupported layer type to convert to dense tensor: {type(layer)}')

    # Test for the shape.
    assert actual_dense_tensor.shape == expected_dense_tensor_shape

    # Test that the calculated tensor contains the expected number of values.
    actual_num_values_val_one = torch.sum(actual_dense_tensor == block_tensor_val_one).item()
    actual_num_values_val_two = torch.sum(actual_dense_tensor == block_tensor_val_two).item()
    assert actual_num_values_val_one == expected_num_values_val_one
    assert actual_num_values_val_two == expected_num_values_val_two


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_is_block_allocated(layer_type: LayerTypes) -> None:
    layer = layer_type(VOXEL_SIZE)
    index = torch.IntTensor([0, 0, 0])
    layer.allocate_block_at_index(index)
    assert layer.is_block_allocated(index)


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_query_feature_layer(layer_type: LayerTypes) -> None:
    # TODO(cvolk): Generalize to all layers
    if issubclass(layer_type, FeatureLayer):
        mapper = Mapper(voxel_sizes_m=[VOXEL_SIZE],
                        integrator_types=[ProjectiveIntegratorType.TSDF])
        feature_layer = mapper.feature_layer_view()

        block_indices = [
            torch.IntTensor([0, 0, 0]),
            torch.IntTensor([0, 2, 0]),
            torch.IntTensor([1, 2, 3])
        ]
        for test_block_index in block_indices:
            test_feature_value = 5.5
            feature_layer.allocate_block_at_index(test_block_index)
            num_elements_per_voxel = feature_layer.num_elements_per_voxel()

            test_feature = torch.full((num_elements_per_voxel - 1, ),
                                      test_feature_value,
                                      dtype=torch.float16,
                                      device='cuda')
            test_weight = torch.tensor([0.5], device='cuda')

            feature_block = feature_layer.get_block_at_index(test_block_index)
            # Set a voxel in this block
            feature_block[0][0][0] = torch.cat((test_feature, test_weight))

            # Query the point corresponding to this voxel in the block
            hv = VOXEL_SIZE * 0.5
            query_point = torch.tensor([
                BLOCK_SIZE * test_block_index[0] + hv, BLOCK_SIZE * test_block_index[1] + hv,
                BLOCK_SIZE * test_block_index[2] + hv
            ],
                                       device='cuda').unsqueeze(0)

            # Test to retrieve the voxel we just set
            query_features_and_weight = mapper.query_layer(QueryType.FEATURE,
                                                           query_point,
                                                           mapper_id=0)
            query_features = query_features_and_weight[:, :-1]
            query_weight = query_features_and_weight[:, -1]
            assert torch.equal(query_features.squeeze(0), test_feature)
            assert test_weight == query_weight

            # Test to retrieve an empty voxel
            query_features_and_weight = mapper.query_layer(
                QueryType.FEATURE,
                query_point + torch.tensor([VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE], device='cuda'),
                mapper_id=0)
            query_features = query_features_and_weight[:, :-1]
            query_weight = query_features_and_weight[:, -1]
            assert torch.equal(query_features.squeeze(0),
                               torch.zeros(query_features.shape[1], device='cuda'))
            assert query_weight == 0.

    else:
        pytest.skip('Not a FeatureLayer instance')


@pytest.mark.parametrize('layer_type', LAYER_TYPES)
def test_get_tsdfs_below_zero(layer_type: LayerTypes) -> None:
    if issubclass(layer_type, TsdfLayer):
        layer = layer_type(VOXEL_SIZE)
        test_block_indices = [[0, 0, 0], [1, 0, 0], [3, 0, 3], [4, 1, 9]]
        test_tsdf_below_zero = -0.1
        test_weight = 0.5
        hv = VOXEL_SIZE * 0.5

        # Initialize the test_positions tensor with the same length as test_block_indices.
        # We add only one voxel per block.
        expected_positions = torch.zeros((len(test_block_indices), 3), device='cuda')

        # Set a voxel per block to a negative tsdf value.
        for num_test_block, test_block_index in enumerate(test_block_indices):
            layer.allocate_block_at_index(torch.IntTensor(test_block_index))
            tsdf_block = layer.get_block_at_index(torch.IntTensor(test_block_index))
            # Set the lower left voxel.
            tsdf_block[0][0][0] = torch.tensor([test_tsdf_below_zero, test_weight], device='cuda')
            # Store the lower left voxel position per block.
            expected_position = torch.tensor([
                BLOCK_SIZE * test_block_index[0] + hv, BLOCK_SIZE * test_block_index[1] + hv,
                BLOCK_SIZE * test_block_index[2] + hv
            ],
                                             device='cuda')
            expected_positions[num_test_block] = expected_position

        actual_tsdf_and_weights, actual_positions = layer.get_tsdfs_below_zero()

        # All expected positions can be found in the actual position tensor.
        for expected_position in expected_positions:
            assert torch.any(
                torch.all(torch.isclose(actual_positions, expected_position, atol=1e-5), dim=1))
        assert torch.all(actual_tsdf_and_weights[:, 0] == test_tsdf_below_zero)
        assert torch.all(actual_tsdf_and_weights[:, 1] == test_weight)

        # Expect one voxel per block.
        assert actual_tsdf_and_weights[:, 0].shape[0] == len(test_block_indices)
        assert actual_positions.shape[0] == len(test_block_indices)
    else:
        pytest.skip('Not a FeatureLayer instance')
