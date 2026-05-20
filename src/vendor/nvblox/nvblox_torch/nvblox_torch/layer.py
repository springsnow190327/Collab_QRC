import torch
import abc
from nvblox_torch.lib.utils import get_nvblox_torch_class
from nvblox_torch import indexing
from typing import Callable, Optional, Type, Union, Tuple, List
from nvblox_torch.constants import constants


class Layer:
    """Base class for nvblox voxelblock layers.

    A layer represent one reconstruction modality (e.g. TSDF, Color, occupancy).
    A map typically consists of multiple layers.

    This class provides a Python API to interact with the underlying C++ layer.
    """

    # TODO-RELEASE(dtingdahl): use value fron nvblox constants
    block_dim_in_voxels = 8

    def __init__(self,
                 voxel_size_m: float,
                 torch_class_name: str,
                 c_layer: Union[Type, None] = None):
        """Initialize the layer.

        If c_layer is None, a new layer will be created.
        Otherwise, the layer will be wrapped around the provided c_layer.

        Accessors are zero-copy, i.e. they provide views around the wrapped C++ layer.
        This means that the returned tensors will be invalidated if the layer is modified.

        Args:
            voxel_size_m: Size of a voxel in meters.
            torch_class_name: Name of the torch class to use when c_layer is None.
            c_layer: Optional c-layer to wrap.
        """
        if c_layer is None:
            self._c_layer = get_nvblox_torch_class(torch_class_name)(voxel_size_m)
        else:
            self._c_layer = c_layer

    @staticmethod
    @abc.abstractmethod
    def num_elements_per_voxel() -> int:
        """Return the number of elements per voxel."""
        pass

    def voxel_size(self) -> float:
        """Return the size of a voxel in meters."""
        return self._c_layer.voxel_size()

    def num_blocks(self) -> int:
        """Return the number of active blocks in the layer."""
        return self._c_layer.num_blocks()

    def num_allocated_bytes(self) -> int:
        """Return the number of allocated bytes in the layer."""
        return self._c_layer.num_allocated_bytes()

    def num_allocated_blocks(self) -> int:
        """Return the number of allocated blocks in the layer.

        Note that this is typically larger than the number of blocks in the layer.
        """
        return self._c_layer.num_allocated_blocks()

    def clear(self) -> None:
        """Clear the layer."""
        return self._c_layer.clear()

    def allocate_block_at_index(self, index: torch.Tensor) -> None:
        """Allocate a block at the given index."""
        return self._c_layer.allocate_block_at_index(index)

    def is_block_allocated(self, index: torch.Tensor) -> bool:
        """Check if a block is allocated at the given index."""
        return self._c_layer.is_block_allocated(index)

    def get_block_at_index(self, index: torch.Tensor) -> torch.Tensor:
        """Get a view of a block at the given index."""
        return self._c_layer.get_block_at_index(index)

    def get_all_block_indices(self) -> torch.Tensor:
        """Get all block indices."""
        return self._c_layer.get_all_block_indices()

    def get_all_blocks(self) -> Tuple[List[torch.Tensor], List[torch.Tensor]]:
        """Get tensor views of all blocks in the layer.

        Return:
            Block tensors and block indices.

        """
        return self._c_layer.get_all_blocks()

    def get_block_limits(self) -> Tuple[torch.Tensor, torch.Tensor]:
        """Return the extents of the layer, expressed in block indices.

        Return:
            Min and Max extents in block indices

        """
        block_indices_tensor = self.get_all_block_indices()
        aabb_max_indices, _ = torch.max(block_indices_tensor, dim=0)
        aabb_min_indices, _ = torch.min(block_indices_tensor, dim=0)
        return aabb_min_indices, aabb_max_indices

    def get_voxels_matching_condition(
            self, get_voxel_mask: Callable) -> Tuple[torch.Tensor, torch.Tensor]:
        """Return voxels that match a condition.

        Return the values (the first self._num_elements_per_voxel values within a voxel) and
        3D positions of the input Layer which meet the passed condition.

        Args:
            get_voxel_mask: A function generating a mask per voxel
                block tensor.

        Returns
            An Nxself._num_elements_per_voxel tensor containing the values of the voxels and a Nx3
            tensor containing voxel centers.

        """
        # Get the voxel blocks and voxel center positions
        voxel_size_m = self.voxel_size()
        voxel_blocks, indices = self.get_all_blocks()
        voxel_centers_per_block_w = indexing.get_voxel_center_grids(indices, voxel_size_m)
        points = torch.zeros((0, 3), device='cuda')
        values = torch.zeros((0, self.num_elements_per_voxel()), device='cuda')
        for layer_block, voxel_centers_w in zip(voxel_blocks, voxel_centers_per_block_w):
            # Get the passing voxels
            mask = get_voxel_mask(layer_block)
            assert mask.shape == torch.Size([8, 8,
                                             8]), 'Your condition should generate a 8x8x8 mask.'
            # Apply mask
            points_in_this_block = voxel_centers_w[mask, :]
            values_in_this_block = layer_block[mask]
            points = torch.vstack((points, points_in_this_block))
            values = torch.vstack((values, values_in_this_block))
        return values, points


class TsdfLayer(Layer):
    """Specialization for the TSDF layer."""

    def __init__(self, voxel_size_m: float, c_layer: torch.classes.TsdfLayer = None):
        """Initialize the TSDF layer.

        Args:
            voxel_size_m: Size of a voxel in meters.
            c_layer: Optional c-layer to wrap.
        """
        super().__init__(voxel_size_m, c_layer=c_layer, torch_class_name='TsdfLayer')

    @staticmethod
    def num_elements_per_voxel() -> int:
        """Return the number of elements per voxel."""
        return 2

    def get_tsdf_mask_negative_distance(self, tsdf_block: torch.Tensor) -> torch.Tensor:
        """Get TSDF voxels that are inside objects and have nonzero weight.

        Return an 8x8x8 mask which is true where TSDF distance < 0.0 and
        weight > 0.0.

        Args:
            tsdf_block: An 8x8x8x2 tensor representing TSDF block.

        Returns
            An 8x8x8 mask.

        """
        tsdf = tsdf_block[..., 0]
        weight = tsdf_block[..., 1]
        mask = torch.logical_and(tsdf < 0.00, weight > 0.01)
        return mask

    def get_tsdfs_below_zero(self) -> Tuple[torch.Tensor, torch.Tensor]:
        """Get TSDF voxels inside objects.

        Return TSDF values, weights and 3D positions of voxels in the TsdfLayer which have
        distance values less than zero.

        Returns
            An Nx2 tensor containing TSDF values and weights respectively and a Nx3 tensor
            containing voxel centers.

        """
        return self.get_voxels_matching_condition(self.get_tsdf_mask_negative_distance)


class ColorLayer(Layer):
    """Specialization for the color layer."""

    def __init__(self, voxel_size_m: float, c_layer: torch.classes.ColorLayer = None):
        """Initialize the color layer.

        Args:
            voxel_size_m: Size of a voxel in meters.
            c_layer: Optional c-layer to wrap.
        """
        super().__init__(voxel_size_m, c_layer=c_layer, torch_class_name='ColorLayer')

    @staticmethod
    def num_elements_per_voxel() -> int:
        """Return the number of elements per voxel."""
        return 3


class OccupancyLayer(Layer):
    """Specialization for the occupancy layer."""

    @staticmethod
    def num_elements_per_voxel() -> int:
        """Return the number of elements per voxel."""
        return 1


class EsdfLayer(Layer):
    """Specialization for the ESDF layer."""

    @staticmethod
    def num_elements_per_voxel() -> int:
        """Return the number of elements per voxel."""
        return 4    # x, y, z, distance


class FeatureLayer(Layer):
    """Specialization for the feature layer."""

    def __init__(self, voxel_size_m: float, c_layer: torch.classes.FeatureLayer = None):
        """Initialize the feature layer.

        Args:
            voxel_size_m: Size of a voxel in meters.
            c_layer: Optional c-layer to wrap.
        """
        super().__init__(
            voxel_size_m,
        # Num elements per voxel is F feature elements + 1 weight. This is the size return by the
        # block getters. TODO(dtingdahl) The query functions in mapper returns features and weigths
        # separately. We should do the same here in order to harmonize the interfaces.
            c_layer=c_layer,
            torch_class_name='FeatureLayer')

    @staticmethod
    def num_elements_per_voxel() -> int:
        """Return the number of elements per voxel."""
        return constants.feature_array_num_elements() + 1    # +1 for the weight


# TODO(dtingdahl) Replace with GPU-accelerated version from core library
def convert_layer_to_dense_tensor(
        layer: Union[TsdfLayer, FeatureLayer],
        unobserved_value: float = 0.0,
        aabb_min_m: Optional[torch.Tensor] = None,
        aabb_max_m: Optional[torch.Tensor] = None) -> Tuple[torch.Tensor, torch.Tensor]:
    """Convert a sparse tsdf or feature layer into a dense tensor.

    Takes a sparse layer representation (e.g., TSDFLayer or FeatureLayer)
    and creates a dense grid tensor that contains only the layer's values
    (e.g., TSDF or feature values) without including any weight information.

    The given aabb range can span a larger area than the layer's workspace. In this case,
    the output tensor is filled with the given unobserved value.

    Args:
        layer: The layer to be converted
        unobserved_value: The value to fill for unobserved voxel in the dense grid.
        aabb_min_m: The minimum extents (blockwise inclusive) of the axis-aligned
            bounding box (AABB) in meters. If not provided, the minimum extend will be determined
            automatically using the layer's `get_block_limits` method.
        aabb_max_m: The maximum extents (blockwise inclusive) of the axis-aligned
            bounding box (AABB) in meters. If not provided, the maximum extent will be determined
            automatically using the layer's `get_block_limits` method.

    Returns
    -------
        Tuple[torch.Tensor, torch.Tensor]:
          - A dense grid tensor with the following shape:
              - `(H, W, D, 1)` for `TsdfLayer`
              - `(H, W, D, F)` for `FeatureLayer` where F is number of features elements.
          - A (H, W, D, 3) tensor containing voxel centers.

    """
    if aabb_min_m is None or aabb_max_m is None:
        aabb_min_block_indices, aabb_max_block_indices = layer.get_block_limits()
    else:
        # Compute the needed block ranges, inclusive on the minimum and maximum extend in meters.
        aabb_min_block_indices = torch.floor(aabb_min_m / layer.block_dim_in_voxels /
                                             layer.voxel_size()).to(torch.int)
        aabb_max_block_indices = torch.ceil(aabb_max_m / layer.block_dim_in_voxels /
                                            layer.voxel_size()).to(torch.int)

    # Initialize the inclusive (including last block) aabb range.
    aabb_range_in_blocks = aabb_max_block_indices - aabb_min_block_indices + torch.ones_like(
        aabb_min_block_indices)
    aabb_range_in_voxels = aabb_range_in_blocks * layer.block_dim_in_voxels

    if isinstance(layer, TsdfLayer):
        # TODO(cvolk): Update once we are able to return voxel data as separate arrays to
        # not hardcode this value here.
        layer_value_depth = 1
    elif isinstance(layer, FeatureLayer):
        # Substract one element for the weight.
        # TODO(cvolk): Update once we are able to return voxel data as separate arrays to
        # not hardcode this value here.
        layer_value_depth = layer.num_elements_per_voxel() - 1
    else:
        raise TypeError(f'Unsupported layer type to convert to dense tensor: {type(layer)}')

    # The output tensor spans the requested aabb range and is allowed to be larger than
    # the global aabb workspace of the layer.
    out_tensor = torch.full(aabb_range_in_voxels.tolist() + [layer_value_depth],
                            fill_value=unobserved_value,
                            dtype=torch.float32,
                            device='cuda')

    # Iterate over the requested aabb range.
    for i in range(aabb_range_in_blocks[0]):
        for j in range(aabb_range_in_blocks[1]):
            for k in range(aabb_range_in_blocks[2]):
                local_tensor_block_index = torch.tensor([i, j, k])
                local_tensor_voxel_index = local_tensor_block_index * layer.block_dim_in_voxels
                x_start = local_tensor_voxel_index[0].item()
                y_start = local_tensor_voxel_index[1].item()
                z_start = local_tensor_voxel_index[2].item()

                # Map from the local tensor index to the global layer index.
                global_layer_block_index = local_tensor_block_index + aabb_min_block_indices
                block_tensor = layer.get_block_at_index(
                    global_layer_block_index.type(torch.IntTensor))

                # Only write if we have a valid block.
                if block_tensor is not None:
                    out_tensor[x_start:(x_start + layer.block_dim_in_voxels),
                               y_start:(y_start + layer.block_dim_in_voxels),
                               z_start:(z_start + layer.block_dim_in_voxels
                                        )] = block_tensor[:, :, :, :layer_value_depth]

    # Generate the voxel center grid
    min_voxel_index = aabb_min_block_indices * layer.block_dim_in_voxels
    max_voxel_index = (aabb_max_block_indices + 1) * layer.block_dim_in_voxels
    x_range = torch.arange(min_voxel_index[0], max_voxel_index[0], device='cuda')
    y_range = torch.arange(min_voxel_index[1], max_voxel_index[1], device='cuda')
    z_range = torch.arange(min_voxel_index[2], max_voxel_index[2], device='cuda')
    x_grid, y_grid, z_grid = torch.meshgrid(x_range, y_range, z_range)
    voxel_index_grid = torch.stack([x_grid, y_grid, z_grid], dim=-1)
    voxel_center_grid = (voxel_index_grid + 0.5) * layer.voxel_size()

    assert out_tensor.shape[:-1] == voxel_center_grid.shape[:-1]

    return out_tensor, voxel_center_grid
