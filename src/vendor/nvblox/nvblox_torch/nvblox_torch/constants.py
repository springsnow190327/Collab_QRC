#
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
import torch

from nvblox_torch.lib.utils import get_nvblox_torch_class


class _Constants:
    """Collection of nvblox constants."""

    def __init__(self, c_constants: torch.classes.Constants = None) -> None:
        if c_constants is None:
            self._c_constants = get_nvblox_torch_class('Constants')()
        else:
            self._c_constants = c_constants

    def feature_array_num_elements(self) -> int:
        return self._c_constants.feature_array_num_elements()

    def feature_array_element_size(self) -> int:
        return self._c_constants.feature_array_element_size()

    def esdf_unknown_distance(self) -> float:
        return self._c_constants.esdf_unknown_distance()


# Instantiate at module level
constants = _Constants()
