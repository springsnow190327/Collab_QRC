#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#
from typing import Any
import os

import torch


# get paths
def get_module_path() -> str:
    """Get the path to the nvblox_torch module."""
    path = os.path.dirname(__file__)
    return path


def get_nvblox_py_library_path() -> str:
    """Get the path to the nvblox_torch .so library."""
    path = os.path.join(get_module_path(), 'nvblox_torch/cpp/libpy_nvblox.so')
    return path


def get_nvblox_torch_class(class_name: str) -> Any:
    """Get one of the C++classes wrapped in the nvblox_torch library."""
    torch.classes.load_library(get_nvblox_py_library_path())
    return getattr(torch.classes.pynvblox, class_name)
