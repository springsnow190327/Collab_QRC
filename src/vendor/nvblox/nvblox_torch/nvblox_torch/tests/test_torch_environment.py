#!/usr/bin/env python
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


def test_import_torchvision() -> None:
    """Importing torchvision fails on Jetson if a non-CUDA enabled version is installed"""

    # pylint: disable=unused-import
    # pylint: disable=import-outside-toplevel
    import torchvision


def test_cuda_enabled() -> None:
    assert torch.cuda.is_available(), 'CUDA is not available'
