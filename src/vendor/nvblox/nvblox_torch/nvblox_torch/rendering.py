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
from typing import Tuple

import torch

from nvblox_torch.layer import TsdfLayer, ColorLayer

# pylint: disable=W0212


def render_depth_image(tsdf_layer: TsdfLayer, camera_pose: torch.Tensor, intrinsics: torch.Tensor,
                       height: int, width: int, max_ray_length: float,
                       max_steps: int) -> torch.Tensor:
    """Render a depth image from a TSDF layer using sphere tracing.

    Args:
        tsdf_layer: The TSDF layer to render
        camera_pose: 4x4 camera pose matrix (world to camera transform)
        intrinsics: 3x3 camera intrinsics matrix
        height: Height of output image in pixels
        width: Width of output image in pixels
        max_ray_length: Maximum length of ray to trace in meters
        max_steps: Maximum number of steps to take along ray

    Returns:
        torch.Tensor: Depth image of size (height, width). Pixels with no valid depth
            will have value -1.
    """
    return torch.ops.pynvblox.render_depth_image(tsdf_layer._c_layer, camera_pose, intrinsics,
                                                 height, width, max_ray_length, max_steps)


def render_depth_and_color_image(tsdf_layer: TsdfLayer, color_layer: ColorLayer,
                                 camera_pose: torch.Tensor, intrinsics: torch.Tensor, height: int,
                                 width: int, max_ray_length: float,
                                 max_steps: int) -> Tuple[torch.Tensor, torch.Tensor]:
    """Render depth and color images from TSDF and color layers using sphere tracing.

    Args:
        tsdf_layer: The TSDF layer to render
        color_layer: The color layer to render
        camera_pose: 4x4 camera pose matrix (world to camera transform)
        intrinsics: 3x3 camera intrinsics matrix
        height: Height of output images in pixels
        width: Width of output images in pixels
        max_ray_length: Maximum length of ray to trace in meters
        max_steps: Maximum number of steps to take along ray

    Returns:
        Tuple[torch.Tensor, torch.Tensor]: Tuple containing:
            - Depth image of size (height, width). Pixels with no valid depth will have value -1.
            - Color image of size (height, width, 3). Pixels with no valid color will be black.
    """
    return torch.ops.pynvblox.render_depth_and_color_image(tsdf_layer._c_layer,
                                                           color_layer._c_layer, camera_pose,
                                                           intrinsics, height, width,
                                                           max_ray_length, max_steps)
