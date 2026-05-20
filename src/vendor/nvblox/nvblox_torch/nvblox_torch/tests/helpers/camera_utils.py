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
import math
import torch


def make_camera_intrinsics_matrix(h_fov: float, height: float, width: float, device: str) \
        -> torch.Tensor:
    """
    Create a camera intrinsics matrix based on the horizontal field of view and image dimensions.

    Args:
        h_fov (float): Horizontal field of view in degrees.
        height (float): Image height in pixels.
        width (float): Image width in pixels.
        device (str): The device on which to create the tensor (e.g., 'cuda:0' or 'cpu').

    Returns:
        torch.Tensor: A 3x3 tensor representing the camera intrinsics matrix:
            [[fx,  0, cx],
             [ 0, fy, cy],
             [ 0,  0,  1]],
        where fx and fy are the focal lengths in pixels, and (cx, cy) is the image center.
    """
    fx = width / (2 * math.tan(math.radians(h_fov) / 2))
    fy = fx    # square pixels
    return torch.tensor([[fx, 0, width / 2], [0, fy, height / 2], [0, 0, 1]], device=device)
