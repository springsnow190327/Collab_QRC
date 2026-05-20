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

import matplotlib.pyplot as plt
import torch

from nvblox_torch.examples.utils.scenes import get_single_sphere_scene_mapper
from nvblox_torch.rendering import render_depth_image, render_depth_and_color_image

# pylint: disable=C0103

WIDTH = 640
HEIGHT = 480
FU = 570
FV = 570
CU = 320
CV = 240
MAX_RAY_LENGTH = 20.0
MAX_STEPS = 100


def test_rendering() -> None:
    # Test scene
    mapper = get_single_sphere_scene_mapper(center=[0.0, 0.0, 3.0], )

    K = torch.tensor([[FU, 0, CU], [0, FV, CV], [0, 0, 1]], dtype=torch.float32)
    T_W_C = torch.eye(4, dtype=torch.float32)

    tsdf_layer = mapper.tsdf_layer_view()
    depth_image = render_depth_image(
        tsdf_layer,
        T_W_C,
        K,
        HEIGHT,
        WIDTH,
        MAX_RAY_LENGTH,
        MAX_STEPS,
    )

    # Check that we get some non-background depths
    non_background_depths = depth_image[depth_image > 0.0]
    assert len(non_background_depths) > 0

    # Allocate color blocks, and set all the color blocks to red
    block_indices = tsdf_layer.get_all_block_indices()
    color_layer = mapper.color_layer_view()
    for block_index in block_indices:
        color_layer.allocate_block_at_index(block_index)
        color_block = color_layer.get_block_at_index(block_index)
        color_block[..., 0] = 255
    assert tsdf_layer.num_allocated_blocks() == color_layer.num_allocated_blocks()

    # Color + Depth
    depth_image, color_image = render_depth_and_color_image(
        tsdf_layer,
        color_layer,
        T_W_C,
        K,
        HEIGHT,
        WIDTH,
        MAX_RAY_LENGTH,
        MAX_STEPS,
    )

    plot = False
    if plot:
        _, (ax1, ax2) = plt.subplots(1, 2)
        ax1.imshow(depth_image.cpu().numpy())
        ax1.set_title('Depth')
        ax2.imshow(color_image.cpu().numpy())
        ax2.set_title('Color')
        plt.show()

    # Check that the color image is red where the rays hit the sphere
    non_background_colors = color_image[color_image > 128]
    assert len(non_background_colors) > 0
    assert torch.all(non_background_colors == 255)
