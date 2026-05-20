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

# pylint: disable=invalid-name

import torch


def look_at_to_rotation_matrix(center_W: torch.Tensor, look_at_point_W: torch.Tensor,
                               camera_up_W: torch.Tensor) -> torch.Tensor:
    """Generate a rotation matrix from a look-at view-point description.

    Args:
        center_W (torch.Tensor): The eye center in the world frame.
        look_at_point_W (torch.Tensor): The point the eye looks at in the world frame.
        camera_up_W (torch.Tensor): The up direction of the camera frame in the world frame.

    Returns:
        torch.Tensor: The 3x3 rotation matrix R_W_C rotating from camera to world.
    """
    assert len(center_W) == 3
    assert len(look_at_point_W) == 3
    assert len(camera_up_W) == 3
    # The camera-z is the unit vector pointing from the center to the look at point.
    z_vec = look_at_point_W - center_W
    z_vec = z_vec / torch.norm(z_vec)
    # Camera up is not necessarily perpendicular to z_vec, so use it to calculate x_vec
    x_vec = -1.0 * torch.cross(z_vec, camera_up_W)
    x_vec = x_vec / torch.norm(x_vec)
    # Calculate remaining vector
    y_vec = torch.cross(z_vec, x_vec)
    # Use the unit vectors to form the rotation matrix
    R_W_C = torch.hstack((x_vec.view((3, 1)), y_vec.view((3, 1)), z_vec.view((3, 1))))
    assert R_W_C.shape == torch.Size([3, 3])
    return R_W_C


def look_at_to_transformation_matrix(center_W: torch.Tensor, look_at_point_W: torch.Tensor,
                                     camera_up_W: torch.Tensor) -> torch.Tensor:
    """Generate a transformation matrix from a look-at view-point description.

    Args:
        center_W (torch.Tensor): The eye center in the world frame.
        look_at_point_W (torch.Tensor): The point the eye looks at in the world frame.
        camera_up_W (torch.Tensor): The up direction of the camera frame in the world frame.

    Returns:
        torch.Tensor: The 4x4 transformation matrix R_W_C rotating from camera to world.
    """
    R_W_C = look_at_to_rotation_matrix(center_W, look_at_point_W, camera_up_W)
    t_W_C = center_W
    T_W_C = torch.vstack((torch.hstack((R_W_C, t_W_C.view(
        (3, 1)))), torch.tensor([0.0, 0.0, 0.0, 1.0], device=center_W.device)))
    assert T_W_C.shape == torch.Size([4, 4])
    return T_W_C
