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


def truncate_distances(distances: torch.Tensor, clip_value: float) -> torch.Tensor:
    assert clip_value >= 0.0
    output = distances.clone()
    output = torch.max(output, torch.tensor([-clip_value], device='cuda'))
    output = torch.min(output, torch.tensor([clip_value], device='cuda'))
    return output
