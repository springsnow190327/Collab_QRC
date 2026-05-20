#! /usr/bin/env python3
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

from nvblox_torch.datasets.sun3d_dataset import Sun3dDataset
from nvblox_torch.mapper import Mapper
from nvblox_torch.sensor import Sensor
from nvblox_torch.constants import constants
from nvblox_torch.tests.helpers.data import get_sun3d_test_data_dir
from nvblox_torch.timer import Timer, timer_status_string

import sys
import torch
from typing import Dict

VOXEL_SIZE_M = 0.05
NUM_DATASET_ITERATIONS = 10


def process_frame(mapper: Mapper, data: Dict[str, torch.Tensor]) -> None:
    """Process and time single frame of data."""
    depth: torch.Tensor = data['depth'][0].squeeze(-1)
    rgb: torch.Tensor = data['rgb'][0]
    pose: torch.Tensor = data['pose'][0].cpu()
    sensor: Sensor = data['sensor'][0]

    feature_frame = torch.rand(rgb.shape[0],
                               rgb.shape[1],
                               constants.feature_array_num_elements(),
                               dtype=torch.float16,
                               device=rgb.device)

    # Integrate data
    with Timer('add_depth_frame'):
        mapper.add_depth_frame(depth, pose, sensor)
    with Timer('add_color_frame'):
        mapper.add_color_frame(rgb, pose, sensor)
    with Timer(f'add_feature_frame (dim: {constants.feature_array_num_elements()})'):
        mapper.add_feature_frame(feature_frame, pose, sensor)

    # Updates
    with Timer('update_color_mesh'):
        mapper.update_color_mesh()
    with Timer('update_feature_mesh'):
        mapper.update_feature_mesh()
    with Timer('update_esdf'):
        mapper.update_esdf()

    # Getters
    with Timer('get_color_mesh'):
        color_mesh = mapper.get_color_mesh()
    with Timer('get_feature_mesh'):
        feature_mesh = mapper.get_feature_mesh()

    # Decay
    with Timer('decay'):
        mapper.decay()

    # Mesh getters
    with Timer('color_mesh/vertices'):
        color_mesh.vertices()
    with Timer('color_mesh/appearance'):
        color_mesh.vertex_appearances()
    with Timer('color_mesh/triangles'):
        color_mesh.triangles()
    with Timer('feature_mesh/vertices'):
        feature_mesh.vertices()
    with Timer('feature_mesh/appearance'):
        feature_mesh.vertex_appearances()
    with Timer('feature_mesh/triangles'):
        feature_mesh.triangles()


def run_benchmark() -> None:
    """Run the benchmark."""
    dataset_dir = str(get_sun3d_test_data_dir())
    dataloader = Sun3dDataset.create_dataloader(root_dir=dataset_dir, sequence_name='seq-01')

    for _ in range(NUM_DATASET_ITERATIONS):
        mapper = Mapper(voxel_sizes_m=VOXEL_SIZE_M)
        for _, data in enumerate(dataloader):
            process_frame(mapper, data)


def main() -> int:
    run_benchmark()
    print(timer_status_string())
    return 0


if __name__ == '__main__':
    sys.exit(main())
