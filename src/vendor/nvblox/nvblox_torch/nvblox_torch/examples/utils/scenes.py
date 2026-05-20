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
from typing import Optional
import math

import torch

from nvblox_torch.datasets.sun3d_dataset import Sun3dDataset
from nvblox_torch.mapper import Mapper
from nvblox_torch.mapper_params import MapperParams, ProjectiveIntegratorParams, EsdfIntegratorParams
from nvblox_torch.sensor import Sensor
from nvblox_torch.scene import Scene


def get_single_sphere_scene_mapper(
    scene_size_m: float = 10.0,
    voxel_size_m: float = 0.05,
    center: Optional[list[float]] = None,
    radius_m: float = 1.0,
) -> Mapper:
    """Get a mapper containing a scene with a single sphere at a given position.

    Args:
        scene_size_m (float): The side length of the bounding cube spanning the scene.
        voxel_size_m (float): The size of the voxels.
        center (list[float]): The 3D center of the sphere. Defaults to [0.0, 0.0, 0.0].
        radius_m (float): The radius of the sphere.
    """
    if center is None:
        center = [0.0, 0.0, 0.0]
    assert len(center) == 3
    scene = Scene()
    scene.set_aabb([-scene_size_m / 2, -scene_size_m / 2, -scene_size_m / 2],
                   [scene_size_m / 2, scene_size_m / 2, scene_size_m / 2])
    scene.add_primitive('sphere', center + [radius_m])

    # Set the max ESDF distance to the diagonal of the scene to
    # propagate distances all over the scene
    esdf_integrator_params = EsdfIntegratorParams()
    esdf_integrator_params.esdf_integrator_max_distance_m = math.sqrt(3) * scene_size_m
    mapper_params = MapperParams()
    mapper_params.set_esdf_integrator_params(esdf_integrator_params)

    # Mapper with the scene inside
    return scene.to_mapper([voxel_size_m])


def get_sun3d_scene_mapper(dataset_path: str,
                           voxel_size_m: float = 0.05,
                           num_frames: Optional[int] = None) -> Mapper:
    """Map a SUN3D scene and return a mapper containing the map.

    Args:
        dataset_path (str): The path to the SUN3D dataset.
        voxel_size_m (float): The size of the voxels.
        num_frames (Optional[int]): The number of frames to process.
            If None, all frames are processed.
    Returns:
        Mapper: A mapper containing the scene.
    """
    # Create the dataset
    dataloader = Sun3dDataset.create_dataloader(root_dir=dataset_path, sequence_name='seq-01')

    # Configure mapper parameters
    projective_integrator_params = ProjectiveIntegratorParams()
    projective_integrator_params.projective_integrator_max_integration_distance_m = 5.0
    mapper_params = MapperParams()
    mapper_params.set_projective_integrator_params(projective_integrator_params)

    # Do some mapping
    mapper = Mapper(
        voxel_sizes_m=voxel_size_m,
        mapper_parameters=mapper_params,
    )
    for idx, data in enumerate(dataloader):
        print(f'Integrating frame: {idx}')

        depth: torch.Tensor = data['depth'][0].squeeze(-1)
        rgb: torch.Tensor = data['rgb'][0]
        pose: torch.Tensor = data['pose'][0].cpu()
        sensor: Sensor = data['sensor'][0]

        mapper.add_depth_frame(depth, pose, sensor)
        mapper.add_color_frame(rgb, pose, sensor)

        if num_frames and idx > num_frames:
            break
    mapper.update_color_mesh()
    mapper.update_esdf()
    print('Done.')
    return mapper
