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
from typing import Tuple, List
import torch
from nvblox_torch.mapper import Mapper
from nvblox_torch.lib.utils import get_nvblox_torch_class
from nvblox_torch.projective_integrator_types import ProjectiveIntegratorType
from nvblox_torch.mapper_params import MapperParams
from typing import Optional


class Scene:
    """Wrapper around the nvblox Scene class.

    Allows for creating artificial scenes with primitives.
    """

    def __init__(self, c_scene: torch.classes.Scene = None) -> None:
        """Constructor.

        Args:
            c_scene: Optional nvblox Scene object to wrap. If None, a new one will be created.
        """
        if c_scene is None:
            self._c_scene = get_nvblox_torch_class('Scene')()
        else:
            self._c_scene = c_scene

    def set_aabb(self, low: List[float], high: List[float]) -> None:
        """Set the Axis-Aligned Bounding Box (AABB) of the scene.

        Args:
            low: Lower bounds of the AABB.
            high: Upper bounds of the AABB.
        """
        assert len(low) == 3
        assert len(high) == 3
        self._c_scene.set_aabb(low, high)

    def get_aabb(self) -> Tuple[List[float], List[float]]:
        """Get the Axis-Aligned Bounding Box (AABB) of the scene.

        Returns:
            Tuple of lower and upper bounds of the AABB.
        """
        return self._c_scene.get_aabb()

    def add_plane_boundaries(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        """Add plane boundaries to the scene.

        Args:
            x_min: Minimum x-coordinate of the plane boundaries.
            x_max: Maximum x-coordinate of the plane boundaries.
            y_min: Minimum y-coordinate of the plane boundaries.
            y_max: Maximum y-coordinate of the plane boundaries.
        """
        self._c_scene.add_plane_boundaries(x_min, x_max, y_min, y_max)

    def add_ground_level(self, level: float) -> None:
        """Add a ground level to the scene.

        Args:
            level: Height of the ground level.
        """
        self._c_scene.add_ground_level(level)

    def add_ceiling(self, ceiling: float) -> None:
        """Add a ceiling to the scene.

        Args:
            ceiling: Height of the ceiling.
        """
        self._c_scene.add_ceiling(ceiling)

    def add_primitive(self, primitive_type: str, params: List[float]) -> None:
        """Add a primitive to the scene.

        For a cube, the parameters are the center and size.
        For a sphere, the parameters are thecenter and radius.
        For a plane, the parameters are the normal and a point on the plane.

        Args:
            primitive_type: Type of the primitive. One of "cube", "sphere", or "plane".
            params: Parameters of the primitive.
        """
        self._c_scene.add_primitive(primitive_type, params)

    def create_dummy_map(self) -> None:
        """Create a map that's a box with a sphere in the middle."""
        self._c_scene.create_dummy_map()

    def get_primitives_type_list(self) -> List[str]:
        """Get the list of primitive types in the scene."""
        return self._c_scene.get_primitives_type_list()

    def get_c_scene(self) -> torch.classes.Scene:
        """Get the underlying nvblox Scene object."""
        return self._c_scene

    def append_to_mapper(self, mapper: Mapper, mapper_id: int = -1) -> None:
        """Append the scene to a mapper with given ID.

        Args:
            mapper: Mapper to append the scene to.
            mapper_id: ID of the mapper to append the scene to  (-1 for all mappers).
        """
        self._c_scene.to_mapper(mapper.get_c_mapper(), mapper_id)

    def to_mapper(self,
                  voxel_sizes_m: List[float],
                  integrator_types: Optional[List[ProjectiveIntegratorType]] = None,
                  mapper_parameters: MapperParams = MapperParams(),
                  mapper_id: int = -1) -> Mapper:
        """Create a new mapper and append scene to it.

        Args:
            voxel_sizes_m: List of voxel sizes in meters.
            integrator_types: List of integrator types.
            mapper_parameters: Mapper parameters.
            mapper_id: ID of the mapper to append the scene to.
        """
        if integrator_types is None:
            integrator_types = [ProjectiveIntegratorType.TSDF] * len(voxel_sizes_m)
        mapper = Mapper(voxel_sizes_m=voxel_sizes_m,
                        integrator_types=integrator_types,
                        mapper_parameters=mapper_parameters)
        self.append_to_mapper(mapper, mapper_id)

        return mapper
