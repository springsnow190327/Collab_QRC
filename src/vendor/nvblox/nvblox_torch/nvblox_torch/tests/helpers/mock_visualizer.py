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

from typing import Any


class MockO3dViewControl:
    """Mock of Open3d ViewControl that does nothing"""

    def convert_from_pinhole_camera_parameters(self, _: Any, allow_arbitrary: bool = False) -> None:
        pass

    def convert_to_pinhole_camera_parameters(self) -> None:
        return None


class MockO3dVisualizer:
    """Mock of Open3d Visualizer that does nothing"""

    def create_window(self, _: Any = None) -> None:
        pass

    def add_geometry(self, _: Any) -> None:
        pass

    def get_view_control(self) -> MockO3dViewControl:
        return MockO3dViewControl()

    def clear_geometries(self) -> None:
        pass

    def update_renderer(self) -> None:
        pass

    def poll_events(self) -> None:
        pass

    def destroy_window(self) -> None:
        pass

    def run(self) -> None:
        pass


def mock_draw_geometries(_: Any) -> None:
    pass
