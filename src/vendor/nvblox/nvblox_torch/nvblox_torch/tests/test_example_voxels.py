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
from contextlib import redirect_stdout
import io
from typing import List
from unittest.mock import patch

import re
import nvblox_torch.examples.voxels.voxels as example
from nvblox_torch.tests.helpers.mock_visualizer import mock_draw_geometries

NUMBER_OF_VOXELS_INSIDE_SPHERE = 33552


def get_num_voxels_inside_sphere(buffer: io.StringIO) -> List[int]:
    """Gets the maximum number of valid queries from the output of the example."""
    lines = buffer.getvalue().split('\n')
    query_stdout_lines = [line for line in lines if 'Found' in line]
    num_voxels = []
    for line in query_stdout_lines:
        match = re.search(r'Found (\d+)', line)
        if match:
            num_voxels.append(int(match.group(1)))
    return num_voxels


@patch('nvblox_torch.examples.voxels.voxels.o3d.visualization.draw_geometries',
       mock_draw_geometries)
def test_voxels_example() -> None:
    # Run the example, redirecting stdout to a buffer
    buffer = io.StringIO()
    with redirect_stdout(buffer):
        print('Running example')
        assert example.main(visualize=True) == 0
    # Check that the optimization started and stopped.
    assert 'Visualizing voxels using sparse access...' in buffer.getvalue()
    assert 'Visualizing voxels using dense access...' in buffer.getvalue()
    assert 'Done' in buffer.getvalue()

    # Check that sparse and dense access give the same (correct) number of voxels inside the sphere
    num_voxels_inside_sphere = get_num_voxels_inside_sphere(buffer)
    print(f'Number of voxels inside sphere: {num_voxels_inside_sphere}')
    assert len(num_voxels_inside_sphere) == 2
    assert num_voxels_inside_sphere[0] == NUMBER_OF_VOXELS_INSIDE_SPHERE
    assert num_voxels_inside_sphere[1] == NUMBER_OF_VOXELS_INSIDE_SPHERE
