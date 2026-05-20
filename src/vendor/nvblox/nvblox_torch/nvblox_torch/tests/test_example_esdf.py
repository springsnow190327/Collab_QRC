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
import sys
import re
from unittest.mock import patch

import nvblox_torch.examples.esdf.esdf as example
from .helpers.data import get_sun3d_test_data_dir
from nvblox_torch.tests.helpers.mock_visualizer import MockO3dVisualizer


def get_max_num_valid_queries(buffer: io.StringIO) -> int:
    """Gets the maximum number of valid queries from the output of the example."""
    lines = buffer.getvalue().split('\n')
    query_stdout_lines = [line for line in lines if 'Slice at index' in line]
    num_valid_queries = []
    for line in query_stdout_lines:
        match = re.search(r'had (\d+)', line)
        if match:
            num_valid_queries.append(int(match.group(1)))
    max_num_valid_queries = max(num_valid_queries)
    return max_num_valid_queries


@patch('nvblox_torch.examples.esdf.esdf.o3d.visualization.Visualizer', MockO3dVisualizer)
def test_esdf_example() -> None:
    sun3d_test_data_dir = get_sun3d_test_data_dir()
    assert sun3d_test_data_dir.exists()

    # We pass/get CLI input/output by:
    # - Pass CLI args by using the unittest.mock.patch.
    # - Redirect stdout to a buffer for inspection.
    test_args = [
        'esdf_example.py',
        '--dataset_path',
        str(sun3d_test_data_dir),
    ]
    buffer = io.StringIO()
    with (patch.object(sys, 'argv', test_args), redirect_stdout(buffer)):
        assert example.main(visualize=True) == 0
    assert 'Integrating frame: 4' in buffer.getvalue()
    assert 'Done' in buffer.getvalue()

    # Check that at least one slice resulted in valid queries.
    num_valid_queries = get_max_num_valid_queries(buffer)
    print(f'The test resulted in {num_valid_queries} valid queries.')
    assert num_valid_queries > 0
