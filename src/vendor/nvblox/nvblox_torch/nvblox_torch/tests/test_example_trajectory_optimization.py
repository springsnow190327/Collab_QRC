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

import numpy as np
import re
import nvblox_torch.examples.gradients.trajectory_optimization_example as example
from nvblox_torch.tests.helpers.mock_visualizer import MockO3dVisualizer
from unittest.mock import patch

EXPECTED_NUM_COSTS = 500


def check_costs(text: str, tag: str) -> None:
    costs = np.array(re.findall(fr'{tag}:\s*([0-9]+\.[0-9]+)', text), dtype=float)
    assert len(costs) == EXPECTED_NUM_COSTS
    assert not np.all(costs == 0)
    assert not np.all(costs[0] == costs)


@patch(
    'nvblox_torch.examples.gradients.trajectory_optimization_example.o3d.visualization.Visualizer',
    MockO3dVisualizer)
def test_trajectory_optimization_example() -> None:
    # Run the example, redirecting stdout to a buffer
    buffer = io.StringIO()
    with redirect_stdout(buffer):
        print('Running example')
        assert example.main(visualize=True) == 0
    # Check that the optimization started and stopped.
    assert 'Iteration 0' in buffer.getvalue()
    assert 'Iteration 499' in buffer.getvalue()
    assert 'Done' in buffer.getvalue()

    # Check that the costs are sensible
    check_costs(buffer.getvalue(), 'Collision cost')
    check_costs(buffer.getvalue(), 'Boundary cost')
    check_costs(buffer.getvalue(), 'Stretching cost')
