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
from typing import List, Optional
import sys

from unittest.mock import patch

import nvblox_torch.examples.reconstruction.sun3d as example
from .helpers.data import get_sun3d_test_data_dir
from .helpers.data import get_orbbec_test_data_dir
import pathlib


def run_sun3d_example(data_dir: pathlib.Path,
                      sequence_name: str = 'seq-01',
                      additional_args: Optional[List[str]] = None) -> None:
    sun3d_test_data_dir = get_sun3d_test_data_dir()
    assert sun3d_test_data_dir.exists()

    # We pass/get CLI input/output by:
    # - Pass CLI args by using the unittest.mock.patch.
    # - Redirect stdout to a buffer for inspection.
    test_args = [
        'sun3d_example.py',
        '--dataset_path',
        str(data_dir),
        '--sequence_name',
        sequence_name,
        '--dont_visualize',
    ]
    if additional_args is not None:
        test_args += additional_args
    buffer = io.StringIO()
    with patch.object(sys, 'argv', test_args):
        with redirect_stdout(buffer):
            assert example.main() == 0
    assert 'Integrating frame: 4' in buffer.getvalue()
    assert 'Done' in buffer.getvalue()


def test_sun3d_example() -> None:
    run_sun3d_example(data_dir=get_sun3d_test_data_dir())


def test_sun3d_example_with_feature_mapping() -> None:
    run_sun3d_example(data_dir=get_sun3d_test_data_dir(),
                      additional_args=['--deep_feature_mapping'])


def test_orbbec_example() -> None:
    run_sun3d_example(data_dir=get_orbbec_test_data_dir(), sequence_name='g1_sim')
