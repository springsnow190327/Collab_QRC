#!/bin/env python3
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
"""Regression test for plotting of timing data."""

import tempfile
import json
import argparse
from pathlib import Path
import os

from nvblox_plot_timing_data.__main__ import main

SCRIPT_DIR = Path(__file__).parent.resolve()
LOGFILE_PATHS = [os.path.join(SCRIPT_DIR, 'logfile1.txt'), os.path.join(SCRIPT_DIR, 'logfile2.txt')]
BASELINE_PATH = os.path.join(SCRIPT_DIR, 'baseline.json')


def read_json(path: str) -> dict:
    """Load and return a json file."""
    with open(path, 'r', encoding='utf-8') as fp:
        return json.load(fp)


def test_plot_timing_data() -> None:
    """End-to-end test of replica benchmark."""
    with tempfile.TemporaryDirectory() as tmp_dir:
        # Run plotting and data as json
        output_path = os.path.join(tmp_dir, 'output.json')
        args = argparse.Namespace(input_logfiles=LOGFILE_PATHS,
                                  mode='timings',
                                  label='mean_time_s',
                                  regexp='.*color.*|.*depth.*',
                                  max_namespace_level=2,
                                  output_html=None,
                                  output_json=output_path)
        main(args)

        generated = read_json(output_path)
        baseline = read_json(BASELINE_PATH)
        print(baseline)

        assert generated == baseline, 'Plotted data is different from baseline.'


if __name__ == '__main__':
    test_plot_timing_data()
