#!/usr/bin/python3

#
# Copyright 2022 NVIDIA CORPORATION
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import subprocess
from typing import Dict, Optional


def run_single(executable_path: str,
               dataset_base_path: str,
               timing_output_path: Optional[str] = None,
               num_frames: Optional[int] = None,
               flags: Optional[Dict] = None) -> None:
    """set up and run 3DMatch reconstruction"""
    # Launch the 3DMatch reconstruction
    args_string = dataset_base_path
    if timing_output_path:
        args_string += ' --timing_output_path ' + timing_output_path
    if num_frames:
        args_string += ' --num_frames ' + str(num_frames)
    if flags:
        for key, value in flags.items():
            args_string += ' --' + key + ' ' + str(value)

    run_string = executable_path + ' ' + args_string
    print('Running 3DMatch as:\n ' + run_string)
    subprocess.call(run_string, shell=True)


def run_multiple(num_runs: int,
                 executable_path: str,
                 dataset_base_path: str,
                 timing_output_dir: str,
                 num_frames: Optional[int] = None,
                 warmup_run: Optional[bool] = True,
                 flags: Optional[Dict] = None) -> None:
    """set up and run 3DMatch reconstruction multiple times"""
    run_indices = list(range(num_runs))
    if warmup_run:
        run_indices.insert(0, 0)
    for run_idx in run_indices:
        print(f'Run: {run_idx}')

        timing_output_name = f'run_{run_idx}.txt'
        timing_output_path = os.path.join(timing_output_dir, timing_output_name)

        run_single(executable_path, dataset_base_path, timing_output_path, num_frames, flags)
