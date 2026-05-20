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

import pathlib


# The current script dir
def get_sun3d_test_data_dir() -> pathlib.Path:
    script_dir = pathlib.Path(__file__).parent
    sun3d_data_dir = script_dir / '..' / 'data' / '3dmatch'
    return sun3d_data_dir


def get_orbbec_test_data_dir() -> pathlib.Path:
    script_dir = pathlib.Path(__file__).parent
    orbbec_data_dir = script_dir / '..' / 'data' / 'orbbec'
    return orbbec_data_dir
