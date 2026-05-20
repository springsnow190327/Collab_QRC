# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#

import time

import numpy as np
import pytest

from nvblox_torch.timer import Accumulator, Timer, get_last_time, timer_status_string

SLEEP_S = 1


def test_accumulator() -> None:
    acc = Accumulator()
    values = [1, 3, 7]
    for val in values:
        acc.accumulate(val)
    assert np.isclose(acc.mean(), np.mean(values))


def test_timing() -> None:
    t = Timer('test_timer')
    time.sleep(SLEEP_S)
    elapsed = t.stop()
    assert elapsed >= SLEEP_S

    assert elapsed == get_last_time('test_timer')


def test_context() -> None:
    with Timer('test_timer') as _:
        time.sleep(SLEEP_S)

    string = timer_status_string()
    assert 'test_timer' in string

    print(string)


def _test_throw_internal() -> None:
    with Timer('test_timer'):
        # pylint: disable=undefined-variable
        function_that_does_not_exist()    # type: ignore[name-defined]
        # pylint: enable=undefined-variable


def test_throw() -> None:
    with pytest.raises(NameError):
        _test_throw_internal()
