import numpy as np

from door_task.perception.inspector import temporal_pool


def test_pool_single_frame_returns_normalized():
    p = np.array([0.1, 0.6, 0.3])
    out = temporal_pool([p])
    assert np.isclose(out.sum(), 1.0)
    assert out.argmax() == 1


def test_pool_stable_beats_noisy():
    # 5 stable "red button" detections vs 1 huge "door" outlier
    stable = np.array([0.0, 0.6, 0.4])   # index 1 wins each frame
    outlier = np.array([0.9, 0.05, 0.05])  # index 0 spikes once
    pooled = temporal_pool([stable, stable, stable, stable, outlier])
    # The stable class should still win the pooled argmax
    assert pooled.argmax() == 1


def test_pool_window_of_zeros_is_uniform():
    n = 3
    zero = np.zeros(n)
    out = temporal_pool([zero, zero])
    assert np.allclose(out, np.ones(n) / n)


def test_pool_renormalizes():
    a = np.array([0.2, 0.4, 0.4])
    b = np.array([0.1, 0.8, 0.1])
    out = temporal_pool([a, b])
    assert np.isclose(out.sum(), 1.0)
    assert out.argmax() == 1
