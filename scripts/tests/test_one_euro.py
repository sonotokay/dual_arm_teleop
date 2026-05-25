import random

import pytest

from one_euro import OneEuro


def test_first_sample_passes_through():
    f = OneEuro()
    assert f(0.3, 0.0) == 0.3


def test_zero_dt_returns_previous():
    f = OneEuro()
    f(0.5, 0.0)
    # Same timestamp -> no update, return previous filtered value.
    assert f(99.0, 0.0) == 0.5


def test_negative_dt_returns_previous():
    f = OneEuro()
    f(0.5, 1.0)
    assert f(99.0, 0.5) == 0.5


def test_steady_signal_converges_to_value():
    f = OneEuro(min_cutoff=1.0, beta=0.0)
    t = 0.0
    out = None
    for _ in range(200):
        out = f(0.42, t)
        t += 1.0 / 60.0  # 60 Hz
    assert out == pytest.approx(0.42, abs=1e-6)


def test_smooths_zero_mean_noise():
    """A steady signal corrupted by noise should be smoothed substantially."""
    random.seed(0)
    f = OneEuro(min_cutoff=1.0, beta=0.0)
    t = 0.0
    samples_in = []
    samples_out = []
    for _ in range(300):
        noisy = 0.5 + random.gauss(0.0, 0.05)
        samples_in.append(noisy)
        samples_out.append(f(noisy, t))
        t += 1.0 / 60.0

    # Compare variance over the last 100 samples (filter has converged).
    def var(xs):
        m = sum(xs) / len(xs)
        return sum((x - m) ** 2 for x in xs) / len(xs)

    assert var(samples_out[-100:]) < 0.25 * var(samples_in[-100:])


def test_tracks_ramp_with_minimal_lag_when_beta_high():
    """High beta = adaptive cutoff rises quickly under motion -> low lag."""
    f = OneEuro(min_cutoff=1.0, beta=1.0)
    t = 0.0
    out = None
    for i in range(120):  # 2 s @ 60 Hz
        # Ramp from 0 to 1 over 2 s.
        x = i / 120.0
        out = f(x, t)
        t += 1.0 / 60.0
    # At the end of the ramp the filtered output should be close to the input.
    assert out == pytest.approx(1.0, abs=0.05)


def test_reset_clears_state():
    f = OneEuro()
    f(0.5, 0.0)
    f(0.6, 0.1)
    f.reset()
    # After reset, the next sample should pass straight through again.
    assert f(0.9, 0.0) == 0.9
