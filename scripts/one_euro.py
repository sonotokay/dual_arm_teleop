"""One-Euro filter (Casiez, Roussel, Vogel, CHI 2012).

A first-order low-pass whose cutoff frequency scales with signal speed:
- slow signal -> low cutoff -> heavy smoothing (kills jitter at rest)
- fast signal -> high cutoff -> light smoothing (keeps lag low under motion)

Tuning:
- min_cutoff (Hz): baseline cutoff when the signal is still. Lower = smoother.
- beta:           how aggressively cutoff rises with speed. Higher = less lag.
- d_cutoff (Hz):  cutoff of the inner derivative estimator. 1.0 is fine.
"""
import math


class OneEuro:
    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.05,
                d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x_prev: float | None = None
        self._dx_prev: float = 0.0
        self._t_prev: float | None = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self) -> None:
        self._x_prev = None
        self._dx_prev = 0.0
        self._t_prev = None

    def __call__(self, x: float, t: float) -> float:
        if self._t_prev is None:
            self._t_prev = t
            self._x_prev = x
            return x
        dt = t - self._t_prev
        # Non-positive dt would blow up the alpha computation. Skip the update.
        if dt <= 0.0:
            return self._x_prev  # type: ignore[return-value]

        dx = (x - self._x_prev) / dt  # type: ignore[operator]
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self._x_prev  # type: ignore[operator]

        self._x_prev = x_hat
        self._dx_prev = dx_hat
        self._t_prev = t
        return x_hat
