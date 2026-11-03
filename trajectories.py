"""Sample joint-space trajectories. Angles are radians, time is seconds."""

import numpy as np


def quintic_trajectory(start, end, duration, sample_period=0.02):
    """Return time, position, velocity, and acceleration arrays.

    Each joint starts/ends at rest with zero acceleration. All joints finish
    together. Output joint arrays have shape (samples, joints). Sampling uses
    equal intervals no larger than sample_period and includes both endpoints.
    This generates references only; it does not enforce hardware limits.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    if start.ndim != 1 or start.size == 0 or end.shape != start.shape:
        raise ValueError("Start and end must be matching, nonempty joint vectors")
    if not np.all(np.isfinite(start)) or not np.all(np.isfinite(end)):
        raise ValueError("Joint angles must be finite")
    if any(not np.isfinite(x) or x <= 0 for x in (duration, sample_period)):
        raise ValueError("Duration and sample period must be positive and finite")

    intervals = max(1, int(np.ceil(duration / sample_period)))
    time = np.linspace(0, duration, intervals + 1)
    u = time / duration  # Normalized time: 0 at the start, 1 at the end.

    # These coefficients satisfy s(0)=0, s(1)=1, s'(0)=s'(1)=s''(0)=s''(1)=0.
    progress = 10 * u**3 - 15 * u**4 + 6 * u**5
    progress_velocity = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
    progress_acceleration = (60 * u - 180 * u**2 + 120 * u**3) / duration**2

    change = end - start
    return {
        "time_s": time,
        "position_rad": start + progress[:, None] * change,
        "velocity_rad_s": progress_velocity[:, None] * change,
        "acceleration_rad_s2": progress_acceleration[:, None] * change,
    }
