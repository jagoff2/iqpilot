"""Decode a plan's current curvature from one spatial body-heading reference.

The caller supplies the selected plan, its actual inference-time vehicle speed,
and its actual preview horizon. Low-speed hold, smoothing, and control limits
remain the caller's responsibility. This module does not extrapolate a plan.
"""

import math

import numpy as np
from numpy.typing import ArrayLike


MIN_DISTANCE_STEP = 1e-4  # metres; avoid ill-conditioned duplicate spatial knots


class SpatialHeadingError(ValueError):
  def __init__(self, reason: str):
    self.reason = reason
    super().__init__(reason)


def _endpoint_slope(h0: float, h1: float, m0: float, m1: float) -> float:
  slope = ((2 * h0 + h1) * m0 - h0 * m1) / (h0 + h1)
  if np.sign(slope) != np.sign(m0):
    return 0.0
  if np.sign(m0) != np.sign(m1) and abs(slope) > 3 * abs(m0):
    return 3 * m0
  return slope


class SpatialHeading:
  """PCHIP heading on the maximal finite, forward-moving prefix of a plan.

  Times, longitudinal velocities, and headings must share the same knots.
  The first knot is the current plan origin. A negative-velocity endpoint is
  excluded even when its trapezoidal interval has positive net displacement:
  that interval contains a reversal. An exact zero endpoint can be included.
  """

  def __init__(self, times: ArrayLike, longitudinal_velocity: ArrayLike, heading: ArrayLike):
    try:
      t, v, yaw = (np.asarray(a, dtype=np.float64) for a in (times, longitudinal_velocity, heading))
    except (TypeError, ValueError) as exc:
      raise SpatialHeadingError("shape") from exc
    if t.ndim != 1 or len(t) < 3 or v.shape != t.shape or yaw.shape != t.shape:
      raise SpatialHeadingError("shape")
    if not all(math.isfinite(float(a[0])) for a in (t, v, yaw)):
      raise SpatialHeadingError("invalidOrigin")
    if v[0] <= 0:
      raise SpatialHeadingError("initialNonpositiveVelocity")

    distance = [0.0]
    self.end_reason = "end"
    for i in range(1, len(t)):
      if not all(math.isfinite(float(a[i])) for a in (t, v, yaw)):
        self.end_reason = "nonfiniteKnot"
        break
      dt = float(t[i]) - float(t[i - 1])
      if dt <= 0:
        self.end_reason = "unorderedTime"
        break
      if v[i] < 0:
        self.end_reason = "negativeVelocity"
        break
      step = 0.5 * (float(v[i - 1]) + float(v[i])) * dt
      next_distance = distance[-1] + step
      if not math.isfinite(next_distance) or not math.isfinite(float(yaw[i]) - float(yaw[i - 1])):
        self.end_reason = "nonfiniteInterval"
        break
      if next_distance - distance[-1] <= MIN_DISTANCE_STEP:
        self.end_reason = "distanceDegenerate"
        break
      distance.append(next_distance)
      if v[i] == 0:
        self.end_reason = "zeroVelocity"
        break
    if len(distance) < 3:
      raise SpatialHeadingError("insufficientForwardKnots")

    self._distance = np.asarray(distance, dtype=np.float64)
    self._heading = np.unwrap(yaw[:len(distance)])
    self._heading -= self._heading[0]
    h = np.diff(self._distance)
    secants = np.diff(self._heading) / h
    self._slopes = np.zeros(len(distance), dtype=np.float64)
    same_sign = (np.sign(secants[:-1]) == np.sign(secants[1:])) & (secants[:-1] != 0) & (secants[1:] != 0)
    w1, w2 = 2 * h[1:] + h[:-1], h[1:] + 2 * h[:-1]
    self._slopes[1:-1][same_sign] = ((w1[same_sign] + w2[same_sign]) /
                                   (w1[same_sign] / secants[:-1][same_sign] + w2[same_sign] / secants[1:][same_sign]))
    self._slopes[0] = _endpoint_slope(h[0], h[1], secants[0], secants[1])
    self._slopes[-1] = _endpoint_slope(h[-1], h[-2], secants[-1], secants[-2])
    if not np.isfinite(self._slopes).all():
      raise SpatialHeadingError("nonfiniteCurvature")

  @property
  def forward_distance(self) -> float:
    return float(self._distance[-1])

  @property
  def knot_count(self) -> int:
    return len(self._distance)

  def curvature(self, preview_distance: float) -> float:
    """Return 2 * (heading(q) - heading(0)) / q - heading'(0)."""
    if not math.isfinite(preview_distance) or preview_distance <= 0:
      raise SpatialHeadingError("invalidPreviewDistance")
    if preview_distance > self.forward_distance:
      raise SpatialHeadingError("outsideForwardDomain")

    i = min(int(np.searchsorted(self._distance, preview_distance, side="right")) - 1, self.knot_count - 2)
    h = self._distance[i + 1] - self._distance[i]
    u = (preview_distance - self._distance[i]) / h
    y0, y1 = self._heading[i], self._heading[i + 1]
    d0, d1 = self._slopes[i], self._slopes[i + 1]
    heading = ((2 * u**3 - 3 * u**2 + 1) * y0 + (-2 * u**3 + 3 * u**2) * y1 +
               (u**3 - 2 * u**2 + u) * h * d0 + (u**3 - u**2) * h * d1)
    curvature = float(2 * heading / preview_distance - self._slopes[0])
    if not math.isfinite(curvature):
      raise SpatialHeadingError("nonfiniteCurvature")
    return curvature


def get_spatial_curvature(times: ArrayLike, longitudinal_velocity: ArrayLike, heading: ArrayLike,
                          source_speed: float, preview_seconds: float) -> float:
  """Decode the current scalar using the producer's unfloored speed and horizon.

  Raises SpatialHeadingError when no supported current reference can be formed.
  The caller must apply its low-speed hold before calling this function.
  """
  if not math.isfinite(source_speed) or source_speed <= 0:
    raise SpatialHeadingError("invalidSourceSpeed")
  if not math.isfinite(preview_seconds) or preview_seconds <= 0:
    raise SpatialHeadingError("invalidPreviewSeconds")
  return SpatialHeading(times, longitudinal_velocity, heading).curvature(float(source_speed) * float(preview_seconds))
