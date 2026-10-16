import math

import numpy as np
import pytest

from iqpilot.selfdrive.controls.lib.spatial_heading import SpatialHeading, SpatialHeadingError, get_spatial_curvature


MODEL_TIMES = np.arange(33, dtype=float) ** 2 * (10 / 1024)


@pytest.mark.parametrize("curvature", [-0.03, 0.0, 0.02])
@pytest.mark.parametrize("source_speed", [0.31, 4.0, 15.0, 24.0])
@pytest.mark.parametrize("preview_seconds", [0.05, 0.35, 0.42202078104019164])
def test_constant_curvature_survives_source_and_model_speed_mismatch(curvature, source_speed, preview_seconds):
  model_velocity = np.linspace(25.0, 15.0, len(MODEL_TIMES))
  distance = np.r_[0, np.cumsum(0.5 * (model_velocity[:-1] + model_velocity[1:]) * np.diff(MODEL_TIMES))]
  heading = curvature * distance
  assert get_spatial_curvature(MODEL_TIMES, model_velocity, heading, source_speed, preview_seconds) == pytest.approx(curvature, abs=1e-12)


def test_early_heading_recovery_is_not_discarded_when_preview_reaches_plateau():
  distance = np.array([0, 0.1, 0.2, 0.5, 1.0, 2.0, 3.2, 4.0, 6.0])
  u = np.clip((distance - 0.2) / 3.0, 0, 1)
  heading = 0.03 * (3 * u**2 - 2 * u**3)
  # The heading is constant at the preview location, but the early recovery
  # remains necessary. A decoder using only heading'(q) would return zero.
  assert heading[-3] == heading[-2] == heading[-1]
  actual = get_spatial_curvature(distance / 15.0, np.full(len(distance), 15.0), heading, 15.0, 0.35)
  assert actual == pytest.approx(2 * 0.03 / (15 * 0.35), abs=1e-12)


def test_linear_curvature_ramp_at_a_sampled_preview():
  times = np.linspace(0, 2, 201)
  distance = 15 * times
  k0, rate = 0.02, -0.0006
  heading = k0 * distance + 0.5 * rate * distance**2
  actual = get_spatial_curvature(times, np.full(len(times), 15.0), heading, 15.0, 0.35)
  assert actual == pytest.approx(k0 + rate * 15 * 0.35, abs=1e-12)


@pytest.mark.parametrize("offset", [-3.14, 0.15, 3.14])
def test_heading_origin_and_wrap_do_not_change_curvature(offset):
  times = np.linspace(0, 2, 21)
  heading = (0.03 * 15 * times + offset + math.pi) % (2 * math.pi) - math.pi
  before = heading.copy()
  assert get_spatial_curvature(times, np.full(len(times), 15.0), heading, 15.0, 0.35) == pytest.approx(0.03, abs=1e-12)
  np.testing.assert_array_equal(heading, before)


@pytest.mark.parametrize(("source_speed", "preview_seconds", "expected"), [
  (12, 0.05, 0.010714499992140875),
  (12, 0.1, 0.012523730078537326),
  (12, 0.2, 0.011599165744451086),
  (12, 0.35, 0.008666906415724925),
  (12, 0.5, 0.005029140416573799),
  (12, 0.7, 0.0008829211462031868),
  (12, 0.9, -0.0020723063446398656),
  (0.31, 0.35, 0.0091812086122958),
  (12, 0.42202078104019164, 0.0068278467274454045),
])
def test_independent_pchip_reference(source_speed, preview_seconds, expected):
  # Frozen SciPy 1.15.3 PchipInterpolator reference, including a heading extremum,
  # unequal knots, actual startup/route horizons, and a speed below the old floor.
  # SciPy is deliberately not a runtime or test dependency on the device.
  times = [0, 0.01, 0.04, 0.09, 0.16, 0.25, 0.36, 0.49, 0.64, 0.81, 1.0]
  velocity = [13.2, 13.2, 13.18, 13.14, 13.08, 12.99, 12.87, 12.70, 12.48, 12.20, 11.87]
  heading = [0.001, 0.0022, 0.0061, 0.0137, 0.0229, 0.0326, 0.0398, 0.043, 0.0422, 0.0389, 0.032]
  assert get_spatial_curvature(times, velocity, heading, source_speed, preview_seconds) == pytest.approx(expected, abs=1e-13)


@pytest.mark.parametrize(("field", "value", "reason"), [
  ("times", math.nan, "nonfiniteKnot"),
  ("velocity", math.inf, "nonfiniteKnot"),
  ("heading", math.nan, "nonfiniteKnot"),
  ("times", 2.0, "unorderedTime"),
  ("velocity", -0.01, "negativeVelocity"),
])
def test_invalid_future_tail_preserves_supported_current_query(field, value, reason):
  data = {"times": np.arange(6, dtype=float), "velocity": np.full(6, 2.0), "heading": np.arange(6) * 0.04}
  data[field][4] = value
  curve = SpatialHeading(data["times"], data["velocity"], data["heading"])
  assert curve.knot_count == 4
  assert curve.forward_distance == 6.0
  assert curve.end_reason == reason
  assert curve.curvature(3.0) == pytest.approx(0.02)
  with pytest.raises(SpatialHeadingError, match="outsideForwardDomain"):
    curve.curvature(6.01)


def test_negative_endpoint_is_excluded_even_when_trapezoid_is_positive():
  curve = SpatialHeading([0, 1, 2, 3, 4], [2, 2, 2, -0.1, 2], [0, 0.04, 0.08, 0.1, 0.12])
  assert curve.forward_distance == 4.0
  assert curve.end_reason == "negativeVelocity"


def test_exact_zero_endpoint_is_included_once():
  curve = SpatialHeading([0, 1, 2, 3, 4], [2, 2, 2, 0, 2], [0, 0.04, 0.08, 0.1, 0.12])
  assert curve.knot_count == 4
  assert curve.end_reason == "zeroVelocity"
  assert curve.curvature(5.0) == pytest.approx(0.02)
  with pytest.raises(SpatialHeadingError, match="outsideForwardDomain"):
    curve.curvature(np.nextafter(5.0, math.inf))


def test_degenerate_future_distance_ends_prefix():
  curve = SpatialHeading([0, 1, 2, 2.000001, 3], [2, 2, 2, 2, 2], [0, 0.04, 0.08, 0.08, 0.12])
  assert curve.forward_distance == 4.0
  assert curve.end_reason == "distanceDegenerate"
  assert curve.curvature(4.0) == pytest.approx(0.02)


def test_finite_tail_overflow_preserves_supported_prefix():
  curve = SpatialHeading([0, 1, 2, 1e308], [2, 2, 2, 1e308], [0, 0.04, 0.08, 0.1])
  assert curve.end_reason == "nonfiniteInterval"
  assert curve.curvature(4.0) == pytest.approx(0.02)


def test_numpy_scalar_inputs_do_not_round_the_preview_product_to_float32():
  speed, horizon = np.float32(7.123), np.float32(0.35)
  times = np.linspace(0, 2, 21)
  heading = 0.001 * times**2
  actual = get_spatial_curvature(times, np.full(len(times), 8.0), heading, speed, horizon)
  expected = get_spatial_curvature(times, np.full(len(times), 8.0), heading, float(speed), float(horizon))
  assert actual == expected


@pytest.mark.parametrize(("times", "velocity", "heading", "reason"), [
  ([0, 1], [2, 2], [0, 0.1], "shape"),
  ([0, 1, 2], [2, 2], [0, 0.1, 0.2], "shape"),
  ([[0, 1, 2]], [[2, 2, 2]], [[0, 0.1, 0.2]], "shape"),
  ([math.nan, 1, 2], [2, 2, 2], [0, 0.1, 0.2], "invalidOrigin"),
  ([0, 1, 2], [2, 2, 2], [math.inf, 0.1, 0.2], "invalidOrigin"),
  ([0, 1, 2], [0, 2, 2], [0, 0.1, 0.2], "initialNonpositiveVelocity"),
  ([0, 1, 2], [-1, 2, 2], [0, 0.1, 0.2], "initialNonpositiveVelocity"),
  ([0, 1, 2], [2, -0.1, 2], [0, 0.1, 0.2], "insufficientForwardKnots"),
  ([0, 1, 2], [2, 2, math.nan], [0, 0.1, 0.2], "insufficientForwardKnots"),
])
def test_invalid_current_geometry_has_explicit_reason(times, velocity, heading, reason):
  with pytest.raises(SpatialHeadingError) as error:
    get_spatial_curvature(times, velocity, heading, 2.0, 0.35)
  assert error.value.reason == reason


@pytest.mark.parametrize("value", [0.0, -1.0, math.nan, math.inf])
@pytest.mark.parametrize("field", ["source_speed", "preview_seconds"])
def test_invalid_producer_inputs_are_not_floored_or_defaulted(field, value):
  kwargs = {"source_speed": 2.0, "preview_seconds": 0.35, field: value}
  with pytest.raises(SpatialHeadingError):
    get_spatial_curvature([0, 1, 2], [2, 2, 2], [0, 0.04, 0.08], **kwargs)


def test_nonfinite_preview_product_is_rejected():
  with pytest.raises(SpatialHeadingError, match="invalidPreviewDistance"):
    get_spatial_curvature([0, 1, 2], [2, 2, 2], [0, 0.04, 0.08], 1e308, 1e308)
