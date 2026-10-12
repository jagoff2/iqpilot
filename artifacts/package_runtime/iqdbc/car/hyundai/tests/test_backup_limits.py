"""Offline boundary coverage for the modified Sunnypilot limit port."""
from types import SimpleNamespace

import pytest

from iqdbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from iqdbc.car.lateral import (
  CurvatureSteeringLimits,
  apply_driver_steer_torque_limits,
  apply_std_curvature_limits,
  apply_std_steer_angle_limits,
)


def controller_params(flags=0, car=CAR.HYUNDAI_SONATA):
  return CarControllerParams(SimpleNamespace(flags=flags, carFingerprint=car))


@pytest.mark.parametrize("car", CAR)
def test_all_platform_controller_limits(car):
  params = controller_params(car.config.flags, car)
  canfd = bool(car.config.flags & HyundaiFlags.CANFD)
  assert params.STEER_MAX == 400
  assert params.STEER_DELTA_UP == (3 if canfd else 5)
  assert params.STEER_DELTA_DOWN == (3 if canfd else 8)
  assert params.STEER_DRIVER_ALLOWANCE == (350 if canfd else 70)
  assert params.STEER_THRESHOLD == (300 if canfd else 200)
  assert params.STEER_DRIVER_MULTIPLIER == 2
  assert params.STEER_DRIVER_FACTOR == 1
  assert params.STEER_STEP == 1


@pytest.mark.parametrize("alternate", (HyundaiFlags.ALT_LIMITS, HyundaiFlags.ALT_LIMITS_2,
                                      HyundaiFlags.ALT_LIMITS | HyundaiFlags.ALT_LIMITS_2))
@pytest.mark.parametrize("canfd", (0, HyundaiFlags.CANFD))
@pytest.mark.parametrize("car", (CAR.HYUNDAI_SONATA, CAR.HYUNDAI_ELANTRA))
def test_alternate_flags_and_canfd_precedence(alternate, canfd, car):
  # ALT_LIMITS_2 has no current IQ platform, but must use the backup's limits
  # if selected. CAN FD retains its own ramp and driver-override limits.
  params = controller_params(alternate | canfd, car)
  assert params.STEER_MAX == 400
  assert params.STEER_DELTA_UP == (3 if canfd else 5)
  assert params.STEER_DELTA_DOWN == (3 if canfd else 8)
  assert params.STEER_DRIVER_ALLOWANCE == (350 if canfd else 70)


def test_kona_flag_scope():
  assert CAR.HYUNDAI_KONA_EV.config.flags == HyundaiFlags.EV
  assert CAR.HYUNDAI_KONA_EV_2022.config.flags & HyundaiFlags.ALT_LIMITS
  assert CAR.HYUNDAI_KONA.config.flags & HyundaiFlags.ALT_LIMITS
  assert CAR.HYUNDAI_KONA_HEV.config.flags & HyundaiFlags.ALT_LIMITS


@pytest.mark.parametrize("flags,up,down", ((0, 5, 8), (HyundaiFlags.ALT_LIMITS, 5, 8),
                                          (HyundaiFlags.ALT_LIMITS_2, 5, 8), (HyundaiFlags.CANFD, 3, 3)))
@pytest.mark.parametrize("sign", (-1, 1))
def test_torque_cap_and_rate_boundaries(flags, up, down, sign):
  params = controller_params(flags)

  def limit(request, last):
    return apply_driver_steer_torque_limits(sign * request, sign * last, 0, params)

  assert limit(401, 400) == sign * 400
  assert limit(400, 399) == sign * 400
  assert limit(400, 100) == sign * (100 + up)
  assert limit(0, 100) == sign * (100 - down)
  assert limit(-400, 2) == sign * max(2 - down, -up)


@pytest.mark.parametrize("flags,allowance", ((0, 70), (HyundaiFlags.ALT_LIMITS, 70),
                                            (HyundaiFlags.ALT_LIMITS_2, 70), (HyundaiFlags.CANFD, 350)))
@pytest.mark.parametrize("sign", (-1, 1))
def test_driver_override_boundaries(flags, allowance, sign):
  params = controller_params(flags)
  # Driver torque at the allowance leaves full output available;
  # one unit farther against the request reduces output by two.
  for driver, expected in ((allowance, 400), (allowance + 1, 398)):
    actual = apply_driver_steer_torque_limits(sign * 400, sign * 400, -sign * driver, params)
    assert actual == sign * expected
  actual = apply_driver_steer_torque_limits(sign * 400, 0, -sign * (allowance + 200), params)
  assert actual == 0


def test_existing_iq_angle_control():
  params = controller_params(HyundaiFlags.CANFD | HyundaiFlags.ANGLE_CONTROL)
  limits = params.ANGLE_LIMITS
  assert (params.ANGLE_MAX_TORQUE, params.ANGLE_MIN_TORQUE) == (200, 25)
  assert (params.ANGLE_TORQUE_UP_RATE, params.ANGLE_TORQUE_DOWN_RATE) == (8, 12)
  assert apply_std_steer_angle_limits(100, 0, 16, 0, True, limits) == pytest.approx(1.3)
  assert apply_std_steer_angle_limits(0, 100, 16, 0, True, limits) == pytest.approx(98.4)
  assert apply_std_steer_angle_limits(0, 0, 16, 190, False, limits) == 175


@pytest.mark.parametrize("speed", (0, 0.5, 1, 10, 30))
@pytest.mark.parametrize("step", (1, 2))
@pytest.mark.parametrize("sign", (-1, 1))
def test_jerk_limit_at_speed_and_step_boundaries(speed, step, sign):
  limits = CurvatureSteeringLimits(CURVATURE_MAX=10)
  actual = apply_std_curvature_limits(sign, 0, speed, 0, False, step, True, limits)
  expected = sign * 10.0 / max(speed, 1) ** 2 * step * 0.01
  assert actual == pytest.approx(expected)


@pytest.mark.parametrize("speed", (0, 1, 10, 30))
@pytest.mark.parametrize("sign", (-1, 1))
def test_acceleration_limit_preserves_iq_roll_allowance(speed, sign):
  limits = CurvatureSteeringLimits(CURVATURE_MAX=10)
  maximum = (5.0 + 9.81 * 0.06) / max(speed, 1) ** 2
  actual = apply_std_curvature_limits(sign * (maximum + 0.01), sign * maximum, speed, 0, False, 1, True, limits)
  assert actual == pytest.approx(sign * maximum)


@pytest.mark.parametrize("sign", (-1, 1))
def test_inactive_and_absolute_curvature_caps(sign):
  limits = CurvatureSteeringLimits(CURVATURE_MAX=0.02)
  assert apply_std_curvature_limits(0, 0, 20, sign * 0.01, False, 1, False, limits) == sign * 0.01
  assert apply_std_curvature_limits(0, 0, 20, sign * 0.03, False, 1, False, limits) == sign * 0.02
  assert apply_std_curvature_limits(sign, 0, 1, 0, False, 1, True, limits) == sign * 0.02
