from dataclasses import dataclass
from functools import partial

import pytest

from iqdbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags, HyundaiSafetyFlags
from iqdbc.car.lateral import apply_driver_steer_torque_limits
from iqdbc.car.structs import CarParams
from iqdbc.safety.tests.hyundai_common import canfd_steer, classic_steer
from iqdbc.safety.tests.libsafety import libsafety_py


@dataclass(frozen=True)
class Limits:
  torque: int
  rate_up: int
  rate_down: int
  rt_delta: int
  driver_allowance: int
  valid_request_frames: int


CLASSIC = Limits(404, 7, 8, 400, 150, 85)
CANFD = Limits(400, 3, 3, 112, 350, 89)
PROFILES = [
  pytest.param(mode, param, classic_steer, CLASSIC, id=f"{mode}-{param}")
  for mode in (CarParams.SafetyModel.hyundai, CarParams.SafetyModel.hyundaiLegacy)
  for param in (0, HyundaiSafetyFlags.ALT_LIMITS, HyundaiSafetyFlags.ALT_LIMITS_2)
] + [
  pytest.param(CarParams.SafetyModel.hyundaiCanfd, param, partial(canfd_steer, addr, length), CANFD,
               id=f"canfd-{param}-{addr:x}")
  for param, addr, length in (
    (0, 0x12A, 16),
    (HyundaiSafetyFlags.CANFD_LKA_STEERING, 0x50, 16),
    (HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT, 0x110, 32),
    (HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.LONG, 0x12A, 16),
  )
]


def reset_safety(mode, param):
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(mode, param) == 0
  safety.init_tests()
  safety.set_aol_params(False, False, False)
  safety.set_controls_allowed(True)
  safety.set_torque_driver(0, 0)
  return safety


def controller_params(param, limits):
  if limits is CANFD:
    candidate, flags = CAR.KIA_EV6, HyundaiFlags.CANFD
  elif param & HyundaiSafetyFlags.ALT_LIMITS_2:
    candidate, flags = CAR.HYUNDAI_KONA_2022, HyundaiFlags.ALT_LIMITS_2
  elif param & HyundaiSafetyFlags.ALT_LIMITS:
    candidate, flags = CAR.HYUNDAI_KONA, HyundaiFlags.ALT_LIMITS
  else:
    candidate, flags = CAR.HYUNDAI_SONATA, 0
  cp = CarParams.new_message(carFingerprint=candidate.value, flags=int(flags))
  return CarControllerParams(cp)


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
@pytest.mark.parametrize("sign", (-1, 1))
def test_torque_rate_boundaries(mode, param, message, limits, sign):
  # Torque may always unwind faster; the per-frame maximum constrains windup.
  for delta, allowed in ((limits.rate_up, True), (limits.rate_up + 1, False), (-100, True)):
    safety = reset_safety(mode, param)
    safety.set_desired_torque_last(sign * 100)
    safety.set_rt_torque_last(sign * 100)
    assert bool(safety.safety_tx_hook(message(sign * (100 + delta)))) is allowed


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
@pytest.mark.parametrize("sign", (-1, 1))
def test_driver_override_requires_minimum_unwind(mode, param, message, limits, sign):
  for reduction, allowed in ((limits.rate_down, True), (limits.rate_down - 1, False)):
    safety = reset_safety(mode, param)
    driver = -sign * (limits.driver_allowance + limits.torque)
    safety.set_torque_driver(driver, driver)
    safety.set_desired_torque_last(sign * limits.torque)
    safety.set_rt_torque_last(sign * limits.torque)
    assert bool(safety.safety_tx_hook(message(sign * (limits.torque - reduction)))) is allowed


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
@pytest.mark.parametrize("sign", (-1, 1))
def test_realtime_torque_boundary(mode, param, message, limits, sign):
  for delta, allowed in ((limits.rt_delta, True), (limits.rt_delta + 1, False)):
    safety = reset_safety(mode, param)
    desired = sign * delta
    safety.set_desired_torque_last(desired)
    safety.set_rt_torque_last(0)
    assert bool(safety.safety_tx_hook(message(desired))) is allowed


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
@pytest.mark.parametrize("sign", (-1, 1))
def test_opposing_driver_torque_boundary(mode, param, message, limits, sign):
  for driver_extra, commanded_reduction, allowed in ((0, 0, True), (1, 2, True), (1, 1, False)):
    safety = reset_safety(mode, param)
    driver = -sign * (limits.driver_allowance + driver_extra)
    safety.set_torque_driver(driver, driver)
    safety.set_desired_torque_last(sign * limits.torque)
    safety.set_rt_torque_last(sign * limits.torque)
    command = sign * (limits.torque - commanded_reduction)
    assert bool(safety.safety_tx_hook(message(command))) is allowed


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
def test_request_cut_boundaries(mode, param, message, limits):
  for valid_frames in (limits.valid_request_frames - 1, limits.valid_request_frames):
    safety = reset_safety(mode, param)
    for frame in range(valid_frames):
      safety.set_timer(1_000_000 + frame * 10_000)
      assert safety.safety_tx_hook(message(1))
    safety.set_timer(1_000_000 + valid_frames * 10_000)
    allowed = valid_frames == limits.valid_request_frames
    assert bool(safety.safety_tx_hook(message(1, False))) is allowed
    if allowed:
      safety.set_timer(1_010_000 + valid_frames * 10_000)
      assert safety.safety_tx_hook(message(1, False))
      safety.set_timer(1_020_000 + valid_frames * 10_000)
      assert not safety.safety_tx_hook(message(1, False))


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
@pytest.mark.parametrize("sign", (-1, 1))
def test_controller_rate_reaches_full_torque_without_rejection(mode, param, message, limits, sign):
  safety = reset_safety(mode, param)
  params = controller_params(param, limits)
  torque = 0
  for frame in range(150):
    safety.set_timer(frame * 10_000)
    torque = apply_driver_steer_torque_limits(sign * params.STEER_MAX, torque, 0, params)
    assert safety.safety_tx_hook(message(torque))
  assert torque == sign * 400


@pytest.mark.parametrize("mode,param,message,limits", PROFILES)
@pytest.mark.parametrize("sign", (-1, 1))
def test_controller_driver_override_matches_safety(mode, param, message, limits, sign):
  safety = reset_safety(mode, param)
  params = controller_params(param, limits)
  torque = sign * params.STEER_MAX
  driver = -sign * (750 if limits is CANFD else 400)
  safety.set_desired_torque_last(torque)
  safety.set_rt_torque_last(torque)
  safety.set_torque_driver(driver, driver)

  # Feed every real controller output continuously through the firmware hook.
  # With classic down-rate 7, the first command was 393 instead of the required 392.
  for frame in range(150):
    safety.set_timer(frame * 10_000)
    torque = apply_driver_steer_torque_limits(sign * params.STEER_MAX, torque, driver, params)
    assert safety.safety_tx_hook(message(torque)), f"driver override rejected torque={torque}, frame={frame}"
  assert torque == 0
