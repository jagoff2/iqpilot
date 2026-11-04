import math

import pytest

from iqpilot.cereal import car, messaging
from iqpilot.common.params import Params
from iqpilot.selfdrive.locationd.paramsd import VehicleParamsEstimator
from iqpilot.selfdrive.locationd.torqued import TorqueEstimator


@pytest.fixture
def cp():
  result = car.CarParams.new_message()
  result.carFingerprint = "HYUNDAI_KONA_EV"
  result.mass = 1700.
  result.rotationalInertia = 2500.
  result.wheelbase = 2.6
  result.centerToFront = 1.1
  result.tireStiffnessFront = 100000.
  result.tireStiffnessRear = 110000.
  result.steerRatio = 15.
  result.lateralTuning.init("torque")
  result.lateralTuning.torque.latAccelFactor = 3.
  result.lateralTuning.torque.friction = .12
  return result


@pytest.fixture
def isolated_params(tmp_path, monkeypatch):
  params = Params(str(tmp_path / "params"))
  monkeypatch.setattr("iqpilot.selfdrive.locationd.torqued.Params", lambda: params)
  return params


def pose():
  event = messaging.new_message("deviceMotion")
  msg = event.deviceMotion
  msg.sensorsOK = msg.posenetOK = True
  msg.orientationNED.x = .06
  msg.orientationNED.xStd = msg.angularVelocityDevice.zStd = .01
  msg.orientationNED.valid = msg.angularVelocityDevice.valid = True
  return event


def test_same_corrected_frame_reaches_both_roll_consumers(cp, isolated_params):
  params = VehicleParamsEstimator(cp, 15., 1., 0.)
  torque = TorqueEstimator(cp, roll_correction=True, track_all_points=True)
  cs = messaging.new_message("carState").carState
  cs.vEgo = 20.
  cc = messaging.new_message("carControl").carControl
  cc.latActive = True
  output = messaging.new_message("carOutput").carOutput
  output.actuatorsOutput.torque = -.2
  for i in range(torque.hist_len):
    for kind, msg in [("carState", cs), ("carControl", cc), ("carOutput", output)]:
      torque.handle_log(i * .05, kind, msg)
  event = pose()
  params.handle_log(5., "deviceMotion", event.deviceMotion, roll_override=.01)
  torque.handle_log(5., "deviceMotion", event.deviceMotion, roll_override=.01)
  assert params.observed_roll == pytest.approx(.01)
  assert torque.all_torque_points[-1][1] == pytest.approx(-math.sin(.01) * 9.81)


def test_legacy_frame_is_unchanged_without_override(cp, isolated_params):
  params = VehicleParamsEstimator(cp, 15., 1., 0.)
  event = pose()
  for i in range(5):
    params.handle_log(i * .05, "deviceMotion", event.deviceMotion)
  assert params.observed_roll == pytest.approx(.06)


@pytest.mark.parametrize("cached_version", [1, 2])
def test_corrected_mode_never_restores_points_from_unknown_roll_baseline(cp, isolated_params, cached_version):
  cache = messaging.new_message("lateralTorqueParameters")
  msg = cache.lateralTorqueParameters
  msg.version = cached_version
  msg.valid = True
  msg.latAccelFactorFiltered = 3.1
  msg.latAccelOffsetFiltered = .5
  msg.frictionCoefficientFiltered = .1
  msg.points = [[.2, .3]]
  isolated_params.put("CarParamsPrevRoute", cp.to_bytes())
  isolated_params.put("LiveTorqueParameters", cache.to_bytes())
  estimator = TorqueEstimator(cp, roll_correction=True)
  assert len(estimator.filtered_points) == 0
  assert estimator.filtered_params["latAccelOffset"].x == 0.
  assert estimator.filtered_params["latAccelFactor"].x == pytest.approx(3.)
  assert estimator.get_msg().lateralTorqueParameters.version == 2


def test_moving_roll_reference_reset_preserves_published_offset_and_slope(cp, isolated_params):
  estimator = TorqueEstimator(cp, roll_correction=True)
  estimator.filtered_points.add_point(.2, .3)
  estimator.filtered_params["latAccelOffset"].x = .2
  estimator.filtered_params["latAccelFactor"].x = 3.1
  estimator.reset_roll_reference()
  assert len(estimator.filtered_points) == 0
  assert estimator.get_msg().lateralTorqueParameters.latAccelOffsetFiltered == pytest.approx(.2)
  assert estimator.filtered_params["latAccelFactor"].x == 3.1
