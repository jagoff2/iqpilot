import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from iqpilot.selfdrive.locationd.roll_correction import (
  ChassisIMU, MAX_RATE_DEG_S, RollOffsetLearner, STATE_VERSION, SUPPORTED_CAR, VehicleRollCorrection, read_state, write_state,
)


def state():
  return {"version": STATE_VERSION, "carFingerprint": SUPPORTED_CAR, "offsetDeg": 2.5717551236271947,
          "yawZeroDps": .10824475850514448, "calibrationRpy": [0., .053, .032]}


def frame(address, counter=0, yaw=.1, lateral_g=.017, status=False, bus=0):
  if address == 0x130:
    bits = round((yaw + 163.84) / .005) | (round((lateral_g + 4.17677312) / .000127465) << 16) | (counter << 48)
    bits |= int(status) << 40
  elif address == 0x140:
    bits = round(4.17677312 / .000127465) | (counter << 48) | (int(status) << 20)
  else:
    bits = round((lateral_g * 9.81 + 10.23) / .01) | (1023 << 13) | (round((yaw + 40.95) / .01) << 40) | (counter << 60)
  return SimpleNamespace(address=address, src=bus, dat=bits.to_bytes(8, "little"))


def bundle(counter=0, **kwargs):
  return [frame(address, counter, **kwargs) for address in (0x130, 0x140, 0x220)]


def test_can_decoder_units_freshness_and_bus_isolation():
  decoder = ChassisIMU()
  decoder.feed(1., bundle(bus=130))
  assert decoder.read(1.) is None
  decoder.feed(1., bundle())
  yaw, lat = decoder.read(1.01)
  assert yaw == pytest.approx(.1, abs=.005)
  assert lat == pytest.approx(.017, abs=.00013)
  assert decoder.read(.99) is None
  assert decoder.read(1.051) is None


@pytest.mark.parametrize("failure", ["status", "repeat", "jump", "length", "invalid", "disagreement"])
def test_bad_can_cannot_update_reference(failure):
  decoder = ChassisIMU()
  decoder.feed(1., bundle())
  messages = bundle(1)
  if failure == "status":
    messages[0] = frame(0x130, 1, status=True)
  if failure == "repeat":
    messages[0] = frame(0x130, 0)
  if failure == "jump":
    messages[0] = frame(0x130, 3)
  if failure == "length":
    messages[0].dat = b"bad"
  if failure == "disagreement":
    messages[2] = frame(0x220, 1, lateral_g=.2)
  decoder.feed(1.01, messages, valid=failure != "invalid")
  assert decoder.read(1.01) is None


def run(learner, start, duration, speed=0., accel=0., roll=3.4, lat_g=.017, valid=True, active=False, yaw=.1):
  out = []
  for t in np.arange(start, start + duration, .05):
    out.append(learner.update(float(t), speed, accel, roll, (yaw, lat_g) if valid else None, valid, active))
  return out


def test_seed_applies_without_launch_learning_or_pid_state():
  learner = RollOffsetLearner(2.57, .1)
  run(learner, 0., 5.)
  before = learner.applied
  out = run(learner, 5., 5., speed=.3, accel=.05, roll=6., active=True)
  assert out == [before] * len(out)
  assert not learner.qualified


def test_stopped_bank_is_preserved_and_offset_is_persistent():
  learner = RollOffsetLearner(2.57, .1)
  bank = -2.
  run(learner, 0., 25., roll=bank + 2.57, lat_g=math.sin(math.radians(bank)))
  assert learner.applied == pytest.approx(2.57)
  assert learner.reference_roll == pytest.approx(bank)
  out = run(learner, 25., 2., speed=8., accel=2., valid=False, active=True)
  assert out == [2.57] * len(out)


def test_bootstrap_and_rate_limit_while_active():
  learner = RollOffsetLearner()
  run(learner, 0., 25., roll=3.57, lat_g=math.sin(math.radians(1.)))
  assert learner.offset == pytest.approx(2.57)
  before = learner.applied
  out = run(learner, 25., 10., speed=8., roll=4.1, lat_g=math.sin(math.radians(1.)), active=True)
  assert learner.offset > before
  assert np.max(np.abs(np.diff([before, *out]))) <= MAX_RATE_DEG_S * .05 + 1e-12


def test_turn_and_invalid_pose_hold_last_estimate():
  learner = RollOffsetLearner(2.57, .1)
  run(learner, 0., 5.)
  before = learner.offset
  run(learner, 5., 10., speed=8., yaw=15., roll=8., active=True)
  assert learner.offset == before
  run(learner, 15., 10., valid=False, roll=8., active=True)
  assert learner.offset == before


def test_remount_disagreement_requires_reacquisition_without_active_step():
  learner = RollOffsetLearner(2.57, .1)
  out = run(learner, 0., 16., speed=8., roll=0., lat_g=0., active=True)
  assert learner.offset is None
  assert np.max(np.abs(np.diff(out))) <= MAX_RATE_DEG_S * .05 + 1e-12


@pytest.mark.parametrize("change", [{"offsetDeg": float("nan")}, {"offsetDeg": 20.}, {"yawZeroDps": 2.},
                                    {"calibrationRpy": [0., 0.]}, {"carFingerprint": "other"}, {"version": 50}])
def test_corrupt_or_incompatible_state_rejected(tmp_path, change):
  path = tmp_path / "state"
  path.write_text(json.dumps({**state(), **change}))
  assert read_state(path, SUPPORTED_CAR) is None


def test_atomic_state_roundtrip_and_invalidated_state(tmp_path):
  path = tmp_path / "new" / "state"
  write_state(path, state())
  assert read_state(path, SUPPORTED_CAR)["offsetDeg"] == state()["offsetDeg"]
  write_state(path, {**state(), "offsetDeg": None})
  assert read_state(path, SUPPORTED_CAR) is None
  assert list(path.parent.iterdir()) == [path]


def test_wrapper_uses_calibrated_frame_and_rejects_stale_car(tmp_path):
  from iqpilot.common.params import Params
  from iqpilot.common.vehicle_roll_settings import vehicle_roll_state_path
  params = Params(str(tmp_path / "params"))
  write_state(vehicle_roll_state_path(params), {**state(), "calibrationRpy": [0., 0., 0.]})
  correction = VehicleRollCorrection(params, SUPPORTED_CAR)
  orientation = SimpleNamespace(x=math.radians(3.4), y=0., z=0., xStd=.01, valid=True)
  pose = SimpleNamespace(orientationNED=orientation, sensorsOK=True, posenetOK=True)
  correction.handle_log(0., "extrinsicsCalibration", SimpleNamespace(calStatus="calibrated", rpyCalib=[0., 0., 0.]))
  output = correction.handle_log(1., "deviceMotion", pose)
  assert math.degrees(output) == pytest.approx(3.4 - state()["offsetDeg"])
  assert not correction.learner.qualified
  correction.handle_log(1., "carState", SimpleNamespace(canValid=True, vEgo=0., aEgo=0.))
  correction.handle_log(1., "carControl", SimpleNamespace(latActive=False))
  correction.handle_log(1.01, "extrinsicsCalibration", SimpleNamespace(calStatus="calibrated", rpyCalib=[0., .1, 0.]))
  assert correction.learner.offset is None


def test_nonfinite_input_and_long_gaps_do_not_integrate():
  learner = RollOffsetLearner(2.57, .1)
  run(learner, 0., 5.)
  before = learner.offset
  learner.update(10., 0., 0., 3.4, (.1, .017), True, False)
  assert learner.offset == before
  learner.update(10.05, 0., 0., float("nan"), (.1, .017), True, False)
  assert learner.offset == before


def test_cold_calibration_establishes_remount_reference(tmp_path):
  from iqpilot.common.params import Params
  correction = VehicleRollCorrection(Params(str(tmp_path)), SUPPORTED_CAR)
  correction.handle_log(0., "extrinsicsCalibration", SimpleNamespace(calStatus="calibrated", rpyCalib=[0., 0., 0.]))
  correction.learner.offset = correction.learner.applied = 2.57
  correction.handle_log(1., "carState", SimpleNamespace(canValid=True, vEgo=0., aEgo=0.))
  correction.handle_log(1., "carControl", SimpleNamespace(latActive=False))
  correction.handle_log(1., "extrinsicsCalibration", SimpleNamespace(calStatus="calibrated", rpyCalib=[0., .1, 0.]))
  assert correction.learner.offset is None


def test_extrinsics_change_while_moving_cannot_step_corrected_roll(tmp_path):
  from iqpilot.common.params import Params
  from iqpilot.common.vehicle_roll_settings import vehicle_roll_state_path
  params = Params(str(tmp_path))
  write_state(vehicle_roll_state_path(params), {**state(), "calibrationRpy": [0., 0., 0.]})
  correction = VehicleRollCorrection(params, SUPPORTED_CAR)
  correction.handle_log(0., "carState", SimpleNamespace(canValid=True, vEgo=10., aEgo=0.))
  correction.handle_log(0., "carControl", SimpleNamespace(latActive=True))
  pose = SimpleNamespace(orientationNED=SimpleNamespace(x=.06, y=.02, z=.3, xStd=.01, valid=True), sensorsOK=True, posenetOK=True)
  before = correction.handle_log(0., "deviceMotion", pose)
  correction.handle_log(.01, "extrinsicsCalibration", SimpleNamespace(calStatus="calibrated", rpyCalib=[.08, .1, .1]))
  after = correction.handle_log(.05, "deviceMotion", pose)
  assert before == after
  assert not correction.learner.qualified
