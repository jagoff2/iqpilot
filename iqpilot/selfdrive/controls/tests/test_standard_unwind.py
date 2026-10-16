"""Exercise delayed entry, unwind and reversal through the actual torque PID."""
import math

import numpy as np
import pytest

from iqpilot.cereal import car, log
from iqpilot.selfdrive.controls.tests.test_lateral_low_speed import controller_factory


@pytest.mark.parametrize("direction", [-1.0, 1.0])
@pytest.mark.parametrize("speed", [2.0, 6.0, 15.0, 30.0])
@pytest.mark.parametrize("old,current,actual,expected", [
  (0.4, 1.2, 0.3, 0.4),    # Delayed entry target is retained.
  (2.0, 0.8, 1.2, 0.8),    # Stale deeper turn cannot drive inward P.
  (1.2, -0.4, 0.7, 0.0),   # Old direction disappears on reversal.
  (1.2, 0.0, 0.3, 0.0),    # Straight request unwinds immediately.
])
def test_standard_feedback_reference(controller_factory, direction, speed, old, current, actual, expected):
  controller, vm, cp = controller_factory()
  controller.turn_guards_enabled = False
  controller.lat_accel_request_buffer.extend([direction * old] * controller.lat_accel_request_buffer_len)
  angle = math.degrees(vm.get_steer_from_curvature(-direction * actual / speed ** 2, speed, 0.0))
  state = car.CarState.new_message(vEgo=speed, steeringAngleDeg=angle)
  params = log.VehicleParameters.new_message(stiffnessFactor=1.0, steerRatio=cp.steerRatio)
  output, _, trace = controller.update(True, state, vm, params, True, direction * current / speed ** 2,
                                       None, False, 0.3)
  assert trace.actualLateralAccel == pytest.approx(direction * actual, abs=2e-6)
  assert trace.desiredLateralAccel == pytest.approx(direction * expected, abs=2e-6)
  assert trace.p == pytest.approx(controller.pid.k_p * direction * (expected - actual), abs=2e-6)
  assert np.isfinite(output)
  assert controller.pid.i == 0.0  # frozen by the supplied actuator-limit flag


def test_standard_projection_preserves_toyota_lead(controller_factory):
  controller, vm, cp = controller_factory("TOYOTA_RAV4_TSS2")
  controller.turn_guards_enabled = False
  assert controller.setpoint_lead_enabled
  controller.lat_accel_request_buffer.extend([2.0] * controller.lat_accel_request_buffer_len)
  state = car.CarState.new_message(vEgo=15.0)
  params = log.VehicleParameters.new_message(stiffnessFactor=1.0, steerRatio=cp.steerRatio)
  _, _, trace = controller.update(True, state, vm, params, True, 0.8 / 225.0, None, False, 0.3)
  assert 0.0 <= trace.desiredLateralAccel <= 0.8 + 1e-7
