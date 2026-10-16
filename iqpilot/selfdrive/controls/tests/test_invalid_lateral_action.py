"""Invalid model output cannot reach steering; a valid next frame can recover."""
import math

import pytest

from iqpilot.selfdrive.controls.lib.curvature_lookahead import Selection
from iqpilot.selfdrive.controls.tests.lateral_control_scenarios import build_controls, load_controls_module, set_frame


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), -float("inf")])
def test_invalid_lateral_action_resets_and_recovers(monkeypatch, tmp_path, invalid):
  monkeypatch.setenv("PARAMS_ROOT", str(tmp_path))
  controls = build_controls(load_controls_module(monkeypatch), "torque", lookahead=False)
  set_frame(controls, 125)
  controls.LaC.pid.i = 0.4
  controls.sm["modelV2"].action.desiredCurvature = invalid
  command, _ = controls.state_control()
  assert not command.latActive
  assert command.actuators.torque == 0.0
  assert controls.LaC.pid.i == 0.0
  assert math.isfinite(command.actuators.curvature)
  assert controls.lateral_action_selection == Selection.inactive

  set_frame(controls, 126)
  command, _ = controls.state_control()
  assert command.latActive
  assert math.isfinite(command.actuators.torque)
  assert math.isfinite(command.actuators.curvature)
