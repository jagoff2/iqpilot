"""Producer selection and hold behavior for the standard torque reference."""
import math

import numpy as np
import pytest

from iqpilot.cereal import log
from iqpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from iqpilot.selfdrive.controls.lib.spatial_heading import get_spatial_curvature
from iqpilot.selfdrive.iqmodeld.config import ModelConstants, Plan
from iqpilot.selfdrive.iqmodeld.daemon import NeuralEngineState
from iqpilot.selfdrive.iqmodeld.driving_action import get_action_from_model
from iqpilot.selfdrive.iqmodeld.tests.test_action_provenance import outputs_for, runtime


@pytest.mark.parametrize("kind", ["plan", "planplus", "synthetic"])
def test_selected_plan_geometry_drives_spatial_action(kind):
  state = runtime(kind == "synthetic")
  state.standard_torque_spatial_reference = True
  outputs = outputs_for(kind)
  plan = outputs["plan"][0]
  if kind == "planplus":
    plan = plan + 0.75 * outputs["planplus"][0]
  previous = log.ModelDataV2.Action(desiredCurvature=0.004)
  expected = get_spatial_curvature(ModelConstants.T_IDXS, plan[:, Plan.VELOCITY][:, 0],
                                    plan[:, Plan.T_FROM_CURRENT_EULER][:, 2], 27.0, 0.45)
  action = NeuralEngineState.get_action_from_model(state, outputs, previous, 0.45, 0.4, 27.0)
  assert action.desiredCurvature == pytest.approx(smooth_value(expected, 0.004, 0.1), rel=1e-7)
  assert action.lateralActionSource == "plan"
  assert action.legacyLookahead.contract == "unsupported"


@pytest.mark.parametrize("kind", ["nativeAction", "nativeCurvature"])
def test_native_action_keeps_precedence(kind):
  state = runtime()
  outputs = outputs_for(kind)
  previous = log.ModelDataV2.Action()
  expected = NeuralEngineState.get_action_from_model(state, outputs, previous, 0.45, 0.4, 20.0)
  state.standard_torque_spatial_reference = True
  actual = NeuralEngineState.get_action_from_model(state, outputs, previous, 0.45, 0.4, 20.0)
  assert actual.to_dict() == expected.to_dict()


@pytest.mark.parametrize("speed", [0.0, 0.2, 0.3])
def test_low_speed_hold_precedes_invalid_geometry(speed):
  state = runtime()
  state.standard_torque_spatial_reference = True
  outputs = outputs_for("plan")
  outputs["plan"][0, :, Plan.T_FROM_CURRENT_EULER] = np.nan
  previous = log.ModelDataV2.Action(desiredCurvature=0.004)
  native = NeuralEngineState.get_action_from_model(state, outputs, previous, 0.45, 0.4, speed)
  offload = get_action_from_model(outputs, previous, speed, 0.45, 0.4, spatial_reference=True)
  assert native.desiredCurvature == previous.desiredCurvature
  assert offload.desiredCurvature == previous.desiredCurvature


def test_invalid_moving_geometry_is_explicit_and_valid_frame_recovers():
  state = runtime()
  state.standard_torque_spatial_reference = True
  outputs = outputs_for("plan")
  outputs["plan"][0, 0, Plan.T_FROM_CURRENT_EULER] = np.nan
  bad = NeuralEngineState.get_action_from_model(state, outputs, log.ModelDataV2.Action(), 0.45, 0.4, 20.0)
  assert math.isnan(bad.desiredCurvature)
  # No smoothing is configured for the tested models. A valid replacement must
  # not inherit the invalid scalar even when smoothing is enabled elsewhere.
  valid = NeuralEngineState.get_action_from_model(state, outputs_for("plan"), bad, 0.45, 0.4, 20.0)
  assert math.isfinite(valid.desiredCurvature)
