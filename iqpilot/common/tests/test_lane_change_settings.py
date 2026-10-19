import pytest

from iqpilot.common.lane_change_settings import LANE_CHANGE_DELAYS, lane_change_mode
from iqpilot.common.params import ParamKeyFlag, Params
from iqpilot.common.ui_defaults import apply_ui_defaults


@pytest.mark.parametrize('preset', [0, 1, *LANE_CHANGE_DELAYS])
def test_lane_change_selection_survives_restart_and_drive_transitions(tmp_path, preset):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', preset)
  for flag in (ParamKeyFlag.CLEAR_ON_MANAGER_START, ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
               ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION, ParamKeyFlag.CLEAR_ON_IGNITION_ON):
    params.clear_all(flag)
    apply_ui_defaults(params)
    saved = Params(str(tmp_path)).get('IQLaneChangeTimer')
    assert saved == preset
    assert lane_change_mode(saved) == (2 if preset in LANE_CHANGE_DELAYS else preset)
