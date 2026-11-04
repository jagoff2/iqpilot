from pathlib import Path
from types import SimpleNamespace

import pytest

from iqpilot.common.params import Params, ParamKeyFlag
from iqpilot.common.vehicle_roll_settings import (
  get_vehicle_roll_correction_enabled, set_vehicle_roll_correction_enabled, vehicle_roll_state_path,
)


def test_default_off_and_state_path_outside_native_params(tmp_path):
  params = Params(str(tmp_path / "params"))
  assert not get_vehicle_roll_correction_enabled(params)
  state = vehicle_roll_state_path(params)
  native = Path(params.get_param_path())
  assert state == native.parent / "iqpilot_preferences" / native.name / "VehicleRollCorrectionState"
  assert not state.parent.exists()


@pytest.mark.parametrize("enabled", [False, True])
def test_selection_survives_native_clears_and_new_reader(tmp_path, enabled):
  params = Params(str(tmp_path / "params"))
  set_vehicle_roll_correction_enabled(params, enabled)
  for flag in (ParamKeyFlag.CLEAR_ON_MANAGER_START, ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
               ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION, ParamKeyFlag.CLEAR_ON_IGNITION_ON):
    params.clear_all(flag)
    assert get_vehicle_roll_correction_enabled(Params(str(tmp_path / "params"))) is enabled


@pytest.mark.parametrize("contents", [b"", b"true", b"1\n", b"0", b"garbage"])
def test_invalid_value_is_off(tmp_path, contents):
  params = Params(str(tmp_path / "params"))
  path = vehicle_roll_state_path(params).with_name("VehicleRollCorrection")
  path.parent.mkdir(parents=True)
  path.write_bytes(contents)
  assert not get_vehicle_roll_correction_enabled(params)


def test_prefixes_and_roots_are_isolated(tmp_path):
  first = SimpleNamespace(get_param_path=lambda: str(tmp_path / "first" / "a"))
  different_prefix = SimpleNamespace(get_param_path=lambda: str(tmp_path / "first" / "b"))
  different_root = SimpleNamespace(get_param_path=lambda: str(tmp_path / "second" / "a"))
  set_vehicle_roll_correction_enabled(first, True)
  assert get_vehicle_roll_correction_enabled(first)
  assert not get_vehicle_roll_correction_enabled(different_prefix)
  assert not get_vehicle_roll_correction_enabled(different_root)


def test_failed_replace_preserves_selection_and_state(tmp_path, monkeypatch):
  params = Params(str(tmp_path / "params"))
  set_vehicle_roll_correction_enabled(params, True)
  state = vehicle_roll_state_path(params)
  state.write_text('{"offset_deg":2.57}')

  def fail_replace(*args):
    raise OSError("simulated write failure")

  monkeypatch.setattr("iqpilot.common.vehicle_roll_settings.os.replace", fail_replace)
  with pytest.raises(OSError, match="simulated write failure"):
    set_vehicle_roll_correction_enabled(params, False)
  assert get_vehicle_roll_correction_enabled(params)
  assert state.read_text() == '{"offset_deg":2.57}'
  assert sorted(p.name for p in state.parent.iterdir()) == ["VehicleRollCorrection", "VehicleRollCorrectionState"]
