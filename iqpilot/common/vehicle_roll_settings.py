"""Persistent opt-in and calibration paths for vehicle IMU roll correction.

Keep these outside the native Params directory: prebuilt registries reject new
keys and clear_all() deletes unknown files from that directory.
"""
import os
import tempfile
from pathlib import Path

from iqpilot.common.params import Params


def vehicle_roll_state_path(params: Params) -> Path:
  # Preserve the lexical prefix, not the random native Params symlink target.
  param_dir = Path(params.get_param_path())
  return param_dir.parent / "iqpilot_preferences" / param_dir.name / "VehicleRollCorrectionState"


def get_vehicle_roll_correction_enabled(params: Params) -> bool:
  try:
    return vehicle_roll_state_path(params).with_name("VehicleRollCorrection").read_bytes() == b"1"
  except OSError:
    return False


def set_vehicle_roll_correction_enabled(params: Params, enabled: bool) -> None:
  path = vehicle_roll_state_path(params).with_name("VehicleRollCorrection")
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = None
  try:
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".vehicle_roll_", delete=False) as stream:
      temporary = Path(stream.name)
      stream.write(b"1" if enabled else b"0")
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(temporary, path)
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)
