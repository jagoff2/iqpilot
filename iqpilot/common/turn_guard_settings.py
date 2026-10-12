"""Persistent preference for the optional turn-guard steering experiment.

Prebuilt releases cannot add native Params keys without rebuilding the registry.
Unknown files inside the native parameter directory are deleted by clear_all(),
so this setting lives in a separate, prefix-scoped preferences directory.
"""
import os
import tempfile
from pathlib import Path

from iqpilot.common.params import Params


def turn_guards_path(params: Params) -> Path:
  # Keep the lexical prefix (normally 'd'), not its random symlink destination.
  param_dir = Path(params.get_param_path())
  return param_dir.parent / "iqpilot_preferences" / param_dir.name / "IQSteeringTurnGuards"


def get_turn_guards_enabled(params: Params) -> bool:
  try:
    return turn_guards_path(params).read_bytes() == b"1"
  except OSError:
    return False


def set_turn_guards_enabled(params: Params, enabled: bool) -> None:
  path = turn_guards_path(params)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = None
  try:
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".turn_guards_", delete=False) as stream:
      temporary = Path(stream.name)
      stream.write(b"1" if enabled else b"0")
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(temporary, path)
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)


def reset_turn_guards(params: Params) -> None:
  turn_guards_path(params).unlink(missing_ok=True)
