from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace


OVERLAY = Path(__file__).resolve().parents[4] / "iqpilot/ui/layouts/settings/iq_panels.py"


def _source() -> str:
  return OVERLAY.read_text(encoding="utf-8")


def _method(name: str):
  tree = ast.parse(_source())
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ModelsLayout")
  return next(node for node in cls.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name)


def _function(name: str):
  tree = ast.parse(_source())
  return next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == name)


def _compile_method(name: str, env: dict):
  node = _method(name)
  module = ast.Module(body=[node], type_ignores=[])
  ast.fix_missing_locations(module)
  exec(compile(module, str(OVERLAY), "exec"), env)
  return env[name]


def _compile_function(name: str, env: dict):
  node = _function(name)
  module = ast.Module(body=[node], type_ignores=[])
  ast.fix_missing_locations(module)
  exec(compile(module, str(OVERLAY), "exec"), env)
  return env[name]


def test_candidate_ui_contract_has_no_new_param_key():
  source = _source()
  tree = ast.parse(source)
  imported = {
    alias.name
    for node in tree.body
    if isinstance(node, ast.ImportFrom) and node.module == "iqpilot.selfdrive.iqmodeld.full_frame_candidate"
    for alias in node.names
  }
  assert imported == {
    "CANDIDATE_REF",
    "DISPLAY_NAME",
    "candidate_ready",
    "candidate_status",
    "is_candidate_selected",
    "set_candidate_enabled",
  }
  assert "IQFullFrame" not in source
  assert "FullFrameCandidate" not in source

  init_source = ast.get_source_segment(source, _method("_initialize_items"))
  candidate_init = init_source.split("self.full_frame_candidate_item =", 1)[1].split("self.full_frame_candidate_item.set_visible", 1)[0]
  assert "param=" not in candidate_init
  assert "initial_state=is_candidate_selected(ui_state.params)" in candidate_init


def test_picker_exposes_development_candidate_and_cloud_path_remains():
  source = _source()
  folders = ast.get_source_segment(source, _method("_get_folders"))
  selected = ast.get_source_segment(source, _method("_on_model_selected"))
  assert 'PickerGroup(tr("Experimental")' in folders
  assert "PickerItem(CANDIDATE_REF" in folders
  assert "FULL_FRAME_CANDIDATE_NAME" in folders
  assert "'detail_text': status" in folders
  assert selected.index("selected_ref == CANDIDATE_REF") < selected.index("ui_state.params.put(_DOWNLOAD_INDEX_KEY")
  assert "selected_bundle.index" in selected


def test_c3x_gate_and_human_readable_candidate_labels():
  hardware = SimpleNamespace(get_device_type=lambda: "tizi")
  is_c3x = _compile_function("_is_c3x", {"HARDWARE": hardware})
  assert is_c3x()
  hardware.get_device_type = lambda: "tici"
  assert not is_c3x()

  params = object()
  state = SimpleNamespace(params=params)
  status_label = _compile_function(
    "_full_frame_candidate_status_label",
    {"candidate_status": lambda actual: "pending_device_compile", "ui_state": state},
  )
  assert status_label() == "pending device compile"

  selected_name = _compile_function(
    "_selected_model_name",
    {
      "FULL_FRAME_CANDIDATE_NAME": "Full Frame FOV (Development)",
      "is_candidate_selected": lambda actual: True,
      "get_selected_model_name": lambda actual: "FullFrameFOVS110V1",
      "ui_state": state,
    },
  )
  assert selected_name() == "Full Frame FOV (Development)"


def test_candidate_selection_is_local_and_onroad_is_guarded():
  writes = []
  state = SimpleNamespace(
    params=SimpleNamespace(put=lambda key, value: writes.append((key, value))),
    is_offroad=lambda: True,
  )
  reset_calls = []
  candidate_calls = []
  self = SimpleNamespace(
    model_dialog=SimpleNamespace(selection_ref="full-frame-fov-s110-v1"),
    model_manager=SimpleNamespace(activeBundle=None, availableBundles=[]),
    _set_full_frame_candidate=lambda enabled: candidate_calls.append(enabled) or True,
    _show_reset_params_dialog=lambda: reset_calls.append(True),
  )
  env = {
    "is_centerline_candidate_ref": lambda ref: ref.startswith("cd210-centerline-"),
    "DialogResult": SimpleNamespace(CONFIRM=1),
    "CANDIDATE_REF": "full-frame-fov-s110-v1",
    "_is_c3x": lambda: True,
    "ui_state": state,
    "is_candidate_selected": lambda params: False,
    "is_default_bundle": lambda bundle: False,
    "select_default_model": lambda params: None,
    "_DOWNLOAD_INDEX_KEY": "ModelManager_DownloadIndex",
  }
  method = _compile_method("_on_model_selected", env)
  method(self, 1)
  assert candidate_calls == [True]
  assert reset_calls == [True]
  assert writes == []

  state.is_offroad = lambda: False
  candidate_calls.clear()
  self.model_dialog = SimpleNamespace(selection_ref="full-frame-fov-s110-v1")
  method(self, 1)
  assert candidate_calls == []
  assert writes == []


def test_existing_catalog_selection_still_queues_download():
  writes = []
  bundle = SimpleNamespace(ref="cloud-model", index=87, generation=12)
  state = SimpleNamespace(
    params=SimpleNamespace(put=lambda key, value: writes.append((key, value))),
    is_offroad=lambda: True,
  )
  self = SimpleNamespace(
    model_dialog=SimpleNamespace(selection_ref="cloud-model"),
    model_manager=SimpleNamespace(activeBundle=SimpleNamespace(generation=12), availableBundles=[bundle]),
    _set_full_frame_candidate=lambda enabled: True,
    _show_reset_params_dialog=lambda: None,
  )
  env = {
    "is_centerline_candidate_ref": lambda ref: ref.startswith("cd210-centerline-"),
    "DialogResult": SimpleNamespace(CONFIRM=1),
    "CANDIDATE_REF": "full-frame-fov-s110-v1",
    "_is_c3x": lambda: True,
    "ui_state": state,
    "is_candidate_selected": lambda params: False,
    "is_default_bundle": lambda active: False,
    "select_default_model": lambda params: None,
    "_DOWNLOAD_INDEX_KEY": "ModelManager_DownloadIndex",
  }
  method = _compile_method("_on_model_selected", env)
  method(self, 1)
  assert writes == [("ModelManager_DownloadIndex", 87)]


def test_toggle_activation_error_reverts_visible_state_and_reports_error():
  states = []
  errors = []
  helper_calls = []
  state = SimpleNamespace(params=object(), is_offroad=lambda: True)

  def fail(params, enabled, *, offroad):
    helper_calls.append((params, enabled, offroad))
    raise RuntimeError("candidate pending device compile")

  self = SimpleNamespace(
    full_frame_candidate_item=SimpleNamespace(action_item=SimpleNamespace(set_state=states.append)),
    current_model_item=SimpleNamespace(action_item=SimpleNamespace(set_value=lambda value: None)),
    _show_full_frame_candidate_error=errors.append,
  )
  env = {
    "_is_c3x": lambda: True,
    "ui_state": state,
    "set_candidate_enabled": fail,
    "is_candidate_selected": lambda params: False,
    "tr": lambda value: value,
    "_selected_model_name": lambda: "Default (CD210)",
  }
  method = _compile_method("_set_full_frame_candidate", env)
  assert method(self, True) is False
  assert helper_calls == [(state.params, True, True)]
  assert states == [False]
  assert len(errors) == 1 and isinstance(errors[0], RuntimeError)

  state.is_offroad = lambda: False
  helper_calls.clear()
  states.clear()
  errors.clear()
  assert method(self, True) is False
  assert helper_calls == []
  assert states == [False]
  assert errors == []
