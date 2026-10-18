"""Default-off activation helper for the C3X full-frame development candidate.

The helper intentionally uses only existing model-manager Params.  A candidate
cannot be selected until a native ModelBundle descriptor, device compilation
receipt, and QCOM PKL form one hash-bound set in ``Paths.model_root()``.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any

from iqpilot.system.hardware.hw import Paths


CANDIDATE_REF = "full-frame-fov-s110-v1"
DISPLAY_NAME = "Full Frame FOV (Development)"

EXPECTED_ONNX_SHA256 = "e73d721007ca366754bafdb8907a973ee506d27bfe027806af0d5aa0b4553e48"
EXPECTED_OVERLAY_MANIFEST_SHA256 = "270ef7664f9e460fd6652c92056000b22881a7cd71b04e91f220cd5f5488c1ec"
EXPECTED_COMPILER_SHA256 = "168b4e318c3bbc78d3fc7502cf202a2ca1a66ce344c8adcee8f6df655125529a"
EXPECTED_INPUT_NAMES = (
  "img", "big_img", "desire_pulse", "traffic_convention", "action_t",
  "features_buffer", "peripheral_img",
)

DESCRIPTOR_FILE = f"{CANDIDATE_REF}.bundle.json"
RECEIPT_FILE = f"{CANDIDATE_REF}.compile-receipt.json"
PREVIOUS_STATE_FILE = f".{CANDIDATE_REF}.previous.json"

_ACTIVE_BUNDLE_KEY = "ModelManager_ActiveBundle"
_RUNNER_CACHE_KEY = "ModelRunnerTypeCache"
_DOWNLOAD_INDEX_KEY = "ModelManager_DownloadIndex"
_PENDING_INDEX_KEY = "ModelManager_PendingIndex"

_validation_cache: tuple[tuple[Any, ...], dict[str, Any] | None,
                         dict[str, Any] | None, Path | None, bool, str] | None = None


def _model_root() -> Path:
  return Path(Paths.model_root())


def _read_json(path: Path) -> dict[str, Any] | None:
  try:
    value = json.loads(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else None
  except (OSError, UnicodeError, json.JSONDecodeError):
    return None


def _sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def _fingerprint(path: Path) -> tuple[bool, int | None, int | None, int | None]:
  try:
    stat = path.stat()
    return True, int(stat.st_size), int(stat.st_mtime_ns), int(stat.st_ino)
  except OSError:
    return False, None, None, None


def _artifact_record(descriptor: dict[str, Any]) -> tuple[str, str] | None:
  models = descriptor.get("models")
  if not isinstance(models, list) or len(models) != 1 or not isinstance(models[0], dict):
    return None
  if models[0].get("type") != "supercombo":
    return None
  artifact = models[0].get("artifact")
  if not isinstance(artifact, dict):
    return None
  file_name = artifact.get("fileName")
  download_uri = artifact.get("downloadUri")
  digest = download_uri.get("sha256") if isinstance(download_uri, dict) else None
  if (not isinstance(file_name, str) or not file_name or Path(file_name).name != file_name
      or not isinstance(digest, str) or len(digest) != 64):
    return None
  return file_name, digest.lower()


def _descriptor_valid(descriptor: dict[str, Any] | None) -> bool:
  if descriptor is None:
    return False
  return bool(
    descriptor.get("ref") == CANDIDATE_REF
    and descriptor.get("displayName") == DISPLAY_NAME
    and descriptor.get("runner") == "tinygrad"
    and descriptor.get("is20hz") is True
    and descriptor.get("environment") == "development"
    and _artifact_record(descriptor) is not None
  )


def _read_evidence() -> tuple[dict[str, Any] | None, dict[str, Any] | None, Path | None]:
  root = _model_root()
  descriptor = _read_json(root / DESCRIPTOR_FILE)
  receipt = _read_json(root / RECEIPT_FILE)
  artifact = _artifact_record(descriptor) if _descriptor_valid(descriptor) else None
  return descriptor, receipt, (root / artifact[0] if artifact else None)


def _receipt_valid(descriptor: dict[str, Any], receipt: dict[str, Any] | None,
                   artifact_path: Path) -> bool:
  if receipt is None:
    return False
  artifact = _artifact_record(descriptor)
  receipt_artifact = receipt.get("artifact")
  if artifact is None or not isinstance(receipt_artifact, dict):
    return False
  file_name, descriptor_sha = artifact
  try:
    expected_bytes = int(receipt_artifact["bytes"])
  except (KeyError, TypeError, ValueError):
    return False
  try:
    return bool(
      receipt.get("schema_version") == 1
      and receipt.get("status") == "passed_qcom_compile"
      and receipt.get("candidate_ref") == CANDIDATE_REF
      and str(receipt.get("source_onnx_sha256", "")).lower() == EXPECTED_ONNX_SHA256
      and str(receipt.get("runtime_overlay_manifest_sha256", "")).lower() == EXPECTED_OVERLAY_MANIFEST_SHA256
      and str(receipt.get("compiler_source_sha256", "")).lower() == EXPECTED_COMPILER_SHA256
      and receipt.get("expected_device") == "QCOM"
      and receipt.get("compiled_device") == "QCOM"
      and tuple(receipt.get("model_input_names", ())) == EXPECTED_INPUT_NAMES
      and receipt_artifact.get("file_name") == file_name
      and str(receipt_artifact.get("sha256", "")).lower() == descriptor_sha
      and artifact_path.is_file()
      and artifact_path.stat().st_size == expected_bytes
      and _sha256(artifact_path) == descriptor_sha
    )
  except OSError:
    # Cache removal can race a UI status poll or an enable request.
    return False


def _validation_snapshot(*, force_rehash: bool = False):
  global _validation_cache
  root = _model_root()
  descriptor_path, receipt_path = root / DESCRIPTOR_FILE, root / RECEIPT_FILE
  descriptor = _read_json(descriptor_path)
  receipt = _read_json(receipt_path)
  artifact = _artifact_record(descriptor) if _descriptor_valid(descriptor) else None
  artifact_path = root / artifact[0] if artifact else None
  fingerprints: tuple[Any, ...] = (
    _fingerprint(descriptor_path), _fingerprint(receipt_path),
    _fingerprint(artifact_path) if artifact_path is not None else (False, None, None, None),
  )
  if not force_rehash and _validation_cache is not None and _validation_cache[0] == fingerprints:
    return _validation_cache[1:]

  descriptor_fingerprint, receipt_fingerprint, artifact_fingerprint = fingerprints
  if (not _descriptor_valid(descriptor) or receipt is None or artifact_path is None
      or not descriptor_fingerprint[0] or not receipt_fingerprint[0]
      or not artifact_fingerprint[0]):
    ready, status = False, "pending_device_compile"
  else:
    ready = _receipt_valid(descriptor, receipt, artifact_path)
    status = "ready" if ready else "invalid_qcom_evidence"
  _validation_cache = (fingerprints, descriptor, receipt, artifact_path, ready, status)
  return _validation_cache[1:]


def candidate_ready() -> bool:
  return bool(_validation_snapshot()[3])


def is_candidate_selected(params) -> bool:
  value = params.get(_ACTIVE_BUNDLE_KEY)
  return isinstance(value, dict) and value.get("ref") == CANDIDATE_REF


def candidate_status(params) -> str:
  descriptor, receipt, artifact_path, ready, status = _validation_snapshot()
  if is_candidate_selected(params):
    return "selected" if ready else "selected_invalid"
  return status


def _json_safe(value: Any) -> Any:
  if isinstance(value, bytes):
    value = value.decode("utf-8")
  # Round trip rejects non-JSON Params values before any active selection changes.
  return json.loads(json.dumps(value))


def _atomic_write_json(path: Path, value: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + ".tmp")
  with temporary.open("x", encoding="utf-8", newline="\n") as stream:
    json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
  os.replace(temporary, path)


def _remove_if_present(path: Path) -> None:
  try:
    path.unlink()
  except FileNotFoundError:
    pass


def _bundle_files_ready(bundle: Any) -> bool:
  if not isinstance(bundle, dict) or not isinstance(bundle.get("models"), list) or not bundle["models"]:
    return False
  saw_file = False
  try:
    for model in bundle["models"]:
      if not isinstance(model, dict):
        return False
      for key in ("metadata", "artifact"):
        record = model.get(key)
        file_name = record.get("fileName") if isinstance(record, dict) else None
        if not file_name:
          continue
        if not isinstance(file_name, str) or Path(file_name).name != file_name:
          return False
        saw_file = True
        path = _model_root() / file_name
        if not path.is_file() or path.stat().st_size <= 0:
          return False
    return saw_file
  except OSError:
    return False


def set_candidate_enabled(params, enabled, *, offroad: bool) -> None:
  """Enable or roll back the local candidate.

  Both transitions require an explicit offroad state.  Enabling never queues a
  download and fails closed until ``candidate_ready``.  Disabling restores only
  while this candidate remains active, so a later user selection is untouched.
  """
  if not offroad:
    raise RuntimeError("full-frame candidate can only be changed while offroad")
  if not isinstance(enabled, bool):
    raise TypeError("enabled must be bool")

  root = _model_root()
  state_path = root / PREVIOUS_STATE_FILE
  if enabled:
    if is_candidate_selected(params):
      if not _validation_snapshot(force_rehash=True)[3]:
        raise RuntimeError("selected full-frame candidate failed forced artifact validation")
      return
    descriptor, _receipt, _artifact_path, ready, status = _validation_snapshot(force_rehash=True)
    if not ready or descriptor is None:
      raise RuntimeError(f"full-frame candidate is not ready: {status}")
    previous = params.get(_ACTIVE_BUNDLE_KEY)
    state = {
      "schema_version": 1,
      "candidate_ref": CANDIDATE_REF,
      "previous_active_bundle": _json_safe(previous) if previous is not None else None,
    }
    try:
      _remove_if_present(state_path.with_name(state_path.name + ".tmp"))
      _atomic_write_json(state_path, state)
    except OSError as error:
      raise RuntimeError(f"failed to save full-frame rollback state: {error}") from error
    params.remove(_DOWNLOAD_INDEX_KEY)
    params.remove(_PENDING_INDEX_KEY)
    params.remove(_RUNNER_CACHE_KEY)
    params.put(_ACTIVE_BUNDLE_KEY, descriptor)
    return

  # Do not undo a model that the operator selected after enabling this one.
  if not is_candidate_selected(params):
    return
  state = _read_json(state_path)
  params.remove(_DOWNLOAD_INDEX_KEY)
  params.remove(_PENDING_INDEX_KEY)
  params.remove(_RUNNER_CACHE_KEY)
  previous = state.get("previous_active_bundle") if isinstance(state, dict) else None
  if (isinstance(previous, dict) and previous.get("ref") != "default"
      and _bundle_files_ready(previous)):
    params.put(_ACTIVE_BUNDLE_KEY, previous)
  else:
    from iqpilot.selfdrive.iqmodeld.models.helpers import select_default_model
    select_default_model(params)
  _remove_if_present(state_path)
