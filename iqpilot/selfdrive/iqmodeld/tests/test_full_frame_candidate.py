from __future__ import annotations
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import pytest
from unittest import mock  # noqa: TID251

SOURCE = Path(__file__).resolve().parents[1] / 'full_frame_candidate.py'


class Params:
  def __init__(self, initial=None):
    self.values = dict(initial or {})

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class TestFullFrameCandidate:
  def setup_method(self):
    self.temp = tempfile.TemporaryDirectory()
    root = Path(self.temp.name)
    iqpilot = types.ModuleType('iqpilot')
    iqpilot.__path__ = []
    system = types.ModuleType('iqpilot.system')
    system.__path__ = []
    hardware = types.ModuleType('iqpilot.system.hardware')
    hardware.__path__ = []
    hw = types.ModuleType('iqpilot.system.hardware.hw')
    hw.Paths = type('Paths', (), {'model_root': staticmethod(lambda: str(root))})
    selfdrive = types.ModuleType('iqpilot.selfdrive')
    selfdrive.__path__ = []
    iqmodeld = types.ModuleType('iqpilot.selfdrive.iqmodeld')
    iqmodeld.__path__ = []
    models = types.ModuleType('iqpilot.selfdrive.iqmodeld.models')
    models.__path__ = []
    helpers = types.ModuleType('iqpilot.selfdrive.iqmodeld.models.helpers')

    def select_default_model(params):
      params.put('ModelManager_ActiveBundle', {'ref': 'default'})
      params.put('ModelRunnerTypeCache', 1)

    helpers.select_default_model = select_default_model
    modules = {'iqpilot': iqpilot,
        'iqpilot.system': system,
        'iqpilot.system.hardware': hardware,
        'iqpilot.system.hardware.hw': hw,
        'iqpilot.selfdrive': selfdrive,
        'iqpilot.selfdrive.iqmodeld': iqmodeld,
        'iqpilot.selfdrive.iqmodeld.models': models,
        'iqpilot.selfdrive.iqmodeld.models.helpers': helpers,}
    self.modules_patch = mock.patch.dict(sys.modules, modules)
    self.modules_patch.start()
    spec = importlib.util.spec_from_file_location('candidate_under_test', SOURCE)
    self.module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(self.module)
    self.root = root

  def teardown_method(self):
    self.modules_patch.stop()
    self.temp.cleanup()

  def descriptor(self, payload=b'qcom-pkl'):
    digest = hashlib.sha256(payload).hexdigest()
    value = {
      'index': 65001,
      'internalName': 'FullFrameFOVS110V1',
      'displayName': self.module.DISPLAY_NAME,
      'models': [
        {
          'type': 'supercombo',
          'artifact': {'fileName': 'full-frame-fov-s110-v1.pkl', 'downloadUri': {'uri': 'local://full-frame-fov-s110-v1.pkl', 'sha256': digest}},
        }
      ],
      'status': 'cached',
      'generation': 1,
      'environment': 'development',
      'runner': 'tinygrad',
      'is20hz': True,
      'ref': self.module.CANDIDATE_REF,
      'minimumSelectorVersion': 20,
      'overrides': [],
    }
    (self.root / self.module.DESCRIPTOR_FILE).write_text(json.dumps(value), encoding='utf-8')
    return (value, payload, digest)

  def make_ready(self):
    (descriptor, payload, digest) = self.descriptor()
    (self.root / 'full-frame-fov-s110-v1.pkl').write_bytes(payload)
    receipt = {
      'schema_version': 1,
      'status': 'passed_qcom_compile',
      'candidate_ref': self.module.CANDIDATE_REF,
      'source_onnx_sha256': self.module.EXPECTED_ONNX_SHA256,
      'runtime_overlay_manifest_sha256': self.module.EXPECTED_OVERLAY_MANIFEST_SHA256,
      'compiler_source_sha256': self.module.EXPECTED_COMPILER_SHA256,
      'expected_device': 'QCOM',
      'compiled_device': 'QCOM',
      'model_input_names': list(self.module.EXPECTED_INPUT_NAMES),
      'artifact': {'file_name': 'full-frame-fov-s110-v1.pkl', 'sha256': digest, 'bytes': len(payload)},
    }
    (self.root / self.module.RECEIPT_FILE).write_text(json.dumps(receipt), encoding='utf-8')
    return descriptor

  def test_pending_states_fail_closed(self):
    params = Params()
    assert self.module.candidate_status(params) == 'pending_device_compile'
    with pytest.raises(RuntimeError):
      self.module.set_candidate_enabled(params, True, offroad=True)
    self.descriptor()
    assert self.module.candidate_status(params) == 'pending_device_compile'
    assert not self.module.candidate_ready()

  def test_ready_requires_artifact_hash(self):
    self.make_ready()
    assert self.module.candidate_ready()
    (self.root / 'full-frame-fov-s110-v1.pkl').write_bytes(b'wrong')
    assert not self.module.candidate_ready()
    assert self.module.candidate_status(Params()) == 'invalid_qcom_evidence'

  def test_repeated_status_polls_do_not_rehash_unchanged_artifact(self):
    self.make_ready()
    params = Params()
    with mock.patch.object(self.module, '_sha256', wraps=self.module._sha256) as sha256:
      assert self.module.candidate_status(params) == 'ready'
      assert self.module.candidate_ready()
      assert self.module.candidate_status(params) == 'ready'
      assert sha256.call_count == 1

  def test_artifact_stat_change_invalidates_cached_validation(self):
    self.make_ready()
    artifact = self.root / 'full-frame-fov-s110-v1.pkl'
    assert self.module.candidate_ready()
    artifact.write_bytes(b'different-length')
    assert not self.module.candidate_ready()

  def test_enable_forces_hash_and_catches_stat_preserving_tamper(self):
    self.make_ready()
    artifact = self.root / 'full-frame-fov-s110-v1.pkl'
    assert self.module.candidate_ready()
    original_stat = artifact.stat()
    artifact.write_bytes(b'tampered')
    assert artifact.stat().st_size == original_stat.st_size
    os.utime(artifact, ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns))
    assert self.module.candidate_ready()
    previous = {'ref': 'previous'}
    params = Params({'ModelManager_ActiveBundle': previous})
    with pytest.raises(RuntimeError, match='not ready'):
      self.module.set_candidate_enabled(params, True, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == previous
    assert not (self.root / self.module.PREVIOUS_STATE_FILE).exists()

  def test_artifact_deletion_during_hash_returns_not_ready(self):
    self.make_ready()
    artifact = self.root / 'full-frame-fov-s110-v1.pkl'

    def disappear(_path):
      artifact.unlink()
      raise FileNotFoundError(str(artifact))

    with mock.patch.object(self.module, '_sha256', side_effect=disappear):
      assert not self.module.candidate_ready()
    assert self.module.candidate_status(Params()) == 'pending_device_compile'

  def test_enable_snapshot_write_failure_is_runtime_error_and_no_selection_change(self):
    self.make_ready()
    previous = {'ref': 'previous'}
    params = Params({'ModelManager_ActiveBundle': previous})
    with mock.patch.object(self.module, '_atomic_write_json', side_effect=OSError('disk unavailable')):
      with pytest.raises(RuntimeError, match='rollback state'):
        self.module.set_candidate_enabled(params, True, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == previous

  def test_enable_and_restore_previous(self):
    descriptor = self.make_ready()
    (self.root / 'previous.pkl').write_bytes(b'previous')
    previous = {'ref': 'previous', 'displayName': 'Previous', 'models': [{'type': 'supercombo', 'artifact': {'fileName': 'previous.pkl'}}]}
    params = Params({'ModelManager_ActiveBundle': previous, 'ModelRunnerTypeCache': 1, 'ModelManager_DownloadIndex': 9, 'ModelManager_PendingIndex': 9})
    self.module.set_candidate_enabled(params, True, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == descriptor
    assert params.get('ModelRunnerTypeCache') is None
    assert params.get('ModelManager_DownloadIndex') is None
    assert (self.root / self.module.PREVIOUS_STATE_FILE).is_file()
    self.module.set_candidate_enabled(params, False, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == previous
    assert params.get('ModelRunnerTypeCache') is None
    assert not (self.root / self.module.PREVIOUS_STATE_FILE).exists()

  def test_disable_does_not_clobber_later_selection(self):
    self.make_ready()
    params = Params({'ModelManager_ActiveBundle': {'ref': 'previous'}})
    self.module.set_candidate_enabled(params, True, offroad=True)
    later = {'ref': 'later'}
    params.put('ModelManager_ActiveBundle', later)
    self.module.set_candidate_enabled(params, False, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == later

  def test_disable_without_previous_selects_default(self):
    self.make_ready()
    params = Params()
    self.module.set_candidate_enabled(params, True, offroad=True)
    self.module.set_candidate_enabled(params, False, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == {'ref': 'default'}
    assert params.get('ModelRunnerTypeCache') == 1

  def test_disable_falls_back_default_when_previous_files_were_purged(self):
    self.make_ready()
    previous_file = self.root / 'previous.pkl'
    previous_file.write_bytes(b'previous')
    previous = {'ref': 'previous', 'models': [{'type': 'supercombo', 'artifact': {'fileName': previous_file.name}}]}
    params = Params({'ModelManager_ActiveBundle': previous})
    self.module.set_candidate_enabled(params, True, offroad=True)
    previous_file.unlink()
    self.module.set_candidate_enabled(params, False, offroad=True)
    assert params.get('ModelManager_ActiveBundle') == {'ref': 'default'}

  def test_offroad_guard_both_directions(self):
    self.make_ready()
    params = Params()
    with pytest.raises(RuntimeError):
      self.module.set_candidate_enabled(params, True, offroad=False)
    with pytest.raises(RuntimeError):
      self.module.set_candidate_enabled(params, False, offroad=False)

  def test_selected_status(self):
    descriptor = self.make_ready()
    params = Params({'ModelManager_ActiveBundle': descriptor})
    assert self.module.is_candidate_selected(params)
    assert self.module.candidate_status(params) == 'selected'
