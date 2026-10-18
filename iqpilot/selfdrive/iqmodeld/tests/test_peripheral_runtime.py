from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import sys
import types
import unittest  # noqa: TID251
from unittest import mock  # noqa: TID251

import numpy as np


HERE = Path(__file__).resolve().parent
STAGE = HERE.parents[3]
ORIGINAL = HERE / "fixtures/temporal_state_before_peripheral.py"


def load_path(name: str, path: Path):
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  assert spec.loader is not None
  sys.modules[name] = module
  spec.loader.exec_module(module)
  return module


def load_staged_runner():
  def package(name):
    module = types.ModuleType(name)
    sys.modules[name] = module
    module.__path__ = []
    return module

  for name in (
    "iqpilot",
    "iqpilot.common",
    "iqpilot.selfdrive",
    "iqpilot.selfdrive.iqmodeld",
    "iqpilot.selfdrive.iqmodeld.models",
    "iqpilot.selfdrive.iqmodeld.models.runners",
    "iqpilot.selfdrive.iqmodeld.models.runners.tinygrad",
  ):
    package(name)
  loader = types.ModuleType("iqpilot._proprietary_loader")

  class Missing(Exception):
    pass

  loader.ProprietaryModuleMissing = Missing
  loader.load_private_module = lambda *_args, **_kwargs: (_ for _ in ()).throw(Missing())
  sys.modules[loader.__name__] = loader
  params = types.ModuleType("iqpilot.common.params")
  params.Params = type("Params", (), {})
  sys.modules[params.__name__] = params
  model_runner = types.ModuleType("iqpilot.selfdrive.iqmodeld.models.runners.model_runner")
  model_runner.CUSTOM_MODEL_PATH = ""
  model_runner.NumpyDict = dict
  model_runner.ShapeDict = dict
  model_runner.SliceDict = dict
  model_runner.ModelRunner = type("ModelRunner", (), {})
  sys.modules[model_runner.__name__] = model_runner
  constants = types.ModuleType("iqpilot.selfdrive.iqmodeld.models.split_model_constants")
  constants.SplitModelConstants = type("SplitModelConstants", (), {})
  sys.modules[constants.__name__] = constants
  parser = types.ModuleType("iqpilot.selfdrive.iqmodeld.parser")
  parser.PhaseParser = type("PhaseParser", (), {})
  sys.modules[parser.__name__] = parser
  iqc = types.ModuleType("iqpilot.selfdrive.iqmodeld.models.iqc")
  iqc.load_pickle = lambda _path: None
  sys.modules[iqc.__name__] = iqc
  load_path("iqpilot.selfdrive.iqmodeld.temporal_state", STAGE / "iqpilot/selfdrive/iqmodeld/temporal_state.py")
  load_path(
    "iqpilot.selfdrive.iqmodeld.models.runners.tinygrad.supercombo_runtime", STAGE / "iqpilot/selfdrive/iqmodeld/models/runners/tinygrad/supercombo_runtime.py"
  )
  return load_path("staged_peripheral_runner", STAGE / "iqpilot/selfdrive/iqmodeld/models/runners/tinygrad/supercombo_runner.py")


class PeripheralRuntimeV2Test(unittest.TestCase):
  def setUp(self):
    self.module_patch = mock.patch.dict(sys.modules)
    self.module_patch.start()

  def tearDown(self):
    self.module_patch.stop()

  @classmethod
  def setUpClass(cls):
    proprietary = types.ModuleType("iqpilot._proprietary_loader")

    class Missing(Exception):
      pass

    proprietary.ProprietaryModuleMissing = Missing
    proprietary.load_private_module = lambda *_args, **_kwargs: (_ for _ in ()).throw(Missing())
    old_iqpilot = sys.modules.get("iqpilot")
    old_loader = sys.modules.get("iqpilot._proprietary_loader")
    sys.modules["iqpilot"] = types.ModuleType("iqpilot")
    sys.modules["iqpilot._proprietary_loader"] = proprietary
    try:
      cls.staged = load_path("staged_peripheral_temporal", STAGE / "iqpilot/selfdrive/iqmodeld/temporal_state.py")
      cls.original = load_path("original_temporal_for_parity", ORIGINAL)
    finally:
      if old_iqpilot is None:
        sys.modules.pop("iqpilot", None)
      else:
        sys.modules["iqpilot"] = old_iqpilot
      if old_loader is None:
        sys.modules.pop("iqpilot._proprietary_loader", None)
      else:
        sys.modules["iqpilot._proprietary_loader"] = old_loader

  def test_abi6_preserves_original_temporal_state(self):
    rng = np.random.default_rng(110)
    old = self.original.TemporalInputState(4)
    new = self.staged.TemporalInputState(4)
    for tick in range(9):
      warped = rng.integers(0, 256, (2, 6, 128, 256), dtype=np.uint8)
      desire = np.eye(8, dtype=np.float32)[tick % 8]
      traffic = np.asarray([1, 0], np.float32)
      action = np.asarray([0.25, 0.35], np.float32)
      expected = old.push_and_materialize(warped, desire, traffic, action)
      actual = new.push_and_materialize(warped, desire, traffic, action)
      self.assertEqual(expected.keys(), actual.keys())
      for name in expected:
        self.assertEqual(expected[name].dtype, actual[name].dtype, name)
        self.assertTrue(np.array_equal(expected[name], actual[name]), name)

  def test_abi7_preserves_legacy_inputs_and_four_frame_peripheral_history(self):
    spec = dict(self.staged.MODEL_INPUT_SPEC)
    spec["peripheral_img"] = ((1, 12, 128, 256), "uint8")
    staged = self.staged.TemporalInputState(4, spec)
    legacy = self.original.TemporalInputState(4)
    rng = np.random.default_rng(220)
    frames = []
    for tick in range(11):
      warped = rng.integers(0, 256, (2, 6, 128, 256), dtype=np.uint8)
      peripheral = rng.integers(0, 256, (6, 128, 256), dtype=np.uint8)
      frames.append(peripheral)
      desire, traffic, action = np.zeros(8, np.float32), np.asarray([1, 0], np.float32), np.asarray([0.25, 0.35], np.float32)
      actual = staged.push_and_materialize(warped, desire, traffic, action, peripheral=peripheral)
      expected = legacy.push_and_materialize(warped, desire, traffic, action)
      for key in expected:
        np.testing.assert_array_equal(actual[key], expected[key])
      older = frames[tick - 4] if tick >= 4 else np.zeros_like(peripheral)
      np.testing.assert_array_equal(actual["peripheral_img"], np.concatenate([older, peripheral])[None])

  def test_dynamic_layout_keeps_peripheral_out_of_packed_scalars(self):
    shapes = {name: row[0] for name, row in self.staged.MODEL_INPUT_SPEC.items()}
    shapes["peripheral_img"] = (1, 12, 128, 256)
    self.assertNotIn("peripheral_img", self.staged.packed_policy_layout(shapes))
    self.assertEqual(self.staged.spec_from_meta({"input_shapes": shapes})["peripheral_img"][1], "uint8")
    bad = dict(self.staged.MODEL_INPUT_SPEC)
    bad["peripheral_img"] = ((1, 6, 128, 256), "uint8")
    with self.assertRaisesRegex(ValueError, "shapes must match"):
      self.staged.TemporalInputState(4, bad)

  def test_compiler_and_runtime_have_dynamic_abi_contracts(self):
    compiler_path = STAGE / "iqpilot/selfdrive/iqmodeld/tools/compile_supercombo.py"
    compiler = compiler_path.read_text(encoding="utf-8")
    tree = ast.parse(compiler)
    functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    self.assertIn("make_full_wide_prepare", functions)
    self.assertIn("policy_inputs", functions)
    self.assertIn("runtime_jit_contract", compiler)
    self.assertIn("Y16 UV128 padding", compiler)
    policy_node = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "policy_inputs")
    base_node = next(
      node
      for node in tree.body
      if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "BASE_POLICY_INPUTS" for target in node.targets)
    )
    namespace = {}
    ast.fix_missing_locations(policy_node)
    ast.fix_missing_locations(base_node)
    exec(compile(ast.Module(body=[base_node, policy_node], type_ignores=[]), str(compiler_path), "exec"), namespace)
    abi6 = namespace["policy_inputs"]({"img": (), "big_img": ()})
    abi7 = namespace["policy_inputs"]({"img": (), "big_img": (), "peripheral_img": ()})
    self.assertEqual(abi7, abi6 + ["peripheral_img_q"])
    runtime = (STAGE / "iqpilot/selfdrive/iqmodeld/models/runners/tinygrad/supercombo_runtime.py").read_text()
    runner = (STAGE / "iqpilot/selfdrive/iqmodeld/models/runners/tinygrad/supercombo_runner.py").read_text()
    self.assertIn("peripheral_img_q", runtime)
    self.assertIn("peripheral_img_q", runner)
    self.assertIn("physical camera buffers", runner.lower())

  def test_full_wide_geometry_keeps_sensor_aspect_and_neutral_padding(self):
    compiler = STAGE / "iqpilot/selfdrive/iqmodeld/tools/compile_supercombo.py"
    tree = ast.parse(compiler.read_text(encoding="utf-8"))
    node = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "full_wide_geometry")
    namespace = {"np": np}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(compiler), "exec"), namespace)
    matrix, mask, padding = namespace["full_wide_geometry"](types.SimpleNamespace(width=1928, height=1208), 512, 256)
    np.testing.assert_allclose(matrix[0, 0], matrix[1, 1])
    # Pixel-center coordinates map the midpoint of the destination to the sensor midpoint.
    np.testing.assert_allclose(matrix @ np.array([255.5, 127.5, 1.0]), [963.5, 603.5, 1.0], atol=1e-4)
    assert mask.shape == (6, 128, 256)
    assert mask[:, :, 0].sum() == 0
    assert mask[:, :, 128].all()
    np.testing.assert_array_equal(padding[:, 0, 0], [16, 16, 16, 16, 128, 128])

  def test_actual_runner_validator_accepts_abi6_and_abi7(self):
    module = load_staged_runner()
    runner_type = module.TinygradSupercomboRunner

    class Jit:
      def __init__(self, names):
        self.captured = types.SimpleNamespace(expected_names=names)

    base_shapes = {
      "img": (1, 12, 128, 256),
      "big_img": (1, 12, 128, 256),
      "features_buffer": (1, 24, 512),
      "desire_pulse": (1, 25, 8),
      "traffic_convention": (1, 2),
      "action_t": (1, 2),
    }
    base_policy = ['img_q', 'big_img_q', 'feat_q', 'desire_q', 'packed_npy_inputs']
    warp_names = ['big_frame', 'big_tfm', 'frame', 'tfm']
    for abi7 in (False, True):
      runner = runner_type.__new__(runner_type)
      runner._ish = dict(base_shapes)
      if abi7:
        runner._ish["peripheral_img"] = (1, 12, 128, 256)
      policy = base_policy + (["peripheral_img_q"] if abi7 else [])
      runner._meta = {
        "input_shapes": runner._ish,
        "input_dtypes": {"peripheral_img": "uint8"} if abi7 else {},
        "runtime_jit_contract": {
          "warp_inputs": ['tfm', 'big_tfm', 'frame', 'big_frame'],
          "warp_outputs": ['img', 'big_img'] + (["peripheral_img"] if abi7 else []),
          "policy_inputs": ['warped', *policy],
          "physical_camera_buffers": ['img', 'big_img'],
        },
      }
      runner._run_policy = Jit(['warped', *policy])
      runner._warp_jits = {(1928, 1208): Jit(warp_names)}
      runner._pkl_path = str(STAGE / "absent-test.pkl")
      runner._expected_sha256 = ""
      runner._validate_input_contract()
      runner._validate_jit_names()

  def test_actual_runner_validator_rejects_wrong_contract_shape_and_dtype(self):
    module = load_staged_runner()
    runner_type = module.TinygradSupercomboRunner
    base = {
      "img": (1, 12, 128, 256),
      "big_img": (1, 12, 128, 256),
      "features_buffer": (1, 24, 512),
      "desire_pulse": (1, 25, 8),
      "traffic_convention": (1, 2),
      "peripheral_img": (1, 12, 128, 256),
    }
    runner = runner_type.__new__(runner_type)
    runner._ish = dict(base)
    runner._meta = {"input_shapes": runner._ish, "input_dtypes": {"peripheral_img": "float16"}}
    with self.assertRaisesRegex(RuntimeError, "dtype must be uint8"):
      runner._validate_input_contract()
    runner._meta["input_dtypes"]["peripheral_img"] = "uint8"
    runner._ish["peripheral_img"] = (1, 6, 128, 256)
    with self.assertRaisesRegex(RuntimeError, "shapes must match"):
      runner._validate_input_contract()
    runner._ish["peripheral_img"] = (1, 12, 128, 256)

    class Jit:
      captured = types.SimpleNamespace(expected_names=[])

    runner._run_policy = Jit()
    runner._warp_jits = {}
    runner._pkl_path = str(STAGE / "absent-test.pkl")
    runner._expected_sha256 = ""
    runner._meta["runtime_jit_contract"] = {
      "warp_inputs": ['tfm', 'big_tfm', 'frame', 'big_frame'],
      "warp_outputs": ['img', 'big_img', 'peripheral_img'],
      "policy_inputs": ['img_q'],
      "physical_camera_buffers": ['img', 'big_img'],
    }
    with self.assertRaisesRegex(RuntimeError, "runtime_jit_contract differs"):
      runner._validate_jit_names()


if __name__ == "__main__":
  unittest.main()
