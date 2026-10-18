"""Exercise a staged release with isolated Params; never replace live runtime."""
import gc
import importlib.abc
import importlib.util
import json
import os
from pathlib import Path
import sys

STAGE = Path(__file__).resolve().parent
PAYLOAD = STAGE / "payload"
LIVE = Path("/data/iqpilot")
os.environ.update(PYTEST_DISABLE_PLUGIN_AUTOLOAD="1", DEV="CPU", OPENBLAS_NUM_THREADS="1")
sys.dont_write_bytecode = True
sys.path[:0] = ["/data/native-action-spatial-stage-20261017/test-deps", str(STAGE / "runtime"),
                str(LIVE / "artifacts/package_runtime"), "/usr/libexec/iqpilot/python", str(LIVE)]
overrides = {p.relative_to(PAYLOAD).with_suffix("").as_posix().replace("/", "."): p
             for p in (PAYLOAD / "iqpilot").rglob("*.py") if p.name != "__init__.py"}


class StagedSource(importlib.abc.MetaPathFinder):
  def find_spec(self, fullname, path, target=None):
    if fullname in overrides:
      return importlib.util.spec_from_file_location(fullname, overrides[fullname])


sys.meta_path.insert(0, StagedSource())
import pytest
from iqpilot.common.prefix import OpenpilotPrefix


class Cleanup:
  @pytest.fixture(autouse=True)
  def collect_native_subscribers(self):
    yield
    gc.collect()


group = sys.argv[1]
tests = json.loads((STAGE / "test-groups.json").read_text())[group]
extra_plugins = ["-p", "pytest_mock"] if group == "inherited" else []
with OpenpilotPrefix(prefix="release-consolidation-" + group, clean_dirs_on_exit=True):
  result = pytest.main(["-c", "/dev/null", "--import-mode=importlib", "-q", "--tb=short", "-o", "addopts=",
                        "-p", "no:cacheprovider", *extra_plugins,
                        *[str(PAYLOAD / name) for name in tests]], plugins=[Cleanup()])
print(json.dumps({"group": group, "exit_code": result,
                  "loaded_staged_sources": {n: str(p) for n, p in overrides.items()
                                             if getattr(sys.modules.get(n), "__file__", None) == str(p)}}, indent=2))
raise SystemExit(result)
