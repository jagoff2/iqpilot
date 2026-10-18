"""Copy release sources to a separate device test directory."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in [HERE.parent / "iqpilot", *HERE.parents] if (p / ".git").exists())
HOST = "iq@192.168.1.95"
REMOTE = "/data/release-consolidation-tests-20261018"
groups = {
  "action": [
    "iqpilot/selfdrive/iqmodeld/tests/test_action_provenance.py",
    "iqpilot/selfdrive/iqmodeld/tests/test_action_dispatch.py",
    "iqpilot/selfdrive/controls/lib/tests/test_curvature_lookahead.py"],
  "inherited": [
    "iqpilot/selfdrive/controls/lib/tests/test_spatial_heading.py",
    "iqpilot/selfdrive/controls/tests/test_lat_jerk_lowpass.py",
    "iqpilot/common/tests/test_nnff_params.py",
    "iqpilot/common/tests/test_turn_guard_settings.py",
    "iqpilot/selfdrive/ui/tests/test_hkg_feedback_settings.py",
    "iqpilot/selfdrive/ui/tests/test_nnff_big_settings.py",
    "iqpilot/selfdrive/ui/tests/test_nnff_settings.py",
    "iqpilot/selfdrive/ui/tests/test_turn_guard_settings.py"],
  "controls": ["iqpilot/selfdrive/controls/tests/" + n for n in [
    "test_actuator_feedback.py", "test_actuator_feedback_integration.py", "test_torque_friction.py",
    "test_lateral_low_speed.py", "test_turn_guards.py", "test_standard_unwind.py",
    "test_hkg_reduced_torque_feedback.py", "test_latcontrol_torque_buffer.py",
    "test_invalid_lateral_action.py", "test_lateral_action_selection.py", "test_nnff_controls.py"]],
  "models": ["iqpilot/selfdrive/iqmodeld/tests/" + n for n in [
    "test_cd210_centerline_candidates.py", "test_full_frame_candidate.py", "test_local_model_picker.py",
    "test_peripheral_runtime.py", "test_standard_spatial_action.py", "test_supercombo_runner_guards.py",
    "test_compile_supercombo_guards.py", "test_temporal_replay.py", "test_memory_carry.py",
    "test_split_input_state.py", "test_default_model_fallback.py"]],
  "ui": ["iqpilot/ui/tests/test_big_model_picker.py", "iqpilot/ui/tests/test_backup_model_path.py",
         "iqpilot/selfdrive/ui/tests/test_offline_model_selection.py"],
  "monitoring": ["iqpilot/selfdrive/monitoring/test_monitoring.py"],
  "nnff": ["iqpilot/selfdrive/controls/lib/neural_network_feed_forward/tests/" + n
           for n in ["test_locator.py", "test_network.py", "test_nnff_assembly.py"]],
}
(HERE / "test-groups.json").write_text(json.dumps(groups, indent=2) + "\n")
names = set(subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-m", "-o", "--exclude-standard", "-z"]).decode().split("\0")) - {""}
names.update(subprocess.check_output(["git", "-C", str(ROOT), "diff", "--name-only", "e415672", "-z"]).decode().split("\0"))
names = {p for p in names if p and not p.startswith("artifacts/release_consolidation_20261018/")}
for folder in ["iqpilot/selfdrive/iqmodeld", "iqpilot/selfdrive/controls/tests",
               "iqpilot/selfdrive/controls/lib/neural_network_feed_forward/tests"]:
  names.update(p.relative_to(ROOT).as_posix() for p in (ROOT / folder).rglob("*")
               if p.is_file() and p.suffix in {".py", ".json", ".npz"})
names.update(n for paths in groups.values() for n in paths)
names.add("iqpilot/selfdrive/ui/mici/layouts/settings/models.py")
manifest = {}
with tarfile.open(HERE / "test-stage.tar.gz", "w:gz") as archive:
  for name in sorted(names):
    p = ROOT / name
    if p.is_file():
      archive.add(p, arcname="payload/" + name)
      manifest[name] = hashlib.sha256(p.read_bytes()).hexdigest()
  for name in ["run_tests.py", "test-groups.json"]:
    archive.add(HERE / name, arcname=name)
(HERE / "test-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=12", HOST]
subprocess.run(ssh + ["mkdir -p " + REMOTE + "/runtime"], check=True)
subprocess.run(["scp", "-q", str(HERE / "test-stage.tar.gz"), HOST + ":" + REMOTE + "/test-stage.tar.gz"], check=True)
subprocess.run(ssh + ["test -d " + REMOTE + "/runtime/iqdbc || cp -a /data/iqpilot/artifacts/package_runtime/iqdbc " + REMOTE + "/runtime/"], check=True)
subprocess.run(ssh + ["tar -xzf " + REMOTE + "/test-stage.tar.gz -C " + REMOTE + " && cp " + REMOTE + "/payload/artifacts/package_runtime/iqdbc/car/car.capnp " + REMOTE + "/runtime/iqdbc/car/car.capnp"], check=True)
print("Staged", len(manifest), "files without changing the live runtime.")
