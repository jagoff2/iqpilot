# Hyundai receive-input firmware correction

The classic Hyundai receive whitelist omitted CLU11 cruise-main input in stock-cruise and camera-SCC configurations, and omitted MDPS12 driver torque in all configurations. The correction registers both inputs in the common receive checks, removes duplicate longitudinal CLU11 checks, and resets button history when initializing safety mode. Validated cruise-main CAN input can now arm AOL, and received driver torque reaches the existing steering checks.

The included firmware retains the custom actuator profile already deployed on this C3X. It is not a generic upstream firmware build. This correction does not change the preceding deployment's actuator limits. The matching controller profile and regression tests accompany the firmware in `artifacts/package_runtime/iqdbc`.

- `firmware-source.patch` reconstructs the complete custom firmware source from matching source commit `ddaf4bd8f8b8924c4d8d1131a6e63e8e936eca0a`, including the earlier actuator profile and this correction.
- `hyundai-rx-fix.patch` contains only this correction: two Hyundai headers and the new receive-input regression tests. Apply it to the preceding custom source, not in addition to the cumulative patch.
- `build_manifest.json` records the source and firmware hashes, retained actuator profile, and validation status.

Both patches were reapplied in independent directories and checked byte-for-byte against all affected source files: 10 cumulative files and 3 correction files.

## Rebuild

Run on Linux with the repository containing the matching source commit. The verified toolchain is `arm-none-eabi-gcc 13.2.1 20231009`; the Python environment needs SCons and PyCryptodome. Build outside a Git worktree to retain the recorded `DEV-unknown-DEBUG` version string. The bundled developer signing key is selected by the default debug build.

```sh
repo_dir="$(pwd)"
task_build_dir="$(mktemp -d)"
git archive --format=tar --output="$task_build_dir/source.tar" \
  ddaf4bd8f8b8924c4d8d1131a6e63e8e936eca0a \
  artifacts/package_sources/panda artifacts/package_sources/iqdbc
tar -xf "$task_build_dir/source.tar" -C "$task_build_dir"

cd "$task_build_dir/artifacts/package_sources/iqdbc"
git -c core.autocrlf=false apply --check -p1 "$repo_dir/artifacts/safety_port/firmware-source.patch"
git -c core.autocrlf=false apply -p1 "$repo_dir/artifacts/safety_port/firmware-source.patch"

cd ../panda
export PATH="/usr/local/venv/bin:$PATH"
export PYTHONPATH="$task_build_dir/artifacts/package_sources/iqdbc"
unset RELEASE DEBUG CERT
/usr/local/venv/bin/python3 -m SCons --minimal board/obj/panda_h7.bin.signed
sha256sum board/obj/panda_h7.bin.signed
```

Expected firmware SHA-256: `9ed595d48b6c7b74cac0f67022df99b7af9470620de8a526b4a5e8564923f3fa`.

The packaged firmware is `artifacts/package_runtime/panda/board/obj/panda_h7.bin.signed`. Merely copying this file does not establish which firmware is running; deployment must confirm the Panda's running signature matches the rebuilt file.

## Validation

The rebuilt software safety library passed **346 tests**: 168 receive-input cases and 178 existing Hyundai/controller-boundary cases. Coverage includes real cruise-main CAN permission transitions, signed MDPS torque samples and driver override, malformed addressing, counter validation, mode reinitialization, receive timeouts, brake configurations, and the retained torque/rate limits.

The native test library was compiled from the same patched headers with `cc -shared -fPIC -O0 -g -DALLOW_DEBUG -std=gnu11 -Wall -Wextra -Werror -Wno-pointer-to-int-cast -I. iqdbc/safety/tests/libsafety/safety.c -o iqdbc/safety/tests/libsafety/libsafety.so`. Runtime-package tests loaded that rebuilt library explicitly.

Replay of all four failed routes processed **1,118,610 received CAN frames with zero invalid frames**. The correction accepted **8,170 of 8,195 nonzero steering commands**, with **zero rejections while lateral control was authorized** across seven engagement windows. The remaining 25 commands were correctly blocked after cruise-main off presses. Three late receive-timeout checks occurred after AOL was already off and matched the original route's existing CAN-fault interval.

Deployment verification confirmed the Panda's running firmware signature matches the packaged file, with UI and Panda processes running, no Panda faults, and ignition off. NNFF and Turn Guards remained off. Software replay and startup verification do not establish real-vehicle steering behavior; no post-fix drive is claimed here.
