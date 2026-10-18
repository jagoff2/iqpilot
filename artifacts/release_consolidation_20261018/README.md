# Release consolidation, 2026-10-18

Release now preserves the completed controller fixes and custom-model integrations
that previously existed across Git history, the working tree, and device-only
installations. This consolidation does not deploy or reset the live device.

## Complete standard torque fix chain

| Included commit or payload | Behavior preserved |
| --- | --- |
| `20ac940` | Controller selection, NNFF settings and torque-controller wiring |
| `1161003` | Optional corrected cut/apex turn guards with independent reference history |
| `648d3c3` | Hyundai AOL and driver-torque CAN reception prerequisites |
| `2a8f53e` | Feedback-unit corrections and Hyundai low-speed steering-oscillation fixes |
| `e415672` | Distance-based heading/velocity path conversion, invalid-reference handling, native-head precedence, and projected standard-controller unwind |
| Qualified October 17 repair payload | Exact originating request/applied-output pairing; independent slew-matched derivative history seeded on engagement; bounded friction preview; directional integrator recovery with conservative reversal and stale-feedback handling |

Both `origin/fix/lateral-low-speed` and
`origin/experiment/steering-turn-guards` were already ancestors of release.
The explicit merge returned `Already up to date`. Their exact tips, the
standard-turn-unwind branch, and the prerequisite branch tips are recorded in
[source-manifest.json](source-manifest.json).

The five latest controller runtime files match the previously qualified and
installed payload byte-for-byte. No additional gain, roll-compensation, integral
leak, or actuator-limit tuning was introduced. The rejected native-action
spatial experiment and unfinished NNFF future-reference prototype are excluded;
neither was a completed, accepted controller fix.

The native `action` output path receives these same shared controller fixes.
The original [repair report](controller-repair-report.md) explicitly identifies
`nativeAction` applicability, and the [continuous replay report](controller-replay-report.md)
retains the incident route's recorded native-action targets. Action dispatch,
provenance, and curvature-selection tests are recorded in [action.log](action.log).

The earlier repair qualification covered 313 device tests, 220,164 replayed
control ticks, and 223,232 candidate CAN messages, with no added software-safety
rejections or trace gaps. The original receipt is preserved in
[controller-qualification.json](controller-qualification.json). These are
software and recorded-input checks, not a physical vehicle-response test.

## Other preserved changes

- All 114 NNFF model JSON files and their MIT license are now tracked.
- Existing monitoring edits are preserved: passive awareness timeout 300 seconds
  and active distraction timeout 1,100 seconds, with their adjusted tests.
- The experimental picker and selection helpers for CD210 Centerline s24,
  CD210 Centerline s71, and Full Frame FOV are restored from their installed
  snapshots and receipts.
- The Full Frame metadata, temporal state, runtime, runner, and compiler support
  are restored, including the seventh peripheral image input and the existing
  six-input contract.
- [Local model descriptors and receipts](../local_models/inventory.json) record
  all nine verified artifacts. Large model binaries remain in persistent
  `/data/media/0/models` and local export packages; Git contains the integration
  source, descriptors, receipts, and hashes, not those binaries.
- Git attributes preserve exact JSON receipt/payload bytes so newline conversion
  cannot break their recorded SHA-256 relationships.

All 126 initially modified/untracked files were backed up and preserved exactly.
Every modified or untracked live source file is accounted for in the source
manifest. The live `k3_slc.txt` debug log is excluded from source control.

## Validation and device state

Focused tests run on the device interpreter with staged release sources,
an isolated schema copy, and separate Params namespaces. See the individual
logs and [test-groups.json](test-groups.json) for exact tests. Runtime source
hashes are also recorded in [test-manifest.json](test-manifest.json).

| Suite | Passed | Log |
| --- | ---: | --- |
| Controllers and actuator feedback | 337 | [controls.log](controls.log) |
| Model runtime, spatial action and custom candidates | 75 | [models.log](models.log) |
| NNFF model library | 156 | [nnff.log](nnff.log) |
| Model selection UI | 18 | [ui.log](ui.log) |
| Monitoring | 23 | [monitoring.log](monitoring.log) |
| Inherited geometry, jerk filter and steering settings | 188 | [inherited.log](inherited.log) |
| Action dispatch, provenance and curvature selection | 61 | [action.log](action.log) |
| **Total** | **858** | |

Test fixture updates supply
the restored runtime's metadata and dynamic policy input list, and explicitly
select CPU for CPU-reference tests. Production controller/runtime payloads were
not changed to accommodate tests.

The test harness is preserved as [run_tests.py](run_tests.py) and
[stage_tests.py](stage_tests.py). It uses the device's installed Python/runtime
dependencies and the existing isolated test dependency directory. From a local
checkout, `python artifacts/release_consolidation_20261018/stage_tests.py` stages
sources to `/data/release-consolidation-tests-20261018`; run each named group with
`/usr/local/venv/bin/python3 /data/release-consolidation-tests-20261018/run_tests.py GROUP`
over SSH. This uses the documented device address in the staging script.

The final precommit observation found `DisableUpdates=1`, `UpdaterState=idle`,
`UpdaterFetchAvailable=0`, `UpdateAvailable=0`, and no updater process. The device
was offroad and its checkout remained at `e415672` with the already installed
controller repairs. Consolidating and pushing release does not activate the
restored picker/runtime on the device; a separate deployment is required.
