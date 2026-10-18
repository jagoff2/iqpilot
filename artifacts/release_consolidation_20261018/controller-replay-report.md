> Archived repair evidence. Relative links below refer to the original controller-repair-20261017 archive.

# Continuous controller replay evidence

Frozen controller SHA256 `7969128a8f82a8710db0c75b25ff870de49a31e812ed9765f4552f41e89983ac`; frozen feedback helper `c055fe831b61304cae3294204507217e711d694c8f2b4eba404adcba9b8be355`.

This executes the saved controller source without injected controller edits. A return-local profiler records the internal terms. Recorded vehicle motion, driver inputs, calibration, and clipped desiredCurvature remain fixed. E3 keeps its recorded native-action targets; no producer recalculation or rejected spatial conversion is applied. These results quantify commands on recorded motion and do not predict a repaired physical trajectory.

| Route | Controller ticks | Active ticks | Recorded baseline CAN exact/generated |
|---|---:|---:|---:|
| 000001d6--1afc400095 | 127,592 | 13,749 | 127,740/127,740 |
| 000001da--8c768c5c86 | 51,294 | 47,426 | 54,262/54,481 |
| 000001e3--bfaaadcd42 | 41,278 | 31,564 | 41,011/41,011 |

The baseline controller uses the recorded deployed source and recorded limiter flags. The installed comparator and candidate use their own continuously generated actuator feedback, start with zero integral, and retain controller and CarInterface state through every segment. Historical comparisons force standard control and guards ON in both comparator and candidate while retaining their recorded target stream. Historical DA actual-baseline NNFF reconstruction and asynchronous command consumption have small residuals; E3 and D6 generated baseline CAN is exact. The first 104 CAN frames on each route precede replay parser discovery and retain actual commands in software-safety replay.

Candidate feedback pairs each generated result with the request that produced it and preserves this pair through the next recorded carOutput publication. The flag and optional directional detail are consumed on the following control tick, matching controlsd. The unlogged pre-apply timestamp is represented by recorded sendcan emission time; recorded controlsState time represents feedback evaluation time. Raw message validity is preserved, but unlogged live SubMaster socket-health checks cannot be reproduced. Origin-age ranges are retained in comparison-summary.json.

## Incident phases

E3 times are seconds from the first road-video frame. Requested torque is positive left and negative right. Values below are phase means.

| Phase | Installed request | Candidate request | Installed frozen ticks | Candidate frozen ticks |
|---|---:|---:|---:|---:|
| straight_before_right (378..382 s) | +0.019936 | +0.014124 | 183 | 7 |
| right_entry (393..397 s) | -0.181399 | -0.161759 | 320 | 34 |
| right_apex (397..399 s) | -0.341335 | -0.363989 | 170 | 51 |
| right_exit (399..401 s) | -0.102749 | -0.126067 | 157 | 38 |
| straight_between (402..405 s) | +0.027504 | +0.006956 | 216 | 53 |
| left_entry (407..410.5 s) | +0.452400 | +0.380133 | 310 | 67 |
| driver_correction (410.5..412 s) | +0.970408 | +0.961578 | 128 | 121 |

Phase decomposition is saved in phase-decomposition.json. The right-exit command difference is predominantly the changed integral history; outward friction during exit is retained. Driver intervention still reaches saturation. The integral-history changes and extra inward requests elsewhere are retained in outlier-summary.json rather than classified as a trajectory improvement.

## Whole-route behavior

| Route | Saturated ticks, installed/candidate | New/removed saturated ticks | Friction inward against outward error, installed/candidate | Newly saturated with inward output, outward error, no driver | Max active request step, installed/candidate |
|---|---:|---:|---:|---:|---:|
| 000001d6--1afc400095 | 102/46 | 0/56 | 1878/0 | 0 | 0.263000/0.252743 |
| 000001da--8c768c5c86 | 1452/1424 | 39/67 | 8761/0 | 2 | 0.235313/0.200453 |
| 000001e3--bfaaadcd42 | 1041/1108 | 118/51 | 4039/0 | 4 | 0.340682/0.236492 |

The candidate changes torque while replayed vehicle tracking errors remain fixed. Its integral therefore continues accumulating recorded errors that a responding physical vehicle would change. Added integral and saturation are observable command differences, not a simulated departure trajectory. They also cannot be treated as proof that lane keeping is fixed.

The category summary also includes stopped and low-speed active samples, straights, left/right requested turns, target sign-crossing neighborhoods, and driver-pressed samples. Command means are not lane-position predictions. No controller output is nonfinite or outside normalized ±1. Generated CAN stays within ±400; consecutive request-on commands change by at most 8 CAN units. Larger reset/disengagement steps are retained in the CAN summary.

## Software safety

| Route | Candidate traces consumed | Candidate-only rejections | Hardware echo decisions matched/observed | Candidate rejected commands |
|---|---:|---:|---:|---:|
| 000001d6--1afc400095 | 127,740 | 0 | 127,841/127,841 | 2 |
| 000001da--8c768c5c86 | 54,481 | 0 | 54,123/54,124 | 3 |
| 000001e3--bfaaadcd42 | 41,011 | 0 | 41,115/41,115 | 5 |

The safety replay uses the preserved matching firmware library and continuous recorded RX/heartbeat streams with independent candidate TX history. Every generated candidate frame is consumed at its exact recorded send timestamp; there are no trace gaps. Route-end main-off rejections remain. DA retains the known hardware/software shutdown-edge disagreement at +532.464265682 seconds. Passing these command gates does not establish lane keeping or on-road safety.

## Provenance and artifacts

- comparison-summary.json: complete phase/category statistics and source hashes.
- outlier-summary.json: new saturation, largest command steps, and sustained extra-inward intervals.
- phase-decomposition.json: integral/feedforward changes for the incident approach.
- incident-joint-repair.png: visual comparison of both bends.
- qualification/*/segment-*.json.zst: full time series, controller locals, command traces and baseline diagnostics.
- dependency-snapshot.json and dependency-snapshot/: the Python sources used by the replay.
- continuous-software-safety-all.json: complete safety results and exact trace-file hashes.
- manifest.json: hashes of the finished bundle files.

The installed/historical source copies in this bundle use CRLF line endings. Their decoded source text and LF-normalized bytes exactly match the earlier frozen baseline files; only newline bytes differ. The actual CRLF hashes are recorded in every replay artifact. See baseline-newline-provenance.json for raw sizes, counts and hashes.
