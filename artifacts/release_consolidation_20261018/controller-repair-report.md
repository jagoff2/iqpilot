> Archived repair evidence. Relative links below refer to the original controller-repair-20261017 archive.

# Controller repair, 2026-10-17

The repair addresses four demonstrated controller mechanisms in the latest
route, including the native-action path. It does not establish that the car
will remain inside its lane: the recorded model path was already biased toward
the inside of both bends, and the available calibration estimates are not qualified.

Deployment status: **installed and independently verified on the c3x while
offroad**, on October 17, 2026 local time. All five runtime hashes match the
tested payload. The restarted service is healthy, steering/model settings and
Git HEAD are unchanged, and existing untracked paths are preserved. The backup
is `/data/controller-repair-backup-20261018T004855Z-180252`.
This qualification covers the diagnosed software fixes, not a physical trajectory.

## Implemented changes

1. **Compare each actuator output with its own input.** Card now publishes the
   actual input, originating command timestamp, actuation timestamp and lateral
   active state alongside the corresponding actuator output. Controlsd retains
   its existing torque/angle thresholds and next-cycle integration timing, but
   no longer compares an older output with the newest request. Missing, invalid,
   inactive or stale provenance freezes integration. Freshness uses the existing
   60 ms command watchdog. Replay uses recorded event time rather than host time.
2. **Keep steering feedback out of requested jerk.** An independent history
   follows the unmodified requested acceleration through the existing optional
   slew limiter. Turn-guard corrections no longer enter its derivative. New
   engagement seeds constant history; disengagement clears history and filter.
3. **Bound friction preview continuously.** With tracking error `e`, measured
   acceleration `m` and clean requested jerk `j`, the friction input is
   `clip(e + 0.3*j, min(e,-m,0), max(e,-m,0))`. This is equivalent to keeping the
   preview target inside the interval containing measured acceleration, the
   delayed target and straight. Friction cannot deepen measured turning when
   tracking error requests outward correction. Useful anticipation toward
   straight remains. There is no sign-switch discontinuity at zero error or
   zero measured acceleration.
4. **Allow integral recovery through a real steering limit.** For a valid paired
   torque limit, the standard controller evaluates this tick's current and
   proposed integral command using its actual torque conversion. Integration is
   allowed only when both commands remain on the observed limit's side and the
   proposed command moves closer to the applied output. Unknown feedback,
   reversed direction, driver intervention and low speed remain frozen. NNFF
   retains its existing integration behavior. PID state is advanced only once.

No feedback gain, friction coefficient, torque factor, roll calibration, delay,
driver-override preference, NNFF selection or model selection was retuned. The
existing 0.3 s preview coefficient remains. The fix is in the shared controller,
so it applies when the selected producer is nativeAction.

The earlier native-action spatial-conversion candidate was removed from the
working tree because its replay increased inward demand. It was never installed.
Its source and evidence remain in [the earlier archive](../native-action-spatial-20261017/).

## Incident replay

Route: `000001e3--bfaaadcd42`. Times below start at the first road video frame.
Vehicle motion, model requests, driver input and calibration are recorded;
candidate PID, CarInterface and limiter feedback evolve independently.
The replay bundle's `installed` series denotes the pre-repair runtime; its
`candidate` series is the repair now installed on the device.

| Window | Original mean inward requested torque | Repair | Change |
| --- | ---: | ---: | ---: |
| Right entry, 393–397 s | 0.18140 | 0.16176 | -10.8% |
| Right apex, 397–399 s | 0.34133 | 0.36399 | +6.6% |
| Right exit, 399–401 s | 0.10275 | 0.12607 | +22.7% |
| Left entry, 407–410.5 s | 0.45240 | 0.38013 | -16.0% |

The right-exit increase is almost entirely changed integral history: normalized
torque changes by about 0.02354 inward from I and 0.00022 outward from feedforward.
The useful unwind friction remains essentially unchanged. This is a real
behavioral tradeoff, not evidence that the vehicle's exit trajectory improves.

Friction opposing outward tracking error falls from 220 to zero samples in the
right-entry window and 217 to zero in the left-entry window. Frozen integration
ticks fall from 320 to 34 and 310 to 67 respectively. Mean leftward request on
the two straight windows falls from 0.01994 to 0.01412 and from 0.02750 to 0.00696.
Driver-intervention saturation remains substantial; it is not resolved by this
controller repair.

Across the incident route, inward friction against outward error falls from
4,039 samples to zero, but total saturated request samples rise from 1,041 to
1,108. Four additional inward-saturated samples without driver intervention
appear around 172 s. A separate trace attributes their larger integral to
restored integration of earlier recorded tracking error. That earlier error
is corroborated by calibrated yaw, rather than a gross VM measurement bias.
Fixed-motion replay cannot let extra candidate actuation reduce that recorded
error. This is a retained command tradeoff and a limit of the replay, not
evidence establishing either an additional software defect or improved physical
behavior. No speculative integral leak, gain reduction or roll change was added.

The original replay reproduces all 41,011 initialized steering CAN frames
exactly. The candidate's 41,011 commands all pass through the software safety
replay with no gaps or added rejections. The same five route-end main-off
rejections remain. All 41,115 matched original Panda echo decisions agree.
These checks establish command behavior and software acceptance on recorded
motion. They do not simulate the repaired vehicle trajectory.

An error-only friction candidate was rejected: it increased right-exit inward
request by about 61%. A switched sign gate was rejected because it created a
finite friction step at zero tracking error. Both investigations are preserved.

## Validation and remaining limits

- 313 regression tests passed on an isolated runtime on the actual c3x,
  including actual card publication, controlsd consumption, schema compatibility,
  replay clock handling, standard/NNFF units, turn guards, action selection,
  friction direction, continuity, request-history provenance and reengagement.
- Independent numerical review checked 200,000 projection cases for symmetry,
  equivalence and outward-error sign behavior.
- An additional 100,000 directional anti-windup cases and explicit boundary,
  reversal, nonfinite and flat-callback cases passed independent review.
- All three routes completed: 220,164 controller ticks, 92,739 active ticks and
  223,232 generated candidate steering CAN commands. Every candidate frame was
  consumed by continuous software-safety replay, with zero trace gaps and zero
  added rejections. The same ten route-end main-off rejections remain.
- No nonfinite or out-of-bounds normalized outputs; CAN stays within +/-400,
  with consecutive request-on changes no greater than eight CAN units.
- Saturated request samples across the three routes change from 2,595 to 2,578:
  157 newly saturated and 174 removed. Six newly saturated samples have inward
  output, outward error and no driver intervention. Their integral-history
  attribution and fixed-motion limitations are preserved in the detailed review.
- Inward friction against outward tracking error changes from 14,678 samples
  to zero. This establishes the repaired friction direction on these logs.
- Baseline CAN reproduces exactly on E3 and D6. DA retains 219 asynchronous
  timing residuals among 54,481 generated frames and one hardware/software
  shutdown-edge safety-decision discrepancy. These are disclosed replay limits.
- Thirty installer/readiness tests passed both locally and on the c3x, including
  real POSIX ownership, fsync and atomic replacement on temporary device roots,
  delayed startup, invalid/stopped process states, offroad enforcement and error
  capture. Production service operations were mocked in those tests.
- Actual installation then passed fresh production imports, loaded-schema and
  feedback wire-roundtrip validation, exact payload hashes, confirmed offroad
  flags, unchanged raw steering/model preferences and fresh manager health.
  A separate post-install process repeated these checks and verified the exact
  four tracked changes plus one new helper, unchanged HEAD and preserved
  preexisting untracked paths. Driving processes are normally stopped offroad;
  these checks do not substitute for responsive-vehicle validation.

The measured model-path bias is outside this controller repair. So are the
unqualified torque/delay calibrations and the modest VM-versus-yaw measurement
discrepancy. Replacing those values with rejected fits or guessed gains would
not constitute a diagnosed fix. NNFF training remains a separate experimental
task; no training or model activation occurred here.

## Deployment and recovery

The installed archive SHA-256 is
`27a16f7ec670102caadd6085cde30877236420d602e1df8988f5d2db411cf121`.
Two earlier attempts restored the original controller files after readiness
checks failed. The second attempt exposed a validator error: manager startup
temporarily clears its offroad flag, which the original validator treated as
an onroad transition. The corrected validator waits through unknown startup
state, aborts on an explicit onroad flag, and requires confirmed offroad flags
plus a fresh healthy managerState. The file-replacement guard remains strict.
Both recoveries were verified before retrying, and their archives and evidence
are retained. The final attempt completed successfully; controller payload
bytes did not change during the installer repairs.

See [deployment result and exact rollback command](deployment/DEPLOYMENT-RESULT.md).

## Evidence

- [Device regression results](test-directional-results.txt)
- [Mathematical and failure-mode review](review/PREVIEW-PROJECTION.md)
- [Directional anti-windup review](review/DIRECTIONAL-ANTIWINDUP.md)
- [Remaining integral-bias investigation](review/INTEGRAL-BIAS-FOLLOWUP.md)
- [Complete three-route replay report](replay/directional-feedback/REPORT.md)
- [Stateful replay summary](replay/directional-feedback/comparison-summary.json)
- [Saturation and command outliers](replay/directional-feedback/outlier-summary.json)
- [Software safety results](replay/directional-feedback/continuous-software-safety-all.json)
- [Complete source and regression-test patch](controller-repair.patch)
- [Deployment and rollback verification](deployment/VERIFICATION.md)
- [Successful installation receipt](deployment/installed.json)
- [Independent post-install verification](deployment/post-install-verification.json)
- [Prior diagnosis and calibration evidence](../preincident-controls-20261017/REPORT.md)

## Expanded other-route testing

The subsequent read-only expansion inventories 22 complete recordings and 226
source-verified segments, covering about 3.6 recorded hours. Twenty-one recordings
have steering CAN and were qualified, including 20 with active steering. DC has
neither active steering nor steering CAN and is excluded; D0 provides inactive
startup coverage only. The complete eight-segment E3 replay supersedes the
earlier seven-segment result. D6 and DA are reused once from the prior frozen
qualification, with the same repaired controller bytes.

The combined controller coverage is 1,253,672 updates, including all 811,753
recorded active updates. Independent state checks found no violations, and all
generated steering traces completed software-safety replay with zero added
rejections. Saturation changes remain mixed and the recorded-motion comparison
does not establish a responsive vehicle trajectory. No production source,
model, gain, preference or service was changed during this expansion.

See the [combined route results and limitations](expanded-review/REPORT.md),
[independent controller-state audit](expanded-review/invariant-audit.json), and
[detailed expanded replay report](replay/expanded-routes/REPORT.md).
