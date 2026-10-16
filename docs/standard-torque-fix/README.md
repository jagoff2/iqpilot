# Standard torque turn-cutting and unwind candidate

This candidate replaces the plan-derived current curvature for standard torque
control and prevents delayed feedback from asking for a deeper turn after the
current target has unwound. Native action/curvature heads retain their precedence.

The original conversion combined time-sampled heading, a separate yaw-rate
estimate, and vehicle speed. The replacement derives heading and its derivative
from one distance-based reference using the model's changing forward velocity.
This addresses the target-conversion inconsistency before feedback runs.

`spatial_heading.py` integrates model body-forward velocity over the model time
grid, builds a monotone cubic heading interpolant on its valid forward prefix,
and evaluates `2 * (heading(q) - heading(0)) / q - heading_derivative(0)`, with
`q = inference-time vehicle speed * producer preview seconds`. Unsupported
references produce an invalid action, not extrapolation. Existing low-speed
hold and smoothing remain in the producer; a subsequent valid action recovers.
Controlsd resets lateral control on a nonfinite selected action.

In standard torque control, the existing delayed/lead feedback setpoint is
projected between zero and the current limited target acceleration. Entry delay
is retained while stale deeper-turn and opposite-sign feedback requests are
removed. This change does not retune gains, integrator rules, or actuator limits.
For example, with a delayed target of 2.0 m/s², current target of 0.8 m/s², and
measured acceleration of 1.2 m/s², the old P error was +0.8 m/s² into the turn.
The projected target produces an error of -0.4 m/s², requesting unwind.

The qualified deployment uses NNFF off, Mac BIG off, eGPU off, native CD210
Centerline s71, turn guards on, reduced Hyundai feedback off, acceleration slew
off, and model smoothing off. The signed Mac worker has its own bundled producer
code and is outside this public producer patch. No onroad controller switching
was added. NNFF future-reference work is not included in this candidate.

Validation on 2026-10-15:

- 259 focused tests passed on the device. The entire selector test file then
  passed with 56 tests after replacing the obsolete standard-output fixture
  scope with analytical unwind coverage; its golden fixture was unchanged.
- Five recorded routes ran continuously from route start, covering 574,370
  control ticks and 235,179 active ticks. Candidate PID, limits, actuator output,
  and CarInterface history were independent of recorded controller state.
- No invalid active reference, nonfinite/out-of-range command, or NNFF activation
  occurred. Inward P feedback was zero across 61,776 samples already turning
  harder than the candidate current target.
- The final Float32 action-message replay of the intersection and strong-curve
  routes generated 182,221 steering frames, all byte-identical to the earlier
  candidate output and independently checked against matching-source software
  safety. There were no missing commands or added rejections. Five recorded
  main-off rejections remained; one baseline shutdown timing edge differed
  between software ordering and the hardware echo.
- Representative decoder timing on the c3x was 0.895 ms median and 1.313 ms p99
  over 1,000 calls.

Saturation was reduced in the investigated events, but whole-route saturation
rose on two comparison drives, including stronger outward corrections. Some
requested torque steps increased; existing CAN rate limits still applied.
Recorded producer speed must partly be reconstructed from the old Float32
action, so logs cannot establish exact inference-time speed in every packet.
These checks establish recorded-input command behavior and software acceptance;
they do not establish closed-loop physical vehicle response.

Focused regression coverage is in:

- `iqpilot/selfdrive/controls/lib/tests/test_spatial_heading.py`
- `iqpilot/selfdrive/controls/tests/test_standard_unwind.py`
- `iqpilot/selfdrive/controls/tests/test_invalid_lateral_action.py`
- `iqpilot/selfdrive/controls/tests/test_lateral_action_selection.py`
- `iqpilot/selfdrive/iqmodeld/tests/test_standard_spatial_action.py`

The candidate was installed on the c3x at 2026-10-15 22:29:37 Eastern and the
device was rebooted. Postboot verification confirmed all seven deployed source
hashes, disabled NNFF/Mac BIG/eGPU settings, the selected native s71 model, public
module import paths, and no unexpectedly stopped manager processes. The device
was offroad during verification; no physical steering test was performed.

The release includes the previously deployed NNFF wiring, turn-guard, Hyundai
CAN/safety, and low-speed-feedback prerequisite commits. This candidate does not
reflash Panda or qualify the unresolved NNFF spatial future-reference changes.
