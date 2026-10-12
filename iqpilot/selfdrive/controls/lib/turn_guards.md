# Experimental turn guards

Open **Settings > Steering > Turn Guards (Experimental)** on C3/C3X. The comma 4 Steering panel has the same setting. It defaults off, can be changed only offroad, and is captured when the steering controller initializes for the next drive. The setting supports the generic torque controller with NNFF on or off. Other steering controllers show it unavailable.

This ports the backup's cut/apex correction gains with the direction and timing errors corrected. It is an A/B experiment for excessive turning relative to the requested curvature. It does not detect lane boundaries or guarantee that the planned path stays inside the lane.

## Corrected rules

The cut and apex gains remain 0.60 and 0.40. Their individual acceleration caps remain `0.50 * (abs(request) + 0.5)` and `0.40 * (abs(request) + 0.5)`.

Both corrections require the current request, the delay-aligned reference, and the measured acceleration to have the same nonzero direction. Excess turning must exceed both the current request magnitude and the delayed reference magnitude. The total reduction is bounded by the current request magnitude, so shaping cannot amplify it or change its sign and tends continuously to zero near a straight-ahead request.

An independent copy of the optional lateral acceleration slew limiter tracks the unguarded request. Its history supplies the reference at IQ.Pilot's selected lateral delay. This prevents normal slew-limited turn exit from being classified as overtracking. Corrected requests never enter that reference history. The guard waits for sufficient history and clears readiness when steering becomes inactive.

The corrected request passes through the existing output slew limiter and request buffer. Standard PID and NNFF both receive the resulting acceleration and curvature. The off path retains the existing controller math.

## A/B state and persistence

The `steering_turn_guards` startup log event records `enabled` and `vehicle`. Use it to identify which setting a drive used. Changing the saved setting does not change a running controller.

The prebuilt native Params registry does not support dynamic keys and deletes unregistered files. The shared settings helper therefore stores this Boolean outside the native key directory, at `/data/params/iqpilot_preferences/d/IQSteeringTurnGuards` for the normal device prefix. It survives manager and drive transitions; Reset All Settings clears it. Missing, unreadable, or malformed values default off. Writes are atomic.

## Validation

Focused tests cover directional cases, output bounds, zero continuity, timing and slew behavior, NNFF and standard control, persistence, real UI callbacks, offroad gating, and failed-save rollback:

```sh
python -m pytest -q \
  iqpilot/common/tests/test_turn_guard_settings.py \
  iqpilot/selfdrive/controls/tests/test_turn_guards.py \
  iqpilot/selfdrive/controls/tests/test_nnff_controls.py \
  iqpilot/selfdrive/ui/tests/test_turn_guard_settings.py
```

Controller software tests do not establish better real-world tracking. The optional setting is intended to make that comparison explicit; no driving-performance improvement is claimed.
