Torque-controller feedback units and low-speed behavior

The standard controller's PID operates in lateral-acceleration units. NNFF's
PID operates in normalized torque units. Switching between them must select
both the correct gains and the correct output limits, and discard the previous
integral. This also applies to missing-model and inference-error fallback.

NNFF uses P=0.8 and I=0.15. Its existing curvature augmentation supplies its
low-speed compensation. Reusing the standard controller's speed-dependent
gain on top of that augmentation amplified model and jerk errors twice. On
the recorded Kona drive, the old NNFF path repeatedly requested opposite
full-torque commands at low speed.

The standard acceleration-space controller holds P=5.5 below 7.5 m/s. This
uses its existing 7.5 m/s breakpoint and preserves the original gain curve
above that speed. The previous curve rose to P=250 below 1 m/s. Recorded
low-speed errors therefore generated disproportionate corrections even with
NNFF disabled. The optional Hyundai reduced-feedback factor still applies
to the standard gains and friction compensation.

No new actuator output clamp is used to hide the feedback problem. Requested
feedforward and the existing maximum torque and CAN rate limits are retained.
The PID is reset on disengagement, along with NNFF history, so a subsequent
engagement cannot reuse its integral or a stale angle-derivative sample.

Classic Hyundai's `SAS_Speed` is unsigned. For that platform, NNFF derives a
signed rate from successive raw steering angles and the controller timestep,
with a 50 ms first-order filter to smooth quantized and repeated CAN samples.
The first sample after reset seeds the derivative with zero. Learned angle
offset is excluded from the derivative. Other brands and Hyundai CAN-FD retain
their existing signed-rate inputs. The estimate is used only by the active
NNFF calculation.

NNFF's optional friction override uses torque-space friction units, and its
acceleration-error weighting returns to the normal value after a zero-jerk
cycle. These corrections also prevent state-dependent feedback scaling.

Validation includes 43 new regression cases for low-speed gains, gain/limit
transitions, fallback, mirrored steering movement, timestep variants,
quantization, reset, and friction units. The affected controller, model,
buffer, Hyundai-feedback, and Turn Guards suites passed 293 tests using the
C3X Python runtime and installed model files.

Recorded-input replay compares the old and fixed implementations using the
actual vehicle state, model predictions, calibration, and actuator-limiting
flags. The old implementation must first reproduce the logged controller
outputs. This checks the changed commands against real inputs; recorded
vehicle motion remains fixed, so it does not establish closed-loop handling
or road stability of the new controller.

Across five recorded Kona routes, replay reproduced the original controller
with a worst normalized-torque error of 0.000589. In active samples without
detected driver override, NNFF saturation below 2 m/s fell from 96.58% to zero;
between 2 and 5 m/s it fell from 65.75% to zero. Standard-controller saturation
between 2 and 5 m/s fell from 18.79% to 0.95%. Full torque remained available
on large-error samples. Standard proportional and feedforward terms above
7.5 m/s were identical; integral state can differ after the changed low-speed
anti-windup history and disengagement reset.
