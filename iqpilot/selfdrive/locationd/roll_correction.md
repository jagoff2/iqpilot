# Vehicle Roll Correction

The c3x Vehicle settings page has an offroad-only **Vehicle Roll Correction**
toggle. It defaults to off and takes effect on the next drive. Runtime support
is currently restricted to `HYUNDAI_KONA_EV`, whose recorded CAN signals were
checked. Turning it off restores the previous roll input and torque-cache mode.

With the option enabled, `estimatord` supplies the same corrected roll to the
vehicle-parameter and torque estimators. It transforms device attitude into the
calibrated vehicle frame, then subtracts a slowly learned effective offset.
This leaves the localization output and lateral-controller PID reset behavior
unchanged.

The reference uses bus-0 YRS11 lateral acceleration and yaw rate, YRS12 health,
and ESP12 agreement. The gravity residual is `lateral_acceleration - speed *
yaw_rate`, with a stationary learned yaw zero. Messages must be fresh, have
valid status and advancing counters, and agree across the two message families.
YRS checksums are not validated. This reference retains chassis accelerometer
bias; it is not a measurement of absolute road level or isolated device IMU bias.

Offset acquisition requires stationary yaw-zero observations, ten seconds of
qualified offset samples and a median absolute deviation at most 0.2 degrees.
Updates require stable motion and valid pose/calibration inputs. The learned
offset is bounded to five degrees, uses a 30-second update time constant, and
changes by at most 0.01 degrees per second while moving or laterally active.
Learning holds during launch, turns, acceleration and invalid inputs. Abrupt
application and calibration-transform changes require a confirmed inactive stop.
Persistent disagreement or a substantial extrinsics change triggers reacquisition.
The estimator cannot reliably distinguish every remount from sensor-bias changes.

State and the toggle persist outside the native Params registry, at
`/data/params/iqpilot_preferences/d/VehicleRollCorrectionState` and
`VehicleRollCorrection` on the device. Prefixes used by tests are isolated.
Saved state is checked for version, fingerprint, finite values and bounds.
Enabled mode starts torque learning afresh each drive rather than restoring
points from another gravity frame. Subsequent reference changes discard sample
history while preserving published torque filters to prevent output steps.

The initial deployment uses a device-specific seed of **2.5717551236 degrees**
and yaw zero **0.1082447585 degrees/second**, derived from this device's recorded
data. These are not hard-coded defaults for other installations. The saved
extrinsics are taken from the device at installation.

Validation includes 58 native device tests covering CAN rejection, persistence,
acquisition, smoothness, both roll consumers, torque cache isolation and settings
behavior. Four minutes of recorded data (4,800 pose samples) replayed through
the production correction and both legacy/corrected vehicle estimators. At the
original launch, recorded roll was 3.3934 degrees, legacy replay produced 3.3927
degrees, and corrected replay produced 0.6374 degrees. The corrected input was
0.8707 degrees, with the adaptive offset at 2.5372 degrees after preceding
observations. The applied offset was constant through the launch interval.
The combined `estimatord.main` loop also completed the same replay with isolated
message transport: 4,800 valid vehicle-parameter messages and 960 messages each
for lateral delay and torque parameters, with 0.6376 degrees roll at launch.
These are estimator replay results; the vehicle response has not been road-tested.

To revert behavior, switch the toggle off while parked and start a new drive.
The saved offset remains available for a later re-enable.
