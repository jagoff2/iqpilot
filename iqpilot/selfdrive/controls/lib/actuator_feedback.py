"""Associate steering feedback with the request actually consumed by card."""
import math
from dataclasses import dataclass, field


# Match card's existing 60 ms carControl watchdog. This is a freshness bound,
# not an estimate of steering delay or of the number of messages in flight.
CAR_CONTROL_TIMEOUT_NS = 60_000_000
STEER_TORQUE_SATURATION_THRESHOLD = 1e-2


@dataclass(frozen=True)
class AppliedActuators:
  actuators_input: dict = field(default_factory=dict)
  car_control_mono_time: int = 0
  actuation_mono_time: int = 0
  lat_active: bool = False

  @classmethod
  def capture(cls, command, car_control_mono_time: int, actuation_mono_time: int):
    # Snapshot before CI.apply: later commands or a mutable builder must not
    # change the input paired with the resulting output.
    return cls(command.actuators.to_dict(), car_control_mono_time, actuation_mono_time, bool(command.latActive))

  def write_to(self, output) -> None:
    output.actuatorsInput = self.actuators_input
    output.carControlMonoTime = self.car_control_mono_time
    output.actuationMonoTime = self.actuation_mono_time
    output.latActive = self.lat_active


@dataclass(frozen=True)
class SteeringActuatorFeedback:
  limited: bool
  # CarControl torque convention: requested minus applied. None means there is
  # no known torque clamp direction, including angle-control feedback.
  torque_difference: float | None = None
  applied_torque: float | None = None


def steering_feedback_from_output(output, *, feedback_valid: bool, output_mono_time: int, now_mono_time: int,
                                 angle_control: bool, angle_threshold: float) -> SteeringActuatorFeedback:
  """Unknown feedback freezes integration; valid pairs retain existing limits.

  Older writers leave the appended metadata at zero. Repeated requests are
  supported because each output carries its actual input, without a guessed
  message offset or a lookup against the newest command.
  """
  request_time, apply_time = output.carControlMonoTime, output.actuationMonoTime
  if not feedback_valid or not output.latActive or not 0 < request_time <= apply_time <= output_mono_time <= now_mono_time:
    return SteeringActuatorFeedback(True)
  if now_mono_time - request_time > CAR_CONTROL_TIMEOUT_NS or now_mono_time - apply_time > CAR_CONTROL_TIMEOUT_NS:
    return SteeringActuatorFeedback(True)

  if angle_control:
    requested, applied = output.actuatorsInput.steeringAngleDeg, output.actuatorsOutput.steeringAngleDeg
    threshold = angle_threshold
  else:
    requested, applied = output.actuatorsInput.torque, output.actuatorsOutput.torque
    threshold = STEER_TORQUE_SATURATION_THRESHOLD
  if not (math.isfinite(requested) and math.isfinite(applied)):
    return SteeringActuatorFeedback(True)
  difference = requested - applied
  return SteeringActuatorFeedback(abs(difference) > threshold, None if angle_control else difference,
                                  None if angle_control else applied)


def steering_limited_from_feedback(output, *, feedback_valid: bool, output_mono_time: int, now_mono_time: int,
                                   angle_control: bool, angle_threshold: float) -> bool:
  return steering_feedback_from_output(output, feedback_valid=feedback_valid, output_mono_time=output_mono_time,
                                       now_mono_time=now_mono_time, angle_control=angle_control,
                                       angle_threshold=angle_threshold).limited
