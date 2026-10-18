from collections import defaultdict
from types import SimpleNamespace

import pytest

from iqpilot.cereal import car, custom, log
from iqpilot.common.iq_perf import PerfTraceRing
from iqpilot.selfdrive.car import card
from iqpilot.selfdrive.controls.lib.actuator_feedback import AppliedActuators, steering_limited_from_feedback
from iqpilot.selfdrive.controls.tests.lateral_control_scenarios import load_controls_module


class Messages(dict):
  def __init__(self):
    super().__init__()
    self.frame = 1
    self.recv_frame = defaultdict(lambda: 1)
    self.logMonoTime = defaultdict(int)
    self.valid = defaultdict(lambda: True)
    self.alive = defaultdict(lambda: True)

  def all_checks(self, services):
    return all(self.valid[s] and self.alive[s] for s in services)

  def all_alive(self, services):
    return all(self.alive[s] for s in services)


class Publisher:
  def __init__(self):
    self.sent = defaultdict(list)

  def send(self, service, message):
    # Preserve the real serialized Event, as the consumer sees it.
    self.sent[service].append(message if isinstance(message, bytes) else message.to_bytes())

  def latest(self, service):
    with log.Event.from_bytes(self.sent[service][-1]) as message:
      return message.as_builder()


class Interface:
  def __init__(self):
    self.calls = []
    self.applied_torque = None

  def apply(self, command, cc_iq, now_ns, model=None):
    self.calls.append((command.to_dict(), now_ns))
    output = command.actuators.as_builder()
    if self.applied_torque is not None:
      output.torque = self.applied_torque
    return output, []


@pytest.fixture
def card_endpoint(monkeypatch):
  now = [1_000_000_000]
  monkeypatch.setattr(card.time, "monotonic_ns", lambda: now[0])
  monkeypatch.setattr(card.time, "monotonic", lambda: now[0] / 1e9)
  monkeypatch.setattr(card, "REPLAY", False)
  endpoint = card.Car.__new__(card.Car)
  endpoint.sm = Messages()
  endpoint.sm.valid["modelV2"] = False
  endpoint.sm.logMonoTime["carControl"] = now[0]
  endpoint.pm = Publisher()
  endpoint.CI = Interface()
  endpoint.initialized_prev = True
  endpoint._needs_iq_lead_data = False
  endpoint.carcontrol_stale_frames = 6
  endpoint._last_sendcan_mono_ns = None
  endpoint._perf_ring = PerfTraceRing()
  endpoint._perf = SimpleNamespace(emit=lambda *args, **kwargs: None)
  endpoint.CC_prev = car.CarControl.new_message()
  endpoint.last_actuators_output = car.CarControl.Actuators.new_message()
  endpoint.last_actuators_feedback = AppliedActuators()
  endpoint.can_rcv_cum_timeout_counter = 0
  endpoint._lag_initialized = False
  return endpoint, now


def publish_output(endpoint):
  endpoint.state_publish(car.CarState.new_message(canValid=True), custom.IQCarState.new_message(), None)
  return endpoint.pm.latest("carOutput")


def apply_command(endpoint, command):
  endpoint.controls_update(car.CarState.new_message(canValid=True), command.as_reader(), custom.IQCarControl.new_message())


def test_card_publishes_the_previous_apply_pair_then_repeated_request(card_endpoint):
  endpoint, now = card_endpoint
  startup = publish_output(endpoint)
  assert startup.carOutput.carControlMonoTime == 0

  first = car.CarControl.new_message(latActive=True, actuators={"torque": 0.2})
  now[0] += 10_000_000
  apply_command(endpoint, first)
  first_apply_ns = now[0]
  first.actuators.torque = -0.5
  now[0] += 10_000_000
  output = publish_output(endpoint)
  assert output.carOutput.actuatorsInput.torque == pytest.approx(0.2)
  assert output.carOutput.actuatorsOutput.torque == pytest.approx(0.2)
  assert output.carOutput.carControlMonoTime == 1_000_000_000
  assert output.carOutput.actuationMonoTime == endpoint.CI.calls[0][1] == first_apply_ns
  assert first_apply_ns < output.logMonoTime

  # The next command has opposite torque, but cannot rewrite the published pair.
  second = car.CarControl.new_message(latActive=True, actuators={"torque": -0.4})
  endpoint.sm.logMonoTime["carControl"] = now[0]
  apply_command(endpoint, second)
  now[0] += 10_000_000
  second_output = publish_output(endpoint)
  assert second_output.carOutput.actuatorsInput.torque == pytest.approx(-0.4)
  assert output.carOutput.actuatorsInput.torque == pytest.approx(0.2)

  # card may apply one external request more than once; retain its origin while
  # tracking the new apply time and the independently limited output.
  endpoint.CI.applied_torque = -0.38
  apply_command(endpoint, second)
  now[0] += 10_000_000
  repeated = publish_output(endpoint)
  assert repeated.carOutput.carControlMonoTime == second_output.carOutput.carControlMonoTime
  assert repeated.carOutput.actuationMonoTime > second_output.carOutput.actuationMonoTime
  assert repeated.carOutput.actuatorsInput.torque == pytest.approx(-0.4)
  assert repeated.carOutput.actuatorsOutput.torque == pytest.approx(-0.38)


@pytest.mark.parametrize("fallback", [False, True])
def test_card_invalid_or_synthetic_input_has_no_external_origin(card_endpoint, fallback):
  endpoint, now = card_endpoint
  command = car.CarControl.new_message(enabled=True, latActive=True, actuators={"torque": 0.4})
  endpoint.sm.valid["carControl"] = False
  if fallback:
    endpoint.sm.alive["carControl"] = False
    endpoint.sm.frame = 7
  now[0] += 70_000_000
  apply_command(endpoint, command)
  now[0] += 10_000_000
  output = publish_output(endpoint)
  assert output.carOutput.carControlMonoTime == 0
  assert output.carOutput.actuationMonoTime == endpoint.CI.calls[0][1]
  if fallback:
    assert not output.carOutput.latActive
    assert output.carOutput.actuatorsInput.torque == 0.0
  assert steering_limited_from_feedback(output.carOutput, feedback_valid=output.valid,
                                       output_mono_time=output.logMonoTime, now_mono_time=now[0],
                                       angle_control=False, angle_threshold=2.5)


def controls_endpoint(module, pm, output, steer_type):
  endpoint = module.Controls.__new__(module.Controls)
  endpoint.CP = car.CarParams.new_message(steerControlType=steer_type)
  endpoint.CP.lateralTuning.init("torque")
  endpoint.CP_IQ = custom.IQCarParams.new_message()
  endpoint.sm = Messages()
  endpoint.sm.update(carState=car.CarState.new_message(canValid=True), carOutput=output.carOutput,
                     selfdriveState=log.SelfdriveState.new_message(), onroadEvents=[],
                     longitudinalPlan=log.LongitudinalPlan.new_message(),
                     driverMonitoringState=log.DriverMonitoringState.new_message(awarenessStatus=1.0))
  endpoint.sm.valid["driverAssistance"] = False
  endpoint.sm.logMonoTime["carOutput"] = output.logMonoTime
  endpoint.sm.valid["carOutput"] = output.valid
  endpoint.pm = pm
  endpoint.steer_limited_by_safety = False
  for flag in ("enable_curvature_controller", "force_rhd_for_bsm", "enable_long_comfort_mode", "enable_speed_limit_control",
               "enable_speed_limit_predicative", "enable_pred_react_to_speed_limits", "enable_pred_react_to_curves"):
    setattr(endpoint, flag, False)
  endpoint.curvature = endpoint.roll_compensation = endpoint.desired_curvature = 0.0
  endpoint.calibrated_pose = None
  endpoint.lateral_action_selection = log.ControlsState.LateralActionSelection.inactive
  endpoint.LoC = SimpleNamespace(long_control_state="off", pid=SimpleNamespace(p=0.0, i=0.0, f=0.0))
  endpoint.feedback_calls = []
  endpoint.LaC = SimpleNamespace(update_actuator_feedback=endpoint.feedback_calls.append)
  return endpoint


@pytest.mark.parametrize("steer_type", ["torque", "angle", "curvatureDEPRECATED"])
def test_controls_publish_uses_paired_input_and_preserves_next_tick_timing(card_endpoint, monkeypatch, steer_type):
  endpoint, now = card_endpoint
  command = car.CarControl.new_message(latActive=True, actuators={"torque": 0.2, "steeringAngleDeg": 5.0})
  now[0] += 10_000_000
  apply_command(endpoint, command)
  now[0] += 10_000_000
  output = publish_output(endpoint)
  module = load_controls_module(monkeypatch)
  monkeypatch.setattr(module, "REPLAY", False)
  controls = controls_endpoint(module, Publisher(), output, steer_type)
  lac_log = (log.ControlsState.LateralTorqueState.new_message() if steer_type == "torque"
             else log.ControlsState.LateralAngleState.new_message())

  # New request differs drastically from the old output, but that old request
  # was applied exactly. This must not fabricate a limiter event.
  newest = car.CarControl.new_message(latActive=True, actuators={"torque": -0.5, "steeringAngleDeg": -10.0})
  controls.publish(newest, lac_log)
  assert not controls.steer_limited_by_safety
  assert not controls.pm.latest("carControl").carControl.steerLimited
  assert not controls.feedback_calls[-1].limited
  assert controls.feedback_calls[-1].torque_difference == (0.0 if steer_type == "torque" else None)
  if steer_type == "torque":
    assert controls.feedback_calls[-1].applied_torque == pytest.approx(0.2)
  else:
    assert controls.feedback_calls[-1].applied_torque is None

  # Real limitation is still detected and reported on the following tick.
  controls.sm["carOutput"].actuatorsOutput.torque = 0.1
  controls.sm["carOutput"].actuatorsOutput.steeringAngleDeg = 1.0
  controls.publish(newest, lac_log)
  assert controls.steer_limited_by_safety
  assert controls.feedback_calls[-1].limited
  if steer_type == "torque":
    assert controls.feedback_calls[-1].torque_difference == pytest.approx(0.1)
    assert controls.feedback_calls[-1].applied_torque == pytest.approx(0.1)
  else:
    assert controls.feedback_calls[-1].torque_difference is None
    assert controls.feedback_calls[-1].applied_torque is None
  assert not controls.pm.latest("carControl").carControl.steerLimited
  controls.publish(newest, lac_log)
  assert controls.pm.latest("carControl").carControl.steerLimited

  # A good pair clears the flag only for the next tick, without replacing it
  # with a comparison against newest. Old replay/schema data remains frozen.
  controls.sm["carOutput"].actuatorsOutput = controls.sm["carOutput"].actuatorsInput
  controls.publish(newest, lac_log)
  assert not controls.steer_limited_by_safety
  assert controls.pm.latest("carControl").carControl.steerLimited
  controls.publish(newest, lac_log)
  assert not controls.pm.latest("carControl").carControl.steerLimited
  newest.latActive = False
  controls.publish(newest, lac_log)
  assert not controls.steer_limited_by_safety
  assert controls.feedback_calls[-1].limited
  assert controls.feedback_calls[-1].torque_difference is None
  assert controls.feedback_calls[-1].applied_torque is None
  newest.latActive = True
  controls.sm["carOutput"] = car.CarOutput.new_message(actuatorsOutput={"torque": -0.5, "steeringAngleDeg": -10.0})
  controls.publish(newest, lac_log)
  assert controls.steer_limited_by_safety
  assert controls.feedback_calls[-1].torque_difference is None
  assert controls.feedback_calls[-1].applied_torque is None


@pytest.mark.parametrize("controller", [None, SimpleNamespace()])
def test_controls_publish_does_not_require_a_feedback_hook(card_endpoint, monkeypatch, controller):
  endpoint, now = card_endpoint
  command = car.CarControl.new_message(latActive=True, actuators={"torque": 0.2})
  now[0] += 10_000_000
  apply_command(endpoint, command)
  now[0] += 10_000_000
  output = publish_output(endpoint)
  module = load_controls_module(monkeypatch)
  monkeypatch.setattr(module, "REPLAY", False)
  controls = controls_endpoint(module, Publisher(), output, "torque")
  if controller is None:
    del controls.LaC
  else:
    controls.LaC = controller
  controls.publish(command, log.ControlsState.LateralTorqueState.new_message())
  assert not controls.steer_limited_by_safety


def test_replay_freshness_uses_recorded_event_clock(card_endpoint, monkeypatch):
  endpoint, now = card_endpoint
  monkeypatch.setattr(card, "REPLAY", True)
  # The host is on a different boot clock from the route. card already uses
  # the recorded CAN time for CI.apply when REPLAY is set.
  now[0] = 9_000_000_000_000
  endpoint.can_log_mono_time = 1_010_000_000
  command = car.CarControl.new_message(latActive=True, actuators={"torque": 0.2})
  apply_command(endpoint, command)
  output = publish_output(endpoint)
  assert output.carOutput.actuationMonoTime == 1_010_000_000
  assert output.logMonoTime == now[0]
  # ProcessContainer.get_output_msgs replaces only the Event envelope time.
  output.logMonoTime = 1_020_000_000
  module = load_controls_module(monkeypatch)
  monkeypatch.setattr(module, "REPLAY", True)
  controls = controls_endpoint(module, Publisher(), output, "torque")
  controls.sm.logMonoTime["selfdriveState"] = 1_030_000_000
  lac_log = log.ControlsState.LateralTorqueState.new_message()
  controls.publish(command, lac_log)
  assert not controls.steer_limited_by_safety

  # Advancing recorded input time still expires the same feedback even though
  # the host clock has not advanced. Missing provenance is never synthesized.
  controls.sm.logMonoTime["selfdriveState"] = 1_060_000_001
  controls.publish(command, lac_log)
  assert controls.steer_limited_by_safety
