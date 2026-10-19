"""Exercise delay timing through the production lane-change state machine."""
from types import SimpleNamespace

import pytest

from iqpilot.cereal import car, log
from iqpilot.common.lane_change_settings import LANE_CHANGE_DELAYS, LaneSwapPreset
from iqpilot.common.params import Params
from iqpilot.common.realtime import DT_MDL
from iqpilot.selfdrive.controls.lib.action_engine import ActionEngine
from iqpilot.selfdrive.controls.lib.helpers.lane_change import LaneSwapEngine

State = log.LaneChangeState
Direction = log.LaneChangeDirection


@pytest.fixture
def make_engine():
  params = Params()
  params.put_bool('IQLaneTurnDesire', False)
  params.put_bool('NavExitLaneChange', False)
  params.put_bool('IQEdgeGuard', False)
  params.put_bool('LaneChangeContinuous', False)
  params.put_bool('IQLaneChangeBsmDelay', True)
  params.put('CarParams', car.CarParams.new_message(enableBsm=True).to_bytes())

  def make(preset=LaneSwapPreset.DELAY_ONE, direction='left'):
    params.put('IQLaneChangeTimer', preset)
    engine = ActionEngine()
    assert hasattr(engine, 'alc'), 'This integration test requires the production action-engine bundle'
    state = car.CarState.new_message(vEgo=25.0)
    engine.update(state, True, 1.0)
    setattr(state, direction + 'Blinker', True)
    engine.update(state, True, 1.0)  # Turn signal detected at t=0.
    assert engine.lane_change_state == State.preLaneChange
    return engine, state

  return make


@pytest.mark.parametrize('direction', ['left', 'right'])
@pytest.mark.parametrize('preset,delay', LANE_CHANGE_DELAYS.items())
def test_starts_at_selected_delay_from_signal_detection(make_engine, direction, preset, delay):
  engine, state = make_engine(preset, direction)
  frames = round(delay / DT_MDL)
  for _ in range(frames - 1):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
    assert engine.desire == log.Desire.none
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting
  assert engine.desire == (log.Desire.laneChangeLeft if direction == 'left' else log.Desire.laneChangeRight)


@pytest.mark.parametrize('preset,delay', LANE_CHANGE_DELAYS.items())
def test_brief_blindspot_never_shortens_delay(make_engine, preset, delay):
  engine, state = make_engine(preset)
  state.leftBlindspot = True
  engine.update(state, True, 1.0)
  state.leftBlindspot = False
  for _ in range(round(delay / DT_MDL) - 2):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange


def test_persistent_blindspot_blocks_and_clearance_wait_is_preserved(make_engine):
  engine, state = make_engine()
  state.leftBlindspot = True
  for _ in range(60):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
  state.leftBlindspot = False
  for _ in range(19):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting


def test_brake_cancels_automatic_attempt_until_signal_rearmed(make_engine):
  engine, state = make_engine()
  state.brakePressed = True
  engine.update(state, True, 1.0)
  state.brakePressed = False
  for _ in range(40):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state != State.laneChangeStarting
  state.leftBlinker = False
  engine.update(state, True, 1.0)
  state.leftBlinker = True
  engine.update(state, True, 1.0)
  for _ in range(19):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting


@pytest.mark.parametrize('cancel', ['signal', 'hazards', 'disengage', 'low_speed'])
def test_cancellation_discards_partial_countdown(make_engine, cancel):
  engine, state = make_engine()
  for _ in range(10):
    engine.update(state, True, 1.0)
  if cancel == 'signal':
    state.leftBlinker = False
  elif cancel == 'hazards':
    state.rightBlinker = True
  elif cancel == 'low_speed':
    state.vEgo = 0.0
  engine.update(state, cancel != 'disengage', 1.0)
  assert engine.lane_change_state == State.off
  assert not engine.alc.ready
  state.leftBlinker = state.rightBlinker = False
  state.vEgo = 25.0
  engine.update(state, True, 1.0)
  state.leftBlinker = True
  engine.update(state, True, 1.0)
  for _ in range(19):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting


def test_switching_signal_direction_does_not_reuse_elapsed_time(make_engine):
  engine, state = make_engine()
  for _ in range(10):
    engine.update(state, True, 1.0)
  state.leftBlinker = False
  state.rightBlinker = True
  engine.update(state, True, 1.0)
  for _ in range(19):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state != State.laneChangeStarting
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting
  assert engine.desire == log.Desire.laneChangeRight


def test_nudge_mode_still_requires_driver_nudge(make_engine):
  engine, state = make_engine(LaneSwapPreset.STEERING_NUDGE)
  for _ in range(80):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
  state.steeringPressed = True
  state.steeringTorque = 1.0
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting


def test_no_nudge_mode_keeps_its_existing_short_wait(make_engine):
  engine, state = make_engine(LaneSwapPreset.DIRECT)
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.preLaneChange
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting


@pytest.mark.parametrize('preset,delay', LANE_CHANGE_DELAYS.items())
def test_new_signal_uses_selection_saved_since_last_periodic_refresh(make_engine, preset, delay):
  engine, state = make_engine(LaneSwapPreset.DIRECT)
  state.leftBlinker = False
  engine.update(state, True, 1.0)
  Params().put('IQLaneChangeTimer', preset)
  state.leftBlinker = True
  engine.update(state, True, 1.0)
  for _ in range(round(delay / DT_MDL) - 1):
    engine.update(state, True, 1.0)
    assert engine.lane_change_state == State.preLaneChange
  engine.update(state, True, 1.0)
  assert engine.lane_change_state == State.laneChangeStarting


@pytest.mark.parametrize('preset', [LaneSwapPreset.DISABLED, LaneSwapPreset.STEERING_NUDGE, 999])
def test_nonautomatic_or_unknown_preset_never_fires(preset):
  Params().put('IQLaneChangeTimer', preset)
  helper = LaneSwapEngine(SimpleNamespace(lane_change_state=State.preLaneChange, lane_change_direction=Direction.left))
  for _ in range(80):
    helper.sample()
    assert helper.preset == preset
    assert not helper.ready
