"""Lane-change modes shared by the controls and both settings layouts."""


class LaneSwapPreset:
  DISABLED = -1
  STEERING_NUDGE = 0
  DIRECT = 1
  # Keep the stored values of the original delay presets compatible.
  DELAY_HALF = 2
  DELAY_ONE = 3
  DELAY_TWO = 4
  DELAY_THREE = 5
  DELAY_ONE_HALF = 6
  DELAY_TWO_HALF = 7
  OFF = DISABLED
  NUDGE = STEERING_NUDGE
  NUDGELESS = DIRECT
  HALF_SECOND = DELAY_HALF
  ONE_SECOND = DELAY_ONE
  TWO_SECONDS = DELAY_TWO
  THREE_SECONDS = DELAY_THREE


LANE_CHANGE_DELAYS = {
  LaneSwapPreset.DELAY_HALF: 0.5,
  LaneSwapPreset.DELAY_ONE: 1.0,
  LaneSwapPreset.DELAY_ONE_HALF: 1.5,
  LaneSwapPreset.DELAY_TWO: 2.0,
  LaneSwapPreset.DELAY_TWO_HALF: 2.5,
  LaneSwapPreset.DELAY_THREE: 3.0,
}
DEFAULT_LANE_CHANGE_DELAY = LaneSwapPreset.DELAY_ONE
DELAYED_MODE = 2  # UI index; the stored preset also selects the delay.


def lane_change_mode(preset) -> int:
  if preset in LANE_CHANGE_DELAYS:
    return DELAYED_MODE
  return 1 if preset == LaneSwapPreset.DIRECT else 0


def lane_change_delay_label(preset) -> str:
  return f"{LANE_CHANGE_DELAYS.get(preset, LANE_CHANGE_DELAYS[DEFAULT_LANE_CHANGE_DELAY]):.1f} s"
