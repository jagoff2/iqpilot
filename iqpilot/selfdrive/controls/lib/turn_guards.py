"""Experimental acceleration shaping derived from the backup's cut/apex rules.

The backup gains and individual caps are retained. Both rules now require a
same-direction, delay-aligned overtracking error, and their combined reduction
cannot reverse the requested acceleration. This detects request overtracking,
not lane boundaries or an apex, and does not establish closed-loop efficacy.
"""


K_CUT = 0.60
K_APEX = 0.40


def apply_turn_guards(desired_lateral_accel: float, actual_lateral_accel: float, expected_lateral_accel: float) -> float:
  """Shape the current target using an unmodified, delayed request (all m/s²)."""
  same_direction = (desired_lateral_accel > 0.0 and actual_lateral_accel > 0.0 and expected_lateral_accel > 0.0 or
                    desired_lateral_accel < 0.0 and actual_lateral_accel < 0.0 and expected_lateral_accel < 0.0)
  if not same_direction:
    return desired_lateral_accel

  a_des_mag = abs(desired_lateral_accel)
  inside_excess = max(0.0, abs(actual_lateral_accel) - max(a_des_mag, abs(expected_lateral_accel)))
  cut_guard = min(K_CUT * inside_excess, 0.50 * (a_des_mag + 0.5))
  apex_guard = min(K_APEX * inside_excess, 0.40 * (a_des_mag + 0.5))
  reduction = min(cut_guard + apex_guard, a_des_mag)
  return desired_lateral_accel - reduction if desired_lateral_accel > 0.0 else desired_lateral_accel + reduction
