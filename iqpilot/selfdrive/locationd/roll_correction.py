"""Optional chassis-referenced roll offset. Device attitude itself is unchanged.

Only the tested Kona's bus-0 YRS/ESP messages are accepted. A slow offset is
shared by paramsd and torqued; CAN gravity residuals never directly drive roll.
"""
import json
import math
import os
import statistics
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np

from iqpilot.common.transformations.orientation import euler_from_rot, rot_from_euler
from iqpilot.common.vehicle_roll_settings import vehicle_roll_state_path

SUPPORTED_CAR = "HYUNDAI_KONA_EV"
STATE_VERSION = 1
MAX_OFFSET_DEG = 5.0
MAX_RATE_DEG_S = 0.01
MAX_INNOVATION_DEG = 0.75
MAX_CAN_AGE = 0.05
G = 9.81


def finite_values(values):
  return all(math.isfinite(v) for v in values)


class ChassisIMU:
  """Bounded decoder with independent-message agreement and freshness checks."""
  def __init__(self):
    self.samples = {}
    self.counters = {}

  def feed(self, t, messages, valid=True):
    if not valid:
      self.samples.clear()
      self.counters.clear()
      return
    for msg in messages:
      address = msg.address
      if msg.src != 0 or address not in (0x130, 0x140, 0x220):
        continue
      data = bytes(msg.dat)
      if len(data) != 8:
        self.samples.pop(address, None)
        continue
      bits = int.from_bytes(data, "little")
      if address == 0x130:
        values = ((bits & 0xffff) * .005 - 163.84, ((bits >> 16) & 0xffff) * .000127465 - 4.17677312)
        healthy = ((bits >> 32) & 0xfff) == 0
        counter = (bits >> 48) & 15
      elif address == 0x140:
        values = ((bits & 0xffff) * .000127465 - 4.17677312,)
        healthy = ((bits >> 16) & 0xff) == 0
        counter = (bits >> 48) & 15
      else:
        values = ((bits & 0x7ff) * .01 - 10.23, ((bits >> 13) & 0x7ff) * .01 - 10.23,
                  ((bits >> 40) & 0x1fff) * .01 - 40.95)
        healthy = ((bits >> 11) & 3) == 0 and ((bits >> 24) & 3) == 0 and ((bits >> 53) & 3) == 0
        counter = (bits >> 60) & 15
      previous = self.counters.get(address)
      self.counters[address] = counter
      if previous is not None and (counter - previous) % 16 != 1:
        healthy = False
      self.samples[address] = (t, values, healthy)

  def read(self, t):
    rows = [self.samples.get(address) for address in (0x130, 0x140, 0x220)]
    if any(row is None or not row[2] or not 0 <= t - row[0] <= MAX_CAN_AGE for row in rows):
      return None
    yaw, lat_g = rows[0][1]
    long_g = rows[1][1][0]
    esp_lat, esp_long, esp_yaw = rows[2][1]
    if abs(lat_g * G - esp_lat) > .3 or abs(long_g * G - esp_long) > .5 or abs(yaw - esp_yaw) > 1.:
      return None
    return yaw, lat_g


class RollOffsetLearner:
  def __init__(self, offset_deg=None, yaw_zero_dps=None):
    self.offset = offset_deg
    self.applied = offset_deg or 0.
    self.yaw_zero = yaw_zero_dps
    self.last_t = None
    self.gravity = self.yaw_lp = self.accel_lp = None
    self.valid_time = self.steady_time = self.mismatch_time = 0.
    self.yaw_samples = deque()
    self.bootstrap = deque()
    self.qualified = False
    self.reference_roll = None
    self.status = "saved" if offset_deg is not None else "waiting"

  def reset_observations(self):
    self.gravity = self.yaw_lp = self.accel_lp = None
    self.valid_time = self.steady_time = self.mismatch_time = 0.
    self.yaw_samples.clear()
    self.bootstrap.clear()
    self.qualified = False
    self.reference_roll = None

  def invalidate(self, reason):
    # Retain the applied value until it can be removed/replaced without a step.
    self.offset = None
    self.yaw_zero = None
    self.reset_observations()
    self.status = reason

  def update(self, t, speed, accel, device_roll_deg, imu, pose_valid, lat_active):
    dt = 0. if self.last_t is None else t - self.last_t
    self.last_t = t
    if not 0 < dt <= .15:
      self.reset_observations()
      return self.applied
    self.qualified = False
    if imu is None or not pose_valid or not finite_values((speed, accel, device_roll_deg, *imu)):
      self.reset_observations()
      self.status = "holding_invalid_input"
    else:
      yaw_dps, lat_g = imu
      stationary = abs(speed) < .05
      if stationary and abs(yaw_dps) <= 1.:
        self.yaw_samples.append((t, yaw_dps))
        while self.yaw_samples and t - self.yaw_samples[0][0] > 2.1:
          self.yaw_samples.popleft()
        if self.yaw_zero is None and t - self.yaw_samples[0][0] >= 2.:
          self.yaw_zero = statistics.median(v for _, v in self.yaw_samples)
        elif self.yaw_zero is not None:
          self.yaw_zero += dt / (30. + dt) * (yaw_dps - self.yaw_zero)
      else:
        self.yaw_samples.clear()
      yaw = math.radians(yaw_dps - (self.yaw_zero or 0.))
      gravity = G * lat_g - speed * yaw
      if self.gravity is None:
        self.gravity, self.yaw_lp, self.accel_lp = gravity, yaw, accel
        yaw_dot = 0.
      else:
        self.gravity += dt / (.5 + dt) * (gravity - self.gravity)
        old_yaw = self.yaw_lp
        self.yaw_lp += dt / (.2 + dt) * (yaw - self.yaw_lp)
        yaw_dot = (self.yaw_lp - old_yaw) / dt
        self.accel_lp += dt / (.3 + dt) * (accel - self.accel_lp)
      self.valid_time += dt
      self.reference_roll = math.degrees(math.asin(float(np.clip(self.gravity / G, -1., 1.))))
      steady = stationary or (speed >= 3. and abs(yaw) < .05 and abs(self.yaw_lp) < .05 and abs(yaw_dot) < .02
                              and abs(accel) < .4 and abs(self.accel_lp) < .4)
      steady = steady and self.yaw_zero is not None and self.valid_time >= 2. and abs(self.gravity) < .3 * G
      self.steady_time = self.steady_time + dt if steady else 0.
      self.qualified = self.steady_time >= 2.
      self.status = "learning" if self.qualified else "holding_motion"
      if self.qualified:
        candidate = device_roll_deg - self.reference_roll
        if self.offset is None:
          if abs(candidate) <= MAX_OFFSET_DEG:
            self.bootstrap.append((t, candidate, dt))
          while self.bootstrap and t - self.bootstrap[0][0] > 60.:
            self.bootstrap.popleft()
          if sum(row[2] for row in self.bootstrap) >= 10.:
            median = statistics.median(row[1] for row in self.bootstrap)
            mad = statistics.median(abs(row[1] - median) for row in self.bootstrap)
            if mad <= .2:
              self.offset = median
              self.bootstrap.clear()
        elif abs(candidate - self.offset) <= MAX_INNOVATION_DEG and abs(candidate) <= MAX_OFFSET_DEG:
          step = dt / (30. + dt) * (candidate - self.offset)
          self.offset += float(np.clip(step, -MAX_RATE_DEG_S * dt, MAX_RATE_DEG_S * dt))
          self.mismatch_time = 0.
        else:
          self.mismatch_time += dt
          self.status = "offset_disagreement"
          if self.mismatch_time >= 10.:
            self.invalidate("reacquiring_after_disagreement")
      else:
        self.mismatch_time = 0.
    target = self.offset if self.offset is not None else 0.
    if not lat_active and abs(speed) < .05:
      self.applied = target
    else:
      self.applied += float(np.clip(target - self.applied, -MAX_RATE_DEG_S * dt, MAX_RATE_DEG_S * dt))
    return self.applied


def read_state(path, fingerprint):
  try:
    state = json.loads(Path(path).read_text())
    if state["version"] != STATE_VERSION or state["carFingerprint"] != fingerprint:
      return None
    offset, yaw = float(state["offsetDeg"]), float(state["yawZeroDps"])
    calibration = [float(v) for v in state["calibrationRpy"]]
    if len(calibration) != 3 or not finite_values((offset, yaw, *calibration)) or abs(offset) > MAX_OFFSET_DEG or abs(yaw) > 1.:
      return None
    return {**state, "offsetDeg": offset, "yawZeroDps": yaw, "calibrationRpy": calibration}
  except (OSError, ValueError, TypeError, KeyError):
    return None


def write_state(path, state):
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = None
  try:
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, prefix=".vehicle_roll_", delete=False) as stream:
      temporary = Path(stream.name)
      json.dump(state, stream, allow_nan=False, separators=(",", ":"))
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(temporary, path)
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)


class VehicleRollCorrection:
  def __init__(self, params, fingerprint):
    self.fingerprint = fingerprint
    self.state_path = vehicle_roll_state_path(params)
    state = read_state(self.state_path, fingerprint)
    self.learner = RollOffsetLearner(state["offsetDeg"] if state else None, state["yawZeroDps"] if state else None)
    self.calibration = np.array(state["calibrationRpy"]) if state else None
    self.saved_calibration = self.calibration.copy() if self.calibration is not None else None
    self.calibration_valid = False
    self.imu = ChassisIMU()
    self.car = None
    self.car_time = self.control_time = -math.inf
    self.lat_active = True

  def handle_log(self, t, which, msg, valid=True):
    if which == "can":
      self.imu.feed(t, msg, valid)
    elif which == "carState":
      self.car = msg if valid and msg.canValid else None
      self.car_time = t
    elif which == "carControl":
      self.lat_active = msg.latActive if valid else True
      self.control_time = t
    elif which == "extrinsicsCalibration":
      self.calibration_valid = valid and str(msg.calStatus) == "calibrated" and len(msg.rpyCalib) == 3 and finite_values(msg.rpyCalib)
      if self.calibration_valid:
        calibration = np.array(msg.rpyCalib)
        changed = self.saved_calibration is not None and np.max(np.abs(calibration - self.saved_calibration)) > math.radians(2.)
        stopped_inactive = (self.car is not None and 0 <= t - self.car_time <= .1 and abs(self.car.vEgo) < .05
                            and 0 <= t - self.control_time <= .1 and not self.lat_active)
        # Fix the transform while moving: changing it directly would bypass the
        # learned-offset slew limit. Adopt new extrinsics only at a safe stop.
        if self.calibration is None or stopped_inactive:
          if changed:
            self.learner.invalidate("calibration_changed")
          if changed or self.saved_calibration is None:
            self.saved_calibration = calibration.copy()
          self.calibration = calibration
        elif changed:
          self.calibration_valid = False
    elif which == "deviceMotion":
      orientation = msg.orientationNED
      if not finite_values((orientation.x, orientation.y, orientation.z)):
        self.learner.reset_observations()
        return None
      # Preserve the last known transform during a calibration interruption.
      # Learning requires a current, calibrated frame; output changes remain slow.
      calibrated = np.array([orientation.x, orientation.y, orientation.z])
      if self.calibration is not None:
        calibrated = euler_from_rot(rot_from_euler(calibrated) @ rot_from_euler(self.calibration))
      car_valid = self.car is not None and 0 <= t - self.car_time <= .1
      speed, accel = (self.car.vEgo, self.car.aEgo) if car_valid else (0., 0.)
      active = self.lat_active if 0 <= t - self.control_time <= .1 and car_valid else True
      good = (valid and car_valid and self.calibration_valid and orientation.valid and msg.sensorsOK and msg.posenetOK
              and math.isfinite(orientation.xStd) and 0 < orientation.xStd < math.radians(1.5))
      offset = self.learner.update(t, speed, accel, math.degrees(calibrated[0]), self.imu.read(t), good, active)
      return float(calibrated[0] - math.radians(offset))
    return None

  def snapshot(self):
    state = {"version": STATE_VERSION, "carFingerprint": self.fingerprint, "updatedMonoTime": time.monotonic(),
             "offsetDeg": self.learner.offset, "yawZeroDps": self.learner.yaw_zero,
             "calibrationRpy": self.calibration.tolist() if self.calibration is not None else None,
             "status": self.learner.status}
    # A null offset deliberately invalidates a previously persisted calibration.
    return state
