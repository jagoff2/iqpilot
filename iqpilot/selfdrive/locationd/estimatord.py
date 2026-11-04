#!/usr/bin/env python3
"""
Copyright © IQ.Lvbs, apart of Project Teal Lvbs, All Rights Reserved, licensed under https://konn3kt.com/tos
"""
import os
import threading

import numpy as np

import iqpilot.cereal.messaging as messaging
from iqpilot.cereal import car
from iqpilot.cereal.services import SERVICE_LIST
from iqpilot.common.params import Params
from iqpilot.common.realtime import config_realtime_process
from iqpilot.common.steer_delay import SteerDelayPublisher
from iqpilot.common.swaglog import cloudlog
from iqpilot.common.vehicle_roll_settings import get_vehicle_roll_correction_enabled
from iqpilot.selfdrive.locationd.lagd import LateralLagEstimator, retrieve_initial_lag
from iqpilot.selfdrive.locationd.paramsd import (
  VehicleParamsEstimator,
  migrate_cached_vehicle_params_if_needed,
  retrieve_initial_vehicle_params,
)
from iqpilot.selfdrive.locationd.torqued import TorqueEstimator
from iqpilot.selfdrive.locationd.roll_correction import SUPPORTED_CAR, VehicleRollCorrection, write_state


PARAMS_SERVICES = ['deviceMotion', 'extrinsicsCalibration', 'carState']
LAG_SERVICES = ['deviceMotion', 'extrinsicsCalibration', 'carState', 'controlsState', 'carControl']
TORQUE_SERVICES = ['carControl', 'carOutput', 'carState', 'extrinsicsCalibration', 'deviceMotion']
SUBSCRIBED_SERVICES = list(dict.fromkeys(PARAMS_SERVICES + LAG_SERVICES + TORQUE_SERVICES))


def main() -> None:
  config_realtime_process([0, 1, 2, 3], 5)

  debug = bool(int(os.getenv('DEBUG', '0')))
  replay = bool(int(os.getenv('REPLAY', '0')))

  pm = messaging.PubMaster(['vehicleParameters', 'lateralDelay', 'lateralTorqueParameters'])
  sm = messaging.SubMaster(SUBSCRIBED_SERVICES, poll='deviceMotion')

  params = Params()
  CP = messaging.log_from_bytes(params.get('CarParams', block=True), car.CarParams)
  # Read once per drive. Turning the option off restores the legacy input path.
  roll_enabled = get_vehicle_roll_correction_enabled(params) and CP.carFingerprint == SUPPORTED_CAR
  roll_correction = VehicleRollCorrection(params, CP.carFingerprint) if roll_enabled else None
  can_socket = messaging.sub_sock('can') if roll_enabled else None
  state_writer = None

  def save_roll_state(snapshot):
    try:
      write_state(roll_correction.state_path, snapshot)
    except OSError:
      cloudlog.exception('Unable to persist vehicle roll correction')

  migrate_cached_vehicle_params_if_needed(params)
  steer_ratio, stiffness_factor, angle_offset_deg, p_initial = retrieve_initial_vehicle_params(params, CP, replay, debug)
  params_estimator = VehicleParamsEstimator(CP, steer_ratio, stiffness_factor, np.radians(angle_offset_deg), p_initial)

  lag_estimator = LateralLagEstimator(CP, 1. / SERVICE_LIST['deviceMotion'].frequency)
  if (initial_lag_params := retrieve_initial_lag(params, CP)) is not None:
    lag, valid_blocks = initial_lag_params
    lag_estimator.reset(lag, valid_blocks)

  torque_estimator = TorqueEstimator(CP, roll_correction=roll_enabled)
  torque_roll_offset = roll_correction.learner.applied if roll_enabled else 0.
  torque_roll_initialized = roll_correction.learner.offset is not None if roll_enabled else False
  steer_delay_publisher = SteerDelayPublisher(CP)
  estimators = (
    (params_estimator, PARAMS_SERVICES),
    (lag_estimator, LAG_SERVICES),
    (torque_estimator, TORQUE_SERVICES),
  )

  while True:
    sm.update()
    valid = sm.all_checks()

    messages = [(sm.logMonoTime[which], which, sm[which], sm.valid[which]) for which in sm.updated if sm.updated[which]]
    if roll_correction is not None:
      messages.extend((event.logMonoTime, 'can', event.can, event.valid) for event in messaging.drain_sock(can_socket))
    for log_time, which, data, input_valid in sorted(messages, key=lambda row: row[0]):
      t = log_time * 1e-9
      roll_override = None
      if roll_correction is not None:
        roll_override = roll_correction.handle_log(t, which, data, input_valid and (valid or which != 'deviceMotion'))
        initialized = roll_correction.learner.offset is not None
        if initialized != torque_roll_initialized or abs(roll_correction.learner.applied - torque_roll_offset) >= .1:
          torque_estimator.reset_roll_reference()
          torque_roll_offset = roll_correction.learner.applied
          torque_roll_initialized = initialized
      if valid:
        for estimator, services in estimators:
          if which in services:
            if which == 'deviceMotion' and estimator in (params_estimator, torque_estimator):
              estimator.handle_log(t, which, data, roll_override=roll_override)
            else:
              estimator.handle_log(t, which, data)
    if valid:
      lag_estimator.update_points()

    if not sm.updated['deviceMotion']:
      continue

    params_msg = params_estimator.get_msg(valid, debug=debug)
    params_msg_dat = params_msg.to_bytes()
    if sm.frame % 1200 == 0:
      params.put_nonblocking('LiveParametersV2', params_msg_dat)
    pm.send('vehicleParameters', params_msg_dat)

    if roll_correction is not None and sm.frame % 200 == 0:
      cloudlog.event('vehicle_roll_correction', **roll_correction.snapshot(), appliedOffsetDeg=roll_correction.learner.applied,
                     learning=roll_correction.learner.qualified, canRollDeg=roll_correction.learner.reference_roll)
    if roll_correction is not None and not replay and sm.frame % 600 == 0 and (state_writer is None or not state_writer.is_alive()):
      state_writer = threading.Thread(target=save_roll_state, args=(roll_correction.snapshot(),), daemon=True)
      state_writer.start()

    if sm.frame % 5 != 0:
      continue

    lag_estimator.update_estimate()
    lag_msg = lag_estimator.get_msg(valid, debug)
    lag_msg_dat = lag_msg.to_bytes()
    pm.send('lateralDelay', lag_msg_dat)

    torque_estimator.handle_log(sm.logMonoTime['deviceMotion'] * 1e-9, 'lateralDelay', lag_msg.lateralDelay)
    pm.send('lateralTorqueParameters', torque_estimator.get_msg(valid=valid, with_points=debug))

    if sm.frame % 1200 == 0:
      params.put_nonblocking('LiveDelay', lag_msg_dat)

    if sm.frame % 60 == 0:
      steer_delay_publisher.update(lag_msg)

    if sm.frame % 240 == 0:
      torque_msg = torque_estimator.get_msg(valid=valid, with_points=True)
      params.put_nonblocking('LiveTorqueParameters', torque_msg.to_bytes())


if __name__ == '__main__':
  main()
