import queue
import threading
import time
from types import SimpleNamespace

import pytest

from iqpilot.system.hardware import hardwared


@pytest.fixture
def run_hardware_loop(monkeypatch, mocker):
  """Run the real publish loop with simulated hardware and no live-device side effects."""
  params = mocker.Mock()
  params.get_bool.return_value = False
  params.get.side_effect = lambda key, **kwargs: {'UptimeOffroad': 0., 'UptimeOnroad': 0.}.get(key)
  monkeypatch.setattr(hardwared, 'Params', lambda: params)
  hardware = mocker.Mock()
  hardware.get_device_type.return_value = 'pc'
  hardware.get_thermal_config.return_value = SimpleNamespace(get_msg=lambda: hardwared.log.DeviceState.new_message(cpuTempC=[40.]))
  for name in ('get_gpu_usage_percent', 'get_current_power_draw', 'get_som_power_draw'):
    getattr(hardware, name).return_value = 0.
  hardware.get_screen_brightness.return_value = 50
  hardware.booted.return_value = True
  monkeypatch.setattr(hardwared, 'HARDWARE', hardware)
  power = mocker.Mock()
  power.get_power_used.return_value = 0
  power.get_car_battery_capacity.return_value = 0
  power.should_shutdown.return_value = False
  monkeypatch.setattr(hardwared, 'PowerMonitoring', lambda: power)
  monkeypatch.setattr(hardwared, '_CarParamsCache', lambda: mocker.Mock(no_sleep=True))
  fan = mocker.Mock()
  fan.update.return_value = 0
  monkeypatch.setattr(hardwared, 'FanController', lambda: fan)
  monkeypatch.setattr(hardwared, 'EgpuDockFlasher', lambda: mocker.Mock(failed=False))
  monkeypatch.setattr(hardwared, 'EgpuDockStatus', mocker.Mock())
  monkeypatch.setattr(hardwared, 'PerfTraceEmitter', mocker.Mock())
  monkeypatch.setattr(hardwared, 'set_offroad_alert_if_changed', mocker.Mock())
  monkeypatch.setattr(hardwared, 'cloudlog', mocker.Mock())
  monkeypatch.setattr(hardwared, 'get_available_percent', lambda **kwargs: 100.)
  monkeypatch.setattr(hardwared.psutil, 'virtual_memory', lambda: SimpleNamespace(percent=40))
  monkeypatch.setattr(hardwared.psutil, 'cpu_percent', lambda **kwargs: [0.])

  def run(samples):
    params.reset_mock()
    end = threading.Event()
    hw_queue = queue.Queue()
    messages = {name: getattr(hardwared.messaging.new_message(name), name)
                for name in ('peripheralState', 'selfdriveState', 'carState', 'egpuDockState', 'gpsLocationExternal')}
    messages['pandaStates'] = []
    sm = mocker.MagicMock()
    sm.__getitem__.side_effect = messages.__getitem__
    sm.frame = 0
    sm.alive = dict.fromkeys(messages, False)
    sm.valid = dict.fromkeys(messages, False)
    sm.updated = dict.fromkeys(messages, False)
    sm.recv_time = {'pandaStates': time.monotonic()}
    pending = iter(samples)

    def update(timeout):
      sm.frame += round(hardwared.SERVICE_LIST['pandaStates'].frequency * hardwared.DT_HW)
      metered = next(pending)
      if metered is not None:
        hw_queue.put_nowait(hardwared.HardwareState(
          network_type=hardwared.NetworkType.wifi, network_info=None, network_strength=hardwared.NetworkStrength.unknown,
          network_stats={'wwanTx': -1, 'wwanRx': -1}, network_metered=metered, modem_temps=[], usb_state=[],
          usb_link_errors=0, usb3_lane='unknown',
        ))

    sm.update.side_effect = update
    published = []

    def send(service, msg):
      assert service == 'deviceState'
      published.append(msg.deviceState.networkMetered)
      if len(published) == len(samples):
        end.set()

    monkeypatch.setattr(hardwared.messaging, 'SubMaster', lambda *args, **kwargs: sm)
    monkeypatch.setattr(hardwared.messaging, 'PubMaster', lambda *args, **kwargs: SimpleNamespace(send=send))
    hardwared.hardware_thread(end, hw_queue, queue.Queue())
    writes = [call.args[1] for call in params.put_bool_nonblocking.call_args_list if call.args[0] == 'NetworkMetered']
    assert all(call.args[0] != 'NetworkMetered' for call in params.get.call_args_list + params.get_bool.call_args_list)
    return published, writes

  return run


@pytest.mark.parametrize('metered', [False, True])
def test_unchanged_network_publishes_every_cycle_but_writes_once(run_hardware_loop, metered):
  samples = [metered] * 120
  published, writes = run_hardware_loop(samples)
  assert published == samples
  assert writes == [metered]


@pytest.mark.parametrize('initial', [False, True])
def test_each_network_transition_is_persisted(run_hardware_loop, initial):
  samples = [initial] * 40 + [not initial] * 40 + [initial] * 40
  published, writes = run_hardware_loop(samples)
  assert published == samples
  assert writes == [initial, not initial, initial]


def test_empty_hardware_queue_keeps_last_known_state_without_rewriting(run_hardware_loop):
  published, writes = run_hardware_loop([None] * 4 + [True] + [None] * 20 + [False] + [None] * 20)
  assert published == [False] * 4 + [True] * 21 + [False] * 21
  assert writes == [False, True, False]


@pytest.mark.parametrize('metered', [False, True])
def test_restarting_hardware_loop_republishes_parameter(run_hardware_loop, metered):
  for _ in range(2):
    published, writes = run_hardware_loop([metered] * 10)
    assert published == [metered] * 10
    assert writes == [metered]
