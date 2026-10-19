from types import SimpleNamespace

import pytest

from iqpilot.common.params import Params
from iqpilot.common.lane_change_settings import DEFAULT_LANE_CHANGE_DELAY, DELAYED_MODE, LANE_CHANGE_DELAYS, lane_change_delay_label
from iqpilot.selfdrive.ui.mici.layouts.settings import steering
from iqpilot.system.ui.widgets import DialogResult
from iqpilot.system.ui.lib.application import MouseEvent, MousePos
from iqpilot.system.ui.widgets import slider as slider_module
from iqpilot.ui.layouts.settings import iq_panels


def test_set_convergence_defaults_on_and_remains_configurable(tmp_path):
  params = Params(str(tmp_path))
  assert params.get_default_value('expSpeedConv') is True
  params.put_bool('expSpeedConv', False)
  assert not params.get_bool('expSpeedConv')


def test_experimental_lead_mpc_defaults_off_and_remains_configurable(tmp_path):
  params = Params(str(tmp_path))
  assert params.get_default_value('newLeadMpc') is False
  assert not params.get_bool('newLeadMpc')
  params.put_bool('newLeadMpc', True)
  assert params.get_bool('newLeadMpc')


@pytest.mark.parametrize('confirmed', [False, True])
def test_mici_no_nudge_waits_for_confirmation(monkeypatch, tmp_path, mocker, confirmed):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', 0)
  selector = SimpleNamespace(_params=params, _param='IQLaneChangeTimer', refresh=mocker.Mock())
  selector._enable_no_nudge = lambda: steering.LaneChangeSelector._enable_no_nudge(selector)
  dialogs = []
  monkeypatch.setattr(steering.BigButton, '_handle_mouse_release', lambda *args: None)
  monkeypatch.setattr(steering, 'NoNudgeConfirmation', lambda callback: callback)
  monkeypatch.setattr(steering, 'gui_app', SimpleNamespace(push_widget=dialogs.append))

  steering.LaneChangeSelector._handle_mouse_release(selector, None)
  assert params.get('IQLaneChangeTimer') == 0
  assert len(dialogs) == 1
  if confirmed:
    dialogs[0]()
  assert params.get('IQLaneChangeTimer') == int(confirmed)


@pytest.mark.parametrize('current,expected', [(1, DEFAULT_LANE_CHANGE_DELAY), *[(preset, 0) for preset in LANE_CHANGE_DELAYS]])
def test_mici_cycles_no_nudge_delayed_nudge(monkeypatch, tmp_path, mocker, current, expected):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', current)
  selector = SimpleNamespace(_params=params, _param='IQLaneChangeTimer', refresh=mocker.Mock())
  monkeypatch.setattr(steering.BigButton, '_handle_mouse_release', lambda *args: None)
  push = mocker.Mock()
  monkeypatch.setattr(steering, 'gui_app', SimpleNamespace(push_widget=push))
  steering.LaneChangeSelector._handle_mouse_release(selector, None)
  assert params.get('IQLaneChangeTimer') == expected
  push.assert_not_called()


@pytest.mark.parametrize('right_to_left', [False, True])
@pytest.mark.parametrize('distance', [0, 80, -216, 216])
def test_slider_requires_a_complete_slide_in_its_direction(monkeypatch, mocker, right_to_left, distance):
  monkeypatch.setattr(slider_module, 'gui_app', SimpleNamespace(
    font=lambda weight: None, target_fps=20,
    texture=lambda path, width, height: SimpleNamespace(width=width, height=height),
  ))
  monkeypatch.setattr(slider_module, 'UnifiedLabel', mocker.Mock())
  monkeypatch.setattr(slider_module.rl, 'get_time', lambda: 1.0)
  confirmed = mocker.Mock()
  slider = slider_module.SmallSlider('slide to enable', confirmed, right_to_left=right_to_left)
  start_x = 274 if right_to_left else 58
  end_x = start_x + (-distance if right_to_left else distance)
  slider._handle_mouse_event(MouseEvent(MousePos(start_x, 50), 0, True, False, True, 1.0))
  slider._handle_mouse_event(MouseEvent(MousePos(end_x, 50), 0, False, False, True, 1.0))
  slider._update_state()
  confirmed.assert_not_called()
  slider._handle_mouse_event(MouseEvent(MousePos(end_x, 50), 0, False, True, False, 1.0))
  monkeypatch.setattr(slider_module.rl, 'get_time', lambda: 2.0)
  for _ in range(30):
    slider._update_state()
  assert confirmed.call_count == int(distance == 216)
  assert slider.slider_percentage == pytest.approx(float(distance == 216), abs=0.001)


@pytest.mark.parametrize('result', [DialogResult.CANCEL, DialogResult.CONFIRM])
@pytest.mark.parametrize('selected,preset', [(1, 1), (DELAYED_MODE, DEFAULT_LANE_CHANGE_DELAY)])
def test_big_automatic_modes_wait_for_confirmation(monkeypatch, tmp_path, mocker, result, selected, preset):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', 0)
  action = mocker.Mock()
  panel = SimpleNamespace(_lane_change_setting=SimpleNamespace(action_item=action))
  dialogs = []
  monkeypatch.setattr(iq_panels, 'ui_state', SimpleNamespace(params=params))
  monkeypatch.setattr(iq_panels, 'ConfirmDialog', lambda *args, **kwargs: args)
  monkeypatch.setattr(iq_panels, 'gui_app', SimpleNamespace(set_modal_overlay=lambda dialog, callback: dialogs.append(callback)))

  iq_panels.SteeringLayout._set_lane_change_mode(panel, selected)
  assert params.get('IQLaneChangeTimer') == 0
  action.set_selected_button.assert_called_once_with(0)
  dialogs[0](result)
  assert params.get('IQLaneChangeTimer') == (preset if result == DialogResult.CONFIRM else 0)


def test_big_nudge_does_not_require_confirmation(monkeypatch, tmp_path, mocker):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', 1)
  monkeypatch.setattr(iq_panels, 'ui_state', SimpleNamespace(params=params))
  overlay = mocker.Mock()
  monkeypatch.setattr(iq_panels, 'gui_app', SimpleNamespace(set_modal_overlay=overlay))
  iq_panels.SteeringLayout._set_lane_change_mode(SimpleNamespace(), 0)
  assert params.get('IQLaneChangeTimer') == 0
  overlay.assert_not_called()


@pytest.mark.parametrize('preset', LANE_CHANGE_DELAYS)
def test_mici_shows_delayed_mode_and_selected_duration(monkeypatch, tmp_path, mocker, preset):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', preset)
  selector = SimpleNamespace(_params=params, _param='IQLaneChangeTimer', _options=['nudge', 'no nudge', 'delayed'], set_value=mocker.Mock())
  steering.LaneChangeSelector.refresh(selector)
  selector.set_value.assert_called_once_with('delayed')

  monkeypatch.setattr(steering, 'ui_state', SimpleNamespace(params=params))
  panel = SimpleNamespace(_delay=mocker.Mock())
  steering.LaneChangePanel._refresh_delay(panel)
  panel._delay.set_visible.assert_called_once_with(True)
  panel._delay.set_value.assert_called_once_with(lane_change_delay_label(preset))


@pytest.mark.parametrize('preset', [0, 1])
def test_mici_delay_submenu_hidden_for_other_modes(monkeypatch, tmp_path, mocker, preset):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', preset)
  monkeypatch.setattr(steering, 'ui_state', SimpleNamespace(params=params))
  panel = SimpleNamespace(_delay=mocker.Mock())
  steering.LaneChangePanel._refresh_delay(panel)
  panel._delay.set_visible.assert_called_once_with(False)


@pytest.mark.parametrize('preset', LANE_CHANGE_DELAYS)
def test_mici_delay_choice_is_saved(monkeypatch, tmp_path, mocker, preset):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', DEFAULT_LANE_CHANGE_DELAY)
  monkeypatch.setattr(steering, 'ui_state', SimpleNamespace(params=params))
  panel = SimpleNamespace(dismiss=mocker.Mock())
  steering.LaneChangeDelayPanel._select_delay(panel, preset)
  assert params.get('IQLaneChangeTimer') == preset
  panel.dismiss.assert_called_once()


@pytest.mark.parametrize('result', [DialogResult.CANCEL, DialogResult.CONFIRM])
@pytest.mark.parametrize('preset', LANE_CHANGE_DELAYS)
def test_big_delay_picker_saves_only_confirmed_choice(monkeypatch, tmp_path, preset, result):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', DEFAULT_LANE_CHANGE_DELAY)
  dialogs = []
  monkeypatch.setattr(iq_panels, 'ui_state', SimpleNamespace(params=params))
  monkeypatch.setattr(iq_panels, 'MultiOptionDialog', lambda title, options, current:
                      SimpleNamespace(options=options, current=current, selection=current))
  monkeypatch.setattr(iq_panels, 'gui_app', SimpleNamespace(set_modal_overlay=lambda dialog, callback: dialogs.append((dialog, callback))))
  iq_panels.SteeringLayout._select_lane_change_delay(None)
  dialog, confirm = dialogs[0]
  assert dialog.options == ['0.5 s', '1.0 s', '1.5 s', '2.0 s', '2.5 s', '3.0 s']
  assert dialog.current == '1.0 s'
  dialog.selection = lane_change_delay_label(preset)
  confirm(result)
  assert params.get('IQLaneChangeTimer') == (preset if result == DialogResult.CONFIRM else DEFAULT_LANE_CHANGE_DELAY)


@pytest.mark.parametrize('preset', LANE_CHANGE_DELAYS)
def test_big_reselecting_delayed_preserves_duration(monkeypatch, tmp_path, mocker, preset):
  params = Params(str(tmp_path))
  params.put('IQLaneChangeTimer', preset)
  monkeypatch.setattr(iq_panels, 'ui_state', SimpleNamespace(params=params))
  overlay = mocker.Mock()
  monkeypatch.setattr(iq_panels, 'gui_app', SimpleNamespace(set_modal_overlay=overlay))
  iq_panels.SteeringLayout._set_lane_change_mode(None, DELAYED_MODE)
  assert params.get('IQLaneChangeTimer') == preset
  overlay.assert_not_called()


@pytest.fixture
def headless_lane_change_ui(monkeypatch):
  params = Params()
  state = SimpleNamespace(params=params, CP=None, is_offroad=lambda: True)
  monkeypatch.setattr(steering, 'ui_state', state)
  monkeypatch.setattr(iq_panels, 'ui_state', state)
  monkeypatch.setattr(steering.gui_app, 'font', lambda weight: steering.rl.Font())
  monkeypatch.setattr(steering.gui_app, 'texture', lambda path, width=None, height=None, **kwargs:
                      SimpleNamespace(width=width or 1, height=height or 1))
  return params


def test_mici_menu_exposes_delay_choices_only_in_delayed_mode(headless_lane_change_ui, monkeypatch, mocker):
  params = headless_lane_change_ui
  panel = steering.LaneChangePanel()
  opened = []
  monkeypatch.setattr(steering.gui_app, 'push_widget', opened.append)
  monkeypatch.setattr(panel._delay_panel, 'dismiss', mocker.Mock())
  for preset in (0, 1, *LANE_CHANGE_DELAYS):
    params.put('IQLaneChangeTimer', preset)
    panel._timer.refresh()
    panel._refresh_delay()
    assert panel._delay.is_visible == (preset in LANE_CHANGE_DELAYS)
  panel._delay._click_callback()
  assert opened == [panel._delay_panel]
  for preset, button in panel._delay_panel._choices.items():
    button._click_callback()
    panel._refresh_delay()
    assert params.get('IQLaneChangeTimer') == preset
    assert panel._delay.value == lane_change_delay_label(preset)


def test_big_menu_exposes_delay_picker_only_in_delayed_mode(headless_lane_change_ui, monkeypatch):
  params = headless_lane_change_ui
  panel = iq_panels.SteeringLayout()
  opened = []
  monkeypatch.setattr(iq_panels.gui_app, 'set_modal_overlay', lambda dialog, callback: opened.append((dialog, callback)))
  for preset in (0, 1, *LANE_CHANGE_DELAYS):
    params.put('IQLaneChangeTimer', preset)
    assert panel._lane_change_delay.is_visible == (preset in LANE_CHANGE_DELAYS)
  panel._lane_change_delay.callback()
  dialog, confirm = opened[0]
  assert dialog.options == ['0.5 s', '1.0 s', '1.5 s', '2.0 s', '2.5 s', '3.0 s']
  dialog.selection = '1.5 s'
  confirm(DialogResult.CONFIRM)
  assert params.get('IQLaneChangeTimer') == 6
  assert panel._lane_change_delay.action_item.text == '1.5 s'
