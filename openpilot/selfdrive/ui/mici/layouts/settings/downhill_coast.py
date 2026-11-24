from collections.abc import Callable

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.layouts.settings.initial_cruise_speed import InitialCruiseSpeedPicker, SpeedScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr

MIN_SPEED_ALLOWANCE_MPH = 0
MAX_SPEED_ALLOWANCE_MPH = 15


def _speed_conversion() -> float:
  return CV.MPH_TO_KPH if ui_state.is_metric else 1


def get_downhill_coast_speed_options() -> tuple[list[int], int]:
  """Speed allowances in display units, and the index of the saved canonical mph value."""
  options = [round(speed * _speed_conversion()) for speed in range(MIN_SPEED_ALLOWANCE_MPH, MAX_SPEED_ALLOWANCE_MPH + 1)]
  saved_speed = int(ui_state.params.get("DownhillCoastSpeedAllowance", return_default=True))
  selected_idx = max(MIN_SPEED_ALLOWANCE_MPH, min(MAX_SPEED_ALLOWANCE_MPH, saved_speed))
  return options, selected_idx


def set_downhill_coast_speed_allowance(speed: int) -> None:
  """Save a display-unit speed allowance as canonical mph."""
  if ui_state.engaged:
    return
  speed_mph = round(speed / _speed_conversion())
  speed_mph = max(MIN_SPEED_ALLOWANCE_MPH, min(MAX_SPEED_ALLOWANCE_MPH, speed_mph))
  ui_state.params.put("DownhillCoastSpeedAllowance", speed_mph, block=True)


class DownhillCoastSpeedPicker(InitialCruiseSpeedPicker):
  def __init__(self, change_callback: Callable[[], None]):
    speed_scroller = SpeedScroller(change_callback, get_downhill_coast_speed_options, set_downhill_coast_speed_allowance)
    super().__init__(change_callback, speed_scroller)
    self._speed_scroller.set_enabled(lambda: self.enabled and not self.is_dismissing and not ui_state.engaged)


class DownhillCoastSpeedButton(BigButton):
  def __init__(self):
    super().__init__("downhill coast allowance", " ", description="Maximum speed above the set speed allowed while coasting downhill.")
    self._picker = DownhillCoastSpeedPicker(self.refresh)
    self.set_click_callback(lambda: gui_app.push_widget(self._picker))
    self.set_enabled(lambda: not ui_state.engaged)
    self.refresh()

  def _get_label_font_size(self):
    return 42

  def refresh(self):
    options, idx = get_downhill_coast_speed_options()
    self.set_value(f"{options[idx]} " + (tr("kph") if ui_state.is_metric else tr("mph")))
