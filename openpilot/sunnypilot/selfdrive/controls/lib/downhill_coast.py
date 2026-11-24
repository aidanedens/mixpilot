"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import time

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from opendbc.car.interfaces import MAX_CTRL_SPEED

MAX_ALLOWANCE_MPH = 15

# Grade qualification runs during the approach to the set speed. These are
# intentionally conservative initial values and are not user-adjustable.
PITCH_FILTER_RC = 0.5
GRADE_ENTER_PITCH = math.radians(-2.0)
GRADE_CLEAR_PITCH = math.radians(-1.0)
GRADE_ENTER_DWELL = 0.5
GRADE_CLEAR_DWELL = 0.5
GRADE_ENTER_FRAMES = round(GRADE_ENTER_DWELL / DT_MDL)
GRADE_CLEAR_FRAMES = round(GRADE_CLEAR_DWELL / DT_MDL)
MAX_PITCH_STD = math.radians(1.0)
MAX_CALIBRATION_SPREAD = math.radians(4.0)
APPROACH_WINDOW = 3.0  # m/s below the driver set speed

SET_EXIT_HYSTERESIS = 0.1  # m/s
CAP_REENTRY_HYSTERESIS = 0.3  # m/s
HOLD_ACCEL_THRESHOLD = 0.0  # m/s^2


class DownhillCoast:
  def __init__(self, openpilot_longitudinal: bool, params: Params | None = None):
    self.openpilot_longitudinal = openpilot_longitudinal
    self.params = params if params is not None else Params()

    self.enabled = False
    self.allowance = 0.0
    self._params_initialized = False
    self._param_update_time = 0.0
    self._engaged_last = False

    self.pitch_filter = FirstOrderFilter(0.0, PITCH_FILTER_RC, DT_MDL, initialized=False)
    self.grade_ready = False
    self.coast_latched = False
    self.cap_braking = False
    self.cap = 0.0
    self._grade_enter_frames = 0
    self._grade_clear_frames = 0

  @staticmethod
  def _service_ok(sm, service: str) -> bool:
    try:
      return bool(sm.seen[service] and sm.alive[service] and sm.freq_ok[service] and sm.valid[service])
    except (AttributeError, KeyError):
      return False

  @classmethod
  def _grade_sample(cls, sm) -> tuple[bool, float]:
    services = ('carControl', 'deviceMotion', 'extrinsicsCalibration')
    if not all(cls._service_ok(sm, service) for service in services):
      return False, 0.0

    dm = sm['deviceMotion']
    if not (dm.orientationNED.valid and dm.inputsOK and dm.sensorsOK and dm.posenetOK):
      return False, 0.0
    if not math.isfinite(dm.orientationNED.yStd) or dm.orientationNED.yStd >= MAX_PITCH_STD:
      return False, 0.0

    cal = sm['extrinsicsCalibration']
    if cal.calStatus != log.ExtrinsicsCalibration.Status.calibrated or cal.calPerc < 100:
      return False, 0.0
    if len(cal.rpyCalib) != 3 or len(cal.rpyCalibSpread) != 3:
      return False, 0.0
    if not all(math.isfinite(v) for v in (*cal.rpyCalib, *cal.rpyCalibSpread)):
      return False, 0.0
    if cal.rpyCalibSpread[1] > MAX_CALIBRATION_SPREAD:
      return False, 0.0

    orientation = sm['carControl'].orientationNED
    if len(orientation) != 3 or not all(math.isfinite(v) for v in orientation):
      return False, 0.0

    return True, float(orientation[1])

  def _reset_grade(self) -> None:
    self.pitch_filter.initialized = False
    self.pitch_filter.x = 0.0
    self.grade_ready = False
    self._grade_enter_frames = 0
    self._grade_clear_frames = 0

  def _reset(self) -> None:
    self.coast_latched = False
    self.cap_braking = False
    self._reset_grade()

  def update_params(self, engaged: bool) -> None:
    now = time.monotonic()
    engagement_changed = engaged != self._engaged_last
    refresh = not self._params_initialized or engagement_changed or (not engaged and now - self._param_update_time > PARAMS_UPDATE_PERIOD)
    if refresh:
      self.enabled = self.params.get_bool("DownhillCoastEnabled")
      allowance_mph = int(self.params.get("DownhillCoastSpeedAllowance", return_default=True))
      allowance_mph = max(0, min(MAX_ALLOWANCE_MPH, allowance_mph))
      self.allowance = allowance_mph * CV.MPH_TO_MS
      self._param_update_time = now
      self._params_initialized = True

    self._engaged_last = engaged

  def _update_grade(self, sm, v_ego: float, v_set: float) -> None:
    valid, pitch = self._grade_sample(sm)
    if not valid:
      self._reset()
      return

    pitch = self.pitch_filter.update(pitch)
    in_approach_window = v_set - APPROACH_WINDOW <= v_ego <= self.cap
    if not in_approach_window:
      self.grade_ready = False
      self._grade_enter_frames = 0
      self._grade_clear_frames = 0
      return

    if pitch <= GRADE_ENTER_PITCH:
      self._grade_enter_frames += 1
      self._grade_clear_frames = 0
      if self._grade_enter_frames >= GRADE_ENTER_FRAMES:
        self.grade_ready = True
    elif pitch >= GRADE_CLEAR_PITCH:
      self._grade_clear_frames += 1
      self._grade_enter_frames = 0
      if self._grade_clear_frames >= GRADE_CLEAR_FRAMES:
        self.grade_ready = False
    else:
      self._grade_enter_frames = 0
      self._grade_clear_frames = 0

  def update(self, sm, driver_v_cruise: float, normal_a_cruise: float, driver_source: bool,
             reset_state: bool, force_decel: bool, cruise_initialized: bool) -> bool:
    engaged = bool(sm['selfdriveState'].enabled)
    self.update_params(engaged)

    v_ego = float(sm['carState'].vEgo)
    self.cap = min(driver_v_cruise + self.allowance, MAX_CTRL_SPEED)

    CS = sm['carState']
    unavailable = (not self.enabled or not self.openpilot_longitudinal or self.allowance <= 0.0 or
                   not engaged or reset_state or force_decel or not cruise_initialized or
                   not math.isfinite(driver_v_cruise) or not math.isfinite(v_ego) or not math.isfinite(normal_a_cruise) or
                   CS.gasPressed or CS.brakePressed or CS.standstill)
    if unavailable:
      self._reset()
      return False

    self._update_grade(sm, v_ego, driver_v_cruise)

    # A valid observation that the descent ended does not clear the latch. Only
    # the current set speed, disengagement, or invalid grade quality can do so.
    if v_ego <= driver_v_cruise - SET_EXIT_HYSTERESIS:
      self.coast_latched = False
      self.cap_braking = False

    if (not self.coast_latched and self.grade_ready and driver_source and
        normal_a_cruise <= HOLD_ACCEL_THRESHOLD and driver_v_cruise <= v_ego < self.cap):
      self.coast_latched = True
      self.cap_braking = False

    if self.coast_latched and v_ego >= self.cap:
      self.cap_braking = True
    elif self.cap_braking and v_ego <= self.cap - CAP_REENTRY_HYSTERESIS:
      self.cap_braking = False

    return bool(self.coast_latched and not self.cap_braking and driver_source and
                driver_v_cruise <= v_ego < self.cap)
