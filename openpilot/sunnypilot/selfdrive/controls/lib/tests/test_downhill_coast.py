"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

from openpilot.cereal import log, messaging
from openpilot.common.constants import CV
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.downhill_coast import DownhillCoast, MAX_ALLOWANCE_MPH

DEFAULT_PITCH = math.radians(-2.5)


class FakeParams:
  def __init__(self, enabled=True, allowance=5):
    self.enabled = enabled
    self.allowance = allowance

  def get_bool(self, key):
    assert key == "DownhillCoastEnabled"
    return self.enabled

  def get(self, key, return_default=False):
    assert key == "DownhillCoastSpeedAllowance"
    return self.allowance


class FakeSM(dict):
  def __init__(self, pitch=DEFAULT_PITCH, v_ego=19.0):
    super().__init__()
    services = ('carControl', 'deviceMotion', 'extrinsicsCalibration')
    self.seen = dict.fromkeys(services, True)
    self.alive = dict.fromkeys(services, True)
    self.freq_ok = dict.fromkeys(services, True)
    self.valid = dict.fromkeys(services, True)

    car_control = messaging.new_message('carControl')
    car_control.carControl.orientationNED = [0.0, pitch, 0.0]
    car_state = messaging.new_message('carState')
    car_state.carState.vEgo = v_ego
    selfdrive_state = messaging.new_message('selfdriveState')
    selfdrive_state.selfdriveState.enabled = True
    device_motion = messaging.new_message('deviceMotion')
    device_motion.deviceMotion.orientationNED.valid = True
    device_motion.deviceMotion.orientationNED.yStd = math.radians(0.1)
    device_motion.deviceMotion.inputsOK = True
    device_motion.deviceMotion.sensorsOK = True
    device_motion.deviceMotion.posenetOK = True
    calibration = messaging.new_message('extrinsicsCalibration')
    calibration.extrinsicsCalibration.calStatus = log.ExtrinsicsCalibration.Status.calibrated
    calibration.extrinsicsCalibration.calPerc = 100
    calibration.extrinsicsCalibration.rpyCalib = [0.0, 0.0, 0.0]
    calibration.extrinsicsCalibration.rpyCalibSpread = [0.0, 0.0, 0.0]

    self.update({
      'carControl': car_control.carControl,
      'carState': car_state.carState,
      'selfdriveState': selfdrive_state.selfdriveState,
      'deviceMotion': device_motion.deviceMotion,
      'extrinsicsCalibration': calibration.extrinsicsCalibration,
    })

  def set_pitch(self, pitch):
    self['carControl'].orientationNED = [0.0, pitch, 0.0]


class TestDownhillCoast(OpenpilotTestCase):
  def make_coast(self, enabled=True, allowance=5):
    params = FakeParams(enabled, allowance)
    return DownhillCoast(True, params), params

  def update(self, coast, sm, v_ego, v_set=20.0, normal_a=-0.1, driver_source=True, engaged=True):
    sm['carState'].vEgo = v_ego
    sm['selfdriveState'].enabled = engaged
    return coast.update(sm, v_set, normal_a, driver_source, False, False, True)

  def qualify(self, coast, sm, frames=10):
    for _ in range(frames):
      self.update(coast, sm, 19.0, normal_a=0.5)

  def test_prequalifies_before_first_hold_frame(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    for _ in range(9):
      self.update(coast, sm, 19.0, normal_a=0.5)
    assert not coast.grade_ready

    self.update(coast, sm, 19.0, normal_a=0.5)
    assert coast.grade_ready
    assert not coast.coast_latched

    assert self.update(coast, sm, 20.0, normal_a=0.0)
    assert coast.coast_latched

  def test_short_pitch_spike_does_not_qualify(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    self.update(coast, sm, 19.0, normal_a=0.5)
    self.update(coast, sm, 19.0, normal_a=0.5)
    sm.set_pitch(0.0)
    for _ in range(20):
      self.update(coast, sm, 19.0, normal_a=0.5)
    assert not coast.grade_ready
    assert not self.update(coast, sm, 20.0)

  def test_latch_survives_valid_grade_ending(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    self.qualify(coast, sm)
    assert self.update(coast, sm, 20.0, normal_a=0.0)

    sm.set_pitch(math.radians(1.0))
    for _ in range(20):
      assert self.update(coast, sm, 20.5)
    assert not coast.grade_ready
    assert coast.coast_latched

  def test_cap_braking_and_reentry_preserve_latch(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    self.qualify(coast, sm)
    assert self.update(coast, sm, 20.0, normal_a=0.0)
    cap = 20.0 + 5 * CV.MPH_TO_MS

    assert not self.update(coast, sm, cap + 0.01)
    assert coast.coast_latched and coast.cap_braking
    assert not self.update(coast, sm, cap - 0.1)
    assert self.update(coast, sm, cap - 0.31)
    assert coast.coast_latched and not coast.cap_braking

  def test_dynamic_set_speed(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    self.qualify(coast, sm)
    assert self.update(coast, sm, 20.0, normal_a=0.0)

    assert self.update(coast, sm, 21.0, v_set=20.5)
    assert not self.update(coast, sm, 22.0, v_set=19.0)
    assert coast.coast_latched and coast.cap_braking
    assert self.update(coast, sm, 20.8, v_set=19.0)
    assert not self.update(coast, sm, 20.0, v_set=20.5)
    assert not coast.coast_latched

  def test_non_cruise_source_overrides_without_clearing_latch(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    self.qualify(coast, sm)
    assert self.update(coast, sm, 20.0, normal_a=0.0)
    assert not self.update(coast, sm, 20.5, driver_source=False)
    assert coast.coast_latched
    assert self.update(coast, sm, 20.5, driver_source=True)

  def test_invalid_quality_fails_closed_and_requires_new_dwell(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    self.qualify(coast, sm)
    assert self.update(coast, sm, 20.0, normal_a=0.0)

    sm['deviceMotion'].inputsOK = False
    assert not self.update(coast, sm, 20.5)
    assert not coast.coast_latched and not coast.grade_ready

    sm['deviceMotion'].inputsOK = True
    for _ in range(9):
      assert not self.update(coast, sm, 20.5)
    sm.set_pitch(math.radians(-2.5))
    assert self.update(coast, sm, 20.5)

  def test_grade_quality_gates(self):
    cases = ('sensors', 'posenet', 'orientation', 'uncertainty', 'calibration', 'calibration_percent', 'spread', 'message_valid')
    for case in cases:
      with self.subTest(case=case):
        coast, _ = self.make_coast()
        sm = FakeSM()
        if case == 'sensors':
          sm['deviceMotion'].sensorsOK = False
        elif case == 'posenet':
          sm['deviceMotion'].posenetOK = False
        elif case == 'orientation':
          sm['deviceMotion'].orientationNED.valid = False
        elif case == 'uncertainty':
          sm['deviceMotion'].orientationNED.yStd = math.radians(2.0)
        elif case == 'calibration':
          sm['extrinsicsCalibration'].calStatus = log.ExtrinsicsCalibration.Status.uncalibrated
        elif case == 'calibration_percent':
          sm['extrinsicsCalibration'].calPerc = 99
        elif case == 'spread':
          sm['extrinsicsCalibration'].rpyCalibSpread = [0.0, math.radians(5.0), 0.0]
        elif case == 'message_valid':
          sm.valid['deviceMotion'] = False

        valid, _ = coast._grade_sample(sm)
        assert not valid

  def test_calibrated_pitch_is_used_instead_of_raw_device_pitch(self):
    coast, _ = self.make_coast()
    sm = FakeSM()
    sm['deviceMotion'].orientationNED.y = math.radians(3.0)
    valid, pitch = coast._grade_sample(sm)
    assert valid
    assert math.isclose(pitch, DEFAULT_PITCH, abs_tol=1e-7)

  def test_zero_allowance_and_flat_grade_never_enter(self):
    coast, _ = self.make_coast(allowance=0)
    sm = FakeSM()
    for _ in range(20):
      assert not self.update(coast, sm, 20.0)

    coast, _ = self.make_coast()
    sm = FakeSM(pitch=0.0)
    for _ in range(20):
      assert not self.update(coast, sm, 20.0)

  def test_allowance_is_clamped_to_15_mph(self):
    coast, _ = self.make_coast(allowance=100)
    sm = FakeSM()
    self.update(coast, sm, 19.0, normal_a=0.5)
    assert coast.allowance == MAX_ALLOWANCE_MPH * CV.MPH_TO_MS

  def test_engaged_settings_are_snapshotted(self):
    coast, params = self.make_coast()
    sm = FakeSM()
    self.qualify(coast, sm)
    params.enabled = False
    params.allowance = 0
    assert self.update(coast, sm, 20.0, normal_a=0.0)

    assert not self.update(coast, sm, 20.0, engaged=False)
    assert not coast.enabled and not coast.coast_latched
