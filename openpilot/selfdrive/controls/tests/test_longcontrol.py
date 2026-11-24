from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState, long_control_state_trans


class TestLongControlStateTransition(OpenpilotTestCase):

  def test_stay_stopped(self):
    CP_SP = custom.CarParamsSP.new_message()
    active = True
    current_state = LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid
    active = False
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.off

  def test_engage(self):
    CP_SP = custom.CarParamsSP.new_message()
    active = True
    current_state = LongCtrlState.off
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=True, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=True, cruise_standstill=False)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=True)
    assert next_state == LongCtrlState.stopping
    next_state = long_control_state_trans(CP_SP, active, current_state,
                             should_stop=False, brake_pressed=False, cruise_standstill=False)
    assert next_state == LongCtrlState.pid


class TestLongControlCoast(OpenpilotTestCase):
  def make_control(self):
    CP = car.CarParams.new_message()
    CP.longitudinalTuning.kiBP = [0.0]
    CP.longitudinalTuning.kiV = [0.1]
    CP.stopAccel = -0.5
    CP_SP = custom.CarParamsSP.new_message()
    control = LongControl(CP, CP_SP)
    CS = car.CarState.new_message()
    CS.vEgo = 20.0
    CS.cruiseState.standstill = False
    return control, CS

  def test_neutral_output_and_pid_reset(self):
    for a_ego in (-0.5, 0.5):
      with self.subTest(a_ego=a_ego):
        control, CS = self.make_control()
        CS.aEgo = a_ego
        control.long_control_state = LongCtrlState.pid
        control.pid.i = 1.0 if a_ego > 0 else -1.0
        output = control.update(True, CS, 0.0, False, [-3.5, 2.0], coast_active=True)
        assert output == 0.0
        assert control.last_output_accel == 0.0
        assert control.pid.i == 0.0

  def test_stopping_has_priority_over_coast(self):
    control, CS = self.make_control()
    control.long_control_state = LongCtrlState.pid
    output = control.update(True, CS, 0.0, True, [-3.5, 2.0], coast_active=True)
    assert control.long_control_state == LongCtrlState.stopping
    assert output < 0.0

  def test_normal_pid_resumes_after_coast(self):
    control, CS = self.make_control()
    CS.aEgo = 0.5
    control.long_control_state = LongCtrlState.pid
    control.pid.i = -1.0
    assert control.update(True, CS, 0.0, False, [-3.5, 2.0], coast_active=True) == 0.0
    assert control.update(True, CS, -0.5, False, [-3.5, 2.0], coast_active=False) < 0.0
    assert control.pid.i != -1.0
