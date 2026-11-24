from unittest.mock import Mock

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant


class TestLongitudinalPlannerCoast(OpenpilotTestCase):
  def test_final_neutral_disallows_brake(self):
    plant = Plant(speed=20.0, enabled=True)
    plant.planner.downhill_coast.update = Mock(return_value=True)
    plant.step(v_cruise=20.0)

    assert plant.planner.output_a_target == 0.0
    assert not plant.planner.output_should_stop
    assert not plant.planner.allow_brake

  def test_negative_e2e_restores_braking(self):
    plant = Plant(speed=20.0, enabled=True)
    plant.acceleration = -2.0
    plant.planner.downhill_coast.update = Mock(return_value=True)
    plant.step(v_cruise=20.0)

    assert plant.planner.output_a_target < 0.0
    assert plant.planner.allow_brake

  def test_should_stop_restores_braking(self):
    plant = Plant(speed=20.0, enabled=True, e2e_should_stop=True)
    plant.planner.downhill_coast.update = Mock(return_value=True)
    plant.step(v_cruise=20.0)

    assert plant.planner.output_should_stop
    assert plant.planner.allow_brake

  def test_accel_boost_receives_normal_cruise_accel(self):
    plant = Plant(speed=20.0, enabled=True)
    plant.planner.downhill_coast.update = Mock(return_value=True)
    plant.planner.accel_boost.update = Mock(wraps=plant.planner.accel_boost.update)
    plant.step(v_cruise=19.9)

    assert plant.planner.accel_boost.update.call_args.args[3] < 0.0

  def test_default_disabled_keeps_brake_allowed(self):
    plant = Plant(speed=20.0, enabled=True)
    plant.step(v_cruise=20.0)
    assert plant.planner.allow_brake
