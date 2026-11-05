"""Position commands leave effort/protection settings alone; verify shutdown."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from gripper_control import release_gripper


class GripperTests(unittest.TestCase):
    def faulted_bus(self):
        bus = Mock()
        bus.disable_torque.side_effect = RuntimeError('Overload error')
        bus.motors = {'gripper': SimpleNamespace(id=6, model='sts3215')}
        bus.model_ctrl_table = {'sts3215': {'Torque_Enable': (40, 1)}}
        bus._read.return_value = (0, 0, 32)
        return bus

    def test_normal_shutdown_verifies_torque_off(self):
        bus = Mock()
        bus.read.return_value = 0
        self.assertEqual(release_gripper(bus), {'confirmed': True, 'fault_flags': 0})
        bus.sync_write.assert_not_called()

    def test_faulted_shutdown_only_disables_torque_and_preserves_alarm(self):
        bus = self.faulted_bus()
        result = release_gripper(bus)
        self.assertTrue(result['confirmed'])
        self.assertEqual(result['fault_flags'], 32)
        bus.sync_write.assert_called_once_with('Torque_Enable', {'gripper': 0}, normalize=False)
        bus._read.assert_called_once_with(40, 1, 6, raise_on_error=False)
        bus.write.assert_not_called()

    def test_shutdown_rejects_communication_failure(self):
        bus = self.faulted_bus()
        bus._read.return_value = (0, -1, 32)
        with self.assertRaisesRegex(RuntimeError, 'Cannot confirm'):
            release_gripper(bus)

    def test_shutdown_rejects_torque_still_enabled(self):
        bus = self.faulted_bus()
        bus._read.return_value = (1, 0, 32)
        with self.assertRaisesRegex(RuntimeError, 'Cannot confirm'):
            release_gripper(bus)

    def test_gripper_command_only_writes_position(self):
        from pick_primitive import set_gripper
        bus = Mock()
        bus.read.side_effect = [70, 57]
        trial = {'gripper': []}
        with patch('pick_primitive.time.sleep'):
            set_gripper(bus, 35, 'close', trial)
        bus.write.assert_called_once_with('Goal_Position', 'gripper', 35)
        self.assertEqual(trial['gripper'][0]['actual_percent'], 57)

    def test_full_sequence_keeps_lower_release_and_return(self):
        from pick_primitive import run_trial
        order = []
        with patch('pick_primitive.set_gripper', side_effect=lambda b, p, label, t: order.append(label)) as grip, \
                patch('pick_primitive.move_to', side_effect=lambda b, q, label, d, t: order.append(label)), \
                patch('builtins.input', return_value=''):
            run_trial(Mock(), {}, {}, {}, {}, {'success': None}, 37.506)
        self.assertEqual(order, ['open', 'approach', 'descend', 'close', 'lift', 'lower',
                                 'release_object', 'retreat', 'return_start'])
        self.assertEqual(grip.call_args_list[1].args[1], 37.506)

    def test_setup_does_not_override_gripper_limits_or_protection(self):
        from pick_primitive import ALL_MOTORS, prepare_motors
        bus = Mock()
        bus.is_calibrated = True
        registers = {'Operating_Mode': 0, 'P_Coefficient': 16, 'Lock': 1}
        bus.read.side_effect = lambda register, motor: registers[register]
        bus.write.side_effect = lambda register, motor, value: registers.update({register: value})
        bus.sync_read.side_effect = lambda register: dict.fromkeys(ALL_MOTORS, 0)
        session = {'motors_enabled': False, 'original_settings': {}}
        with patch('builtins.input', return_value=''), patch('pick_primitive.build_trajectory'):
            prepare_motors(bus, {}, session, {})
        self.assertEqual({call.args[0] for call in bus.write.call_args_list}, {'Lock', 'P_Coefficient'})
        self.assertNotIn('gripper_limit', session['original_settings'])

    def test_cleanup_does_not_write_gripper_torque_limits(self):
        from pick_primitive import cleanup_motors
        bus = Mock()
        bus.is_connected = True
        registers = {'Lock': 1, 'P_Coefficient': 16}
        bus.write.side_effect = lambda register, motor, value: registers.update({register: value})
        bus.read.side_effect = lambda register, motor: registers[register]
        session = {'motors_enabled': True, 'original_settings': {'elbow_p': 16, 'elbow_lock': 1}}
        with patch('builtins.input', return_value=''), patch('pick_primitive.release_gripper',
                return_value={'confirmed': True, 'fault_flags': 32}):
            cleanup_motors(bus, session, {})
        self.assertEqual({call.args[0] for call in bus.write.call_args_list}, {'Lock', 'P_Coefficient'})
        bus.disconnect.assert_called_once_with(disable_torque=False)


if __name__ == '__main__':
    unittest.main()
