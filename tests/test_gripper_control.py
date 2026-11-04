"""Exercise blocked grasps and latched alarms without a hardware connection."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from gripper_control import GripperController, GripperMotionError, release_gripper


class Clock:
    def __init__(self):
        self.now = 0.0
        self.advance = lambda _: None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.advance(seconds)


class Bus:
    def __init__(self, position=70, obstruction=57.4):
        self.position = self.goal = position
        self.obstruction = obstruction
        self.limit = 200
        self.load = 0
        self.commands = []
        self.fault = False
        self.block_open = False
        self.torque = 1
        self.communication = 0
        self.motors = {'gripper': SimpleNamespace(id=6, model='sts3215')}
        self.model_ctrl_table = {'sts3215': {'Torque_Enable': (40, 1)}}

    def advance(self, seconds):
        destination = self.goal
        if self.goal < self.position:
            destination = max(destination, self.obstruction)
        elif self.block_open:
            destination = self.position
        change = max(-25 * seconds, min(25 * seconds, destination - self.position))
        self.position += change
        self.load = min(self.limit, abs(self.goal - self.position) * 100)

    def read(self, register, motor, **kwargs):
        if self.fault:
            raise RuntimeError('[RxPacketError] Overload error!')
        return {'Present_Position': self.position, 'Present_Load': self.load,
                'Torque_Limit': self.limit, 'Torque_Enable': self.torque}[register]

    def write(self, register, motor, value):
        self.commands.append((register, value))
        if register == 'Goal_Position':
            self.goal = value
        elif register == 'Torque_Limit':
            self.limit = value

    def disable_torque(self, motor):
        if self.fault:
            raise RuntimeError('[RxPacketError] Overload error!')
        self.torque = 0

    def sync_write(self, register, values, **kwargs):
        self.commands.append((register, values))
        self.torque = values['gripper']

    def _read(self, address, length, motor_id, **kwargs):
        return self.torque, self.communication, 32 if self.fault else 0


class GripperTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.bus = Bus()
        self.clock.advance = self.bus.advance
        self.controller = GripperController(self.bus)
        self.trial = {'gripper': []}
        patcher = patch('gripper_control.time', self.clock)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_obstructed_close_replaces_unreachable_target_before_lifting(self):
        self.controller.move(35, 'close', self.trial, closing=True)
        record = self.trial['gripper'][0]
        self.assertEqual(record['status'], 'contact_detected')
        self.assertAlmostEqual(self.bus.goal, 56.9)
        self.assertEqual(self.bus.limit, 100)
        self.assertLess(self.clock.now, 1.5)
        # The reduced hold remains healthy over the previously failing lift/lower time.
        phase = {}
        for _ in range(70):
            self.clock.sleep(0.05)
            self.controller.monitor(phase)
        self.assertGreater(len(phase['gripper_samples']), 30)

    def test_open_waits_until_target_instead_of_fixed_half_second(self):
        self.bus.position = 37.4
        self.controller.move(70, 'open', self.trial)
        self.assertGreater(self.clock.now, 1.2)
        self.assertGreaterEqual(self.bus.position, 69)

    def test_release_after_contact_reopens_with_capped_movement_effort(self):
        self.controller.move(35, 'close', self.trial, closing=True)
        self.assertEqual(self.bus.limit, 100)
        self.controller.move(70, 'release_object', self.trial)
        self.assertGreaterEqual(self.bus.position, 69)
        self.assertEqual(self.bus.limit, 200)
        self.assertIsNone(self.controller.hold_target)

    def test_blocked_open_aborts_and_replaces_goal(self):
        self.bus.position = 50
        self.bus.block_open = True
        with self.assertRaisesRegex(GripperMotionError, 'blocked/stalled'):
            self.controller.move(70, 'open', self.trial)
        self.assertEqual(self.bus.goal, 50)
        self.assertEqual(self.trial['gripper'][0]['status'], 'failed')

    def test_unloaded_stall_is_not_claimed_as_object_contact(self):
        self.clock.advance = lambda _: None
        with self.assertRaisesRegex(GripperMotionError, 'blocked/stalled'):
            self.controller.move(35, 'close', self.trial, closing=True)

    def test_latched_fault_does_not_send_new_position_to_clear_alarm(self):
        self.bus.fault = True
        with self.assertRaisesRegex(RuntimeError, 'Overload'):
            self.controller.move(70, 'release_object', self.trial)
        self.assertEqual(self.bus.commands, [])

    def test_monitor_aborts_sustained_holding_effort(self):
        self.controller.hold_target = 57
        self.bus.position = self.bus.goal = 57
        self.clock.advance = lambda _: None
        self.bus.load = 100
        with self.assertRaisesRegex(GripperMotionError, 'holding load/error'):
            for _ in range(12):
                self.clock.sleep(0.05)
                self.controller.monitor({})

    def test_faulted_shutdown_confirms_torque_off_but_preserves_alarm(self):
        self.bus.fault = True
        result = release_gripper(self.bus)
        self.assertTrue(result['confirmed'])
        self.assertEqual(result['fault_flags'], 32)
        self.assertEqual(self.bus.commands, [('Torque_Enable', {'gripper': 0})])
        self.assertTrue(self.bus.fault)

    def test_shutdown_does_not_claim_success_on_communication_error(self):
        self.bus.fault = True
        self.bus.communication = -1
        with self.assertRaisesRegex(RuntimeError, 'Cannot confirm'):
            release_gripper(self.bus)

    def test_pick_does_not_approach_when_gripper_cannot_open(self):
        from pick_primitive import run_trial
        self.bus.position = 50
        self.bus.block_open = True
        with patch('pick_primitive.move_to') as move:
            with self.assertRaises(GripperMotionError):
                run_trial(self.bus, {}, {}, {}, {}, self.trial, self.controller)
        move.assert_not_called()

    def test_monitor_fault_prevents_next_arm_command(self):
        import numpy as np
        from experiments.test_trajectory_tracking import execute
        bus = Mock()
        record = {'samples': []}
        trajectory = {'time_s': np.array([0.0, 0.02]), 'position_rad': np.zeros((2, 5))}
        hook = Mock(side_effect=GripperMotionError('holding load/error'))
        with self.assertRaisesRegex(GripperMotionError, 'holding load/error'):
            execute(bus, trajectory, record, settle_time=0, sample_hook=hook)
        bus.sync_write.assert_not_called()

    def test_cleanup_does_not_raise_effort_when_alarm_remains(self):
        from pick_primitive import cleanup_motors
        bus = Mock()
        bus.is_connected = True
        registers = {'Lock': 1, 'P_Coefficient': 16}
        bus.write.side_effect = lambda register, motor, value: registers.update({register: value})
        bus.read.side_effect = lambda register, motor: registers[register]
        session = {'motors_enabled': True, 'original_settings': {
            'elbow_p': 16, 'elbow_lock': 1, 'gripper_limit': 500}}
        log = {}
        with patch('builtins.input', return_value=''), patch('pick_primitive.release_gripper',
                return_value={'confirmed': True, 'fault_flags': 32}):
            cleanup_motors(bus, session, log)
        self.assertIn('gripper_limit_restore_skipped', log)
        self.assertFalse(any(call.args[0] == 'Torque_Limit' for call in bus.write.call_args_list))
        bus.disconnect.assert_called_once_with(disable_torque=False)


if __name__ == '__main__':
    unittest.main()
