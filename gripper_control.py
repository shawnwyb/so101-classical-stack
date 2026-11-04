"""Contact-aware position control; load values are heuristics, not measured force."""

import time


class GripperMotionError(RuntimeError):
    """Software-detected blocked motion while servo feedback is still healthy."""


class GripperController:
    PERIOD = 0.05
    POSITION_TOLERANCE = 1.0  # normalized travel percentage points
    CONTACT_TIME = 0.15
    HOLD_BIAS = 0.5          # small closing offset from measured contact

    def __init__(self, bus, effort=200, hold_effort=100):
        self.bus = bus
        self.effort = effort
        self.hold_effort = min(effort, hold_effort)
        self.hold_target = None
        self.last_poll = float('-inf')
        self.overload_since = None

    def read(self):
        return (float(self.bus.read('Present_Position', 'gripper')),
                float(self.bus.read('Present_Load', 'gripper', normalize=False)))

    def cancel_closing(self, position, record):
        """Replace an unreachable target; never clear a fault or open on error."""
        try:
            self.bus.write('Goal_Position', 'gripper', position)
        except Exception as error:
            record['cancel_error'] = str(error)

    def move(self, target, label, trial, *, closing=False):
        record = {'phase': label, 'command_percent': target, 'samples': [], 'status': 'started'}
        trial['gripper'].append(record)
        position = None
        self.hold_target = None
        try:
            position, load = self.read()
            record['before_percent'] = position
            effort = min(self.effort, self.bus.read('Torque_Limit', 'gripper'))
            if effort <= 0:
                raise GripperMotionError('Gripper effort limit must be positive')
            self.bus.write('Goal_Position', 'gripper', target)
            # Opening after a grasp restores only the already capped movement effort.
            if not closing:
                self.bus.write('Torque_Limit', 'gripper', self.effort)
                effort = self.effort
            started = time.monotonic()
            high_load_since = None
            stationary_since = started
            anchor = position
            settled_since = None
            timeout = 1.5 if closing else 3.0
            while True:
                time.sleep(self.PERIOD)
                now = time.monotonic()
                position, load = self.read()
                record['samples'].append({'time_s': now - started,
                                          'position_percent': position, 'load_raw': load})
                at_target = abs(position - target) <= self.POSITION_TOLERANCE
                if at_target:
                    settled_since = now if settled_since is None else settled_since
                    if now - settled_since >= 0.1:
                        record['status'] = 'target_reached'
                        break
                else:
                    settled_since = None
                if abs(position - anchor) >= 0.3:
                    anchor, stationary_since = position, now
                if abs(load) >= 0.7 * effort and not at_target:
                    high_load_since = now if high_load_since is None else high_load_since
                else:
                    high_load_since = None
                loaded = (high_load_since is not None and now - high_load_since >= self.CONTACT_TIME
                          and now - stationary_since >= self.CONTACT_TIME)
                stalled = now - stationary_since >= 0.25 and not at_target
                if loaded or stalled:
                    if not closing or abs(load) < 0.3 * effort:
                        raise GripperMotionError(f'Gripper {label} blocked/stalled at {position:.1f}% '
                                           f'(target {target:.1f}%, load {load:g})')
                    record['status'] = 'contact_detected'
                    break
                if now - started >= timeout:
                    raise GripperMotionError(f'Gripper {label} timed out at {position:.1f}% '
                                       f'(target {target:.1f}%)')
            if closing:
                # Stop driving toward the original, unreachable target during the lift.
                hold_target = max(target, position - self.HOLD_BIAS)
                self.bus.write('Goal_Position', 'gripper', hold_target)
                self.bus.write('Torque_Limit', 'gripper', self.hold_effort)
                self.hold_target = hold_target
                self.last_poll = float('-inf')
                self.overload_since = None
                record['hold_target_percent'] = hold_target
                record['hold_effort'] = self.hold_effort
                time.sleep(0.15)
                position, load = self.read()
                if abs(position - hold_target) > 2.0:
                    raise GripperMotionError('Gripper did not maintain the contact holding position')
            record.update(actual_percent=position, load_raw=load)
            print(f'Gripper {label}: {record["before_percent"]:.1f}% -> {position:.1f}% '
                  f'(target {target:.1f}%; {record["status"]})')
        except BaseException as error:
            record.update(status='failed', error=str(error))
            if position is not None and isinstance(error, (GripperMotionError, KeyboardInterrupt)):
                self.cancel_closing(position, record)
            raise

    def monitor(self, record):
        """Poll during lift/lower; stop the trajectory on fault or sustained load."""
        now = time.monotonic()
        if self.hold_target is None or now - self.last_poll < self.PERIOD:
            return
        self.last_poll = now
        position, load = self.read()  # An error aborts; no automatic retry/motion recovery.
        samples = record.setdefault('gripper_samples', [])
        samples.append({'elapsed_s': 0.0 if not samples else now - record['gripper_monitor_started'],
                        'position_percent': position, 'load_raw': load})
        record.setdefault('gripper_monitor_started', now)
        overloaded = abs(load) >= 0.9 * self.hold_effort or abs(position - self.hold_target) > 2.0
        self.overload_since = (now if self.overload_since is None else self.overload_since) if overloaded else None
        if self.overload_since is not None and now - self.overload_since >= 0.25:
            self.cancel_closing(position, record)
            raise GripperMotionError('Gripper holding load/error persisted; stopping arm motion before overload')


def release_gripper(bus):
    """Torque-off only, including when a latched alarm prevents normal status reads.

    Do not send a new position command to clear an alarm. A successful raw read
    can confirm torque-off while still preserving/reporting the servo error bits.
    This Feetech-specific fallback is restricted to shutdown, never motion.
    """
    acknowledgement_error = None
    try:
        bus.disable_torque('gripper')
        if bus.read('Torque_Enable', 'gripper') != 0:
            raise RuntimeError('Gripper torque-off readback mismatch')
        return {'confirmed': True, 'fault_flags': 0}
    except Exception as error:
        acknowledgement_error = str(error)
    bus.sync_write('Torque_Enable', {'gripper': 0}, normalize=False)
    motor = bus.motors['gripper']
    address, length = bus.model_ctrl_table[motor.model]['Torque_Enable']
    value, communication, flags = bus._read(address, length, motor.id, raise_on_error=False)
    if communication != 0 or value != 0:
        raise RuntimeError(f'Cannot confirm gripper torque-off: comm={communication}, '
                           f'value={value}, fault_flags={flags}; {acknowledgement_error}')
    return {'confirmed': True, 'fault_flags': flags,
            'acknowledgement_error': acknowledgement_error}
