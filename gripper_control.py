"""Feetech gripper shutdown verification; no motion or load-feedback controller."""


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
