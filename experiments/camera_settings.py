"""Linux V4L2 image controls shared by camera experiments (no extra dependency)."""

import errno
import fcntl
import json
from pathlib import Path
import struct


DEFAULT_CONFIG = Path(__file__).with_name('camera_settings.json')
CONTROL_IDS = {
    'brightness': 0x00980900,
    'contrast': 0x00980901,
    'saturation': 0x00980902,
    'hue': 0x00980903,
    'auto_white_balance': 0x0098090C,
    'gamma': 0x00980910,
    'gain': 0x00980913,
    'power_line_frequency': 0x00980918,
    'white_balance_temperature': 0x0098091A,
    'sharpness': 0x0098091B,
    'backlight_compensation': 0x0098091C,
    'exposure_auto': 0x009A0901,
    'exposure_absolute': 0x009A0902,
}


def load_config(path):
    config = json.loads(Path(path).read_text())
    if (not isinstance(config, dict) or config.get('version') != 1
            or not isinstance(config.get('cameras'), dict) or not config['cameras']):
        raise ValueError('Camera config needs version=1 and a nonempty cameras object')
    capture = config.get('capture', {})
    if not isinstance(capture, dict):
        raise ValueError('capture must contain width, height and fps')
    for name, value in capture.items():
        if name not in ('width', 'height', 'fps') or type(value) not in (int, float) or not value > 0:
            raise ValueError(f'Invalid capture setting {name}={value!r}')
        if name != 'fps' and type(value) is not int:
            raise ValueError(f'{name} must be an integer')
        if name == 'fps' and value > 240:
            raise ValueError('fps must be at most 240')
    for role, profile in config['cameras'].items():
        if not role or not isinstance(profile, dict) or not isinstance(profile.get('device'), str):
            raise ValueError(f'{role}: a device path is required')
        controls = profile.setdefault('controls', {})
        if not isinstance(controls, dict):
            raise ValueError(f'{role}: controls must be an object')
        for name, value in controls.items():
            if name not in CONTROL_IDS or type(value) is not int:
                raise ValueError(f'{role}: invalid integer control {name}={value!r}')
        if 'exposure_absolute' in controls and controls.get('exposure_auto') != 1:
            raise ValueError(f'{role}: exposure_absolute requires exposure_auto=1 (manual)')
        if 'white_balance_temperature' in controls and controls.get('auto_white_balance') != 0:
            raise ValueError(f'{role}: fixed white balance requires auto_white_balance=0')
    return config


def get_control(device, control_id):
    buffer = bytearray(struct.pack('=Ii', control_id, 0))
    fcntl.ioctl(device, 0xC008561B, buffer)  # VIDIOC_G_CTRL
    return struct.unpack('=Ii', buffer)[1]


def describe_control(device, control_id):
    buffer = bytearray(68)
    struct.pack_into('=I', buffer, 0, control_id)
    fcntl.ioctl(device, 0xC0445624, buffer)  # VIDIOC_QUERYCTRL
    cid, kind, label, low, high, step, default, flags = struct.unpack_from('=II32siiiiI', buffer)
    return {'id': cid, 'type': kind, 'label': label.split(b'\0')[0].decode(),
            'minimum': low, 'maximum': high, 'step': step, 'default': default,
            'flags': flags}


def read_controls(device):
    """Read supported controls, including ranges, auto modes and inactive values."""
    result = {}
    for name, control_id in CONTROL_IDS.items():
        try:
            description = describe_control(device, control_id)
            description['value'] = get_control(device, control_id)
            result[name] = description
        except OSError as error:
            if error.errno not in (errno.EINVAL, errno.ENOTTY):
                raise
    return result


def apply_controls(path, requested):
    """Validate first; set auto modes before dependent values; verify readback.

    Restore the original controls if any requested setting fails.
    """
    with open(path, 'rb', buffering=0) as device:
        original = read_controls(device)
        for name, value in requested.items():
            control = original.get(name)
            if control is None or control['flags'] & 1:
                raise ValueError(f'{path}: unsupported control {name}')
            if not control['minimum'] <= value <= control['maximum']:
                raise ValueError(f'{name}={value} outside {control["minimum"]}..{control["maximum"]}')
            if control['step'] and (value - control['minimum']) % control['step']:
                raise ValueError(f'{name}={value} does not match step {control["step"]}')
        ordered = sorted(requested, key=lambda name: name not in ('exposure_auto', 'auto_white_balance'))
        changed = []
        try:
            for name in ordered:
                fcntl.ioctl(device, 0xC008561C, struct.pack('=Ii', CONTROL_IDS[name], requested[name]))
                changed.append(name)
            actual = read_controls(device)
            mismatches = {name: actual[name]['value'] for name, value in requested.items()
                          if actual[name]['value'] != value}
            if mismatches:
                raise RuntimeError(f'Camera rejected settings; readback: {mismatches}')
        except Exception:
            # Reverse order restores dependent values before turning auto back on.
            for name in reversed(changed):
                try:
                    fcntl.ioctl(device, 0xC008561C,
                                struct.pack('=Ii', CONTROL_IDS[name], original[name]['value']))
                except OSError as error:
                    print(f'Could not restore {name}: {error}', flush=True)
            raise
    return actual


def inspect_controls(path):
    with open(path, 'rb', buffering=0) as device:
        return read_controls(device)
