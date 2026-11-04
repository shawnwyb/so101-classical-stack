"""Failure handling for reproducible camera configuration; no hardware required."""

from copy import deepcopy
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from experiments.camera_settings import CONTROL_IDS, apply_controls, load_config


class CameraSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.device = Path(self.temp.name) / 'device'
        self.device.touch()
        self.controls = {
            'exposure_auto': {'minimum': 0, 'maximum': 3, 'step': 1, 'flags': 0, 'value': 3},
            'exposure_absolute': {'minimum': 1, 'maximum': 5000, 'step': 1, 'flags': 16, 'value': 313},
        }
        self.calls = []
        self.reject = False

    def ioctl(self, device, request, buffer):
        cid, value = struct.unpack('=Ii', buffer)
        name = next(name for name, number in CONTROL_IDS.items() if number == cid)
        self.calls.append((name, value))
        if not (self.reject and name == 'exposure_absolute' and value == 200):
            self.controls[name]['value'] = value

    def apply(self, values):
        with patch('experiments.camera_settings.read_controls', side_effect=lambda _: deepcopy(self.controls)), \
                patch('experiments.camera_settings.fcntl.ioctl', side_effect=self.ioctl):
            return apply_controls(self.device, values)

    def test_manual_mode_is_set_before_exposure(self):
        result = self.apply({'exposure_absolute': 200, 'exposure_auto': 1})
        self.assertEqual(self.calls, [('exposure_auto', 1), ('exposure_absolute', 200)])
        self.assertEqual(result['exposure_absolute']['value'], 200)

    def test_invalid_range_does_not_change_camera(self):
        with self.assertRaisesRegex(ValueError, 'outside'):
            self.apply({'exposure_auto': 1, 'exposure_absolute': 6000})
        self.assertEqual(self.calls, [])

    def test_unsupported_control_does_not_change_camera(self):
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            self.apply({'exposure_auto': 1, 'gain': 20})
        self.assertEqual(self.calls, [])

    def test_readback_mismatch_restores_original_controls(self):
        self.reject = True
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.apply({'exposure_auto': 1, 'exposure_absolute': 200})
        self.assertEqual(self.controls['exposure_auto']['value'], 3)
        self.assertEqual(self.controls['exposure_absolute']['value'], 313)
        self.assertEqual(self.calls[-2:], [('exposure_absolute', 313), ('exposure_auto', 3)])

    def test_config_rejects_fixed_exposure_with_automatic_mode(self):
        path = Path(self.temp.name) / 'config.json'
        path.write_text(json.dumps({'version': 1, 'cameras': {'wide': {
            'device': '/dev/video0', 'controls': {'exposure_auto': 3, 'exposure_absolute': 200}}}}))
        with self.assertRaisesRegex(ValueError, 'manual'):
            load_config(path)


if __name__ == '__main__':
    unittest.main()
