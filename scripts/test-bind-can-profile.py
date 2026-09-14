#!/usr/bin/env python3
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("binding", Path(__file__).with_name("bind-can-profile.py"))
binding = importlib.util.module_from_spec(spec)
spec.loader.exec_module(binding)


class BindingTests(unittest.TestCase):
    def test_renamed_interface_uses_sysfs_channel_and_exact_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            netdev = root / "net" / "arm_bus"; netdev.mkdir(parents=True)
            usb = root / "usb"; device = usb / "interface"; device.mkdir(parents=True)
            driver = root / "drivers" / "gs_usb"; driver.mkdir(parents=True)
            (device / "driver").symlink_to(driver, target_is_directory=True)
            (netdev / "device").symlink_to(device, target_is_directory=True)
            for name, value in {"type":"280", "dev_port":"2", "dev_id":"0x2"}.items():
                (netdev / name).write_text(value)
            for name, value in {"idVendor":"1209", "idProduct":"2323",
                                "serial":"0123456789ABCDEF0123456789ABCDEF"}.items():
                (usb / name).write_text(value)
            actual = binding.adapter_binding("arm_bus", root / "net")
            self.assertEqual(actual["channel"], 2)
            self.assertEqual(actual["serial"], "0123456789ABCDEF0123456789ABCDEF")
            (netdev / "dev_id").write_text("0x1")
            with self.assertRaisesRegex(ValueError, "mismatch"):
                binding.adapter_binding("arm_bus", root / "net")

    def test_only_bus_binding_changes(self):
        source = {"schema_version":3, "joint_coordinate_version":2,
                  "validated":True, "calibrated":False,
                  "joints":[{"identity":{"serial_number":42}, "zero_offset_rad":0.3}],
                  "bus":{"transport":"socket_can", "interface":"can0", "channel":0,
                         "expected_link":{"data_bitrate":4000000, "adapter":{}}}}
        before = copy.deepcopy(source)
        adapter = dict(driver="gs_usb", vendor_id=0x1209, product_id=0x2323,
                       serial="A"*32, channel=3)
        result = binding.bind_profile(source, "can7", adapter)
        self.assertEqual(source, before)
        self.assertEqual(result["joints"], source["joints"])
        self.assertFalse(result["calibrated"])
        self.assertTrue(result["validated"])
        self.assertEqual(result["bus"]["interface"], "can7")
        self.assertEqual(result["bus"]["channel"], 3)
        self.assertEqual(result["bus"]["expected_link"]["data_bitrate"], 4000000)
        self.assertEqual(result["bus"]["expected_link"]["adapter"], adapter)

    def test_invalid_interface_and_transport_are_rejected(self):
        for interface in ["../can0", "can0;id", "", "x"*16]:
            with self.assertRaises(ValueError):
                binding.adapter_binding(interface)
        with self.assertRaises(ValueError):
            binding.bind_profile({"schema_version":3,"joint_coordinate_version":2,
                                  "bus":{"transport":"gs_usb"}}, "can0", {})


if __name__ == "__main__":
    unittest.main()
