import importlib.util
import os
import pathlib
import tempfile
import unittest
import urllib.parse
from unittest import mock


ROOT = pathlib.Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location("fleet", ROOT / "tools" / "fleet.py")
fleet = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(fleet)


class FleetTests(unittest.TestCase):
    def test_encrypted_manifest_round_trip_hides_device_key(self):
        with tempfile.TemporaryDirectory() as directory:
            private_dir = pathlib.Path(directory)
            manifest_path = private_dir / "fleet.enc"
            with mock.patch.object(fleet, "PRIVATE_DIR", private_dir), mock.patch.object(
                fleet, "MANIFEST_PATH", manifest_path
            ):
                manifest = fleet.new_manifest()
                manifest["devices"]["wireless-module-001"] = {
                    "device_key": "BASE64-DEVICE-SECRET",
                    "result": "UNASSIGNED",
                }
                password = b"correct horse battery staple"
                fleet.save_manifest(manifest, password)
                raw = manifest_path.read_bytes()
                self.assertNotIn(b"BASE64-DEVICE-SECRET", raw)
                self.assertEqual(fleet.load_manifest(password), manifest)
                self.assertEqual(os.stat(manifest_path).st_mode & 0o777, 0o600)

    def test_retry_reuses_existing_identity(self):
        manifest = fleet.new_manifest()
        manifest["devices"] = {
            "wireless-module-001": {"device_key": "one", "mac": "AA:BB:CC:00:00:01"},
            "wireless-module-002": {"device_key": "two"},
        }
        name, _ = fleet.allocate_device(manifest, "AA:BB:CC:00:00:01")
        self.assertEqual(name, "wireless-module-001")
        name, _ = fleet.allocate_device(manifest, "AA:BB:CC:00:00:02")
        self.assertEqual(name, "wireless-module-002")

    def test_mac_normalization(self):
        self.assertEqual(
            fleet.normalize_mac("aabbccddeeff"), "AA:BB:CC:DD:EE:FF"
        )
        with self.assertRaises(fleet.FleetError):
            fleet.normalize_mac("not-a-mac")

    def test_openapi_authorization_uses_user_resource(self):
        with mock.patch.object(fleet.time, "time", return_value=1_700_000_000):
            authorization = fleet.openapi_authorization("12345", "dGVzdC1rZXk=")
        values = urllib.parse.parse_qs(authorization)
        self.assertEqual(values["version"], ["2020-05-29"])
        self.assertEqual(values["res"], ["userid/12345"])
        self.assertEqual(values["et"], ["1700003600"])
        self.assertEqual(values["method"], ["sha1"])
        self.assertTrue(values["sign"][0])

    def test_batch_create_saves_every_returned_device_key(self):
        response = {
            "success": True,
            "data": {
                "list": [
                    {"name": "wireless-module-001", "sec_key": "key-1"},
                    {"name": "wireless-module-002", "sec_key": "key-2"},
                ]
            },
        }
        with mock.patch.object(fleet, "api_request", return_value=response) as request:
            result = fleet.batch_create(
                "product", ["wireless-module-001", "wireless-module-002"]
            )
        self.assertEqual(
            result,
            {"wireless-module-001": "key-1", "wireless-module-002": "key-2"},
        )
        request.assert_called_once_with(
            "BatchCreateDevices",
            body={
                "product_id": "product",
                "devices": [
                    {"name": "wireless-module-001"},
                    {"name": "wireless-module-002"},
                ],
            },
        )


if __name__ == "__main__":
    unittest.main()
