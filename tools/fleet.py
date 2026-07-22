#!/usr/bin/env python3
"""OneNET fleet preparation and ESP32-S3 production station.

Secrets are kept only inside an AES-256-GCM encrypted local manifest. The
passphrase and OneNET OpenAPI credential are accepted through environment
variables so neither appears in command history or process arguments.
"""

from __future__ import annotations

import argparse
import base64
import csv
import datetime as dt
import getpass
import hashlib
import hmac
import json
import os
import pathlib
import re
import secrets
import stat
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.kdf.scrypt import Scrypt


ROOT = pathlib.Path(__file__).resolve().parent.parent
PRIVATE_DIR = ROOT / ".factory"
MANIFEST_PATH = PRIVATE_DIR / "fleet.enc"
CONFIG_PATH = ROOT / "tools" / "fleet_config.json"
MACOS_DEFAULT_IDF_PATH = pathlib.Path("/Users/gally/.espressif/v5.5.4/esp-idf")
BUILD_DIR = ROOT / "build"
MAGIC = b"ESP32S3-FLEET-1\0"
NVS_OFFSET = "0x9000"
NVS_SIZE = "0x6000"
API_BASE = "https://iot-api.heclouds.com/common"
APP_NAME = "ESP32S3_Adapter_ESP_IDF"


class FleetError(RuntimeError):
    pass


def idf_path() -> pathlib.Path:
    """Return the active ESP-IDF tree without baking a macOS path into Windows."""
    configured = os.environ.get("IDF_PATH")
    if configured:
        result = pathlib.Path(configured).expanduser().resolve()
    elif os.name == "nt":
        raise FleetError(
            "IDF_PATH is not set; open an ESP-IDF v5.5.4 PowerShell or run "
            "the ESP-IDF export.ps1 script first"
        )
    else:
        result = MACOS_DEFAULT_IDF_PATH
    if not result.exists():
        raise FleetError(f"ESP-IDF path does not exist: {result}")
    return result


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat()


def normalize_mac(value: str) -> str:
    compact = re.sub(r"[^0-9a-fA-F]", "", value).upper()
    if len(compact) != 12:
        raise FleetError(f"Invalid ESP32-S3 MAC: {value}")
    return ":".join(compact[index : index + 2] for index in range(0, 12, 2))


def passphrase() -> bytes:
    value = os.environ.get("FACTORY_MANIFEST_PASSPHRASE")
    if not value and sys.stdin.isatty():
        value = getpass.getpass("Factory manifest passphrase: ")
    if not value or len(value) < 12:
        raise FleetError(
            "Set FACTORY_MANIFEST_PASSPHRASE to at least 12 characters"
        )
    return value.encode("utf-8")


def derive_key(password: bytes, salt: bytes) -> bytes:
    return Scrypt(salt=salt, length=32, n=1 << 15, r=8, p=1).derive(password)


def new_manifest() -> dict[str, Any]:
    return {"schema": 1, "created_at": utc_now(), "devices": {}}


def load_manifest(password: bytes) -> dict[str, Any]:
    if not MANIFEST_PATH.exists():
        return new_manifest()
    payload = MANIFEST_PATH.read_bytes()
    if not payload.startswith(MAGIC) or len(payload) < len(MAGIC) + 28:
        raise FleetError("Factory manifest has an invalid format")
    cursor = len(MAGIC)
    salt = payload[cursor : cursor + 16]
    nonce = payload[cursor + 16 : cursor + 28]
    encrypted = payload[cursor + 28 :]
    try:
        plain = AESGCM(derive_key(password, salt)).decrypt(nonce, encrypted, MAGIC)
        manifest = json.loads(plain)
    except Exception as exc:
        raise FleetError("Cannot decrypt factory manifest") from exc
    if manifest.get("schema") != 1 or not isinstance(manifest.get("devices"), dict):
        raise FleetError("Unsupported factory manifest schema")
    return manifest


def save_manifest(manifest: dict[str, Any], password: bytes) -> None:
    PRIVATE_DIR.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(PRIVATE_DIR, 0o700)
    salt = secrets.token_bytes(16)
    nonce = secrets.token_bytes(12)
    plain = json.dumps(
        manifest, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    encrypted = AESGCM(derive_key(password, salt)).encrypt(nonce, plain, MAGIC)
    fd, temporary_name = tempfile.mkstemp(prefix="fleet.", dir=PRIVATE_DIR)
    temporary = pathlib.Path(temporary_name)
    try:
        os.fchmod(fd, stat.S_IRUSR | stat.S_IWUSR)
        with os.fdopen(fd, "wb") as stream:
            stream.write(MAGIC + salt + nonce + encrypted)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, MANIFEST_PATH)
        os.chmod(MANIFEST_PATH, 0o600)
    finally:
        temporary.unlink(missing_ok=True)


def load_config() -> dict[str, Any]:
    if not CONFIG_PATH.exists():
        raise FleetError(
            "Missing tools/fleet_config.json; copy fleet_config.json.example first"
        )
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    config["product_id"] = os.environ.get(
        "ONENET_PRODUCT_ID", config.get("product_id", "")
    )
    required = ("product_id", "broker_host", "device_prefix")
    for field in required:
        if not config.get(field) or str(config[field]).startswith("REPLACE_"):
            raise FleetError(f"Missing fleet configuration field: {field}")
    config.setdefault("broker_port", 1883)
    config.setdefault("token_ttl", 3600)
    config.setdefault("serial_baud", 115200)
    config.setdefault("verify_timeout_seconds", 240)
    if not 300 <= int(config["token_ttl"]) <= 604800:
        raise FleetError("token_ttl must be between 300 and 604800 seconds")
    return config


def openapi_authorization(user_id: str, access_key: str) -> str:
    expires = str(int(time.time()) + 3600)
    method = "sha1"
    version = "2020-05-29"
    resource = f"userid/{user_id}"
    source = f"{expires}\n{method}\n{resource}\n{version}".encode()
    try:
        key = base64.b64decode(access_key, validate=True)
    except ValueError as exc:
        raise FleetError("ONENET_API_ACCESS_KEY is not valid Base64") from exc
    signature = base64.b64encode(hmac.new(key, source, hashlib.sha1).digest()).decode()
    return "&".join(
        (
            f"version={version}",
            f"res={urllib.parse.quote(resource, safe='')}",
            f"et={expires}",
            f"method={method}",
            f"sign={urllib.parse.quote(signature, safe='')}",
        )
    )


def api_request(
    action: str,
    *,
    query: dict[str, str] | None = None,
    body: dict[str, Any] | None = None,
) -> dict[str, Any]:
    user_id = os.environ.get("ONENET_USER_ID", "")
    access_key = os.environ.get("ONENET_API_ACCESS_KEY", "")
    if not user_id or not access_key:
        raise FleetError("Set ONENET_USER_ID and ONENET_API_ACCESS_KEY")
    parameters = {"action": action, "version": "1"}
    parameters.update(query or {})
    url = API_BASE + "?" + urllib.parse.urlencode(parameters)
    data = None
    headers = {
        "Authorization": openapi_authorization(user_id, access_key),
        "Accept": "application/json",
    }
    if body is not None:
        data = json.dumps(body, separators=(",", ":")).encode()
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, data=data, headers=headers)
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            payload = json.loads(response.read())
    except urllib.error.HTTPError as exc:
        detail = exc.read(512).decode("utf-8", "replace")
        raise FleetError(f"OneNET OpenAPI HTTP {exc.code}: {detail}") from exc
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        raise FleetError(f"OneNET OpenAPI request failed: {exc}") from exc
    if not isinstance(payload, dict):
        raise FleetError("OneNET OpenAPI returned a non-object response")
    return payload


def query_device(product_id: str, name: str) -> dict[str, Any] | None:
    payload = api_request(
        "QueryDeviceDetail",
        query={"product_id": product_id, "device_name": name},
    )
    if payload.get("success") is True:
        data = payload.get("data")
        if not isinstance(data, dict):
            raise FleetError(f"OneNET returned no details for {name}")
        return data
    message = str(payload.get("msg") or payload.get("message") or "").lower()
    code = str(payload.get("code") or "").lower()
    absence_markers = ("not exist", "not found", "不存在", "未找到")
    if any(marker in message for marker in absence_markers) or code in {
        "devicenotexist",
        "devicenotfound",
        "10410",
    }:
        return None
    raise FleetError(f"Cannot query {name}: code={code}, message={message}")


def batch_create(product_id: str, names: list[str]) -> dict[str, str]:
    if not names:
        return {}
    if len(names) > 500:
        raise FleetError("OneNET accepts at most 500 devices per batch")
    payload = api_request(
        "BatchCreateDevices",
        body={
            "product_id": product_id,
            "devices": [{"name": name} for name in names],
        },
    )
    if payload.get("success") is not True:
        raise FleetError(
            "OneNET batch creation failed: "
            + str(payload.get("msg") or payload.get("message") or payload.get("code"))
        )
    data = payload.get("data") or {}
    rows = data.get("list") if isinstance(data, dict) else None
    if not isinstance(rows, list):
        raise FleetError("OneNET batch creation returned no device list")
    result: dict[str, str] = {}
    for row in rows:
        if isinstance(row, dict) and row.get("name") and row.get("sec_key"):
            result[str(row["name"])] = str(row["sec_key"])
    missing = sorted(set(names) - set(result))
    if missing:
        raise FleetError(f"OneNET did not return Device Keys for {len(missing)} devices")
    return result


def command_refresh_key(args: argparse.Namespace) -> None:
    if not args.confirm:
        raise FleetError("Refusing Device Key replacement without --confirm")
    config = load_config()
    password = passphrase()
    manifest = load_manifest(password)
    record = manifest["devices"].get(args.device_name)
    if record is None:
        raise FleetError(f"Unknown local identity: {args.device_name}")
    detail = query_device(config["product_id"], args.device_name)
    if detail is None:
        raise FleetError(f"OneNET device does not exist: {args.device_name}")
    device_key = detail.get("sec_key") or detail.get("device_key")
    if not device_key:
        raise FleetError(f"OneNET returned no Device Key for {args.device_name}")
    record["device_key"] = str(device_key)
    record["key_refreshed_at"] = utc_now()
    record["result"] = "ASSIGNED" if record.get("mac") else "UNASSIGNED"
    record["last_error"] = "Device Key refreshed; NVS reflash required"
    save_manifest(manifest, password)
    print(f"Refreshed encrypted Device Key for {args.device_name}; key not displayed")


def command_prepare(args: argparse.Namespace) -> None:
    config = load_config()
    password = passphrase()
    manifest = load_manifest(password)
    names = [
        f"{config['device_prefix']}{number:03d}"
        for number in range(args.start, args.start + args.count)
    ]
    missing: list[str] = []
    for index, name in enumerate(names, start=1):
        existing = manifest["devices"].get(name)
        if existing and existing.get("device_key"):
            print(f"[{index:03d}/{len(names):03d}] {name}: already secured locally")
            continue
        detail = query_device(config["product_id"], name)
        if detail is None:
            missing.append(name)
            print(f"[{index:03d}/{len(names):03d}] {name}: will create")
            continue
        device_key = detail.get("sec_key") or detail.get("device_key")
        if not device_key:
            raise FleetError(f"OneNET returned no Device Key for existing {name}")
        manifest["devices"][name] = {
            "device_key": str(device_key),
            "cloud_state": "existing",
            "created_at": utc_now(),
            "result": "UNASSIGNED",
        }
        save_manifest(manifest, password)
        print(f"[{index:03d}/{len(names):03d}] {name}: imported")
    if missing:
        print(f"Creating {len(missing)} missing OneNET devices in one batch...")
        created = batch_create(config["product_id"], missing)
        for name in missing:
            manifest["devices"][name] = {
                "device_key": created[name],
                "cloud_state": "created",
                "created_at": utc_now(),
                "result": "UNASSIGNED",
            }
        save_manifest(manifest, password)
    print(f"Prepared {len(names)} encrypted device identities in {MANIFEST_PATH}")


def esptool_path() -> pathlib.Path:
    result = idf_path() / "components" / "esptool_py" / "esptool" / "esptool.py"
    if not result.exists():
        raise FleetError(f"Missing ESP-IDF v5.5.4 esptool: {result}")
    return result


def validate_factory_build() -> None:
    app_bin = BUILD_DIR / f"{APP_NAME}.bin"
    key_path = pathlib.Path(
        os.environ.get("OTA_SIGNING_KEY", ROOT / "keys" / "ota_signing_key.pem")
    )
    if not app_bin.exists() or not key_path.exists():
        raise FleetError("Signed factory application or OTA signing key is missing")
    espsecure = (
        idf_path() / "components" / "esptool_py" / "esptool" / "espsecure.py"
    )
    verify = subprocess.run(
        [
            sys.executable,
            str(espsecure),
            "verify_signature",
            "--version",
            "2",
            "--keyfile",
            str(key_path),
            str(app_bin),
        ],
        text=True,
        capture_output=True,
    )
    if verify.returncode:
        raise FleetError("Factory application is not signed by the configured RSA key")
    information = subprocess.run(
        [
            sys.executable,
            str(esptool_path()),
            "--chip",
            "esp32s3",
            "image_info",
            "--version",
            "2",
            str(app_bin),
        ],
        text=True,
        capture_output=True,
        check=True,
    ).stdout
    expected_version = (ROOT / "version.txt").read_text().strip()
    if f"Project name: {APP_NAME}" not in information:
        raise FleetError("Factory image has the wrong ESP-IDF project identity")
    if f"App version: {expected_version}" not in information:
        raise FleetError("Factory image version does not match version.txt; rebuild it")
    if "ESP-IDF: v5.5.4" not in information:
        raise FleetError("Factory image was not built with ESP-IDF v5.5.4")


def choose_port(requested: str) -> str:
    if requested != "auto":
        return requested
    from serial.tools import list_ports

    ports = []
    for item in list_ports.comports():
        description = " ".join(
            str(value or "")
            for value in (
                item.device,
                item.description,
                item.manufacturer,
                item.hwid,
            )
        ).lower()
        unix_usb = any(
            marker in description
            for marker in (
                "usbmodem",
                "usbserial",
                "slab",
                "wchusbserial",
                "usb jtag",
                "usb serial",
                "espressif",
                "cp210",
                "ch340",
                "ch910",
                "ftdi",
            )
        )
        windows_usb = (
            os.name == "nt"
            and item.device.lower().startswith("com")
            and (getattr(item, "vid", None) is not None or "usb" in description)
        )
        if unix_usb or windows_usb:
            ports.append(item.device)
    if len(ports) != 1:
        raise FleetError(
            "Auto port selection requires exactly one USB serial device; found: "
            + (", ".join(ports) if ports else "none")
        )
    return ports[0]


def read_mac(port: str) -> str:
    command = [
        sys.executable,
        str(esptool_path()),
        "--chip",
        "esp32s3",
        "--port",
        port,
        "read_mac",
    ]
    result = subprocess.run(command, text=True, capture_output=True, timeout=30)
    if result.returncode:
        raise FleetError("Cannot read ESP32-S3 MAC: " + result.stderr.strip())
    match = re.search(r"MAC:\s*([0-9a-fA-F:]{17})", result.stdout)
    if not match:
        raise FleetError("esptool did not report an ESP32-S3 MAC")
    return normalize_mac(match.group(1))


def allocate_device(
    manifest: dict[str, Any], mac: str, requested_name: str | None = None
) -> tuple[str, dict[str, Any]]:
    for name, record in manifest["devices"].items():
        if record.get("mac") == mac:
            return name, record
    if requested_name:
        record = manifest["devices"].get(requested_name)
        if not record:
            raise FleetError(f"Unknown device identity: {requested_name}")
        if record.get("mac") and record["mac"] != mac:
            raise FleetError(f"{requested_name} is already bound to another MAC")
        return requested_name, record
    for name in sorted(manifest["devices"]):
        record = manifest["devices"][name]
        if not record.get("mac"):
            return name, record
    raise FleetError("No unassigned identities remain; run fleet prepare")


def generate_nvs(config: dict[str, Any], name: str, key: str) -> pathlib.Path:
    PRIVATE_DIR.mkdir(mode=0o700, parents=True, exist_ok=True)
    csv_path = PRIVATE_DIR / f"{name}.nvs.csv"
    bin_path = PRIVATE_DIR / f"{name}.nvs.bin"
    rows = [
        ("key", "type", "encoding", "value"),
        ("onenet", "namespace", "", ""),
        ("broker_host", "data", "string", config["broker_host"]),
        ("broker_port", "data", "u16", str(config["broker_port"])),
        ("product_id", "data", "string", config["product_id"]),
        ("device_name", "data", "string", name),
        ("device_key", "data", "string", key),
        ("token_ttl", "data", "u32", str(config["token_ttl"])),
    ]
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            os.chmod(csv_path, 0o600)
            csv.writer(stream).writerows(rows)
        generator = (
            idf_path()
            / "components"
            / "nvs_flash"
            / "nvs_partition_generator"
            / "nvs_partition_gen.py"
        )
        subprocess.run(
            [sys.executable, str(generator), "generate", str(csv_path), str(bin_path), NVS_SIZE],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        os.chmod(bin_path, 0o600)
        return bin_path
    finally:
        csv_path.unlink(missing_ok=True)


def flash_device(port: str, nvs_bin: pathlib.Path) -> None:
    arguments_path = BUILD_DIR / "flasher_args.json"
    if not arguments_path.exists():
        raise FleetError("Missing build/flasher_args.json; build the signed factory app first")
    arguments = json.loads(arguments_path.read_text(encoding="utf-8"))
    files = arguments.get("flash_files") or {}
    required_offsets = {"0x0", "0x8000", "0xf000", "0x20000"}
    if not required_offsets.issubset({key.lower() for key in files}):
        raise FleetError("Factory build is missing bootloader/partition/otadata/application")
    command = [
        sys.executable,
        str(esptool_path()),
        "--chip",
        "esp32s3",
        "--port",
        port,
        "--before",
        "default_reset",
        "--after",
        "hard_reset",
        "write_flash",
        *arguments.get("write_flash_args", []),
    ]
    for offset, filename in files.items():
        command.extend((offset, str(BUILD_DIR / filename)))
    command.extend((NVS_OFFSET, str(nvs_bin)))
    result = subprocess.run(command, cwd=ROOT)
    if result.returncode:
        raise FleetError(f"esptool flash failed with exit code {result.returncode}")


def wait_for_online(port: str, expected_name: str, config: dict[str, Any]) -> str:
    import serial

    deadline = time.monotonic() + int(config["verify_timeout_seconds"])
    last_stage = "reset"
    stream = None
    while time.monotonic() < deadline and stream is None:
        try:
            stream = serial.Serial(port, int(config["serial_baud"]), timeout=1)
        except serial.SerialException:
            time.sleep(0.5)
    if stream is None:
        raise FleetError(f"USB console did not reappear after flashing: {port}")
    with stream:
        stream.dtr = False
        stream.rts = False
        while time.monotonic() < deadline:
            line = stream.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            marker = line.find("FACTORY_STATUS ")
            if marker < 0:
                continue
            try:
                event = json.loads(line[marker + len("FACTORY_STATUS ") :])
            except json.JSONDecodeError:
                continue
            last_stage = str(event.get("stage") or last_stage)
            print(f"  verify: {last_stage}")
            if event.get("stage") == "online":
                if event.get("device") != expected_name or event.get("mqtt") is not True:
                    raise FleetError("Online verification reported the wrong device identity")
                return str(event.get("iccid") or "")
            if event.get("stage") == "error":
                raise FleetError("Firmware factory verification reported an error")
    raise FleetError(f"Timed out waiting for OneNET online; last stage={last_stage}")


def provision_once(port_arg: str, requested_name: str | None = None) -> tuple[str, str]:
    config = load_config()
    password = passphrase()
    manifest = load_manifest(password)
    port = choose_port(port_arg)
    mac = read_mac(port)
    name, record = allocate_device(manifest, mac, requested_name)
    if not record.get("device_key"):
        raise FleetError(f"Encrypted Device Key is missing for {name}")
    record["mac"] = mac
    record["result"] = "ASSIGNED"
    record["assigned_at"] = record.get("assigned_at") or utc_now()
    record["last_error"] = ""
    save_manifest(manifest, password)
    print(f"Provisioning {name} on {mac} through {port}")
    nvs_bin: pathlib.Path | None = None
    try:
        nvs_bin = generate_nvs(config, name, record["device_key"])
        flash_device(port, nvs_bin)
        iccid = wait_for_online(port, name, config)
        record["iccid"] = iccid
        record["version"] = (ROOT / "version.txt").read_text().strip()
        record["flashed_at"] = utc_now()
        record["verified_at"] = utc_now()
        record["result"] = "PASS"
        save_manifest(manifest, password)
        print(f"PASS {name} MAC={mac} ICCID={iccid or '-'}")
        return name, mac
    except Exception as exc:
        record["result"] = "FAIL"
        record["last_error"] = str(exc)[:240]
        record["failed_at"] = utc_now()
        save_manifest(manifest, password)
        raise
    finally:
        if nvs_bin:
            nvs_bin.unlink(missing_ok=True)


def command_station(args: argparse.Namespace) -> None:
    validate_factory_build()
    while True:
        provision_once(args.port)
        if args.once or not sys.stdin.isatty():
            return
        response = input("Remove the PASS unit, insert the next unit, then press Enter (q to quit): ")
        if response.strip().lower() in {"q", "quit", "exit"}:
            return


def command_retry(args: argparse.Namespace) -> None:
    validate_factory_build()
    password = passphrase()
    manifest = load_manifest(password)
    mac = normalize_mac(args.mac)
    name = next(
        (name for name, record in manifest["devices"].items() if record.get("mac") == mac),
        None,
    )
    if not name:
        raise FleetError(f"MAC {mac} has no assigned identity and cannot be retried")
    provision_once(args.port, name)


def public_rows(manifest: dict[str, Any]) -> list[dict[str, str]]:
    fields = (
        "device_name",
        "mac",
        "iccid",
        "version",
        "assigned_at",
        "flashed_at",
        "verified_at",
        "result",
        "last_error",
    )
    rows = []
    for name in sorted(manifest["devices"]):
        record = manifest["devices"][name]
        row = {field: str(record.get(field, "")) for field in fields}
        row["device_name"] = name
        rows.append(row)
    return rows


def command_status(_: argparse.Namespace) -> None:
    manifest = load_manifest(passphrase())
    counts: dict[str, int] = {}
    for record in manifest["devices"].values():
        result = str(record.get("result") or "UNASSIGNED")
        counts[result] = counts.get(result, 0) + 1
    print(f"Encrypted identities: {len(manifest['devices'])}")
    for result in sorted(counts):
        print(f"  {result}: {counts[result]}")
    for row in public_rows(manifest):
        if row["result"] in {"FAIL", "ASSIGNED"}:
            print(f"  {row['device_name']} {row['mac'] or '-'} {row['result']}")


def command_export(args: argparse.Namespace) -> None:
    manifest = load_manifest(passphrase())
    output = pathlib.Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = public_rows(manifest)
    fields = list(rows[0]) if rows else ["device_name", "mac", "result"]
    with output.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Exported {len(rows)} public audit rows to {output}")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(prog="fleet")
    commands = result.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="import/create encrypted identities")
    prepare.add_argument("--start", type=int, default=1)
    prepare.add_argument("--count", type=int, default=100)
    prepare.set_defaults(handler=command_prepare)
    refresh = commands.add_parser(
        "refresh-key", help="securely refresh a reset OneNET Device Key"
    )
    refresh.add_argument("--device-name", required=True)
    refresh.add_argument("--confirm", action="store_true")
    refresh.set_defaults(handler=command_refresh_key)
    station = commands.add_parser("station", help="run the production station")
    station.add_argument("--port", default="auto")
    station.add_argument("--once", action="store_true", help=argparse.SUPPRESS)
    station.set_defaults(handler=command_station)
    retry = commands.add_parser("retry", help="retry a previously assigned MAC")
    retry.add_argument("--mac", required=True)
    retry.add_argument("--port", default="auto")
    retry.set_defaults(handler=command_retry)
    status = commands.add_parser("status", help="show non-secret fleet totals")
    status.set_defaults(handler=command_status)
    export = commands.add_parser("export-audit", help="export a non-secret CSV")
    export.add_argument("--output", default=str(PRIVATE_DIR / "fleet-audit.csv"))
    export.set_defaults(handler=command_export)
    return result


def main() -> int:
    args = parser().parse_args()
    if getattr(args, "start", 1) < 1 or getattr(args, "count", 1) < 1:
        raise FleetError("start and count must be positive")
    args.handler(args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FleetError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
