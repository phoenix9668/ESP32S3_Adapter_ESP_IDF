#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <version, for example 1.1.0>" >&2
  exit 2
fi

VERSION="$1"
if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+([.-][0-9A-Za-z]+)*$ ]] || [[ ${#VERSION} -gt 20 ]]; then
  echo "Invalid OneNET application version: ${VERSION}" >&2
  exit 2
fi

# OneNET rejects upload filenames longer than 20 characters.  Keep the
# platform-facing artifact short while retaining the complete version.
OUT_BASENAME="s3-${VERSION}.bin"
if [[ ${#OUT_BASENAME} -gt 20 ]]; then
  echo "OneNET OTA filename would exceed 20 characters: ${OUT_BASENAME}" >&2
  echo "Use a target version no longer than 13 characters." >&2
  exit 2
fi

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IDF_PATH="/Users/gally/.espressif/v5.5.4/esp-idf"
IDF_TOOLS_PATH="/Users/gally/.espressif"
KEY_PATH="${OTA_SIGNING_KEY:-${PROJECT_DIR}/keys/ota_signing_key.pem}"
CONFIGURED_KEY_PATH="${PROJECT_DIR}/keys/ota_signing_key.pem"
OUT_DIR="${PROJECT_DIR}/dist"
APP_BIN="${PROJECT_DIR}/build/ESP32S3_Adapter_ESP_IDF.bin"
OUT_BIN="${OUT_DIR}/${OUT_BASENAME}"
MANIFEST="${OUT_DIR}/s3-${VERSION}.manifest.json"

if [[ ! -f "${KEY_PATH}" ]]; then
  echo "Missing OTA signing key: ${KEY_PATH}" >&2
  echo "Run tools/generate_signing_key.sh once, then back up the key offline." >&2
  exit 1
fi

FILE_VERSION="$(tr -d '[:space:]' < "${PROJECT_DIR}/version.txt")"
if [[ "${FILE_VERSION}" != "${VERSION}" ]]; then
  echo "Version mismatch: version.txt=${FILE_VERSION}, requested=${VERSION}" >&2
  echo "Update and commit version.txt before creating a release." >&2
  exit 1
fi

KEY_LINK_CREATED=0
KEY_ABS="$(cd "$(dirname "${KEY_PATH}")" && pwd)/$(basename "${KEY_PATH}")"
if [[ "${KEY_ABS}" != "${CONFIGURED_KEY_PATH}" ]]; then
  if [[ -L "${CONFIGURED_KEY_PATH}" && "$(readlink "${CONFIGURED_KEY_PATH}")" == "${KEY_ABS}" ]]; then
    : # The persistent ignored link already targets the selected offline key.
  elif [[ -e "${CONFIGURED_KEY_PATH}" || -L "${CONFIGURED_KEY_PATH}" ]]; then
    echo "Cannot use OTA_SIGNING_KEY while ${CONFIGURED_KEY_PATH} exists." >&2
    exit 1
  else
    mkdir -p "$(dirname "${CONFIGURED_KEY_PATH}")"
    ln -s "${KEY_ABS}" "${CONFIGURED_KEY_PATH}"
    KEY_LINK_CREATED=1
  fi
fi
cleanup_key_link() {
  if [[ ${KEY_LINK_CREATED} -eq 1 ]]; then
    rm -f "${CONFIGURED_KEY_PATH}"
  fi
}
trap cleanup_key_link EXIT

export IDF_PATH IDF_TOOLS_PATH APP_RELEASE_VERSION="${VERSION}"
source "${IDF_PATH}/export.sh" >/dev/null
if [[ "$(idf.py --version)" != *"v5.5.4"* ]]; then
  echo "ESP-IDF v5.5.4 is required." >&2
  exit 1
fi

cd "${PROJECT_DIR}"
idf.py fullclean build
espsecure.py verify_signature --version 2 --keyfile "${KEY_PATH}" "${APP_BIN}"

mkdir -p "${OUT_DIR}"
cp "${APP_BIN}" "${OUT_BIN}"

export OTA_VERSION="${VERSION}" OTA_OUTPUT_BIN="${OUT_BIN}" OTA_MANIFEST="${MANIFEST}"
python3 - <<'PY'
import datetime
import hashlib
import json
import os
import pathlib
import subprocess

binary = pathlib.Path(os.environ["OTA_OUTPUT_BIN"])
payload = binary.read_bytes()
manifest = {
    "artifact": binary.name,
    "version": os.environ["OTA_VERSION"],
    "size": len(payload),
    "md5": hashlib.md5(payload).hexdigest(),
    "sha256": hashlib.sha256(payload).hexdigest(),
    "git_commit": subprocess.check_output(
        ["git", "rev-parse", "HEAD"], text=True).strip(),
    "git_dirty": subprocess.run(
        ["git", "diff", "--quiet", "--ignore-submodules", "HEAD"],
        check=False,
    ).returncode != 0,
    "built_at_utc": datetime.datetime.now(
        datetime.timezone.utc).replace(microsecond=0).isoformat(),
    "idf_version": "v5.5.4",
    "upload_to_onenet": True,
    "image_kind": "signed ESP-IDF application only",
}
pathlib.Path(os.environ["OTA_MANIFEST"]).write_text(
    json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8",
)
PY

echo "OneNET upload file: ${OUT_BIN}"
echo "Local release manifest: ${MANIFEST}"
echo "Do not upload merged factory images, bootloader, partition table, or NVS."
