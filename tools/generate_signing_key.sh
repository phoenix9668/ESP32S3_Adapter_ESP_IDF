#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IDF_PATH="/Users/gally/.espressif/v5.5.4/esp-idf"
IDF_TOOLS_PATH="/Users/gally/.espressif"
KEY_PATH="${OTA_SIGNING_KEY:-${PROJECT_DIR}/keys/ota_signing_key.pem}"
CONFIGURED_KEY_PATH="${PROJECT_DIR}/keys/ota_signing_key.pem"

export IDF_PATH IDF_TOOLS_PATH
source "${IDF_PATH}/export.sh" >/dev/null

if [[ -e "${KEY_PATH}" ]]; then
  echo "Refusing to replace existing signing key: ${KEY_PATH}" >&2
  exit 1
fi

mkdir -p "$(dirname "${KEY_PATH}")"
KEY_ABS="$(cd "$(dirname "${KEY_PATH}")" && pwd)/$(basename "${KEY_PATH}")"
if [[ "${KEY_ABS}" != "${CONFIGURED_KEY_PATH}" &&
      ( -e "${CONFIGURED_KEY_PATH}" || -L "${CONFIGURED_KEY_PATH}" ) ]]; then
  echo "Configured key path already exists: ${CONFIGURED_KEY_PATH}" >&2
  exit 1
fi

espsecure.py generate_signing_key --version 2 "${KEY_PATH}"
chmod 600 "${KEY_PATH}"
if [[ "${KEY_ABS}" != "${CONFIGURED_KEY_PATH}" ]]; then
  mkdir -p "$(dirname "${CONFIGURED_KEY_PATH}")"
  ln -s "${KEY_ABS}" "${CONFIGURED_KEY_PATH}"
  echo "Linked project signing path to the offline key."
fi
echo "Generated RSA-3072 OTA signing key: ${KEY_PATH}"
echo "Back up this file offline before manufacturing devices."
