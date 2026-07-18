#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <serial-port> <signed-application.bin>" >&2
  echo "Example: $0 /dev/tty.usbmodem11201 dist/s3-1.0.2.bin" >&2
  exit 2
fi

PORT="$1"
APP_BIN="$2"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IDF_PATH="/Users/gally/.espressif/v5.5.4/esp-idf"
IDF_TOOLS_PATH="/Users/gally/.espressif"
OTADATA_OFFSET="0xF000"
OTADATA_SIZE="0x2000"
OTA0_OFFSET="0x20000"
OTA_SLOT_SIZE=$((0x400000))

if [[ ! -f "${APP_BIN}" ]]; then
  echo "Application image not found: ${APP_BIN}" >&2
  exit 1
fi
APP_BIN="$(cd "$(dirname "${APP_BIN}")" && pwd)/$(basename "${APP_BIN}")"
APP_SIZE="$(stat -f %z "${APP_BIN}")"
if (( APP_SIZE <= 0 || APP_SIZE > OTA_SLOT_SIZE )); then
  echo "Application size ${APP_SIZE} does not fit the 4 MiB OTA slot." >&2
  exit 1
fi

export IDF_PATH IDF_TOOLS_PATH
# shellcheck disable=SC1091
source "${IDF_PATH}/export.sh" >/dev/null
if [[ "$(idf.py --version)" != *"v5.5.4"* ]]; then
  echo "ESP-IDF v5.5.4 is required." >&2
  exit 1
fi

ESPTOOL=(python "${IDF_PATH}/components/esptool_py/esptool/esptool.py"
         --chip esp32s3 --port "${PORT}")

echo "Recovery image: ${APP_BIN} (${APP_SIZE} bytes)"
echo "Erasing only otadata at ${OTADATA_OFFSET}, size ${OTADATA_SIZE}."
echo "OneNET NVS and rfid_store will not be erased."
"${ESPTOOL[@]}" --after no_reset erase_region "${OTADATA_OFFSET}" "${OTADATA_SIZE}"
"${ESPTOOL[@]}" --after hard_reset write_flash --verify \
  --flash_mode keep --flash_freq keep --flash_size keep \
  "${OTA0_OFFSET}" "${APP_BIN}"

echo "Recovery application written to ota_0 and boot selection reset."
echo "Monitor with: idf.py -p ${PORT} monitor"
