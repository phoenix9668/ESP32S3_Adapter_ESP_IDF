#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
IDF_PATH="/Users/gally/.espressif/v5.5.4/esp-idf"
IDF_TOOLS_PATH="/Users/gally/.espressif"
PYTHON="${IDF_TOOLS_PATH}/python_env/idf5.5_py3.12_env/bin/python"
CSV="${ONENET_NVS_CSV:-${PROJECT_DIR}/tools/onenet_nvs.csv}"
BIN="${ONENET_NVS_BIN:-${PROJECT_DIR}/build/onenet_nvs.bin}"
NVS_SIZE="0x6000"
NVS_OFFSET="0x9000"

if [[ ! -f "${CSV}" ]]; then
  echo "Missing ${CSV}; copy tools/onenet_nvs.csv.example and fill it locally." >&2
  exit 1
fi
if [[ ! -x "${PYTHON}" ]]; then
  echo "Missing ESP-IDF v5.5.4 Python environment: ${PYTHON}" >&2
  exit 1
fi

mkdir -p "$(dirname "${BIN}")"
"${PYTHON}" \
  "${IDF_PATH}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py" \
  generate "${CSV}" "${BIN}" "${NVS_SIZE}"
echo "Generated ${BIN}"

if [[ "${1:-}" != "" ]]; then
  "${PYTHON}" \
    "${IDF_PATH}/components/esptool_py/esptool/esptool.py" \
    --chip esp32s3 --port "$1" write_flash "${NVS_OFFSET}" "${BIN}"
  echo "Flashed OneNET NVS partition through $1"
else
  echo "Pass a serial port as argument to flash offset ${NVS_OFFSET}."
fi
