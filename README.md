# ESP32-S3 Adapter V2

ESP-IDF firmware for the ESP32-S3 adapter, E34 radio, CH9434 serial bridge,
RFID reader, and ML307C cellular/GNSS module. The ESP32-S3 owns the ML307C AT
state machine, OneNET MQTT upload, GNSS polling, and the flash-backed RFID
delivery queue.

## Required toolchain

The project intentionally rejects any ESP-IDF version other than v5.5.4.
If the local Python environment is missing, bootstrap it once with:

```sh
/Users/gally/.espressif/v5.5.4/esp-idf/install.sh esp32s3
```

Then initialize and build with:

```sh
export IDF_TOOLS_PATH=/Users/gally/.espressif
export IDF_PYTHON_ENV_PATH=/Users/gally/.espressif/python_env/idf5.5_py3.12_env
source /Users/gally/.espressif/v5.5.4/esp-idf/export.sh
idf.py --version
idf.py fullclean build
```

The ESP-IDF tools directory is
`/Users/gally/.espressif/tools`, while `IDF_TOOLS_PATH` itself must point
at its parent `/Users/gally/.espressif`; this is required by ESP-IDF's
`export.sh` lookup convention.

## Safe ML307C wiring

- Power the LLMM307R core board through VIN using the external 12 V supply.
- Connect ESP32-S3 and ML307C grounds.
- GPIO43 (ESP TX) connects to the core-board RX.
- GPIO44 (ESP RX) connects to the core-board TX.
- Do not connect H2.3/U2RXD to BAT.
- Do not connect H2.4/U2TXD to EN.
- H2 5 V remains disconnected.

The firmware uses UART2 at 115200 baud and does not attempt a GPIO power cycle.

## OneNET provisioning

Credentials are read from the existing NVS partition and are never compiled
into the application:

```sh
cp tools/onenet_nvs.csv.example tools/onenet_nvs.csv
# Edit the local CSV, then generate only:
bash tools/provision_onenet.sh
# Or generate and flash the NVS partition:
bash tools/provision_onenet.sh /dev/tty.usbmodem1301
```

The local CSV and generated NVS binary are ignored by Git. The required NVS
namespace is `onenet`, with keys `broker_host`, `broker_port`,
`product_id`, `device_name`, `access_key`, and `token_expiry`.

## Delivery behavior

- Every valid RFID tag is stored in the `rfid_store` partition before upload.
- RFID records are removed only after a matching OneNET property reply with
  code 200; retries therefore provide at-least-once delivery.
- GNSS is polled with `AT+MGNSSLOC` every 120 seconds and only the newest
  generation remains pending.
- E34 keeps the existing frame format; its receiver accepts fragmented,
  concatenated, and noise-prefixed UART input.

Host parser tests can be run independently:

```sh
cmake -S tests/host -B /tmp/esp32-adapter-host-tests
cmake --build /tmp/esp32-adapter-host-tests
ctest --test-dir /tmp/esp32-adapter-host-tests --output-on-failure
```
