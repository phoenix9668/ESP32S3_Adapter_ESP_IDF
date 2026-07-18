# ESP32S3 Adapter V2 ML307C Project Context

Updated: 2026-07-17

## Direction

The project is moving from a two-firmware design to a single active ESP-IDF
firmware design.

Old design:

- ESP32-S3 firmware in `ESP32S3_Adapter_ESP_IDF`.
- EG800K QuecPython firmware in `ESP32S3_Adapter_QuecPython`.
- The two processors exchanged a custom CRC-protected text protocol.

New design:

- ESP32-S3 firmware in this repository is the only active firmware.
- ML307C replaces EG800K.
- ML307C runs vendor AT firmware.
- ESP32-S3 must own the AT command state machine, network state, cloud upload,
  GNSS polling/report parsing, RFID offline queue, and LED behavior.

## Implementation Status

The single-firmware refactor is implemented on `wireless-module`:

- ML307C uses fixed 115200-baud UART0 on GPIO43 TX and GPIO44 RX. These are
  ESP32-S3-WROOM-1 physical pins 37 (TXD0) and 36 (RXD0); the console uses USB
  Serial/JTAG, and UART1 remains dedicated to E34.
- H2.3/BAT and H2.4/EN are intentionally disconnected; firmware contains no
  GPIO power-cycle path.
- OneNET credentials live only in the `onenet` NVS namespace and the device
  generates its HMAC-SHA256 token at runtime.
- MQTT property uploads use QoS 0 and are committed locally only after the
  reply topic returns the matching request ID with `code=200`.
- RFID remains a persistent FIFO with at-least-once delivery.
- GNSS is queried every 120 seconds and uses latest-only generation tracking.
- E34 retains the original wire format with a stream-safe receiver.
- The build is pinned to ESP-IDF v5.5.4 and rejects other versions during
  CMake configuration.

## Pre-refactor Repository State (Historical)

Repository:

```text
ESP32S3_Adapter_ESP_IDF
```

Current active branch:

```text
wireless-module
```

Current head when this handoff was written:

```text
07978f1 feat: parse and queue multiple RFID tags per response
```

This section records the EG800K/QuecPython baseline that existed before the
current implementation:

- `main/main.c`: composition root.
- `main/board.c`: LEDs, address switches, and RS485 direction GPIOs.
- `main/ch9434.c`: CH9434 SPI/UART driver.
- `main/serial_router.c`: CH9434 UART polling and serial routing.
- `main/radio_service.c`: E34 packet TX/RX and command dispatch.
- `main/app_protocol.c`: E34 application frame format and CRC32.
- `main/cellular_4g.c`: current EG800K UART2 protocol receiver, watchdog,
  EN control, RFID send, and ACK receive path.
- `main/cellular_4g_protocol.c`: current CRC16 text protocol parser/builder for
  the old ESP32-S3 <-> EG800K/QuecPython status protocol.
- `main/rfid_response.c`: RFID multi-tag response parser; keeps the first 8
  bytes from each 12-byte tag record.
- `main/rfid_store.c`: flash-backed RFID FIFO queue with ordered replay and
  ACK-driven deletion.

Current data flows on `wireless-module`:

- CH9434 UART0 receives instrument/weight frames and forwards complete frames
  through E34.
- CH9434 UART1 polls the RFID reader and parses response frames that may contain
  multiple tags.
- Each valid RFID tag is stored in the `rfid_store` flash partition before
  upload.
- The oldest pending RFID record is sent over UART2 by `cellular_4g_send_rfid`.
- A record is deleted only after a CRC-validated ACK for the matching sequence.
- Green LED is driven by the current cellular/cloud online state.
- Blue LED pulses when a valid RFID tag is read.

The pieces to replace for ML307C are mainly the UART2 peer protocol and the
cloud/GNSS responsibility split. With ML307C AT firmware, ESP32-S3 must talk AT
commands directly instead of expecting QuecPython to send `STAT`, `EVENT`, and
`ACK` frames.

## ML307C Hardware Notes

The top-level `doc` directory now contains the ML307C/LLMM307R/GNSS documents
needed for the new hardware scheme.

### Core Board Interface

From `LLMM307R核心板规格书.pdf`, the mini core board exposes:

Left-side header:

| Pin | Signal | Note |
|---:|---|---|
| 1 | VIN | 5-16 V input |
| 2 | GND | Ground |
| 3 | TX | Serial TX, 3.3 V level on the core board |
| 4 | RX | Serial RX, 3.3 V level on the core board |
| 5 | EN | Default pulled up to VIN |
| 6 | BAT | 3.4-4.2 V battery input, not used together with VIN |

Right-side header:

| Pin | Signal | Note |
|---:|---|---|
| 1 | VIN | 5-16 V input |
| 2 | GND | Ground |
| 3 | DP | USB D+ |
| 4 | DM | USB D- |
| 5 | NET | Network indicator |
| 6 | BOOT | Boot/download pin |

The bare ML307C module UART is 1.8 V logic, while this core-board document says
the board TX/RX pins are 3.3 V level. Before direct ESP32-S3 connection, verify
the actual core board has level shifting and that TX/RX directions are mapped as
expected.

### ML307C Bare Module Constraints

From `ML307C_硬件设计手册_GNSS_AT版本适用.pdf`:

- VBAT range: 3.4 V to 4.5 V, typical 3.8 V.
- 4G transmit current can cause transient current near 2 A.
- The power supply should be able to provide more than 2 A.
- VBAT should have at least two 220 uF storage capacitors near the module.
- VBAT routing should be short and wide.
- On power cycling, hold VBAT below 1.8 V for at least 100 ms before powering on
  again.
- `PWR_ON/OFF` should be pulled low for 2 s to 3.5 s to power on.
- `PWR_ON/OFF` should be pulled low for 3.5 s to 4 s to request shutdown.
- Wait at least 500 ms after VBAT is stable before pulling `PWR_ON/OFF` low.
- `RESET` low for at least 300 ms performs a hardware reset.
- `UART0` is the AT command interface.
- Bare module UART0 is 1.8 V logic and needs level shifting if not using a core
  board that already shifts levels.
- `NETLIGHT` output indicates LTE registration state:
  - 100 ms high / 1900 ms low: LTE registered.
  - 50 ms high / 950 ms low: not registered.
  - low: power off or sleep.
- `STATE` high means module is powered on or sleeping; low means powered off.

### GNSS AT Notes

From `GNSS用户手册_4G系列.pdf`:

- `AT+MGNSS=1` enables continuous GNSS positioning.
- `AT+MGNSS=2` enables single positioning.
- `AT+MGNSS=0` disables GNSS.
- `AT+MGNSSCFG="nmea/mask",<mask>` controls NMEA output.
- `AT+MGNSSCFG="nmea/cycle",<seconds>` controls NMEA report cycle.
- `AT+MGNSSCFG="nmea/port",<port>` controls NMEA output port.
- `AT+MGNSSLOC=1` enables automatic `+MGNSSLOC` location reports.
- `AT+MGNSSLOC` reads the current location once.
- `+MGNSSLOC` fields:
  `UTC,latitude,longtitude,hdop,altitude,fix,cog,spkm,spkn,date,nsat,dtype`.
- ML307C GNSS is disabled by default and must be enabled by AT command.
- This firmware keeps `AT+MGNSS=1` for continuous positioning but enforces
  `AT+MGNSSCFG="nmea/mask",0` and `AT+MGNSSLOC=0`. The NMEA mask is NV-backed,
  so it is queried before writing. Disabling unsolicited output prevents NMEA
  sentences from interleaving with ML307 HTTP HEX frames on the shared AT UART.
- An accepted OTA task enters an exclusive maintenance mode: the GNSS engine
  is stopped with `AT+MGNSS=0`, RFID/CH9434 processing and normal telemetry are
  paused across download retries, and all are restored only after the task is
  gone. Existing RFID FIFO records remain persistent and are uploaded later.

## Old QuecPython Lessons To Preserve

The old QuecPython app is not a runtime dependency anymore, but several design
lessons should be carried into the ESP-IDF ML307C implementation.

### OneNET Property Upload

The old cloud model used OneNET property post topics:

```text
$sys/{product_id}/{device_name}/thing/property/post
$sys/{product_id}/{device_name}/thing/property/post/reply
```

Do not copy credentials into documentation. Reconfirm product ID, device name,
device key, and whether ML307C AT MQTT supports the same authentication format
before implementing the new driver.

Previously used property identifiers:

| Identifier | Purpose |
|---|---|
| `fix_status` | GNSS fix validity |
| `firmware_version` | ESP32-S3 application version shown on the OneNET property page |
| `latitude` | Latitude |
| `longitude` | Longitude |
| `altitude` | Altitude |
| `speed_kph` | Ground speed |
| `course_deg` | Course |
| `satellites` | Satellite count |
| `hdop` | Horizontal dilution |
| `utc_time` | GNSS UTC timestamp |
| `sim_status` | SIM status |
| `sim_csq` | Signal quality |
| `sim_iccid` | SIM ICCID |
| `sim_imsi` | SIM IMSI |
| `sim_phone_number` | Optional phone number |
| `rfid_tag_id` | RFID tag ID |

The most important lesson: a successful local MQTT publish call is not enough.
For durable delivery, match the OneNET reply `id` and require success code
`200` before treating GNSS/RFID data as delivered.

### RFID Offline Queue Semantics

The old 4G branch implemented the right delivery shape:

- Parse every valid RFID response.
- Store each tag in flash before attempting cloud upload.
- Upload pending records in FIFO order.
- Delete a record only after the cloud confirms it.
- Preserve pending records across reset and power loss.
- Accept at-least-once delivery semantics: a lost confirmation can cause a
  duplicate cloud upload.

For the current hardware, the RFID tag sent to the cloud should be the first
8 bytes of the parsed 12-byte tag record, encoded as uppercase hex.

## Implemented ESP-IDF Modules

| Module | Responsibility |
|---|---|
| `components/esp_ml307` | ML307C AT, PDP and MQTT transport with fixed-UART0/raw-field compatibility patches. |
| `cellular_service.cc/.h` | Network state machine, retry policy, OneNET publishing, scheduling and C-compatible status API. |
| `onenet_config.c/.h` | NVS configuration and HMAC-SHA256 token generation. |
| `onenet_reply.c/.h` | OneNET reply ID/code parsing. |
| `gnss_ml307c.c/.h` | GNSS setup, `+MGNSSLOC` parsing and latest snapshot model. |
| `rfid_store.c/.h` | Flash-backed FIFO consumed directly by the OneNET service. |
| `app_protocol_stream.c/.h` | Stream-safe E34 frame accumulation and resynchronization. |

## LED Behavior Target

Keep this behavior in the new implementation:

- Green LED on: 4G/cloud link is online and data can be uploaded.
- Green LED off: 4G/cloud link is offline or not yet ready.
- Blue LED pulse: a valid RFID tag was read.

Do not let unrelated E34 sends extend or cancel the blue RFID pulse.
