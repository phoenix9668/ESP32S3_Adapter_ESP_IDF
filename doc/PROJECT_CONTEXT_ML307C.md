# ESP32S3 Adapter V2 ML307C Project Context

Updated: 2026-07-16

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

## Current ESP-IDF Repository State

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

The current `wireless-module` branch already contains the previous
EG800K/QuecPython-oriented 4G implementation. The next step is to keep the
useful ESP32-S3-side pieces and replace the EG800K peer protocol with an
ML307C AT command driver.

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

## New ESP-IDF Implementation Direction

Recommended refactor/new modules:

| Module | Responsibility |
|---|---|
| `modem_ml307c.c/.h` | Replace or wrap `cellular_4g.c` with UART driver, AT transaction layer, URC parser, and power control. |
| `onenet_client.c/.h` | OneNET property payloads, publish confirmation matching. |
| `gnss_ml307c.c/.h` | GNSS AT setup, `+MGNSSLOC` parsing, location model. |
| `rfid_store.c/.h` | Keep and adapt the existing flash-backed FIFO queue for offline RFID records. |
| `rfid_response.c/.h` | Keep the existing RFID multi-tag response parser unless the reader protocol changes. |
| `cellular_service.c/.h` | Network state machine, retry policy, LED state, upload scheduling. |

The ML307C AT command manual for TCP/MQTT/HTTP is still needed. The copied GNSS
manual covers GNSS commands, but not the full data-service and MQTT command set.

## LED Behavior Target

Keep this behavior in the new implementation:

- Green LED on: 4G/cloud link is online and data can be uploaded.
- Green LED off: 4G/cloud link is offline or not yet ready.
- Blue LED pulse: a valid RFID tag was read.

Do not let unrelated E34 sends extend or cancel the blue RFID pulse.
