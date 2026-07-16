# Project Context Handoff

Updated: 2026-06-08

## Workspace Layout

```text
ESP32S3_Adapter_V2/
|-- AGENTS.md
|-- docs/
|   `-- PROJECT_CONTEXT.md
|-- protocol/
|   `-- 4g_esp32_uart_protocol.md
|-- ESP32S3_Adapter_QuecPython/
`-- ESP32S3_Adapter_ESP_IDF/
```

The child directories are independent Git repositories. The parent directory
is a coordination workspace, not a replacement monorepo.

## Hardware

- 4G/GNSS module: Quectel EG800K-CNGC core board.
- Controller: ESP32-S3.
- Adapter schematic:
  `ESP32S3_Adapter_QuecPython/doc/SCH_ESP32_Adapter_V2_2026-05-25.pdf`.
- UART connection:
  - EG800K UART2 TX -> ESP32-S3 UART2 RX.
  - ESP32-S3 UART2 TX -> EG800K UART2 RX.
  - 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control.
- ESP32-S3 controls the 4G module/core-board EN signal with GPIO44.

### Resolved Restart Issue

The repeated EG800K restarts were caused by the core-board input supply, not by
ESP32-S3 EN control or a Python exception.

- The core board accepts VIN from 5 V to 16 V and recommends 12 V.
- The adapter supplied exactly 5 V, which was at the lower input boundary.
- Cellular transmit current caused the core-board internal VIN-to-VBAT rail to
  become unstable.
- QuecPython diagnostics reported `power_down=3`, interpreted as undervoltage.
- Oscilloscope measurements at the adapter H2 `+5V` pin remained stable because
  this is the upstream VIN input, not the EG800K VBAT rail.
- Supplying the core board with 12 V resolved the spontaneous restarts.

Do not reintroduce a 5 V-only assumption for the core-board VIN supply.

## QuecPython Repository

Repository:
`ESP32S3_Adapter_QuecPython`

Entry point:
`src/main.py`

Current application version:
`1.2.0`

Current head at handoff:

```text
b8279da feat: implement UART status protocol v1
```

Important earlier commits:

```text
8787716 Add ESP32S3 UART heartbeat
f4ecbf1 Forward RFID tags to OneNET
3b49ca9 Upload SIM info on module startup
dd250a2 Fix OneNET GNSS property upload
6227979 Implement GNSS OneNET uploader
```

### Implemented QuecPython Features

- Wait for cellular network registration.
- Initialize and read the internal GNSS with `quecgnss`.
- Parse NMEA GGA, RMC, and VTG information.
- Connect to OneNET over MQTT/OneJSON.
- Upload GNSS properties every 120 seconds.
- Upload SIM information once after MQTT connection on each boot.
- Receive RFID tag text from ESP32-S3 on UART2 and upload `rfid_tag_id`.
- Send protocol-v1 `STAT` frames every 10 seconds with CRC16-CCITT-FALSE.
- Send immediate `EVENT` frames for SIM, registration, PDP, and MQTT changes.
- Respond to valid ESP32-S3 `GET` status requests.
- Publish CRC-protected ESP32-S3 `RFID` frames and return `ACK` only after a
  matching OneNET property reply reports `code=200`.
- Re-ACK the most recently published RFID sequence without republishing it
  when an ACK retry occurs during the same EG800K boot.
- Serialize UART status/event writes and RFID reads with a lock.
- Log power-on reason, power-down reason, VBAT, and heap information.

### MQTT Runtime Decisions

- MQTT port: 1883, non-TLS.
- MQTT keepalive: 300 seconds.
- Property upload interval: 120 seconds.
- `umqtt` internal reconnect: disabled.
- Main loop owns disconnect/reconnect recovery.
- Keepalive must not be set to zero on the current EG800K firmware; doing so
  causes `MQTT Connect error='bytes index out of range'`.

### OneNET Configuration

- Product name: `wireless-module`.
- Device name: `eg800k-gnss-01`.
- Product ID and MQTT host are already configured in `src/main.py`.
- Device credentials are intentionally omitted from this document.
- MQTT property post topic:
  `$sys/{product_id}/{device_name}/thing/property/post`.

Configured or used property identifiers:

| Identifier | Type | Purpose |
|---|---|---|
| `fix_status` | int32 | GNSS fix valid: 0 or 1 |
| `latitude` | double | Latitude, -90 to 90 |
| `longitude` | double | Longitude, -180 to 180 |
| `altitude` | double | Altitude in metres |
| `speed_kph` | double | Ground speed in km/h |
| `course_deg` | double | Course in degrees |
| `satellites` | int32 | Satellites used/visible |
| `hdop` | double | Horizontal dilution |
| `utc_time` | string | GNSS UTC timestamp |
| `sim_status` | int32 | SIM status |
| `sim_csq` | int32 | Signal quality |
| `sim_iccid` | string | SIM ICCID |
| `sim_imsi` | string | SIM IMSI |
| `sim_phone_number` | string | Optional phone number |
| `rfid_tag_id` | string | RFID tag forwarded by ESP32-S3 |

Only properties that exist in the OneNET product model should be included in
uploads.

## ESP-IDF Repository

Repository:
`ESP32S3_Adapter_ESP_IDF`

Branch:
`wireless-module`

Target and toolchain:

- ESP32-S3.
- ESP-IDF 5.5.4.
- 16 MB external flash.
- Custom 16 MB partition table:
  - two 4 MB OTA application slots;
  - about 7.875 MB dedicated `rfid_store` data partition;
  - standard NVS, OTA metadata, and PHY partitions.

Current head at handoff:

```text
4d1eb5b feat: implement 4G status protocol and LED indicators
```

Important earlier commits:

```text
dde9241 fix: 避免 4G 心跳格式差异触发误重启
b5c6851 fix: 整理 RFID 标签上报格式
fb38359 fix: 修复 CH9434 轮询发送与 4G 心跳监控
a8fe306 feat: 增加 RFID 轮询和 4G 转发
b6fff0e refactor: 重构 ESP32 数据转发架构
```

At handoff, the only uncommitted file in this child repository is `.DS_Store`.
Do not include it in future source commits.

### ESP32-S3 Application Architecture

Startup order in `main/main.c`:

1. Initialize the shared application CRC table.
2. Initialize board LEDs, RS485 direction GPIOs, and board address switches.
3. Initialize UART2 and the 4G EN GPIO.
4. Initialize the serial-router queue.
5. Start the E34 radio service.
6. Start the CH9434 serial-router task.

Primary modules:

- `main/main.c`: composition root and startup order.
- `main/app_config.h`: queue sizes, task stacks, timing, and protocol limits.
- `main/app_protocol.c`: E34 application framing and CRC32.
- `main/board.c`: address switches, LEDs, and four RS485 direction GPIOs.
- `main/ch9434.c`: SPI driver and four-channel UART hub access.
- `main/serial_router.c`: CH9434 initialization, RFID polling, response routing,
  and weight-frame collection.
- `main/radio_service.c`: E34 RX/TX tasks, queueing, command dispatch, and
  periodic E34 reset.
- `main/cellular_4g.c`: EG800K UART2, heartbeat receive watchdog, EN restart,
  RFID protocol transmission, and cloud ACK reception.
- `main/rfid_store.c`: power-loss-tolerant RFID flash queue and ordered replay
  after MQTT connectivity recovers.

### RFID Offline Queue

- Every valid RFID tag is written to the `rfid_store` flash partition before
  upload is attempted.
- Records use a fixed-size CRC32-protected circular log with a commit marker.
- Only the oldest pending record is sent to EG800K.
- ESP32-S3 deletes a record only after a matching CRC-validated ACK.
- Queue recovery scans flash at boot, so pending records survive reset and
  power loss.
- Delivery is at least once; a lost ACK can cause a duplicate cloud publish.
- When no reclaimable sector remains, the oldest records are preserved and the
  newest RFID tag is rejected with an error log.

### Hardware Interfaces

CH9434 SPI2:

| Signal | ESP32-S3 GPIO |
|---|---:|
| MISO | 41 |
| MOSI | 40 |
| CLK | 39 |
| CS | 38 |
| RST | 48 |
| INT | 42 |

CH9434 channels correspond directly to the board connector numbering 0 to 3.

- Channel 0: 9600 baud; weight/instrument input.
- Channels 1 to 3: 57600 baud.
- Channel 1 is the RFID channel.
- RS485 direction GPIOs for channels 0 to 3 are GPIO47, GPIO21, GPIO14, and
  GPIO13 respectively.
- The board connectors may expose RS232 or RS485 physical interfaces, but the
  CH9434 channel index remains the same.

E34-2G4D20D:

| Signal | ESP32-S3 resource |
|---|---|
| UART | UART1 |
| TX | GPIO6 |
| RX | GPIO7 |
| M0 | GPIO4 |
| M1 | GPIO5 |
| AUX | GPIO15 |
| Baud | 9600 |

EG800K/4G:

| Signal | ESP32-S3 resource |
|---|---|
| UART | UART2 |
| TX | GPIO2 |
| RX | GPIO1 |
| EN | GPIO44 |
| Baud | 115200, 8N1 |

Board GPIO:

- Green LED: GPIO17.
- Blue LED: GPIO16.
- Address inputs ADDR1 to ADDR4: GPIO9, GPIO3, GPIO8, GPIO18.

### Current Data Flows

Weight data:

```text
CH9434 channel 0
  -> serial_router frame accumulation
  -> app_protocol frame with CRC32
  -> radio_service TX queue
  -> E34-2G4D20D
```

Remote RFID command:

```text
E34-2G4D20D
  -> app_protocol validation
  -> radio_service RX
  -> serial_router command queue
  -> CH9434 channel 1
  -> response routed back through E34
```

Periodic RFID collection:

```text
Every 500 ms:
04 FF 01 1B B4
  -> CH9434 channel 1
  -> RFID response
  -> extract every tag and retain its first 8 bytes
  -> persist each tag as 16-character uppercase hexadecimal ASCII
  -> protocol-v1 RFID frame over UART2
  -> EG800K
  -> OneNET rfid_tag_id
```

RFID response formatting currently interprets:

- Byte offset 0 as the number of following bytes, so complete frame length is
  `data[0] + 1`.
- Byte offset 4 as the tag count.
- Byte offset 5 as the first tag record.
- Each tag record is `0x0C` followed by 12 raw tag bytes.
- The final two bytes after all tag records are RFID-frame CRC bytes.
- Each tag is queued independently for cloud upload.
- Only the first 8 bytes of each 12-byte tag are retained and converted to 16
  uppercase hexadecimal characters; the final 4 bytes are discarded.
- UART1 responses are accumulated to the declared complete length before
  parsing, so split reads and multiple frames in one read are supported.

### CH9434 Debugging History

The CH9434 driver previously produced invalid register reads, very large FIFO
lengths, and no output on board RS232 channel 1. The working implementation:

- Pulses CH9434 RST low for 1 ms and then high before initialization.
- Performs an SPI register operation as two 8-bit transfers while CS remains
  low.
- Does not combine the address and data phases into one incompatible 16-bit
  transaction.
- Waits for TX FIFO drain and applies a minimum line hold delay before changing
  RS485 direction.

Do not revert this SPI transaction shape without validating it on real
hardware.

### ESP32-S3 4G Behavior

- GPIO44 high enables the 4G core board.
- UART input is buffered through complete CRLF-delimited lines.
- Protocol-v1 `STAT` and `EVENT` frames are validated for source, version,
  required fields, ranges, and CRC16 before they refresh the watchdog.
- The latest cellular and MQTT state is retained and exposed through
  `cellular_4g_get_status()`.
- Communication is marked stale after 30 seconds without a valid frame, and
  ESP32-S3 sends a `GET` request for an immediate status report.
- After 60 seconds without a valid frame, ESP32-S3 drives EN low for 2 seconds
  and then high.
- Overlength, malformed, unsupported, and bad-CRC lines do not refresh the
  watchdog.
- RFID responses are formatted as text and forwarded to EG800K through
  `cellular_4g_send()`.

An ESP32-triggered restart can be confirmed only when ESP32 logs contain:

```text
no valid 4G frame for 60000 ms, toggling EN gpio=44
4G EN gpio=44 set low
4G EN gpio=44 set high
```

If the module reboots without the EN-low log, investigate module power or
module-side causes instead of assuming the ESP32 watchdog fired. The previously
observed frequent restart issue was resolved by supplying the core board with
12 V, as described in the hardware section above.

### E34 Application Protocol

- Header: `0xE55E`.
- Board address is included in the header.
- Payload maximum: 512 bytes.
- CRC: CRC32 over header and payload.
- Existing frame types:
  - `0x01`: weight data.
  - `0x02`: RFID command/response.

The current radio parser assumes one complete application packet is returned by
one UART read. If reliability over a fragmented or concatenated radio stream
becomes important, add an explicit stream accumulator.

### Build and Verification Notes

- The repository configuration identifies ESP-IDF 5.5.4 and target
  `esp32s3`.
- A full ESP-IDF 5.5.4 build passed after the protocol-v1 implementation.
- On this Mac, the Espressif Installer environment is under
  `~/.espressif/v5.5.4` and `~/.espressif/tools`.
- The repository `build/` directory has previously contained a CMake cache
  generated from a Windows path. If that cache reappears, use `idf.py
  fullclean` or build in a fresh directory rather than trusting it.
- Sandbox execution may block the ESP-IDF component manager when it calls
  macOS `sysctl`; run the build with the required host permission.
- After the later `sdkconfig` commit or workspace relocation, run a clean build
  before flashing production hardware.

### Remote Upgrade Analysis

No OTA code has been implemented yet.

Current constraints:

- The project still uses the single-application partition table.
- Reliable OTA requires at least `otadata`, `ota_0`, and `ota_1`, plus rollback
  handling.
- Existing deployed devices cannot safely gain an A/B partition layout through
  a normal application-only OTA; the initial migration should update the
  bootloader, partition table, and application through a controlled wired
  flash process.

Preferred options discussed:

1. If stable site WiFi is available, ESP32-S3 WiFi plus HTTPS OTA is the
   simplest implementation because ESP-IDF already provides download,
   verification, partition switching, and rollback support.
2. If WiFi cannot be guaranteed, EG800K can download the image and transfer it
   to ESP32-S3 with a reliable UART chunk protocol. This is more work but fits
   mobile or unattended cellular installations.
3. E34 firmware transfer is best treated as an optional maintenance fallback,
   not the primary upgrade channel.

If WiFi OTA is selected, account for provisioning and 2.4 GHz coexistence with
the E34 module. Enter a dedicated upgrade mode that pauses RFID polling and E34
traffic while downloading and writing firmware.

Required production safeguards:

- HTTPS certificate validation.
- Firmware signature verification.
- Hardware/product compatibility check.
- Version and downgrade policy.
- A/B application slots.
- Boot validation and automatic rollback.
- Upgrade progress and result reporting through EG800K.

## Implemented UART Protocol

Protocol version 1 in `protocol/4g_esp32_uart_protocol.md` is implemented in
both child repositories.

Required behavior:

- EG800K emits `STAT` every 10 seconds.
- EG800K emits `EVENT` immediately on cellular, PDP, or MQTT state changes.
- ESP32-S3 can distinguish:
  - module alive but still booting;
  - SIM unavailable;
  - cellular network searching or rejected;
  - cellular registered but PDP unavailable;
  - data network available but OneNET disconnected;
  - OneNET online.
- ESP32-S3 detects EG800K reboot using `UP` decreasing or `SEQ` restarting.
- ESP32-S3 regards any valid `STAT` or `EVENT` frame as communication
  heartbeat.
- After 30 seconds without a valid frame, state becomes stale.
- After 60 seconds without a valid frame, ESP32-S3 restarts the 4G module via
  EN.

## Implementation Notes

### EG800K Side

- One shared status structure tracks SIM, registration, PDP, MQTT, CSQ, RAT,
  uptime, publish age, and cause.
- Status/event UART writes and RFID reads share the existing UART lock.
- The status worker only encodes and transmits snapshots; modem and MQTT
  operations remain in the main application flow.
- `dataCall.setCallback()` provides immediate PDP events when supported, with
  polling as a fallback.
- Valid ESP `GET` requests are separated from RFID text before upload.

### ESP32-S3 Side

- `cellular_4g_protocol.c` validates bounded complete lines independently from
  the UART/watchdog task.
- `cellular_4g_get_status()` exposes the latest decoded state through a
  critical-section-protected snapshot.
- Legacy `EG800K_HEARTBEAT` text is no longer accepted.

## Suggested Verification

1. Build and flash both child projects independently.
2. Confirm valid `STAT` frames arrive every approximately 10 seconds.
3. Remove the SIM and confirm state transitions to SIM unavailable.
4. Disable or shield the antenna briefly and confirm searching/registration
   transitions.
5. Make the MQTT endpoint unreachable and confirm PDP remains available while
   MQTT becomes disconnected.
6. Restore connectivity and confirm an immediate event plus a later status
   frame.
7. Stop EG800K UART output and confirm stale state at 30 seconds and EN restart
   at 60 seconds.
8. Send malformed frames and bad CRC values; they must not refresh the
   watchdog.
9. Verify RFID forwarding and OneNET upload still work.
