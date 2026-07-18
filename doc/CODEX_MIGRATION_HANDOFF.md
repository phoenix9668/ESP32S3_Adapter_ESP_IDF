# Codex Migration Handoff

Updated: 2026-07-17

Use this note when moving future work into the `ESP32S3_Adapter_ESP_IDF`
project conversation.

## Current Decision and Status

The hardware scheme changed:

- Replace EG800K QuecPython module with ML307C core board.
- ML307C runs AT firmware.
- Stop developing `ESP32S3_Adapter_QuecPython`.
- Continue all new firmware work in `ESP32S3_Adapter_ESP_IDF`.
- H2.3/BAT and H2.4/EN remain physically disconnected.
- The first ML307C/OneNET/GNSS single-firmware implementation is complete on
  `wireless-module`; remaining work is hardware provisioning and integration
  testing.

## Active Repository

```text
/Users/gally/Library/CloudStorage/Dropbox/Git/GitHub/ESP32S3_Adapter_V2/ESP32S3_Adapter_ESP_IDF
```

Current active branch:

```text
wireless-module
```

The repository now ignores generated ESP-IDF configuration exports, build
outputs, local OneNET credentials, and `.DS_Store` files.

## Documentation Now Lives Here

Start with:

```text
doc/README.md
doc/PROJECT_CONTEXT_ML307C.md
```

Key source documents copied into `doc/`:

- `ML307C_硬件设计手册_GNSS_AT版本适用.pdf`
- `LLMM307R核心板规格书.pdf`
- `GNSS用户手册_4G系列.pdf`
- `SCH_ESP32_Adapter_V2_2026-05-25.pdf`

Legacy EG800K/QuecPython references are under:

```text
doc/legacy/
```

Treat those as old-design reference only.

## Pre-refactor Firmware Shape (Historical)

The following describes the EG800K baseline that was replaced and is retained
only to explain the migration:

- CH9434 UART0 receives instrument/weight frames and forwards them through E34.
- CH9434 UART1 polls the RFID reader.
- `main/rfid_response.c` parses RFID responses with multiple 12-byte tag
  records and keeps the first 8 bytes for cloud upload.
- `main/rfid_store.c` stores RFID records in flash and replays them in FIFO
  order.
- `main/cellular_4g.c` currently assumes a QuecPython peer that sends
  CRC-protected `STAT`/`EVENT`/`ACK` text frames over UART2.
- `main/cellular_4g_protocol.c` implements that old text protocol parser.
- E34 uses UART1.
- CH9434 uses SPI2.
- Existing app framing uses CRC32 in `main/app_protocol.c`.

Keep the RFID parser, flash queue, partition strategy, and ACK-driven deletion
semantics. Replace the QuecPython-specific UART2 protocol with an ML307C AT
state machine and an ESP-IDF owned OneNET upload flow.

## ML307C Facts To Keep Handy

- Core board VIN: 5-16 V.
- Core board TX/RX: documented as 3.3 V level.
- Bare ML307C UART0: 1.8 V AT command interface.
- Verify level shifting on the exact core board before direct ESP32-S3 wiring.
- ML307C VBAT: 3.4-4.5 V, typical 3.8 V.
- 4G transmit transient current can approach 2 A.
- `PWR_ON/OFF` low 2-3.5 s powers on.
- `PWR_ON/OFF` low 3.5-4 s requests shutdown.
- `RESET` low at least 300 ms resets the module.
- `NETLIGHT` can indicate registration, but the firmware should still maintain
  its own AT/network/cloud state.
- GNSS stays in continuous mode with `AT+MGNSS=1`; unsolicited output is kept
  off with `AT+MGNSSCFG="nmea/mask",0` and `AT+MGNSSLOC=0`, and the current fix
  is queried every 120 seconds with `AT+MGNSSLOC`.

## Implemented Firmware Shape

- `components/esp_ml307` and `components/uart_uhci` are vendored at recorded
  upstream commits and use fixed 115200-baud UART0 on GPIO43/44. The console
  is routed through USB Serial/JTAG, and UART1 remains dedicated to E34.
- `main/cellular_service.cc` owns modem detection, SIM/registration/PDP
  readiness, MQTT connection, exponential reconnect, GNSS/RFID scheduling,
  and the green network LED.
- `main/onenet_config.c` reads credentials from the `onenet` NVS namespace;
  `main/onenet_token.c` generates the HMAC-SHA256 token locally.
- RFID remains flash-backed FIFO and is removed only after a matching OneNET
  reply ID with `code=200`.
- `main/gnss_ml307c.c` parses `+MGNSSLOC`, including empty fields and signed
  NMEA degree-minute coordinates. Only the latest generation is retained.
- Accepted OTA tasks run in an exclusive maintenance mode. GNSS is stopped,
  RFID/CH9434 polling and serial commands are paused, and ordinary property
  uploads stay suspended across download retries. Persistent RFID records are
  retained and normal services resume only after the OTA task is gone.
- `main/app_protocol_stream.c` keeps the existing E34 frame format while
  handling fragmented, concatenated, noise-prefixed, and CRC-damaged streams.
- Host tests cover the protocol parsers, OneNET token vector, reply matching,
  GNSS coordinate cases, RFID multi-tag parsing, FIFO order, and reboot
  recovery.

## Remaining Hardware Work

1. Generate and flash the local OneNET NVS image.
2. Verify ML307C AT communication and core-board UART voltage levels.
3. Exercise registration/PDP/MQTT, RFID confirmation, and 120-second GNSS
   uploads on hardware.
4. Inject network loss, non-200 replies, reboots, and queue backlog while
   observing that RFID records remain pending until cloud confirmation.
