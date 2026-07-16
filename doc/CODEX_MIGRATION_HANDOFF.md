# Codex Migration Handoff

Updated: 2026-07-16

Use this note when moving future work into the `ESP32S3_Adapter_ESP_IDF`
project conversation.

## Current Decision

The hardware scheme changed:

- Replace EG800K QuecPython module with ML307C core board.
- ML307C runs AT firmware.
- Stop developing `ESP32S3_Adapter_QuecPython`.
- Continue all new firmware work in `ESP32S3_Adapter_ESP_IDF`.

## Active Repository

```text
/Users/gally/Library/CloudStorage/Dropbox/Git/GitHub/ESP32S3_Adapter_V2/ESP32S3_Adapter_ESP_IDF
```

Current active branch:

```text
wireless-module
```

Current head:

```text
07978f1 feat: parse and queue multiple RFID tags per response
```

Current uncommitted state at migration time:

- `doc/` has been populated with ML307C/current-project documents.
- Build outputs and `.DS_Store` files may exist locally; do not commit them
  unless intentionally requested.

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

## Current Firmware Shape

The current `wireless-module` branch is the active implementation branch for
the 4G/RFID/cloud scheme. It already contains the old EG800K/QuecPython 4G
integration that must be converted to ML307C AT control.

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
- GNSS commands include `AT+MGNSS=1`, `AT+MGNSSLOC=1`, and `AT+MGNSSLOC`.

## Next Engineering Target

Implement ML307C support in ESP-IDF:

1. Replace or refactor `cellular_4g.c` into a UART/AT transaction layer for
   ML307C.
2. Add a cellular state machine for SIM, registration, PDP, and cloud state.
3. Add OneNET upload through ML307C AT MQTT or HTTP commands.
4. Require platform confirmation before deleting queued data.
5. Add GNSS location parsing from `+MGNSSLOC` or NMEA.
6. Reintroduce RFID multi-tag parsing and flash-backed offline queue.
7. Drive green LED from actual online state.
8. Pulse blue LED only when a valid RFID tag is read.

Missing input before full cloud implementation:

- ML307C AT command manual for network/PDP/MQTT/HTTP commands.
- Confirm final OneNET access method: MQTT over AT, HTTP over AT, or another
  module-supported transport.
