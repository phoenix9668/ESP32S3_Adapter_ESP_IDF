# ESP32S3 Adapter V2 Documentation

Updated: 2026-07-16

This directory is now the documentation home for the active firmware project:
`ESP32S3_Adapter_ESP_IDF`.

The hardware plan has changed from an EG800K QuecPython module to an ML307C
core board running AT firmware. New development should happen in this ESP-IDF
repository only. The previous QuecPython project is retained as legacy
reference material, not as an active firmware target.

## Current Scheme

- ESP32-S3 remains the main controller.
- ML307C replaces the previous EG800K module.
- ML307C is controlled by ESP32-S3 through AT commands.
- QuecPython code is no longer part of the runtime design.
- OneNET upload, GNSS acquisition, RFID offline storage, and network status
  handling should be implemented in ESP-IDF.

## Primary Documents

| File | Purpose |
|---|---|
| `PROJECT_CONTEXT_ML307C.md` | Current ML307C/AT project context and implementation direction. |
| `CODEX_MIGRATION_HANDOFF.md` | Short handoff note for moving Codex work into this ESP-IDF project. |
| `ML307C_硬件设计手册_GNSS_AT版本适用.pdf` | ML307C module hardware design reference. |
| `LLMM307R核心板规格书.pdf` | ML307R/ML307C-compatible mini core-board pinout and interface reference. |
| `GNSS用户手册_4G系列.pdf` | GNSS AT commands and GNSS location/NMEA reporting format. |
| `SCH_ESP32_Adapter_V2_2026-05-25.pdf` | ESP32 Adapter V2 board schematic. |

## Legacy Documents

Files under `legacy/` describe the old EG800K + QuecPython scheme. Keep them
only for design lessons, property-model history, and old hardware reference.

Important distinction:

- Do reuse the delivery semantics learned there: wait for platform confirmation
  before deleting queued RFID/GNSS data.
- Do not reuse the QuecPython UART status protocol as the final ML307C design,
  because ML307C runs AT firmware and cannot execute the old Python protocol
  peer.

