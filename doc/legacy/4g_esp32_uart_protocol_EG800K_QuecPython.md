# EG800K to ESP32-S3 UART Status Protocol

Protocol version: 1

Status: Implemented in both firmware repositories

## Purpose

This protocol replaces the legacy `EG800K_HEARTBEAT` message with a framed
status heartbeat. It lets ESP32-S3 distinguish module liveness, cellular
registration, packet-data availability, and OneNET MQTT connectivity.

## Transport

- UART2.
- 115200 baud.
- 8 data bits.
- No parity.
- 1 stop bit.
- No hardware flow control.
- ASCII text.
- Every frame ends with `\r\n`.
- Maximum complete frame length: 192 bytes including CRC and CRLF.

## Frame Syntax

```text
$<SOURCE>,V=<version>,T=<type>,<fields>*<crc16>\r\n
```

Sources:

- `Q4G`: EG800K/QuecPython.
- `ESP`: ESP32-S3.

Rules:

- Field names are uppercase ASCII.
- Numeric values use decimal notation.
- Fields are comma-separated and must not contain commas, `*`, CR, or LF.
- Unknown optional fields may be ignored.
- Unknown versions or message types must be rejected.
- Required fields must be present exactly once.

## CRC

- Algorithm: CRC16-CCITT-FALSE.
- Polynomial: `0x1021`.
- Initial value: `0xFFFF`.
- Reflect input: false.
- Reflect output: false.
- Final XOR: `0x0000`.
- Encoded as four uppercase hexadecimal characters.
- CRC input starts with the `$` character and ends immediately before `*`.
- CRLF and the `*` separator are not included.

Example CRC input:

```text
$Q4G,V=1,T=STAT,SEQ=42,UP=187,STATE=5,SIM=1,REG=2,PDP=1,MQTT=2,CSQ=29,RAT=LTE,LASTTX=8,CAUSE=0
```

The example frames below contain verified CRC values.

## `STAT`: Periodic Status

EG800K sends a `STAT` frame every 10 seconds.

```text
$Q4G,V=1,T=STAT,SEQ=42,UP=187,STATE=5,SIM=1,REG=2,PDP=1,MQTT=2,CSQ=29,RAT=LTE,LASTTX=8,CAUSE=0*AD3C\r\n
```

Required fields:

| Field | Meaning |
|---|---|
| `V` | Protocol version, currently `1` |
| `T` | `STAT` |
| `SEQ` | Unsigned sequence counter; wraps after implementation-defined maximum |
| `UP` | Seconds since current EG800K application boot |
| `STATE` | Summary state |
| `SIM` | SIM state |
| `REG` | Cellular registration state |
| `PDP` | Packet-data context state |
| `MQTT` | OneNET MQTT state |
| `CSQ` | Signal quality, 0-31 or 99 when unknown |
| `RAT` | Radio access technology |
| `LASTTX` | Seconds since last successful OneNET property publish, or -1 |
| `CAUSE` | Current reason/error code |

### Summary State

| Value | Meaning |
|---:|---|
| 0 | Module/application booting |
| 1 | SIM unavailable |
| 2 | Searching for cellular network |
| 3 | Cellular registered, packet data unavailable |
| 4 | Packet data available, OneNET MQTT unavailable |
| 5 | OneNET MQTT online |
| 6 | Network or application error |

### SIM State

| Value | Meaning |
|---:|---|
| 0 | Unknown |
| 1 | Ready |
| 2 | Missing |
| 3 | Error |

### Registration State

| Value | Meaning |
|---:|---|
| 0 | Unknown/not registered |
| 1 | Searching |
| 2 | Registered on home network |
| 3 | Registered while roaming |
| 4 | Registration rejected |

### PDP State

| Value | Meaning |
|---:|---|
| 0 | Inactive |
| 1 | Active |

### MQTT State

| Value | Meaning |
|---:|---|
| 0 | Disconnected |
| 1 | Connecting |
| 2 | Connected to OneNET |

### RAT Values

- `LTE`
- `GSM`
- `UNKNOWN`

### Cause Codes

| Value | Meaning |
|---:|---|
| 0 | No current error |
| 1 | Booting |
| 2 | SIM missing |
| 3 | SIM error |
| 4 | Searching for network |
| 5 | Registration rejected |
| 6 | Packet-data context disconnected |
| 7 | DNS failure |
| 8 | MQTT connection failure |
| 9 | MQTT communication failure |
| 10 | MQTT authentication failure |
| 99 | Unknown error |

## `EVENT`: Immediate State Change

EG800K sends `EVENT` immediately when important state changes. A later `STAT`
frame still carries the complete current state.

```text
$Q4G,V=1,T=EVENT,SEQ=43,UP=190,EV=MQTT_DOWN,STATE=4,CAUSE=9*1BDC\r\n
```

Required fields:

- `V`
- `T=EVENT`
- `SEQ`
- `UP`
- `EV`
- `STATE`
- `CAUSE`

Initial event names:

- `BOOT`
- `SIM_READY`
- `SIM_LOST`
- `REG_SEARCH`
- `REG_HOME`
- `REG_ROAM`
- `REG_REJECTED`
- `PDP_UP`
- `PDP_DOWN`
- `MQTT_CONNECTING`
- `MQTT_UP`
- `MQTT_DOWN`

## `GET`: ESP32-S3 Status Query

ESP32-S3 may request an immediate status report:

```text
$ESP,V=1,T=GET,SEQ=18,WHAT=STAT*86C0\r\n
```

EG800K responds with a normal `STAT` frame using its own next sequence number.

Required fields:

- `V`
- `T=GET`
- `SEQ`
- `WHAT=STAT`

## `RFID`: Durable RFID Upload

ESP32-S3 stores every valid RFID tag in its local flash queue before sending
it to EG800K. It sends only the oldest pending record while MQTT is online:

```text
$ESP,V=1,T=RFID,SEQ=123,TAG=0102030405060708*CE92\r\n
```

Required fields:

- `V`
- `T=RFID`
- `SEQ`: persistent RFID queue sequence number
- `TAG`: uppercase hexadecimal RFID identifier, maximum 64 characters

EG800K publishes the tag to OneNET, waits for a matching property-post reply,
and returns an ACK only when the reply reports `code=200`:

```text
$Q4G,V=1,T=ACK,REF=123,RESULT=OK*04BE\r\n
```

ESP32-S3 marks the corresponding flash record delivered only after receiving
this CRC-validated ACK. Missing ACKs are retried in queue order after network
recovery. A retry may produce a duplicate cloud publish if the previous ACK
was lost, so delivery semantics are at least once.

## Timing and Watchdog Rules

- EG800K status period: 10 seconds.
- Status-period tolerance: approximately +/-2 seconds during normal operation.
- ESP32-S3 refreshes the communication watchdog only after receiving a valid
  CRC-checked `STAT` or `EVENT` frame.
- After 30 seconds without a valid status/event frame, ESP32-S3 marks 4G
  communication as stale.
- After 60 seconds without a valid status/event frame, ESP32-S3 drives 4G EN
  low for 2 seconds and then high.
- A decreasing `UP` value or restarted `SEQ` sequence indicates EG800K reboot.
- A disconnected network state is not itself a reason to restart EG800K as
  long as valid status frames continue to arrive.

## Parser Requirements

- Buffer input until CRLF.
- Handle frames split across multiple UART reads.
- Handle multiple frames in one UART read.
- Discard overlength frames and resume at the next CRLF boundary.
- Do not refresh the watchdog for malformed, unsupported, or bad-CRC frames.
- Keep RFID traffic from ESP32-S3 to EG800K functional while status frames are
  sent in the opposite direction.
- `ACK` frames do not refresh the 4G communication watchdog; only valid
  `STAT` and `EVENT` frames do.

## Migration

The implemented ESP32-S3 firmware no longer accepts the legacy
`EG800K_HEARTBEAT` token. Only validated protocol-v1 `STAT` and `EVENT` frames
refresh the watchdog.
