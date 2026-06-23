# ESP32S3 Adapter ESP-IDF Firmware

ESP32-S3 firmware for the adapter board that bridges a CH9434 four-channel
UART hub with an E34-2G4D20D 2.4 GHz transparent radio module.

## Runtime Architecture

- `main/main.c`: composition root. Initializes protocol, board IO, radio
  service, and serial router.
- `main/app_config.h`: queue sizes, task stack sizes, timing constants, and
  protocol limits.
- `main/app_protocol.c`: RF application framing, CRC32, packet parsing, and
  weight-frame tail detection.
- `main/board.c`: board address switches, LEDs, and RS485 direction GPIOs.
- `main/e34_2g4d20d.c`: low-level E34 UART, AUX, mode, parameter, and reset
  driver.
- `main/radio_service.c`: E34 RX/TX tasks, packet queueing, command dispatch,
  and periodic E34 reset isolation.
- `main/ch9434.c`: CH9434 SPI register and UART helper driver.
- `main/serial_router.c`: CH9434 polling/interrupt draining, channel routing,
  RS485 direction control, and frame accumulation.

## Data Flows

1. CH9434 UART0 receives instrument/table data. The serial router accumulates
   bytes until a known complete frame tail is seen, then sends a CRC-protected
   `APP_FRAME_TYPE_WEIGHT` packet through the E34 radio service. The green LED
   pulses only after the frame is queued for radio TX.
2. E34 receives external commands as CRC-protected
   `APP_FRAME_TYPE_CHANNEL1` packets. The radio service validates the packet
   and queues the payload for CH9434 UART1. UART1 responses are accumulated by
   idle timeout and sent back through E34 as `APP_FRAME_TYPE_CHANNEL1`. The
   blue LED pulses only after the response is queued for radio TX.

This removes the old timing dependency where DEBUG logging accidentally delayed
FIFO reads long enough to make partial serial chunks look like complete frames.

## Build

The project is currently built with ESP-IDF 5.5.4 for `esp32s3`.

```sh
idf.py build
```

The default log level is intentionally set to INFO in `sdkconfig` so the
firmware is built in the same mode as the field failure:

```text
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_MAXIMUM_LEVEL=3
```

## Host Tests

Run the lightweight host-side protocol tests with:

```sh
test/run_host_tests.sh
```

These tests cover CRC32, packet build/parse, bad CRC rejection, stream packet
discovery, address/type rejection, and the supported UART0 frame tails.

## Hardware Validation

After connecting the ESP32-S3 serial port:

```sh
idf.py -p /dev/tty.usbmodem101 flash monitor
```

Use the actual `/dev/tty.*` or `/dev/cu.*` device name shown by macOS if it is
different.

Validation checklist:

- Boot log reaches `radio service started` and `serial router started`.
- With `sdkconfig` at INFO level, CH9434 UART0 input completes a known table
  frame and the green LED pulses.
- The RF receiver receives the UART0 frame.
- An external RF command reaches CH9434 UART1.
- CH9434 UART1 feedback is returned through RF and the blue LED pulses.

Do not treat a successful build alone as proof of hardware-level data-flow
completion.
