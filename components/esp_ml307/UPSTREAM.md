# Upstream provenance

- Project: https://github.com/78/esp-ml307
- Version: 3.6.6
- Commit: 9f2a278ac6bf4ca3d6bc5057e8fcf10097cc583f
- License: Apache-2.0 (LICENSE)

Local compatibility changes are intentionally limited to selecting a UART
port at runtime, preserving raw AT argument fields, safely parsing empty
arguments, declaring the local UART-UHCI dependency, defensive MQTT URC
bounds checking, explicitly selecting MQTT 3.1.1 before connecting, and
refusing to report the data network ready until an IP address is present.
The ML307C HTTPS path also uses a larger DMA receive-task stack, frames
`MHTTPURC content` by its declared HEX payload length when the modem inserts
an intermediate CRLF, and reports truncated/overflowed HTTP streams to the
caller so OTA can resume from its last durable checkpoint.
