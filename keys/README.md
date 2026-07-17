# OTA signing key

Generate `ota_signing_key.pem` with `tools/generate_signing_key.sh`. The PEM is
ignored by Git. Back it up offline: devices that trust the corresponding public
key cannot install future OTA releases if this private key is lost.

Do not replace the key after devices have been shipped. This project enables
signed OTA validation but intentionally does not burn Secure Boot or Flash
Encryption eFuses.
