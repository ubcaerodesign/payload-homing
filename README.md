# payload-homing

Firmware for the 2026 / 2027 payload (ESP32-S3).

## What you need

[arduino-cli](https://arduino.github.io/arduino-cli/latest/installation/), then the ESP32 core:

    arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
    arduino-cli core install esp32:esp32@3.3.12 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json

## Compile

From this folder:

    arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc .

## Flash

Battery off, USB in:

    arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc .
