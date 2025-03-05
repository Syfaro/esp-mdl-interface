# esp-mdl-interface

A way to use an ESP32 for user device communication to faciliate the ISO
18013-5 mDL verification process.

Supports NFC engagements and BLE transmission.

## Usage

This currently only works with ESP32C6 modules. It also relies on a PN532 module
for NFC connectivity using SPI for communication.

### Wiring

|Pin   |Connection|
|------|----------|
|gpio1 |LED strip |
|gpio2 |PN532 IRQ |
|gpio18|PN532 MOSI|
|gpio20|PN532 MISO|
|gpio19|PN532 SCK |
|gpio21|PN532 SS  |
