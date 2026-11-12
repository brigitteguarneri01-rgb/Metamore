# Metamore — prototype firmware

Firmware of the mid-fidelity prototype of Metamore, an affective wearable for deafblind people, developed for an MSc thesis in Digital and Interaction Design at Politecnico di Milano (2026).

This is the version used in the user test (configuration A): the ESP32 is powered by the laptop USB, the loads by an external 5 V adapter through the Disconnect button.

**Board:** NodeMCU ESP-32S (38 pins)

| Pin | Function |
|---|---|
| GPIO 13 / 14 | heating bands A / B (via MOSFET module) |
| GPIO 16 / 17 | pump / exhaust valve (via MOSFET module) |
| GPIO 4 | two DS18B20 temperature sensors (OneWire) |
| GPIO 34 | slide potentiometer (session control) |
| GPIO 21-22 / 18-19 | DRV2605L haptic drivers, left / right (two I2C buses) |
| GPIO 25 / 26 | indicator LEDs |

Comments in the code are in Italian.

This is a research prototype, not a device ready for use: it was only run under supervision and has no certification or clinical validation.
