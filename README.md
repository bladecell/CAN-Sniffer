# ESP32 CAN Sniffer

An open-source OBD-II diagnostics and data logger based on the ESP32-S3. It reads PIDs, DTCs and the VIN from the car's CAN bus and serves a web dashboard from the device.

![Render Image](https://github.com/bladecell/CAN-Sniffer/raw/main/hw/img/can-sniffer.png)

## Hardware

- ESP32-S3 with a CAN transceiver, 12 V automotive input, SD card slot
- HW v1.2 is the current board. HW v2 (LTE modem, GNSS, MQTT, ignition sense) is planned.
- Schematics and PCB files are in `hw/`.

## Features (working)

- CAN (TWAI) driver and OBD-II: live PIDs, DTCs, VIN, PID definitions from JSON on the SD card
- Web dashboard (Svelte) over REST and WebSocket, API in `docs/api.md`
- PIN login: the first user claims the device with a BOOT press
- Wi-Fi station mode, with a fallback access point when the network is missing
- Battery-aware sleep and wake: sleeps after inactivity, wakes only when the bus is alive, power settings kept in flash, manual sleep from the API
- Failure handling: a failed Wi-Fi, web server or SD card disables only that feature; a critical failure backs off into deep sleep instead of a reboot loop

## Plan

| Phase | Goal                                                 | State       |
| :---- | :--------------------------------------------------- | :---------- |
| P0-P1 | Tooling, quality gate, verify existing features      | Done        |
| P2    | Robustness, power, security, API cleanup             | In progress |
| P3    | Trip recorder on the SD card, per-component settings | Next        |
| P4    | ECU scan and CAN bus view                            | Planned     |
| P5    | HW v2: modem, GNSS, MQTT                             | Planned     |
| P6    | OTA updates                                          | Planned     |

## Build

Needs ESP-IDF 6.1 and bun. From the repo root:

```sh
tools/ci.sh                                  # full gate: frontend, firmware, static checks, host tests
idf.py -C sw/firmware -p /dev/ttyACM0 flash  # flash
```

Docs are in `docs/`: architecture, API, power, hardware, SD card format and decisions.
