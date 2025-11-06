# ESP32 CAN Sniffer

An open-source vehicle diagnostics and data logging project based on the ESP32-S3 and SN65HVD230 CAN transceiver.  
This project enables real-time vehicle data monitoring, OBD-II diagnostics, and fault code management through a web interface hosted on the ESP.

![Render Image](https://github.com/bladecell/CAN-Sniffer/blob/04f474c9713baed74a16c7adc6aa91a1fa6a33a9/hw/img/can-sniffer.png?raw=true)

---

## Hardware Overview

- MCU: ESP32-S3
- CAN Transceiver: SN65HVD230
- Power: 12V automotive input, regulated to 5V/3.3V
- PCB: Custom-designed 2-layer board (tested and verified)

Schematics and PCB files are available in the `hw/` directory.

---

## Features

- Real-time CAN bus data logging  
- OBD-II communication (PID requests)  
- Live web dashboard hosted on the ESP  
- Display of:
  - Live OBDII data
  - Diagnostic Trouble Codes (DTCs)
- Clear DTCs through the web interface  
- JSON API for external applications  
- Persistent logging to SD card

---

## Roadmap

| Stage | Task                                             | Status      |
| :---- | :----------------------------------------------- | :---------- |
| 1     | Design and fabricate PCB                         | Completed   |
| 2     | Test hardware and verify CAN transceiver         | Completed   |
| 3     | Implement ESP32 TWAI CAN driver                  | Completed   |
| 4     | Implement OBD-II driver for PID parsing          | In Progress |
| 5     | Add DTC read/clear functionality                 | In Progress |
| 6     | Create web server backend and API                | Planned     |
| 7     | Define REST/JSON API for frontend                | Planned     |
| 8     | Create Svelte frontend project (layout, routing) | Planned     |
| 9     | Add data logging to SD card                      | Planned     |
| 10    | Optimize for performance and stability           | Planned     |

---

## Web Dashboard (Planned)

The onboard web interface will display:
- Real-time vehicle parameters
- Diagnostic Trouble Codes
- Buttons to clear DTCs
- Graphs for engine data over time

Planned technologies:
- HTML5 and JavaScript frontend  
- WebSocket and REST API backend  
- JSON-based communication

## Tools and Libraries

-   [ESP-IDF Core for ESP32](https://idf.espressif.com)   
-   [TWAI (CAN) Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/twai.html)
