# Hardware

Pick the board in menuconfig (`CAN Sniffer` > `Hardware revision`) or in `sdkconfig.defaults`: `CONFIG_SNIFFER_HW_REV_V1_1`, `CONFIG_SNIFFER_HW_REV_V1_2` (default, the current board) or `CONFIG_SNIFFER_HW_REV_V2`. `sw/firmware/main/include/utilities.h` maps the choice to a pin set. v1.1 and v1.2 differ only in a few pin assignments, so the same firmware runs on both once the choice matches the board. To flash a board that is not the default, build with an untracked overlay instead of editing the tracked files: `idf.py -C sw/firmware -B build_v11 -D SDKCONFIG=build_v11/sdkconfig -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;/path/hw.defaults" -p /dev/ttyACM0 flash`, where `hw.defaults` holds `CONFIG_SNIFFER_HW_REV_V1_1=y`. A v1.1 board running the V1_2 build reads the wrong card-detect pin and reports "SD Card removed". v2 has no pin set yet: selecting it stops the build with `#error`.

## Revisions
| Feature | v1.2 | v2 |
|---|---|---|
| ESP32-S3 | ✓ | ✓ |
| CAN transceiver | ✓ | ✓ |
| SD card | ✓ | ✓ |
| IMU (motion wake) | ✓ | ✓ |
| Ignition sense (optocoupler, OBD pin 1, VAG) | – | ✓ |
| LTE modem with GNSS (UART, AT) | – | ✓ |
| PSRAM | 8 MB octal (seen in the boot log) | TODO |
| Flash size | 16 MB (`esptool flash_id`) | TODO |

## Pinout v1.2
| Signal | GPIO | Notes |
|---|---|---|
| CAN TX | 5 | |
| CAN RX | 4 | |
| CAN RS (transceiver slope/standby) | 6 | v1.1: 7 |
| CAN loopback (LBK) | 7 | v1.1: 6 |
| SD (SPI) | MISO 12, MOSI 10, SCLK 11, CS 9 | v1.1: MISO 10, MOSI 12, SCLK 11, CS 13 |
| SD detect | 13 | v1.1: 9 |
| LED | 47 | |
| IMU (I²C / SPI) | TODO | |
| IMU interrupt | TODO | must be an RTC GPIO for deep-sleep wake |
| OBD voltage sense (ADC) | TODO | divider ratio: TODO |

## Pinout v2
_To be added._
