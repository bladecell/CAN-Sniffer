# OTA updates (planned)

Requirements: OTA-1…9, UI-OTA-1/2. Open decisions: D8 (signing), D9 (server, trigger, frontend packaging).

## Partition layout
`sw/firmware/partitions.csv`, 16 MB flash (checked on the board with `esptool flash_id`). No factory partition; the bootloader falls back between `ota_0` and `ota_1`. App at P0.7: 1.2 MB, so each slot has about 43% headroom.

| Name | Type | Subtype | Offset | Size | Notes |
|---|---|---|---|---|---|
| nvs | data | nvs | 0x9000 | 24K | |
| otadata | data | ota | 0xf000 | 8K | |
| phy_init | data | phy | 0x11000 | 4K | |
| ota_0 | app | ota_0 | 0x20000 | 2M | |
| ota_1 | app | ota_1 | 0x220000 | 2M | |
| www | data | spiffs | 0x420000 | 1M | Frontend, LittleFS, flashed with the app image build. Whether OTA also updates it is D9 |
| coredump | data | coredump | 0x520000 | 256K | ELF format; a dump is about 50 KB |

0x620000 to 0x1000000 is unallocated. Changing the table moves partitions: reflash everything (`www` is rewritten by the normal flash). `nvs` keeps its offset.

## Update flow
1. Check that it's safe: no trip recording, supply voltage above TODO V, sleep blocked (OTA-6).
2. Receive the image: web upload (v1.2) or HTTPS download through the modem (v2).
3. Write it to the inactive slot with `esp_ota_*` and verify it: project name, chip, version, signature (OTA-5).
4. Set the boot partition and reboot.
5. The new image boots in pending-verify state. It runs the self-test, then calls `esp_ota_mark_app_valid_cancel_rollback()`. If the self-test fails or the device resets first, the bootloader rolls back (OTA-4). Needs `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`.

## Self-test
TODO: what has to pass before the image is marked valid (e.g. supervisor init done, web server up, CAN driver started).

## Frontend versioning
TODO (D9).
