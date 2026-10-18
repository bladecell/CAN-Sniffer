# SD card

The card is a FAT volume on the SDMMC peripheral (slot 1, 1-bit bus, max 20 MHz, `max_files` 5, allocation unit 16 KiB), mounted at `/sdcard` (`SDCard` in `components/sd_card/`, config in `SUPERVISOR::setup_sd_card`). Card detect is a GPIO that reads 0 when a card is present. The supervisor loop checks it once per second, waits 100 ms to debounce, then mounts or unmounts.

Settings are not on the card. Wi-Fi, CAN and supervisor settings are in NVS (namespace `storage`), see `GET /api/v1/settings` in `api.md`.

## Layout
```
/sdcard
├── config/              created on every mount if missing
│   ├── pid_def.json     PID config JSON (pid-config.md). Written by POST /api/v1/pid_def/save. Path is configurable
│   └── dtcs.bin         DTC description database (binary, below). Copied on by the user, never written by the firmware. Path is configurable
└── logs/                created on every mount if missing; nothing in the firmware writes to it
```
The paths of the two files are `pid_def_path` and `dtc_desc_path` in NVS. Defaults are in `sw/firmware/main/include/supervisor.hpp`. The REST file calls (`api.md`) can read, write, copy and delete any path on the card, so other files may exist.

There is no trip recording code, so no trips folder exists.

`/logs` is reserved for the recorder (REC-1).

## Behaviour without a card
No card, or a card that is unreadable, never stops the boot: the SD step succeeds even with an empty slot, and mounting is retried by the 1 s card-detect loop. Per feature:
- PID database: nothing is loaded and the database stays empty. There is no default PID set. The `OBD` init step reports `ESP_ERR_INVALID_STATE` in `component_status`. OBD2 itself runs. A PID set that was loaded earlier stays in RAM if the card is removed later.
- `POST /pid_def/save` and `/pid_def/load`: 503.
- DTC descriptions (`GET /dtc?codes=`): 503. A mounted card without `dtcs.bin` gives 404. DTC codes themselves still work, they just have no text.
- SD REST routes (`/sd_card/…`, `/system/copy_file`): 503, except `GET /sd_card/info` and `GET /system` which report `is_mounted`, `is_present`, `sd_card_detected`.
- All SD operations take one recursive mutex. A caller waits at most 250 ms to 5 s depending on the route, then gets 503 `SD card busy`.
- A full card shows up as a write error: upload returns 500 `Disk write failed` and removes the partial file if it created it. `pid_def/save` returns 500.
- A card that is present but fails to mount (blank, other file system, corrupt) is left untouched and reported as not mounted, because `format_if_mount_failed` is false. Format it explicitly with `POST /api/v1/sd_card/format`.
- `POST /sd_card/format` formats a mounted card and remounts it.
- The card stays mounted while the device is in the sleep state (`power.md`). It is not deinitialized before sleep.

## DTC description database
- File: `/sdcard/config/dtcs.bin` by default (`DTC_DESC_DB_PATH`).
- Format: a flat array of fixed 128-byte records, no header, no index, no checksum, no multi-byte numbers (so no endianness).

| Offset | Size | Content |
|---|---|---|
| 0 | 6 | DTC code as ASCII, 5 characters (`P0301`) padded with NUL to 6 bytes |
| 6 | 122 | Description, ASCII, at most 121 characters, padded with NUL |

- Record count is `file size / 128`. A trailing partial record is ignored. Fewer than one record is an error (500, `No records found`).
- The records must be sorted by the code, byte-wise ascending.
- Lookup (`m_dtc_description_get` in `middleware.cpp`): binary search over the records. For each probe it reads the first 6 bytes, compares the first 5 characters to the requested code (`strcmp`), and on a match reads the description. Codes must be 5 characters, upper case, `[PBCU][0-3][0-9A-F]{3}`. The 122 description bytes are returned verbatim. A code that is not in the file gives `"Description not found"`.
- Built by `sw/firmware/csv_to_bin.py` from `sw/firmware/obd-trouble-codes.csv` (two columns: code, description). Run it from `sw/firmware/`. It strips whitespace, sorts by code, drops non-ASCII characters and cuts descriptions to 121 characters. The result in the repo, `sw/firmware/dtcs.bin`, has 3071 records (393088 bytes). Copy it to `/config/dtcs.bin` on the card, for example with `POST /api/v1/sd_card/file/config/dtcs.bin`.
- The file is read per request with a lock held for the whole lookup, so a lookup of 30 codes holds the SD mutex for the full time.

## PID config file
JSON array, written as one line without whitespace by `save_pid_def_to_json`, with keys `id, mode, pid, length, name, unit, desc, formula, minV, maxV, priority, interval, color, icon`. Format and validation are in `pid-config.md`. The write is crash-safe (`atomic_file.hpp`): the data goes to `<path>.tmp`, is flushed and fsynced, then `<path>` is renamed to `<path>.bak`, the tmp file to `<path>`, and the bak file deleted. A failed write or rename keeps the previous file. After a power cut between the two renames `<path>` is missing for a moment: the next load (`atomic_file::recover`) restores it from `.tmp` (when `.bak` exists, which proves the tmp write had finished) or from `.bak`. A leftover `.tmp` or `.bak` next to the PID file is therefore normal after a power cut and can be deleted. Paths longer than 255 bytes are refused.

## Trip CSV (planned, REC-1)
Not implemented.

TBD: trip CSV file naming and columns. Unbuilt; trip start and end are decision D3.
