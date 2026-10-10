# Decisions

Decisions that shape the code, and why. Add one when an open decision (`D#` in the plan) is made, or when a design choice would surprise someone reading the code.

Format: `## <date>: <decision>`, then context, the choice, and the alternatives that were rejected.

## 2026-10-10: Agent docs live in `docs/`, planning in a gitignored `plan/`
Code layout is found by reading the code, so there is no separate codemap. `docs/` holds what the code can't tell you: contracts, formats, state machines, hardware and decisions.

## 2026-10-10: HW revision is a Kconfig choice that includes v1.1
`main/Kconfig.projbuild` offers V1_1, V1_2 (default) and V2. The task text names only V1_2 and V2, but v1.1 boards exist and run the same firmware, so dropping its pin set would break them. V2 has no pin set; selecting it is a build `#error`, not invented pins.

## 2026-10-10: Partition table and sdkconfig baseline (P0.7)
16 MB flash: `ota_0` and `ota_1` are 2 MB each (the app is 1.2 MB), `www` 1 MB, `coredump` 256 KB, laid out once so OTA-3 needs no table change. `sdkconfig.defaults` is the config source of truth, generated from the previous `sdkconfig` with `idf.py save-defconfig`, plus the P0.7 debug options. `PANIC_PRINT_HALT` was replaced by `PANIC_PRINT_REBOOT`: an unattended device must reboot, not halt.

## 2026-10-10: Static analysis setup (P0.2)
`tools/clang-tidy.sh` runs `run-clang-tidy` from esp-clang itself instead of `idf.py clang-check`. pyclang 0.6.3 leaves `-mdisable-hardware-atomics` in the command line, clang rejects it and skips those files, and its default `-checks=` would override `.clang-tidy`. cppcheck runs with `-D__cplusplus=201703L` because 2.17 cannot preprocess IDF's `__VA_OPT__` log macros. Vendored `tinyexpr` is excluded from clang-tidy and cppcheck, and builds with `-Wno-array-bounds` (known GCC false positive on its allocation trick) but otherwise `-Wall -Wextra -Werror`.

## 2026-10-10: v1.1 stays in the HW choice; v2 is an `#error` by design
Decided by the user. `CONFIG_SNIFFER_HW_REV_V2` exists so the choice is complete, but selecting it fails the build with a clear `#error` until the v2 pin map is known. No guessed pins.

## 2026-10-10: cert-err33-c is on
Write paths check `fflush`, `fsync` and `fclose` and return `ESP_FAIL` (`SDCard::write_file`, `create_file`). Read-only `fclose` and sized `snprintf` are cast to `(void)` with the reason in a comment where it isn't obvious.

## 2026-10-10: Parsing code lives in hardware-free files, tested on the host
`obd2_frame.cpp` (single frame, ISO-TP first and consecutive frames, DTC and VIN extraction) and `pid_json.cpp` (PID JSON) hold the logic the OBD2 class and the middleware used to inline. The OBD2 class calls them; host tests (`test/host/`) cover them without CAN hardware. Reassembled ISO-TP payloads are capped at 512 bytes, and `CONFIG_CJSON_NESTING_LIMIT` is 32 (the default 1000 overflowed a small task stack in a host test).

