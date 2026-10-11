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


## 2026-10-11: The SD card is never formatted automatically
Decided by the user. `format_if_mount_failed` is false. A corrupt or foreign card stays untouched and is reported as not mounted; formatting happens only through `POST /api/v1/sd_card/format`. A boot without a card keeps running with no default PID set (B1 stays a no-card limitation by choice).

## 2026-10-11: v2 overlay, size numbers and coredump fit (R3, R5, R6)
`sdkconfig.defaults.v2` layers v2 flash, PSRAM and partitions over the v1 defaults. R5 (v1-pin scratch build with v2 keys): app 1205600 B with PERF, 1101328 B with SIZE (-8.6%); default stays PERF, not changed. R6 (estimate, no forced panic): the ELF dump stores each task's used stack (SP to top), TCB and registers, not the whole 8K. About 22 tasks (6 async workers of 8K, httpd 8K, OBD2 poll and receive 8K, main 8K, ...) total about 127K of stack if dumped whole, which would not fit 64K. By the measured high-water marks the used stacks sum to about 40K, plus about 0.7K per task of headers: about 55K, against a 50 KB dump measured at P0.7. It fits with thin margin and not with a guarantee; a dump that does not fit is dropped, not a crash. `CONFIG_ESP_COREDUMP_STACK_SIZE` is the handler's own stack, not a cap. `CONFIG_ESP_COREDUMP_DATA_FORMAT_BIN` still dumps the same stacks, so it only trims header overhead (unmeasured) and loses ELF/GDB tooling. Real levers: fewer async workers (6 to 3 saves about 11K) or `ESP_COREDUMP_MAX_TASKS_NUM`.
