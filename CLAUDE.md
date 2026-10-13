# CAN Sniffer: instructions for agents

This is an ESP32-S3 CAN/OBD-II sniffer. The firmware runs on ESP-IDF 5.5.3 with FreeRTOS, in C and C++. It reads PIDs, DTCs and the VIN, records trips to the SD card, and serves a Svelte web frontend. HW v1.2 is the current board; HW v2 adds an LTE modem with GNSS, MQTT and ignition sense.

## Repo layout
```
/                   git root: CLAUDE.md, tools/, docs/, .claude/rules/, plan/ (gitignored)
sw/firmware/        ESP-IDF project (main/, components/, sdkconfig.defaults, partitions.csv, test/host/)
sw/frontend/        Svelte frontend, served by the firmware
```
All paths are relative to the git root. Firmware and frontend conventions are in `.claude/rules/`, and load when you work in those folders.

## Start every session here
1. Read `plan/STATUS.md` for the current phase, the next unchecked task and any blockers.
2. Read that task in `plan/todo.md`, its phase in `plan/plan.md`, and the requirements it serves in `plan/requirements.md`.
3. Read the docs the task touches (see **Docs** below). Find the code itself by reading and searching it.
4. Implement the task, then run `tools/ci.sh`, then run the task's Verify step. Only then check the box. Never check a box without running its verification.
5. Update `plan/STATUS.md`: the next task, blockers, and metrics if any were measured. When a requirement is verified met, check it in `plan/requirements.md`.
6. New work enters in this order: a requirement in `requirements.md`, then a phase in `plan.md`, then an item in `todo.md`, then code. Don't start work that isn't in `todo.md`.
7. Open decisions (`D#` in `plan/plan.md`) are the user's to make. Don't build on an undecided one. Ask, or pick other work. When one is decided, record it in `docs/decisions.md`.

`plan/` is gitignored and local only. `docs/` is committed.

## Parallel sessions
Several sessions can share this working tree. Before editing or committing, read `plan/LANES.md` (file ownership, commit rules, requests) and update your row when you start or finish. Edit only files your lane owns. Commit with `git commit -m "<task id>: ..." -- <paths>`, never `git add -A`, `git add .` or `git commit -a`. Don't `reset`, `stash` or amend commits another lane may have built on.

## Docs
Read a doc before changing what it describes. If your change alters a contract or behaviour in a doc, update the doc in the same commit. A doc that disagrees with the code is a bug: fix whichever one is wrong.
- `docs/architecture.md`: components, FreeRTOS tasks, data flow, startup order
- `docs/api.md`: REST endpoints and WebSocket messages, the contract between firmware and frontend
- `docs/pid-config.md`: the PID JSON schema, the formula language, `getPID()` and `getBit()`
- `docs/sd-card.md`: the SD card file layout, the trip CSV format and the DTC description bin format
- `docs/power.md`: the supervisor's sleep and wake state machine, wake sources and deinit order
- `docs/hardware.md`: pinouts and the differences between HW v1.2 and v2
- `docs/ota.md`: partition layout, the update flow and rollback
- `docs/decisions.md`: decisions made and why

## Never crash (outranks feature work)
The device sits in a car unattended. A reboot loop drains the battery, and a hang stops logging. Bad input must produce an error, never an abort.
- **Inputs are hostile.** That covers CAN frames (DLC, ISO-TP lengths and sequence numbers), PID JSON, formulas, HTTP and WebSocket params, files on the SD card, the DTC bin and OTA images. Bounds-check every index and length derived from them, and cap any allocation sized by input.
- **No aborts at runtime.** `ESP_ERROR_CHECK`, `assert` and `abort()` are allowed only in early init, for errors that are truly unrecoverable. Everywhere else, propagate the error with `ESP_RETURN_ON_ERROR`, `ESP_RETURN_ON_FALSE` or `ESP_GOTO_ON_ERROR` from `esp_check.h`.
- **No throwing APIs.** C++ exceptions are off, so a throw aborts. Don't use `std::stoi`, `.at()`, or `new` without `std::nothrow`. Use `strtol` or `std::from_chars`, checked indexing, and check every allocation.
- **Formulas fail safe.** A TinyExpr parse or eval error, NaN, inf or division by zero marks the PID invalid. It must never crash the poll task.
- **Degrade, don't die.** A missing or full SD card, a disconnected bus, or a failed component init means that feature is disabled and the error is reported. The rest of the device keeps running.
- **Prove it.** Every crash fix lands with a host test that reproduces the crash.

## Non-negotiables
- **Battery.** Every component supports `init()` and `deinit()`, because the supervisor tears components down before sleep. Deinit releases its drivers, ISRs, tasks and allocations, so an init/deinit cycle leaks nothing. No busy loops. Keep polling and wake time minimal.
- **Vehicle safety.** Only standard OBD requests go to a real car's bus. Raw CAN transmission, ECU scans and DTC clearing on a real vehicle need explicit approval from the user each time.
- **Layering.** The frontend reaches firmware data only through the REST API and the WebSocket (`docs/api.md`), which go through the web server middleware. Components talk through each other's public headers only.
- **One contract, both sides.** When an endpoint or message format changes, update the firmware, the frontend and `docs/api.md` in the same commit.
- **Config persistence.** PID changes made at runtime are persisted to the PID JSON on the SD card.
- **Quality gate.** `tools/ci.sh` must be green before every commit.
- **Commits.** One task id per commit, e.g. `P2.3: ISO-TP length bounds check`. Commit only green states, and commit small.

## Commands
Run commands from the git root. The IDF environment isn't in a plain shell, so source it in the same command, and point `idf.py` at the project with `-C sw/firmware`:
```sh
. ~/.esp/esp-idf/export.sh && idf.py -C sw/firmware build
```
- Quality gate: `tools/ci.sh`. It runs the frontend check and build, the firmware build (`-Werror`), the size report, cppcheck, clang-tidy and host tests.
- Host tests only: `tools/ci.sh --host`
- Set target (once): `idf.py -C sw/firmware set-target esp32s3`
- Flash: `idf.py -C sw/firmware -p /dev/ttyACM0 flash`
- Logs: `timeout 15 idf.py -C sw/firmware -p /dev/ttyACM0 monitor`. Monitor is long-running, so always wrap it.
- Size: `idf.py -C sw/firmware size`, or `size-components` to see which component grew.
- Crash dump: `idf.py -C sw/firmware -p /dev/ttyACM0 coredump-info`
- Frontend: run in `sw/frontend/` with bun. `bun install --frozen-lockfile`, `bun run dev` (long-running, not a completion check), `bun run build` (writes `dist/index.html.gz`), `bun run check` (svelte-check). No lint script.

## Verifying on hardware
- After a firmware change that affects the device's behaviour, flash it and capture the monitor output with `timeout`. Read the log for errors, the reset reason (logged at boot), the free heap and panics.
- If the reset reason is a panic or watchdog, read the core dump with `coredump-info` before changing anything.
- If no car or CAN source is connected, say so. Report bus-dependent checks as unverified rather than passing.
- Record measured numbers in `plan/STATUS.md` → Metrics: sleep current, poll rate, free heap, stack high-water marks and app size.

## Map of the code
`Supervisor` → `CAN driver` (TWAI) → `OBD2` (PID DB, TinyExpr, poll task, DTCs, VIN) → `Web server` (REST, WebSocket, middleware) → Svelte frontend.
Planned: `Recorder` on top of OBD2, `OTA`, and `Modem` (v2) for AT, GNSS, time and MQTT. Details are in `docs/architecture.md`.
