# CAN Sniffer: instructions for agents

This is an ESP32-S3 CAN/OBD-II sniffer. The firmware runs on ESP-IDF 6.1 (Picolibc, GCC 15) with FreeRTOS, in C and C++. It reads PIDs, DTCs and the VIN, records trips to the SD card, and serves a Svelte web frontend. HW v1.2 is the current board; HW v2 adds an LTE modem with GNSS, MQTT and ignition sense.

## Repo layout
```
/                   git root: CLAUDE.md, tools/, docs/, .claude/rules/, plan/ (gitignored)
sw/firmware/        ESP-IDF project (main/, components/, sdkconfig.defaults, partitions.csv, test/host/)
sw/frontend/        Svelte frontend, served by the firmware
```
All paths are relative to the git root. Firmware and frontend conventions are in `.claude/rules/`, and load when you work in those folders.

## Start every session here
1. Task state lives on the board (see **Board workflow** below), not in `plan/todo.md` or `plan/STATUS.md` (both kept as backup, no longer updated for task state). Run `board list in-progress` and `board list todo` for the current and next task, and `board show <id>` for its detail and How to verify. Blockers are the blocked flag on a card.
2. Read the task's phase in `plan/plan.md` and the requirements it serves in `plan/requirements.md`.
3. Read the docs the task touches (see **Docs** below). Find the code itself by reading and searching it.
4. Implement the task, then run `tools/ci.sh`, then run the card's How to verify steps. Only then move the card to verify with `board move <id> verify --notes "what ran + result"`. Never move a card without running its verification.
5. Measured metrics still go in `plan/STATUS.md` -> Metrics. When a requirement is verified met, check it in `plan/requirements.md`.
6. New work enters in this order: a requirement in `requirements.md`, then a phase in `plan.md`, then a card on the board (`board add`, one per plan step, created as soon as the plan exists), then code. Don't start work that isn't a card in todo.
7. Open decisions (`D#` in `plan/plan.md`) are the user's to make. Don't build on an undecided one. Don't ask in chat: put the question on the board as a decision card with options (see **Board workflow**), then pick other work. When one is decided, record it in `docs/decisions.md`.

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

## Frontend UI rules
UI work follows `plan/DESIGN.md`. Read its Surfaces table, "Blur variables and in-flow tiers" and "Reuse before you write CSS" before writing any style.
- **Reuse.** Use the shared classes (`.card`, `.btn`, `.in`, `.seg`, `.chip`, `.float-layer`, `.popup`) and components (`Dialog`, `Sheet`, `Toast`, `SelectMenu`, `RangeSlider`, `SettingRow`, `MasterDetail`). No second dropdown, dialog, sheet, toast, slider or view-prefixed copy of a shared class. If you add a token, class or component, add it to DESIGN.md in the same commit.
- **Tokens.** Fills come from the Surfaces table, radii from `--r-*`, sizes from `--ctl-h` and `--pad-card`. No literal colours, `color-mix` fills or one-off radii.
- **Blur.** Three tokens only: `--blur` (user setting), `--float-blur` (float layers), `--card-blur` (in-flow surfaces; `none` on the plain background, blurred over a generated one). Never write your own `blur()`. Every in-flow fill that sits directly on the page (`--card`, or `--row`/`--card2`/`--ctl` on a top-level panel or button) is followed by `-webkit-backdrop-filter: var(--card-blur); backdrop-filter: var(--card-blur);`.
- **No nesting.** A blurred box never contains another blurred box (Firefox drops the blur). Rows, chips and buttons inside a blurred card get no blur. A panel holding blurred children puts its fill and blur on a `::before` (see `.db-section.editing`).
- **Firefox.** No page cross-fade on Gecko, and no opacity, transform, filter or mask animation on an ancestor of a blurred layer.
- **Flat surfaces.** No borders or outlines on cards, no tinted "selected" backgrounds, no 3D. The only shadow is the soft one on floating layers. A selected or active state is a 3px accent bar plus accent text; inside a segmented control or chip the text stays plain, because accent text fails AA on the control fill.
- **Float layers.** Anything drawn over other content, including sticky or fixed bars, uses `.float-layer` or `.popup`. In-flow surfaces use `--card-blur` instead.
- **Copy.** No subtitles or hints that explain how something works. Add text only when needed, short and to the point.
- **Accessibility floor.** Text contrast at least AA 4.5, targets at least 44px on `pointer:coarse`, `:focus-visible` never removed, `prefers-reduced-motion` honoured, status colours always carry an icon or label, light and dark get equal care.
- **Dropdowns.** Native `<select>` is not used: use `SelectMenu`.

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
. ~/.espressif/tools/activate_idf_v6.1.sh && idf.py -C sw/firmware build
```
IDF 6.1 lives in `~/.espressif/v6.1/esp-idf` (installed by eim, tools and venv under `~/.espressif/tools`). `~/.esp/esp-idf-v6.1/export.sh` fails: its Python env `idf6.1_py3.14_env` was never created. The activate script also runs `eim select v6.1`. To avoid that, export `IDF_PATH`, `IDF_TOOLS_PATH`, `IDF_PYTHON_ENV_PATH=~/.espressif/tools/python/v6.1/venv` and the tool `bin` dirs yourself (see `activate_idf_v6.1.sh -e`). `tools/ci.sh` sources `$IDF_PATH/export.sh` only if `idf.py` is not on PATH, so run it from an activated shell. `~/.esp/esp-idf` is still 5.5.3 and no longer builds this tree.
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

<!-- project-board:start -->
## Board workflow
Work is tracked on a local kanban board (`board` command). Data lives in `plan/board/` and is gitignored.
- Columns: inbox, planned, todo, in-progress, verify, done. Blocked is a flag, not a column.
- Inbox -> Todo is the user's approval. Build only Todo items.
- At the start of each session run `board list todo` and `board list in-progress`.
- `board start <id>` moves a todo card to in-progress and marks you as working on it; use it (or `claim_card`) before doing any work. When done, move it to verify with `board move <id> verify --notes "what ran + result" --steps "- [ ] check one\n- [ ] check two"` and run `board stop <id>`. Notes are for what you ran and the result; anything the user should check by hand goes in How to verify as `- [ ]` lines (`--steps`).
- Only the user moves cards to done.
- If the `board` MCP server is configured, use its tools instead (`next_card`, `claim_card`, `add_note`, `move_to_verify` with `howToVerify`, `set_how_to_verify`, `add_card`, `release_card`); agents cannot move cards to todo, planned or done.
- Add new items you find with `board add "title"` (or `add_card`); they land in inbox. Use `--needs-you` for unsure items.
- Always set `type`, `phase`, `lane` and task id (`--task`, `taskId`) on every card you create, decisions and questions included. Reuse the phase and lane names already on the board (`board list`). Keep cards without a phase to a minimum; use none only for genuinely cross-cutting items.
- Plan to cards: when a plan or phase is written or changed, every step that has no card gets one in inbox right away (`board add "title" --task <plan id> --phase <phase> --type task`, with the plan step in the description and a testable How to verify). Do not leave work only in plan files; the user approves by moving cards to todo. Before starting a phase, check `board list` for steps that are missing.
- Questions go on the board, not in chat. When you need a choice from the user (design option, library, scope, an open `D#` decision), do not ask in prose and do not guess. Create a question card with the MCP `add_card`: `type` decision, `options` (2 to 12 short labels, each `"Label :: one-line reason"`), `recommended` (your pick), `multi: true` when several can apply, and enough context in `description`. Never leave the choices only in the title or description: they must be `options`, so the user gets radio buttons or checkboxes. When one decision has several independent sub-questions, pass `questions` (a list of `{text, options, multi, recommended}`) instead of `options`; the user answers each one. Then continue with unblocked work. The user answers in the board and the card moves to done; read the answer with `board_show` (CLI: `board show <id>`, Answer section) before building on it, and record it in the project's decision log. Without MCP: `board add "question" --type decision --needs-you` with the options in the description.
<!-- project-board:end -->
