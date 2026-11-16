# VAG Diagnostics Scanner (vag_diag) — Requirements

Component to create: `components/vag_diag` (ESP-IDF C++, house conventions as `components/obd2`).

Purpose: the "AutoScan" orchestrator — enumerate VAG modules on the Golf VI (PQ35) CAN bus, probe
each address over TP 2.0 + KWP2000, collect ident and fault codes, store the address database and
scan reports on the SD card, and expose everything over the existing REST/WebSocket API.

Companion docs: `requirements/tp20.md`, `requirements/kwp.md` (their APIs are dependencies),
background: `~/dev/vcds/VCDS_AutoScan_Analysis.md` (VCDS.exe scan logic, AutoScan.txt format).

RFC 2119 MUST/SHOULD/MAY. Each FR has a test requirement (TR).

---

## 1. Dependencies (existing code — do not modify)

| API | Use |
|---|---|
| `tp20::Tp20` / `kwp::KwpClient` | probing and data collection (see their requirements) |
| `CanDriver` | bitrate config + connection state only (no direct TX/RX here) |
| `SDCard::Operation` lease, `sd_path::validRelative`, `atomic_file` | address DB + reports on SD |
| `api_routes.hpp` X-macro table | new routes (§5) |
| WebSocket `/ws` | progress + result events |
| OBD2 "diagnostic bus owner" pattern (`tryHoldDiagBus`) | mutual exclusion with OBD2 polling |

## 2. Address database on the SD card

The address list MUST live on the SD card as user-editable files, in **VCDS `AutoScan.txt` format**
so a stock database can be dropped in unmodified:

```
;
; VCDS Auto-Scan File 20240323
;
2H,VW Amarok (2010 > 2022),00
1K,VW Golf/Jetta V (2004 > 2009),00
...
3D,VW Phaeton (2002 > 2006),01,02,03,05,...,77
```

- **DB-1 Location**: `<SD mount point>/vag/autoscan.txt` (mount point resolved via
  `SDCard`/`sd_path`; never hard-code `/sdcard`). Optional per-user override file
  `<mount>/vag/myautoscan.txt` loaded on top (later entries win, same as VCDS).
- **DB-2 Format rules** (from the VCDS parser, `FUN_0042a5f0`):
  - lines starting `;` are comments; empty lines ignored.
  - `chassis,description,addr[,addr...]` — chassis = up to 3 alnum chars, description = free text
    (no commas), addresses = hex, `0x` prefix optional.
  - `LOOP` keyword in the address field = rescan this chassis's list a second time.
  - A first address of `00` sets the chassis "auto-detect via gateway" flag (no address 0x00 is added).
  - Limits: max 350 chassis, max 255 addresses per chassis, address values 1..0xC003.
  - Attribute-tagged addresses (`0x4000/0x8000/0xC000 + addr`): v1 MUST accept them in the parser and
    treat the low 11 bits as the address, storing the tag for future protocol hints; no behavior change.
- **DB-3 Loading**: parsed once at scan start into RAM (fixed pool: 350 * (255+1) * 4 B max ≈ 350 KB
  — MUST be allocated once at init or the DB MUST be streamed per chassis; implementer picks, but
  no per-scan heap churn). SD access only under an `SDCard::Operation` lease (task-affine!); parsing
  happens after copying the file content to RAM, not while holding the lease.
- **DB-4 Defaults**: if the SD file is missing/unmountable, a compiled-in fallback table with the
  Golf VI-relevant entries (from VCDS 25.3 `AutoScan.txt`: 5K/5F/A5/... PQ35 rows) MUST be used and
  the API must surface `dbSource: "builtin" | "sd"`.
- **DB-5 Reports**: every finished scan writes
  `<mount>/vag/scans/Scan-YYYYmmdd-HHMMSS.txt` (human-readable, VCDS-like layout) and
  `Scan-...json` (machine-readable, same data as the API) via `atomic_file`. Listing/deleting scans
  reuses the `recorder`'s trip-listing UX patterns.
- **DB-6 Editing**: v1 read-only from the API (upload via file manager out of scope); `POST reload`
  re-reads the files.

## 3. Scan engine

- **FR-1 Address strategy** per VAG address `A` (default 0x01..0x7F or the DB list):
  1. TP 2.0 probe: `tp20.openChannel(A, 250ms)`. Positive 0xD0 -> classify `TP20`, continue with
     KWP ident + faults. `0xD6..0xD8` or timeout -> mark `ABSENT`.
  2. (v2, stub interface now) UDS/ISO-TP probe on `0x7E0+` -> classify `UDS`.
  3. Record result, always `closeChannel` (teardown on every path, including errors/abort).
- **FR-2 Gateway pass**: before the address sweep, open a channel to the gateway (VAG 0x19; some
  PQ35 cars answer on 0x1F — try both) and read its installation/chassis info via KWP. Whatever the
  gateway yields (chassis code, installed-module hints) feeds the DB row selection and progress UI.
  The exact gateway records REQUIRE a bench capture — until then this pass is best-effort and
  non-blocking: on failure, continue with the plain sweep and flag `gateway: false` in the result.
- **FR-3 Scan session**: `start(chassis | "auto", options)` -> engine state machine
  `IDLE -> RUNNING -> CANCELLING -> DONE/ABORTED`. Options: address list source, per-probe timeout
  (default 250 ms), inter-address delay (default 50 ms), collect faults (default true).
- **FR-4 Progress**: per address, an event is emitted (§5 WS): `{addr, state, protocol?, partNumber?,
  dtcCount?}`. A full result record per module:
  `{ addr, protocol, present, partNumber, swVersion, hwVersion, coding, dtcCount, faults[], error }`.
- **FR-5 Cancellation**: `stop()` at any point; MUST tear down the open channel of the current
  address before returning `ABORTED` (never leave TP 2.0 channels open).
- **FR-6 Arbitration**: the scanner and OBD2 polling are mutually exclusive. Take the same
  diagnostic-bus owner slot OBD2 uses (`tryHoldDiagBus` pattern); if OBD2 is busy, `start` returns
  `ESP_ERR_BUSY` with a JSON body saying what holds the bus. Auto-resume of OBD2 after scan end.
- **FR-7 Read-only default**: nothing in the scan path writes to ECUs. `clearFaults` is exposed only
  as a separate explicit endpoint with a `confirm: true` field and a `token` echoed from the GET
  (two-step confirm, mirrors the existing destructive-API patterns in this codebase).
- **FR-8 Single scan**: only one scan session at a time; second `start` => `ESP_ERR_INVALID_STATE`.
  Results of the last scan stay queryable until the next `start`.

## 4. Non-functional requirements

- NRF-1 Scan task priority below tp20 RX task; core `CORE_ID_CAN_TASKS`; no heap churn per address
  (one reusable result record + one JSON build buffer).
- NRF-2 Whole-scan duration for a 40-address sweep MUST stay under ~2 min at default timeouts.
- NRF-3 SD writes never block the scan path: reports are written after `DONE/ABORTED` (writer step
  in the scan task, under `SDCard::Operation`).
- NRF-4 Log tag `vag`; per-address summary at INFO; raw frames NEVER logged here (tp20/kwp already do).

## 5. API surface (add to the `api_routes.hpp` X-macro table)

| Route | Method | Body / Params | Returns |
|---|---|---|---|
| `/api/v1/vag/db` | GET | — | chassis list `{source, chassis:[{code,description,addresses,loop}]}` |
| `/api/v1/vag/db/reload` | POST | — | re-read SD files |
| `/api/v1/vag/scan` | POST | `{action:"start", chassis:"5K"|"auto", options{...}}` or `{action:"stop"}` | `{started, busBusy?}` |
| `/api/v1/vag/scan` | GET | — | `{state, currentAddr, progress:[...], gateway, dbSource, durationMs}` |
| `/api/v1/vag/results` | GET | `?format=json\|text` | last scan's result records / report text |
| `/api/v1/vag/module/{addr}` | GET | — | fresh single-module probe + ident + faults (on demand) |
| `/api/v1/vag/module/{addr}/faults` | DELETE | `{confirm:true, token}` | clear faults (two-step confirm) |
| `/api/v1/vag/groups/{addr}/{group}` | GET | — | live group read via KWP `21 group` |

WebSocket events on `/ws` (same envelope as existing events): `vag_scan_progress`,
`vag_module_result`, `vag_scan_done`. Auth: same middleware/session rules as the rest of the API.

## 6. Test requirements

- **TR-1 DB parser host tests**: fixtures covering: stock VCDS AutoScan.txt excerpt (parse all rows),
  comments, `LOOP`, `00`-first-address flag, attribute-tagged addresses, over-limit chassis/addresses
  (error rows skipped with counters), malformed lines (never crash; line numbers reported).
- **TR-2 Scan-flow host tests**: mocked `tp20`/`kwp` (interface seam: `vag_diag` takes
  `ITp20*`/`IKwp*`): present module (D0->ident->faults), absent module (timeout), negative response,
  abort mid-sweep (assert closeChannel called for the in-flight address), bus-busy arbitration.
- **TR-3 Target loopback test**: fake ECU answering on 0x200+addr; scan over TWAI loopback finds it,
  produces the JSON result, writes both report files (SD or tmpfs in test).
- **TR-4 API tests**: extend the existing REST check script/pytest for the new routes incl. auth
  rejection and the two-step confirm flow for DELETE faults.

## 7. Acceptance criteria

1. TR-1..TR-4 pass in CI / on target.
2. Bench run against the Golf VI: a sweep finds the engine (0x01) and other TP 2.0 modules, returns
   part numbers matching ECU labels, writes `/vag/scans/Scan-*.txt|.json` on the SD card, and a
   WebSocket client sees per-address progress events.
3. Stopping a scan mid-way never leaves an open channel (verify via tp20 counters/status).
4. With OBD2 auto-poll enabled, starting a scan returns `ESP_ERR_BUSY`; after the scan completes,
   OBD2 polling resumes automatically.
5. Dropping the stock VCDS `AutoScan.txt` onto the SD card works without code changes.

## 8. Out of scope (v1)

- UDS/ISO-TP probing (interface stubbed now; plug-in point is FR-1 step 2).
- Gateway installation-list parsing (needs a bench capture first; FR-2 stays best-effort).
- Writing/editing the DB or measures via the API; measuring-block formula interpretation (raw values
  + hex only — interpretation belongs to the frontend/tinyexpr layer).
- Multiple simultaneous scans, scheduled scans, scan-diff tooling.
