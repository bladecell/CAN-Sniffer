# PID configuration

The PID database is loaded from JSON on the SD card when OBD2 starts, and can be changed at runtime (add, replace, remove). Runtime changes live in RAM only until `POST /api/v1/pid_def/save` writes them back to the file. There is no automatic persistence.

- File: `/sdcard/config/pid_def.json` by default (`PID_DEF_DB_PATH` in `supervisor.hpp`). The path is stored in NVS (`pid_def_path`, `system` section of `/api/v1/settings`) and can be changed there. It must start with `/sdcard`.
- Load: `setup_obd()` in the supervisor reads the file at boot and again on every wake from sleep. It goes through the same code as `PUT /api/v1/pid_def`, so the file must pass the validation below, and an invalid file loads nothing (all or nothing). A missing file is not an error: the database stays empty.
- There is no built-in default PID set. With no SD card or no file there are no PIDs.
- Sample files in the repo: `sw/firmware/pid_def.json` (10 PIDs) and `sw/firmware/asd.json` (derived PID tests).

## PID attributes
The file, `POST` and `PUT` use these names. `GET` responses use other names for some fields (last column).

| Field | Type | Meaning | GET name |
|---|---|---|---|
| `id` | uint, 0–0x7FF | CAN arbitration ID that the request is sent to (not an id of the entry). Typically 0x7DF (2015), the OBD functional address. For mode 0x01 the firmware replaces it with the responding ECU's ID (response ID minus 8) when discovery finds the PID supported | `id` |
| `mode` | uint8 | `0x01` (1) current data, `0x22` (34) read data by identifier, `0x45` (69) derived PID (internal, no CAN request) | `mode` |
| `pid` | uint16 | PID number (mode 0x01: 1–0xFF) or data identifier (mode 0x22, 16 bit) or an arbitrary number for derived PIDs. **The database key**: unique across all modes together | `pid` |
| `length` | uint8 | The ISO-TP length byte of the request: 2 for mode 0x01 (mode + PID), 3 for mode 0x22 (mode + 2-byte DID), 0 for derived. `len` is accepted as an alias on input; if both are given they must be equal. Saved files use `length`. Max 8 | `length` |
| `name` | string ≤64 | Display name | `name` |
| `unit` | string ≤32 | Display unit | `unit` |
| `desc` | string ≤256 | Description | `description` |
| `formula` | string 1–1024 | TinyExpr expression (below) | `formula` |
| `minV`, `maxV` | float or null, optional | Expected range. Absent or `null` means unset (NaN internally, returned as `null`). When both are set, `minV <= maxV`. The firmware stores and returns them; it does not clamp or check values against them | `minValue`, `maxValue` |
| `priority` | uint8 | Lower number is polled first when two requests are due at the same tick | `priority` |
| `interval` | uint16 | Poll interval in ms. `0` means "not polled recurrently"; it is only read by `POST /api/v1/req/static_pid`. Otherwise at least 4 (`MIN_TRANSMIT_PERIOD_MS`). The frontend enforces at least 16 | `update_interval_ms` |
| `color` | uint, 0–0xFFFFFF | Display color, RGB as one number (e.g. `16096779` = `0xF59F0B`) | `color` |
| `icon` | string ≤64 | Display icon name | `icon` |

Poll timing as built: a request is sent only while the CAN bus is connected and auto poll is on (`POST /api/v1/req/auto_poll?auto_poll=true`). Requests to the same CAN ID are at least 30 ms apart and the TX task sends at most one frame every 4 ms. Each time a recurring PID comes due with no update for `max(3 x interval, 1000 ms)`, one of 3 retries is used up. When they are used up it is marked invalid and dropped from the recurring schedule until auto poll is set again or discovery runs again.

Mode 0x01 PIDs are polled only when the ECU's supported-PID bitmaps (PIDs 0x00, 0x20 … 0xC0, probed on every bus connect) list them. Mode 0x22 and derived PIDs are always treated as supported.

Derived (mode 0x45) PIDs are evaluated locally. The poll task queues them like any other PID; the receive task then evaluates the formula with A–D set to 0 and stores the result with `id` = 0x7DF.

## Validation
A received frame is rejected (value not stored, PID marked invalid) if it carries fewer data bytes than the formula reads: the highest of `A`..`D` used in the formula sets the minimum.

Implemented in `components/obd2/pid_json.cpp` (host-tested). JSON nesting deeper than 32 levels is a parse error (`CONFIG_CJSON_NESTING_LIMIT`). A formula result that is NaN, inf (division by zero) or overflows float marks that reading invalid and the value is not stored; a formula that does not compile is rejected when the set is loaded. Applied to the whole array by `PUT /api/v1/pid_def`, `POST /api/v1/pid_def/load` and the boot-time file load. Any failure rejects the entire set:
- the root is a JSON array of at most 256 objects, no trailing data after the JSON;
- required on every item: `id`, `mode`, `pid`, `length` (or `len`), `priority`, `interval`, `color`, and the strings `name`, `unit`, `desc`, `formula`, `icon` (empty strings are allowed except `formula`); keys the firmware does not know are ignored;
- numbers must be finite integers within the ranges above (floats for `minV`, `maxV`, which are optional: absent or `null` is unset, any other non-number is rejected);
- valid combinations: mode 0x01 with `pid` 1–255 and `length` 2; mode 0x22 with `length` 3; mode 0x45 with `length` 0. Any other mode is rejected;
- `interval` is 0 or ≥ 4; `minV <= maxV` only when both are finite numbers; `formula` not empty;
- `pid` is unique in the array;
- the formula must compile (TinyExpr). A formula that does not compile makes the whole replace fail with 422.

`POST /api/v1/pid_def` (add) is looser, per item: it needs `id, mode, pid, name, formula, interval`; every number is range-checked against the table above (`id` 0-0x7FF, `mode` and `priority` 0-255, `pid` and `interval` 0-65535, `color` 0-0xFFFFFF, `minV`/`maxV` finite floats; fractions, NaN, negative or out-of-range values reject the item with `ESP_ERR_INVALID_ARG`; optional fields may be absent or `null`), the (mode, PID, length) rule above applies, `length` defaults to 3 for mode 0x22, 0 for derived and 2 otherwise, `interval` below 4 is raised to 4 (so 0 becomes 4 there), a formula that does not compile is accepted and the PID never gets a value, and a `pid` that already exists is rejected. Prefer `PUT`.

## Formulas
Formulas are compiled by TinyExpr (`components/tinyexpr`) when the definition is created.

- Variables `A`, `B`, `C`, `D`: the first four data bytes after the service and PID header of the response frame (mode 0x01: bytes 3–6 of the CAN frame, mode 0x22: bytes 4–7). Bytes past the end of the frame are 0. Only single-frame responses are evaluated. For derived PIDs all four are 0.
- Operators: `+ - * / % ^` (power), parentheses, `,` argument separator. Numbers are `double` and accept hex (`0x0C`) because they go through `strtod`.
- TinyExpr functions: `abs acos asin atan atan2 ceil cos cosh e exp fac floor ln log log10 ncr npr pi pow sin sinh sqrt tan tanh`. `log` is base 10 here.
- Extra functions registered by the firmware (`pid_def.cpp`, `tinyexpr_helpers.h`):
  - `getBit(value, bit)`: bit number `bit` of `value` (cast to uint32), 0 is the least significant bit. Returns 0 or 1, and 0 when `bit > 31`.
  - `bitMask(value, start, len)`: `(value >> start) & ((1 << len) - 1)`. Returns 0 when `start > 31` or `len == 0`.
  - `getPID(pid)`: the last stored value of another PID, looked up by the `pid` key (not the `id` field), as a double. If that PID is not defined it returns NaN. If it is defined but has no data yet it returns 0. It reads the stored value and never evaluates the other formula, so a derived PID that refers to itself or forms a loop just reads the previous value. There is no cycle check and none is needed.
  - `getPIDRaw(pid, byte)`: byte `byte` (0–7) of the last raw frame stored for that PID. The stored bytes start at the CAN frame byte 0 (the ISO-TP length byte), so for mode 0x01 byte 3 is `A`. Returns 0 for an unknown PID or an index above 7.

Evaluation runs in the receive task while the PID map lock is held.

Known gap against the rule in `CLAUDE.md` ("NaN, inf or division by zero marks the PID invalid"): `PIDDefinition::evaluate` returns `ESP_OK` for any result, so a NaN or infinite value (for example `A / 0`) is stored and the PID is marked valid. Only a formula that did not compile or a 10 ms evaluation lock timeout leaves the PID without a value. A PID whose formula did not compile stays invalid.

## Example
Mode 0x01 engine RPM and a derived PID (both valid for `PUT`):
```json
[
  {
    "id": 2015, "mode": 1, "pid": 12, "length": 2,
    "name": "Engine RPM", "unit": "rpm", "desc": "Engine speed",
    "formula": "((A * 256) + B) / 4",
    "minV": 0.0, "maxV": 16383.75,
    "priority": 1, "interval": 256, "color": 3899638, "icon": "droplet"
  },
  {
    "id": 2015, "mode": 69, "pid": 50002, "length": 0,
    "name": "RPM per coolant degree", "unit": "", "desc": "Derived from two PIDs",
    "formula": "getPID(0x0C) / getPID(0x05)",
    "minV": 0.0, "maxV": 1000.0,
    "priority": 2, "interval": 1024, "color": 16347926, "icon": "gauge"
  }
]
```
