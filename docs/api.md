# API: firmware ↔ frontend

This is the contract between `sw/firmware` and `sw/frontend`. A change here lands in both sides and this doc in the same commit.

Source of truth in the firmware: the route table `api_routes` in `sw/firmware/main/src/webserver.cpp` (handlers), `sw/firmware/main/src/middleware.cpp` (response bodies and status mapping). Frontend callers are in `sw/frontend/src/lib/canStore.svelte.ts`, `lib/api.ts` and `lib/components/`.

## Conventions
- Server: port 80, host `can-sniffer.local` (mDNS) or the device IP. No TLS, no authentication.
- Base path: `/api/v1`. The frontend page itself is `GET /`.
- Format: JSON, `Content-Type: application/json`, unless noted. Numbers that are NaN or infinite are serialized by cJSON as `null`.
- Requests are queued to 6 worker tasks. The server rate limit is global, not per client: more than 50 requests in a one-second window get `429`.
- Request body limits: 2048 bytes for JSON bodies, except `PUT /pid_def` (64 KiB), the two PID file calls (256 bytes), and SD upload (16 MiB). JSON bodies need a `Content-Length` header; chunked bodies are rejected with 400.
- Versioning: none. There is no API version field. `GET /api/v1/system` returns `app_version` (`APP_VERSION_STRING`, currently `"0.5.0"`), which is the only thing the frontend could compare.

### Error shapes
Three shapes occur. The frontend (`lib/api.ts`) treats a non-2xx status or a body with `"status":"error"` as an error and shows `reason`.
1. Middleware errors: `{"status":"error","reason":"<text>"}` with the HTTP status. `reason` is usually an `esp_err_to_name` string such as `ESP_ERR_NOT_FOUND`. A handler that gets no body from the middleware answers with a fixed body for the status: 400 "Bad request", 404 "Not found", 408 "Request timeout", 413 "Content too large", 422 "Unprocessable entity", 503 "Service unavailable", 504 "Gateway timeout", else 500 "Internal server error".
2. Handler errors from `httpd_resp_send_err`: the status is set and the body is a plain text message (for example `Invalid PID requested`, `Invalid JSON`, `Content-Length required`, `JSON too large`, `Incomplete request body`, `Invalid SD card path`, `File not found`). Not JSON.
3. Async layer errors: `{"error":"<text>"}` with status `429 Too Many Requests`, `503 Busy` or `503 Shutting Down` (not valid reason phrases). No `Content-Type` is set.

Unknown paths and unsupported methods get the ESP-IDF defaults (404, 405, 414 and so on), as plain text.

Status mapping used by `status_for_error` in the middleware, where a route says "mapped": `ESP_ERR_INVALID_ARG` → 400 (or the route's own default), `ESP_ERR_NOT_FOUND` → 404, `ESP_ERR_TIMEOUT` → 504, `ESP_ERR_INVALID_STATE` and `ESP_ERR_INVALID_SIZE` → 503, everything else → 500. For SD routes `ESP_ERR_TIMEOUT` from acquiring the SD lock (1 s) is reported as 503 `"SD card busy"` instead.

### Path parameters
- `<pid>` is a decimal integer 0–65535 (`strtol` base 10). Anything else gives 400. PIDs are keys in one map, shared by all modes (`pid-config.md`).
- `<path>` (SD routes) is relative to the SD mount (`/sdcard`). It is percent-decoded. Separators must be a literal `/` (`%2F` is rejected). Rejected: empty or `.` or `..` components, `//`, control characters, `\`, `?`, `#`, `"`, and anything that does not fit in 512 bytes. A single trailing `/` is meaningful (directory).

## REST endpoints
31 routes are registered in `api_routes`: `GET /` and 30 under `/api/v1`. `GET /dtc` is listed as three rows because one handler serves three query forms. Three registered patterns end in `*` and match more than the documented path: `/api/v1/dtc*`, `/api/v1/req/dtc*` and `/api/v1/req/pid_poll*` (for example `GET /api/v1/dtcfoo` reaches the DTC handler). In the table "Q" is the query string, "B" is the request body. Frontend caller `–` means no caller exists in the frontend today.

| Method | Path | Request | Response | Errors | Frontend caller |
|---|---|---|---|---|---|
| GET | `/` | – | gzipped HTML from `/www/index.html.gz` (`Content-Encoding: gzip`, `text/html`) | 404 if the file is missing | browser |
| GET | `/api/v1/system` | – | `{app_version, uptime_s, restart_reason, mac, state, battery_voltage, sd_card_detected, component_status:[{name,status}]}`. `state` is the supervisor enum: 0 UNINITIALIZED, 1 STARTING, 2 NOT_CONNECTED, 3 RUNNING, 4 STOPPING, 5 SLEEPING, 6 ERROR. `component_status.status` is an `esp_err_to_name` string, `ESP_FAIL` for steps that never ran | 503 `SD card busy` (SD lock held over 1 s) | `canStore.svelte.ts:376` |
| POST | `/api/v1/system/reboot` | – | `{"status":"success"}`, then the device restarts after 500 ms | 500 | – |
| POST | `/api/v1/system/copy_file` | B `{"source_path","destination_path"}`, both absolute paths under `/sdcard` (not relative like the other SD routes) | `{"status":"success"}` | 400 bad body or path outside `/sdcard` or copy onto itself, 404 source missing, 503 SD busy or not mounted, 500 | – |
| GET | `/api/v1/can_bus` | – | `{state, bitrate, rs_mode, debug_mode, initialized, bus_connected, node_status:{twai_error_state, tx_error_count, rx_error_count}}`. `state`: `not_initialized`, `bus_off`, `not_connected`, `connected`. `rs_mode`: `high_speed`, `slope_control`. `twai_error_state`: `error_active`, `error_warning`, `error_passive`, `bus_off` | 500 | `canStore.svelte.ts:422` |
| GET | `/api/v1/obd2` | – | `{continuous_running, pid_initialized, pid_def_count, pid_data_count, poll_task_utilization, pid_def_path, dtc_desc_path, supported_pids:{count, groups:[7 x uint32 bitmap]}}`. `poll_task_utilization` is 0.0–1.0 | 500 | `canStore.svelte.ts:434` |
| GET | `/api/v1/pid_def` | – | `{"data":[PIDDef…],"count":n}`, see PIDDef below | 504 PID map lock busy, 500 | `canStore.svelte.ts:402` |
| GET | `/api/v1/pid_def/<pid>` | – | same shape with one item | 400 bad `<pid>`, 404 not defined, 504 | – |
| POST | `/api/v1/pid_def` | B JSON array (max 2048 bytes) of PID objects. Required: `id, mode, pid, name, formula, interval`. Optional: `unit, desc, minV, maxV, priority, color, icon, length` (alias `len`) | `{"added":[pid…],"failed":[{"pid":n or null,"error":"ESP_ERR_…"}…],"status":"success"\|"partial_success"\|"error","reason"?}`. HTTP 200 unless every item failed | 400 body not an array, 413 over 2048 bytes, plain 400 invalid JSON, 422 all items failed (503 or 500 if any failure was of that kind) | – |
| PUT | `/api/v1/pid_def` | B JSON array (max 64 KiB, max 256 items) of full PID objects, replaces the whole set atomically. Rules in `pid-config.md` | `204`, no body | 400 not an array or an item is not an object, 413, 422 field or range invalid, duplicate `pid`, formula does not compile, more than 256 items, 503 poll queue capacity or PID map lock timeout, 500 OBD2 not running or out of memory | `canStore.svelte.ts:687` |
| DELETE | `/api/v1/pid_def/<pid>` | – | `{"status":"success"}` | 400 bad `<pid>`, 404, 503 OBD2 not running, 504 | – |
| POST | `/api/v1/pid_def/save` | optional B `{"pid_def_path":"/sdcard/…"}` (max 256 bytes). Without a body it uses the configured path (`/sdcard/config/pid_def.json` by default) | `{"status":"success"}`. Writes the current definitions as a JSON array to the file | 400 body not an object, `pid_def_path` missing or not under `/sdcard`, 503 `SD card busy` or card not mounted, 500 | `canStore.svelte.ts:715` |
| POST | `/api/v1/pid_def/load` | same body as save | `{"status":"success"}`. Reads the file and replaces the whole definition set like PUT | 404 file not found, 400 not valid JSON, not an array, or validation failed, 503 SD busy or not mounted, 500 | – |
| GET | `/api/v1/pid_data` | – | `{"data":[PIDData…],"count":n}` | 504, 500 | – |
| GET | `/api/v1/pid_data/<pid>` | – | one item | 400, 404, 504 | – |
| POST | `/api/v1/req/pid_poll?running=true\|1\|false` | Q `running`: `true` (any case) or `1` starts continuous polling, any other value stops it | `201`, no body | 400 `running` missing | `canStore.svelte.ts:475` |
| POST | `/api/v1/req/static_pid` | – | `{"status":"success"}`. Queues one poll for every PID whose `interval` is 0 | 500 | – |
| GET | `/api/v1/vin` | – | `{"vin":""\|"<17 chars>"}`. Empty until a VIN was received | 500 | `canStore.svelte.ts:494` |
| POST | `/api/v1/req/vin` | – | `{"status":"success"}` once the VIN arrived (waits up to 500 ms). Read it with `GET /vin` | 503 OBD2 not running, 504 no answer | `canStore.svelte.ts:505` |
| GET | `/api/v1/dtc` | no Q | `{"dtcs":[{mode,type,dtc:[code…],dtc_count}…],"count":3,"status":"success"}` for modes 3 (`confirmed_dtcs`), 7 (`pending_dtcs`) and 10 (`permanent_dtcs`) | 500 | `canStore.svelte.ts:524` |
| GET | `/api/v1/dtc?mode=3\|7\|10` | Q `mode`, decimal | same shape with one section | 400 `mode` not an integer, 422 other number (`status:"error"`, `reason:"Unsupported DTC mode"`) | – |
| GET | `/api/v1/dtc?codes=P0301,P0420` | Q `codes`: comma separated, 1–30 codes, each exactly 5 chars `[PBCU][0-3][0-9A-F]{3}` (upper case) | `{"status":"success","dtc_count":n,"dtcs":[{"dtc","description"}…]}`. `description` is `"Description not found"` for unknown codes | 400 bad code, bad list, any other query key, 404 DTC bin missing (`status:"error"`, `reason`, `dtc_count:0`), 503 SD busy or not mounted, 500 read error | `canStore.svelte.ts:552` |
| POST | `/api/v1/req/dtc` | Q `mode` optional: 3, 7 or 10. No query requests all three | single mode: `{"status":"success"}`. All: `{"successful_modes":[mode…],"failed_modes":[{"mode","error"}…],"status":"success"\|"partial_success"\|"error","reason"?}` (200 for success and partial). Results are read with `GET /dtc` | 400 `mode` not an integer or other query, 422 unsupported mode, 503, 504 no answer within 500 ms | `canStore.svelte.ts:609` |
| POST | `/api/v1/req/clear_dtc` | – | `{"status":"success"}` when the ECU acknowledged within 500 ms. Sends OBD mode 0x04 to the functional ID 0x7DF. **This clears codes on a real vehicle. No confirmation exists in the firmware.** | 503, 504 | `canStore.svelte.ts:631` |
| GET | `/api/v1/sd_card/info` | – | `{name, mount_path, capacity, used_space_mb, max_freq_mhz, is_sdio, is_mmc, is_mounted, is_present}`. `capacity` is MB | 503 busy | `canStore.svelte.ts:389` |
| POST | `/api/v1/sd_card/format` | – | `{"status":"success"}`. Formats the card and remounts. **Destroys all files, no confirmation.** | 503 card missing or not mounted, 400, 500 | – |
| GET | `/api/v1/sd_card/tree` and `/api/v1/sd_card/tree/<path>` | – | `{"path","children":[…],"status":"success"}`. A file child is `{name,type:"file",size}`, a directory child is another `{path,children}` object. Nested up to 5 levels below the requested directory | 400 bad path, 404 not a directory or missing, 503, 500 (also when nesting exceeds 5 levels) | `SDCardImportModal.svelte:28`, `FileTree.svelte` |
| GET | `/api/v1/sd_card/file/<path>[?download=true\|1\|false\|0]` | only the `download` query is allowed, once | the file, chunked. Content-Type from the extension (`.txt` text/plain, `.csv` text/csv, `.json` application/json, `.png` image/png, `.jpg`/`.jpeg` image/jpeg, else application/octet-stream sent as attachment). `Content-Disposition` is `inline` or `attachment; filename="…"`. Headers `X-Content-Type-Options: nosniff`, `Cache-Control: no-store` | 400 bad path or query, 404 not a regular file, 503 busy or not mounted, 500 | `SDCardImportModal.svelte:48` |
| POST | `/api/v1/sd_card/file/<path>` | B raw bytes, `Content-Length` required, max 16 MiB. A `<path>` ending in `/` creates a directory and needs an empty body. An existing file is truncated and overwritten in place | `{"status":"success"}` | `{"status":"error","reason"}` with 400 invalid length or path, 408 stalled (5 s no progress, 60 s total), 413, 503 `SD card busy` or `SD Card not mounted`, 500 | `FileTree.svelte:101` (directory), `FileTree.svelte:144` (upload) |
| DELETE | `/api/v1/sd_card/file/<path>` | a path ending in `/` deletes that directory recursively | `{"status":"success"}` | 400 bad path, 404, 503, 500 | `FileTree.svelte:57` |
| GET | `/api/v1/settings` | – | array of `{name,settings}` for `wifi`, `can`, `system`. Passwords are never returned. Fields below | 500 if any section could not be read | – |
| POST | `/api/v1/settings` | B (max 2048 bytes) one `{name,settings:{…}}` object or an array of them. Partial: only the given fields change. Fields may also sit directly on the item | `{"status":"success"}`. Values go to NVS and take effect after a reboot. Nothing is range-checked | 400 not an object or array, or no known fields, 422 unknown `name`, 500 | – |

Settings fields: `wifi`: `ssid, password, sta_ssid, sta_password, channel, max_connections, auth_mode, ssid_hidden, pmf_required, gtk_rekey_interval, sta_auth_mode, mode, sta_max_retry` (`auth_mode` is `wifi_auth_mode_t`, `mode` is `wifi_mode_t`: 1 STA, 2 AP, 3 AP+STA). `can`: `bitrate` (125000, 250000, 500000, 1000000), `tx_pin, rx_pin, lbk_pin, rs_pin, debug, rs_mode` (0 high speed, 1 slope control), `tx_queue_depth, rx_queue_size, filter, mfilter_cfg:{id,mask,is_ext}`. `system`: `pid_def_path, dtc_desc_path`.

### PIDDef (responses of `GET /pid_def`)
`{pid, mode, id, length, name, unit, description, minValue, maxValue, priority, update_interval_ms, color, icon, formula}`

The request and file format uses different names for the same fields: `desc`, `minV`, `maxV`, `interval` (and `len` as an alias of `length`). See `pid-config.md`. The frontend converts in `serializePidDefinition`.

### PIDData (responses of `GET /pid_data`)
`{id, pid, value, lastUpdated, isSupported, isValid, update_interval_ms}`. `id` is the CAN ID of the ECU that answered (or the configured request ID before support is known). `lastUpdated` is milliseconds since boot. `value` is the formula result, `null` when it is NaN.

## WebSocket
- Path: `/ws` (plain `ws://`, GET upgrade). Binary frames only. All multi-byte values are little-endian.
- There is no per-client state. The flags below are global. The frontend sends a ping every 2 s and reconnects when no pong arrives within 1 s.
- On connect the server starts the 1 Hz CAN status stream. The stream stops when the last client closes.

| Type | Direction | Payload | Rate |
|---|---|---|---|
| `0xA0` start PID stream | client → server | 1 byte `0xA0`. Enables PID packets for all clients | on demand |
| `0xA1` stop PID stream | client → server | 1 byte `0xA1` | on demand |
| `0xA2` ping | client → server | 1 byte `0xA2` | frontend: every 2 s |
| `0xA3` pong | server → all clients | 1 byte `0xA3`, reply to a ping | per ping |
| `0x02` PID update | server → all clients, only after `0xA0` | 20 bytes: `[0]=0x02`, `[1]=18` (payload length), `[2..5]` uint32 `pid`, `[6..9]` float32 value, `[10..13]` uint32 `lastUpdated` (ms since boot), `[14..17]` uint32 update interval ms, `[18]` isSupported (0/1), `[19]` isValid (0/1) | once per PID update, so it follows the poll rate. Not throttled |
| `0x03` CAN status | server → all clients | 12 bytes: `[0]=0x03`, `[1]=10`, `[2]` CAN state (0 not initialized, 1 bus off, 2 not connected, 3 connected), `[3..6]` float32 poll utilization 0.0–1.0, `[7..10]` float32 battery voltage in V, `[11]` bus connected (0/1) | 1 Hz while at least one client is connected |

Other behaviour:
- Text frames are logged and ignored. Unknown command bytes are logged and ignored.
- A close frame is echoed. When at most one WebSocket client remains, both streams are turned off. A client that drops without a close frame leaves the flags on.
- `0x01` (`MSG_TYPE_LOG`) is defined in `middleware.hpp` but nothing sends it.
- Up to 20 WebSocket clients are considered by a broadcast. The server allows 7 open sockets in total.
