# Architecture

Facts below were read from the code at app version 0.5.0 (`sw/firmware/main/include/utilities.h`). ESP-IDF 5.5.3, ESP32-S3, C++17.

## Components
All paths are relative to `sw/firmware/`. Dependencies are the `REQUIRES` / `PRIV_REQUIRES` in each `CMakeLists.txt`, plus the includes the code really uses.

| Component | Path | Responsibility | Depends on |
|---|---|---|---|
| main (app) | `main/` | `app_main`, then the pieces below that have no component of their own: Supervisor (`src/supervisor.cpp`), Settings (NVS, `src/settings.cpp`), Battery ADC (`src/battery.cpp`), LED (`include/led_status.hpp`), Web server routes and WebSocket (`src/webserver.cpp`), Middleware (`src/middleware.cpp`) | driver, can_driver, obd2, obd2_simulator, wifi, async_web_server, esp_http_server, esp_adc, sd_card; private: json (`espressif/cjson`); managed: `joltwallet/littlefs` |
| Supervisor | `main/src/supervisor.cpp` | Runs the init steps in order, owns the state machine, sleep and wake (`power.md`), loads and saves the PID JSON on the SD card, file copy | Settings, SD card, CAN driver, OBD2, WiFi, web server, Battery, LED, middleware (`m_pid_def_put`) |
| CAN driver | `components/can_driver/` | TWAI node, RX and TX queues, TX task, bus health check task, connected/disconnected callback, loopback/debug mode, `isBusConnected()`, `quickCheckBus()` | esp_driver_twai, driver |
| OBD2 | `components/obd2/` | PID database (`obd2_data_model.cpp`), formulas (`pid_def.cpp`), poll scheduler (`include/pid_priority_queue.hpp`), receive task, ISO-TP multi-frame (DTC, VIN), supported-PID discovery, DTC and VIN requests | can_driver, tinyexpr. It does not touch the SD card; the supervisor does the JSON file I/O |
| TinyExpr | `components/tinyexpr/` | Vendored expression parser (compiled with `-w`) | – |
| OBD2 simulator | `components/obd2_simulator/` | Answers requests with synthetic frames when the CAN config has `debug=true` | can_driver, obd2 |
| SD card | `components/sd_card/` | SDMMC mount, hot-plug via the card-detect pin, task-affine operation lease (recursive mutex), file ops, path validation | driver, fatfs, freertos, sdmmc; private: json |
| WiFi | `components/wifi/` | AP, STA or AP+STA, STA retry then AP fallback, mDNS | esp_wifi, nvs_flash, espressif/mdns |
| Async web server | `components/async_web_server/` | Wraps `esp_http_server`: request queue, worker tasks, rate limit, WebSocket broadcast | esp_http_server, esp_timer |
| Web server (routes) | `main/src/webserver.cpp`, `main/src/middleware.cpp` | REST routes, WebSocket handler. `webserver.cpp` does HTTP parsing and calls `m_*` functions in `middleware.cpp`, which return cJSON | OBD2, SD card, Settings, Supervisor, CAN driver |
| Frontend | `../frontend/` (Svelte, built with bun) | UI, served as one gzipped file from the `www` LittleFS partition | REST and WebSocket only |
| Recorder (planned) | – | Trip detection, CSV logging, 0–100 timing. Not in the code | OBD2, SD |
| OTA (planned) | – | Not in the code (`ota.md`). `partitions.csv` already has `otadata`, `ota_0`, `ota_1` | – |
| Modem (v2, planned) | – | Not in the code | – |

Layering discrepancies, as built:
- `webserver.cpp` calls `SDCard` directly for SD file upload and download, instead of going through `middleware.cpp`. The rule in `CLAUDE.md` says firmware data goes through the middleware.
- `supervisor.cpp` includes `middleware.hpp` (it calls `m_pid_def_put` to load PID JSON), so the supervisor depends on the web layer.
- `main/main.cpp.disabled_sd` is a stray, unused file.

### Frontend packaging (as built)
`main/CMakeLists.txt` defines a custom target that runs `bun run build` in `sw/frontend`, then `littlefs_create_partition_image(www "../../frontend/dist" FLASH_IN_PROJECT)` builds a LittleFS image from `sw/frontend/dist` and flashes it to the `www` partition (`partitions.csv`, 1M, type `data`, subtype `spiffs`, mounted by label as LittleFS). The web server serves only `/` and it reads `/www/index.html.gz`. Other files in `dist` are not served. Frontend commands (bun install, dev, build, check) are in `.claude/rules/frontend.md`; `vite.config.js` inlines everything into one HTML file and gzips it.

## FreeRTOS tasks
Priorities are absolute FreeRTOS numbers (`tskIDLE_PRIORITY` is 0). "Core" is the pinned core, "any" is `tskNO_AFFINITY`. Stack is in bytes, as passed to `xTaskCreate*`. The tick rate is 1000 Hz (`CONFIG_FREERTOS_HZ=1000`).

| Task | Owner | Core | Stack | Priority | Blocks on |
|---|---|---|---|---|---|
| `main` (`app_main`) | ESP-IDF | 0 | 8192 (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) | 1 (IDF default) | `vTaskDelay` 250 ms until the supervisor reports RUNNING, then `vTaskDelay(1000)` forever |
| `SupervisorTask` | `main/src/supervisor.cpp` | 1 | 4096 | 1 | `vTaskDelay(1000 ms)` at the end of every loop; in SLEEPING also `vTaskDelay` of the sleep time |
| `led_worker` | `main/include/led_status.hpp` | any | 1536 | 2 | binary semaphore with timeout |
| `WSDataStreamTask` | `main/src/webserver.cpp` | 1 | 4096 | 1 | `vTaskDelayUntil` 1000 ms; when no WebSocket client is connected, `xSemaphoreTake(WSDataStreamSemaphore, portMAX_DELAY)` |
| `httpd` | ESP-IDF HTTP server (`httpd_start`) | any | 8192 (set in `async_web_server.cpp`) | 5 (`HTTPD_DEFAULT_CONFIG`) | `select()` on sockets. Also runs the WebSocket send work queued by `wsBroadcast` |
| `async_req_worker` x6 | `components/async_web_server/` | any | 8192 | 5 | `xQueueReceive(request_queue, portMAX_DELAY)`. Queue depth is 12 (2 x workers). All REST handlers run here |
| `can_tx` | `components/can_driver/` | 1 | 4096 | 4 (`TX_TASK_PRIO`) | `canConnectedSemaphore` while the bus is not CONNECTED; otherwise `txQueue` with a 5000 ms timeout. After each frame it waits until 4 ms (`MIN_TRANSMIT_PERIOD_MS`) have passed |
| `can_health_check` | `components/can_driver/` | 1 | 4096 | 3 (`HEALTH_CHECK_TASK_PRIO`) | task notification: forever when CONNECTED, 2000 ms in NOT_CONNECTED (ping period), 5000 ms in BUS_OFF |
| `OBD2_receive_task` | `components/obd2/` | 1 | 8192 | 2 | startup gate event bit, then `xBusConnectionSemaphore` while the bus is down; otherwise `CanDriver::receive` with a 10 ms timeout |
| `OBD2_PollTask` | `components/obd2/` | 1 | 8192 | 1 | startup gate, then `ulTaskNotifyTake` with the wait time of the poll queue (next due request) |
| `OBD_Connection_callback_task` | `components/obd2/` | 1 | 4096 | 0 | `xQueueReceive(event_queue)` with a 100 ms timeout. Runs the supervisor's VIN and DTC requests |
| `DataSimTask` | `components/obd2_simulator/` | 1 | 4096 | 2 | `xTaskNotifyWait(portMAX_DELAY)`. Exists only when the CAN config has `debug=true` |
| `sys_reboot` | `main/src/middleware.cpp` | any | 2048 | 5 | one-shot: waits 500 ms, then `esp_restart()` |

Not listed: ESP-IDF system tasks (IDLE, `esp_timer`, event loop, lwIP, Wi-Fi, mDNS).

Workers and CAN tasks that the supervisor stops are deleted: OBD2 tasks exit on their own, `can_tx`, `can_health_check` and `WSDataStreamTask` are deleted with `vTaskDelete` from another task.

Task watchdog: enabled, 5 s timeout, panic on timeout is off, idle tasks of both cores are checked (`sdkconfig`).

## Startup order
`app_main` (`main/main.cpp`):
1. Sets the `CAN_DRIVER` log level to DEBUG.
2. `LedStatus::init(LED_GPIO)` (GPIO 47 in both pin sets).
3. `SUPERVISOR::start()` creates `SupervisorTask` on core 1. If the task cannot be created the state becomes ERROR.
4. Waits in a 250 ms loop until the supervisor state is RUNNING. It never times out.

`SupervisorTask` in state STARTING runs these steps in this order (`_setup_steps` in `supervisor.hpp`). Each result is kept and shown by `GET /api/v1/system` as `component_status`.

| # | Step | What it does | Critical |
|---|---|---|---|
| 1 | SETTINGS | `Settings::init()` (`nvs_flash_init`, erase and retry on `NO_FREE_PAGES` or `NEW_VERSION_FOUND`), then loads the supervisor config (PID and DTC file paths) from NVS | yes |
| 2 | POWER_MANAGEMENT | `esp_pm_configure` max 240 MHz, min 40 MHz, automatic light sleep on. Creates and takes an `ESP_PM_NO_LIGHT_SLEEP` lock | yes |
| 3 | FLASH_FILESYSTEM | Mounts the `www` partition read-only as LittleFS at `/www` (no format on failure) | yes |
| 4 | SD_CARD | `SDCard::init` (SDMMC slot 1, 1-bit, card-detect GPIO). Returns OK even when no card is inserted. If a card is present it mounts it, and the mount callback creates `/config` and `/logs`. `format_if_mount_failed` is true | no |
| 5 | BATTERY | ADC1 channel 1 oneshot | no |
| 6 | CAN | Reads the CAN config from NVS (defaults on first boot), installs the simulator hooks if `debug`, `CanDriver::init` | no |
| 7 | OBD | Registers the connected-callbacks, `OBD2::init()` (needs the CAN driver initialized), waits 1 s, loads the PID JSON from the SD card | no |
| 8 | WIFI | Reads the Wi-Fi config from NVS (defaults on first boot), `WIFI::init` + `start` | yes |
| 9 | WEBSERVER | `setup_web_server()`: starts `AsyncWebServer`, registers routes, subscribes `pid_stream_callback` to PID updates, creates `WSDataStreamTask` | yes |

Failure handling:
- A step that fails and is not critical is logged and skipped. The next step still runs and the state goes on to NOT_CONNECTED. The web server then runs with that feature missing (for example OBD calls return 503).
- A critical step that fails sets state ERROR and stops the sequence. The remaining steps are not run, and `component_status` shows `ESP_FAIL` for them (the initial value). In ERROR the supervisor only blinks the LED (`blink(0xFFFFFFFF, 100, 100)`). It does not reboot and does not sleep.
- NVS read errors other than "not found" in settings, CAN or Wi-Fi setup call `Settings::factoryReset()` (wipes all of NVS) and `esp_restart()`.
- With no SD card, OBD step 7 initializes OBD2 but `load_pid_def_from_json` returns `ESP_ERR_INVALID_STATE`, so the OBD step result is an error and the PID database is empty. There is no built-in default PID set.
- On a first boot the Wi-Fi default is STA mode with an empty SSID. After `sta_max_retry` (5) failed connects it switches to AP mode (SSID "Can Sniffer", open, channel 6).
- After step 9 the state is NOT_CONNECTED. It becomes RUNNING when the CAN driver reports the bus connected (`power.md`).
- The default CAN config sets `debug = DEBUG_MODE` which is 1 (`utilities.h`). With `debug` on, the TWAI node runs in loopback and self-test mode, the loopback pin is driven high, the looped-back request frames go to the simulator and not to the OBD2 RX queue, and `DataSimTask` pushes synthetic ECU responses (ID 0x7E8) straight into the RX queue. A fresh device therefore runs on simulated data until `debug` is turned off in the CAN settings and the device is rebooted.

## Data flow
```
CAN RX ISR → rxQueue (size from CAN config, default 20) → OBD2_receive_task → parseRecFrame ─┬→ PID data map (+ formula) ─→ pid_stream_callback → WebSocket 0x02 packets
   (debug: ISR → simulator instead of rxQueue)                                              ├→ DTC / VIN data ─→ REST (GET /dtc, /vin)
                                                                                             └→ supported-PID bitmaps (discovery)
OBD2_PollTask (priority heap of PollRequest) → queryMsg → CanDriver::transmit → txQueue (10 frames) → can_tx → TWAI TX
Derived PIDs: PollTask → derivedPidQueue (10) → receive task → parseDerivedData → formula with getPID()
WSDataStreamTask → 1 Hz CAN status packet 0x03 → WebSocket
Browser → REST (AsyncWebServer workers) → webserver.cpp handlers → middleware m_* → OBD2 / SD / Settings / Supervisor
```
There is no Recorder. Nothing writes to the SD card besides the PID JSON save, file upload and the SD file copy. Details of each interface are in `api.md`, `pid-config.md` and `sd-card.md`.

## Sleep and power
The supervisor puts the device into automatic light sleep when the bus stays down. See `power.md`.
