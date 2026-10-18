# Power, sleep and wake

The supervisor keeps the device from draining the car battery when the car is off. Everything under "As built" was read from `sw/firmware/main/src/supervisor.cpp`, `battery.cpp` and the CAN driver. Everything under "Design intent" is not in the code.

## As built

### What exists
- One sleep mode: **automatic light sleep**, driven by a timer. No deep sleep, no GPIO wake, no IMU code, no ignition sense, no modem (`grep` finds `esp_sleep_enable_timer_wakeup` only in `supervisor.cpp`).
- Power management is configured at startup: CPU 240 MHz max, 40 MHz min, `light_sleep_enable = true` (`CONFIG_PM_ENABLE=y`). The supervisor creates an `ESP_PM_NO_LIGHT_SLEEP` lock and holds it for the whole awake time. It releases the lock only in the SLEEPING state.
- The only wake source is the timer. The sleeping task just calls `vTaskDelay(sleep time)`; the FreeRTOS idle task then enters light sleep, and the same delay brings the device back. `esp_sleep_enable_timer_wakeup()` is also called with the same time.

### States (`SUPERVISOR::State`, shown as a number by `GET /api/v1/system`)
```
UNINITIALIZED(0) → STARTING(1) ─ all critical steps ok → NOT_CONNECTED(2) ⇄ RUNNING(3)
                        │ critical step failed
                        ▼
                     ERROR(6)  (LED blinks 30 s, then sleeps with the PM lock released and restarts; see below)

NOT_CONNECTED ─ bus down, V ≤ 13.2, 60 s ─→ STOPPING(4) ─ deinit ok ─→ SLEEPING(5)
SLEEPING ─ wake, CAN init ok, bus alive ─→ restart OBD, Wi-Fi, web server ─→ NOT_CONNECTED
SLEEPING ─ wake, bus dead ─→ stay SLEEPING (CAN deinit, transceiver standby, sleep again)
SLEEPING ─ CAN init fails after wake ─→ ERROR (same back-off and restart)
SLEEPING ─ OBD / Wi-Fi / web server start fails after wake ─→ STOPPING (retry the shutdown)
STOPPING ─ web server, OBD2 or CAN deinit fails ─→ stays STOPPING, retried every second
```
The supervisor loop runs once per second (`vTaskDelay(1000)`), and each pass also calls `SDCard::update_card_status()`.

- `NOT_CONNECTED` → `RUNNING` when `CanDriver::isBusConnected()` is true. The driver declares the bus connected after 3 successful pings, sent every 2 s (`can_health_check`), with no ACK or stuff errors.
- `RUNNING` → `NOT_CONNECTED` when the bus is no longer connected. The OBD2 "disconnected" callback does the same, but only from RUNNING (an atomic compare-and-set), so it cannot overwrite ERROR, STOPPING or SLEEPING. `eState` is a `std::atomic`.
- `ERROR` (`main/include/error_policy.hpp`): the LED blinks for 30 s, then the supervisor releases the PM lock, light-sleeps for a back-off and calls `esp_restart()`. The back-off is 60 s doubled for every consecutive ERROR restart, capped at 3600 s, so a permanent fault costs one short boot per hour rather than a reboot loop or an awake device. The count is kept in RTC memory (valid only with a magic value, so power-on garbage reads as 0) and cleared when a start completes. A critical setup step that fails at boot (settings, power management, flash file system, Wi-Fi, web server) takes this path.
- There is no path from `RUNNING` to sleep. Sleep always goes through `NOT_CONNECTED`.

### Sleep decision
| Input | HW | Signal | Threshold / timeout |
|---|---|---|---|
| CAN bus health | v1.2 | `CanDriver::isBusConnected()`: false when more than 3 consecutive ACK errors or stuff errors are seen, or the controller is bus-off | the bus must stay down for 60 s (`SLEEP_TRANSITION_TIMER_S`, one count per 1 s loop) |
| OBD port voltage | v1.2 | `battery_read()`: ADC1 channel 1 (GPIO 2 on the ESP32-S3), 12 dB attenuation, 12 bit, 16 sample average, divider 1 MΩ / 270 kΩ (ratio 4.704), rounded to 0.1 V. Uses the raw count scaled to 3300 mV, no ADC calibration | above **13.2 V** (alternator running): stay awake and restart the 60 s count. At or below: the count runs |
| Battery voltage while asleep | v1.2 | same ADC | below **11.4 V** (`LONG_SLEEP_VOLTAGE_LIMIT`, hard coded, a `TODO: save this to nvs` is in the source): sleep 60 s per cycle instead of 10 s |
| Ignition sense (OBD pin 1) | v2 | – | not built |

The old doc guessed "engine off ≈ below 13.0 V"; the code uses 13.2 V.

### Sleep cycle (SLEEPING state)
1. Read the voltage. Sleep time is 10 s, or 60 s below 11.4 V.
2. Release the PM lock, `esp_sleep_enable_timer_wakeup`, `vTaskDelay`. Re-acquire the lock after the delay.
3. Set the CAN transceiver RS pin low (high speed), run `setup_can()` (re-reads the CAN config from NVS).
4. `quickCheckBus()`: sends one OBD request (`0x7DF`, mode 0x01 PID 0x00), waits 50 ms and returns true only if no ACK or stuff errors were counted.
5. Bus alive: `setup_obd()`, `setup_wifi()`, `setup_webserver()`, state NOT_CONNECTED. Bus dead: stop the web server, deinit the CAN driver, RS pin high (standby) and sleep again.

The cycle is about `sleep time + 1 s` (supervisor loop delay) plus the wake work. Each wake transmits one CAN frame on the real bus, a standard OBD request. Sleep current and wake duration have not been measured (`plan/STATUS.md` → Metrics).

### Observed on a board (2026-10-10, HW v1.1, bench, `DEBUG_MODE 0`, nothing on the OBD port)
- `DEBUG_MODE` is only the default for the CAN `debug` setting. A stored `c_debug` in NVS wins, so a device that ever ran with debug on keeps the loopback and simulator until `POST /api/v1/settings` sets `{"name":"can","settings":{"debug":false}}` and the device reboots. Without that the bus is "connected" and the device never sleeps.
- Boot, then `Waiting for CAN bus connection...` every second (NOT_CONNECTED, Wi-Fi and web server up).
- 60 s after boot: `CAN disconnected for 60s (Voltage 1.00V), transitioning to SLEEP`, one second later `Shutting down modules for sleep...` (Wi-Fi goes down), then `Entering Light Sleep...`.
- The voltage read 0.9 to 1.0 V (no supply on the OBD pin), which is below 11.4 V, so every cycle is 60 s with `Battery low ... Sleeping aggressively...`. Cycles seen: sleep at 64.3 s, wake at 124.3 s (`CAN driver initialized`, `Bus is dead. Going back to sleep.`), sleep at 125.3 s, wake at 185.3 s, and again. So a cycle is the sleep time plus about 1 s, as written above. No panic, no reset loop in 220 s.
- While it sleeps the board has no Wi-Fi and no API, and the USB serial port does not answer `esptool` until the next wake window; reflashing needs several `--connect-attempts` (or a power cycle).
- Sleep current was not measured (software cannot measure it).

### Deinit and init order
Before sleep (STOPPING):
1. `stop_web_server()`: asks `WSDataStreamTask` to leave its loop and waits up to 1 s for it to exit by itself, stops `AsyncWebServer` (workers, httpd), and only then deletes the task's semaphore, so the WebSocket handshake handler never sees a deleted semaphore. Waits at most 3 s for the workers; if one is still inside a handler it returns `ESP_ERR_TIMEOUT`, nothing further is stopped and the sequence retries on the next pass.
2. `OBD2::deinit()`: stops its three tasks, clears PID values, VIN, DTCs and all subscriber lists, deletes its queues and semaphores. PID definitions stay. If it times out (5 s), the sequence stops here and retries on the next pass.
3. `WIFI::deinit()` (returns void, so nothing is checked).
4. `CanDriver::deinit()`: stops the simulator, disables the node, then asks the TX and health tasks to exit (flag and notify, bounded 3 s wait, each task deletes itself) before deleting the TWAI node, queues, semaphore and TX ring. Safe to call twice. A task that does not leave in time returns `ESP_ERR_TIMEOUT` with nothing freed; an error stops the sequence and retries.
5. RS pin high (transceiver standby), state SLEEPING.

After wake: CAN, then (bus alive) OBD (which re-subscribes the connected-callbacks and reloads the PID JSON from the SD card), Wi-Fi, web server (which re-subscribes the PID stream callback).

Not deinitialized before sleep, and have no `deinit()`: Settings (NVS), power management, the `www` LittleFS mount, the SD card (stays mounted, still polled for hot-plug every second), the battery ADC unit, the LED worker task. `CLAUDE.md` requires every component to support `deinit()`; these do not.

### Where it differs from a battery-safe design
- ERROR state: fixed by P2.9e, see States.
- While the bus is up (RUNNING) the device never sleeps. While the bus is down and the voltage stays above 13.2 V it never sleeps either.
- `main()` keeps polling the supervisor state every 250 ms until it is RUNNING, including while the device sleeps.

## Design intent (not built)
Kept from the first draft of this doc. None of it exists in the code.
```
BOOT → INIT → RUNNING ⇄ IDLE → DEINIT → SLEEP → (wake) → BOOT
```
| Wake source | HW | ESP32 wake type |
|---|---|---|
| IMU motion interrupt | v1.2 | ext0 / ext1 / GPIO, TBD. Needs an RTC GPIO for deep sleep |
| Ignition sense | v2 | TBD |
| Timer (periodic DTC scan, parked MQTT) | v2 | timer (this one exists today, for the polling wake above) |
| Others | | D7 |

TBD: IMU interrupt GPIO and wake type. No IMU code exists.

## Measured
Sleep current and other numbers are recorded in `plan/STATUS.md` → Metrics.
