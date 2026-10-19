# Power, sleep and wake

The supervisor keeps the device from draining the car battery when the car is off. The production state machine is in `sw/firmware/main/src/supervisor_core.cpp`; `supervisor.cpp` supplies ESP policy operations, `application_components.cpp` configures components through their public APIs, and main owns their registration descriptors. Everything under "Design intent" is not in the code.

## As built

### What exists
- Normal sleep uses **automatic light sleep**, driven by a timer. ERROR uses timer deep sleep and reboots on wake. No GPIO wake, IMU code, ignition sense or modem.
- Power management is configured at startup: CPU 240 MHz max, 40 MHz min, `light_sleep_enable = true` (`CONFIG_PM_ENABLE=y`). Application wiring creates an `ESP_PM_NO_LIGHT_SLEEP` lock, private to that module, and holds it for the whole awake time. Supervisor sleep operations release it only in SLEEPING and reacquire it after the delay through narrow error-returning wrappers. Null-handle success, create/acquire failure cleanup and SDK error propagation are unchanged.
- The only wake source is the timer. The sleeping task just calls `vTaskDelay(sleep time)`; the FreeRTOS idle task then enters light sleep, and the same delay brings the device back. `esp_sleep_enable_timer_wakeup()` is also called with the same time.

### States (`SUPERVISOR::State`, shown as a number by `GET /api/v1/system`)
```
UNINITIALIZED(0) → STARTING(1) ─ all critical steps ok → NOT_CONNECTED(2) ⇄ RUNNING(3)
                        │ critical step failed
                        ▼
                     ERROR(6)  (LED blinks 30 s, modules shut down, deep sleep on a timer, reboot on wake; see below)

NOT_CONNECTED ─ bus down, V ≤ 13.2, 60 s ─→ STOPPING(4) ─ deinit ok ─→ SLEEPING(5)
SLEEPING ─ wake, CAN init ok, bus alive ─→ restart OBD, Wi-Fi, web server ─→ NOT_CONNECTED
SLEEPING ─ wake, bus dead ─→ stay SLEEPING (CAN deinit, transceiver standby, sleep again)
SLEEPING ─ CAN init fails after wake ─→ ERROR (same back-off and deep sleep)
SLEEPING ─ OBD / Wi-Fi / web server start fails after wake ─→ STOPPING (retry the shutdown)
STOPPING ─ web server, OBD2 or CAN deinit fails ─→ stays STOPPING, retried every second
STOPPING ─ ten failed attempts, next pass ─→ ERROR (bounded blink, deep-sleep backoff)
```
The supervisor loop runs once per second (`vTaskDelay(1000)`), and each pass also calls `SDCard::update_card_status()`, which waits at most 250 ms for the SD lease and skips the round when a download, upload or delete holds it (those end by deadline: download 5 min, upload 60 s, delete 30 s). `card_present()` and `is_mounted()` take no lease.

- `NOT_CONNECTED` → `RUNNING` when `CanDriver::isBusConnected()` is true. The driver declares the bus connected after 3 successful pings, sent every 2 s (`can_health_check`), with no ACK or stuff errors.
- `RUNNING` → `NOT_CONNECTED` when the bus is no longer connected. The OBD2 "disconnected" callback does the same, but only from RUNNING (an atomic compare-and-set), so it cannot overwrite ERROR, STOPPING or SLEEPING. State lives in the core's `std::atomic`.
- `ERROR` (`main/include/error_policy.hpp`): the LED blinks for 30 supervisor iterations, then the supervisor makes one ordered cleanup attempt and calls `esp_deep_sleep_start()` with a timer wake. Cleanup stops at the first failure, retaining providers still used by a dependent until deep sleep powers the CPU off. The backoff is 60 s doubled for every recovery episode, capped at 3600 s; the counter increments once per episode and saturates at 16. The count lives in `RTC_NOINIT_ATTR` variables and is trusted only with the matching magic word. Completing startup does not clear it: only successful orderly STOPPING teardown before normal sleep does. ERROR cleanup does not clear it. A factory reset after `ESP_ERR_NVS_CORRUPT_KEY_PART` also counts in it; after 3 in a row the section fails instead. Critical boot init failure, CAN wake init failure and exhausted shutdown attempts all use ERROR. The SDK deep-sleep call does not return.
- Without an explicit `stop()`, RUNNING never sleeps; normal inactivity sleep goes through NOT_CONNECTED. `stop()` requests STOPPING directly from an awake state.

### Cooperative stop and lifetime
`SUPERVISOR` is process-lifetime: only `getInstance()` constructs it; external construction/destruction is not supported. `stop()` is nonblocking and never deletes its task. One internal mailbox coalesces requests. A request during STARTING waits for startup to finish; the supervisor performs teardown on its own task. Successful cleanup acknowledges SLEEP_READY, while entry to ERROR acknowledges RECOVERY, not successful teardown. Calls before start, in ERROR or while SLEEPING are ignored. The acknowledgment is not a task-exit notification: normal timer wake/probe still follows. There is no public waitable stop API, so workers and callbacks cannot block a teardown dependency, and there is no waiter timeout/cancellation or caller-owned completion storage.

`init(SettingsBootstrap)` first validates/freezes the bounded component registry, then synchronously runs the application-owned settings bootstrap on its caller before asynchronously starting the supervisor. Graph rejection has no bootstrap/NVS/component/task side effects and leaves registration open; main logs the error and returns rather than waiting for RUNNING. A missing bootstrap callback or policy reader returns INVALID_ARG without freezing. SETTINGS is a reserved compatibility prefix, not a normal initializer. Its exact bootstrap result is published first; failure starts only an ERROR recovery task, never any ordinary component. If task creation fails, `init()` runs that same bounded ERROR recovery on its caller's stack without another allocation. This exceptional path blocks the caller for the blink interval and normally never returns because it enters deep sleep. LED resources initialized by `app_main` are reused, not recreated. An accepted init returns task-admission success, not component/bootstrap success; failed bootstrap with an ERROR owner is not discarded as an ordinary main rejection.

Bootstrap work is synchronous and may take SDK initialization time on main's stack; no new timeout/retry/erase policy is introduced. It performs the unchanged NVS init/erase/retry and System-section loading, leaving CAN/Wi-Fi section loading to their own application init adapters. A proven corrupt section still uses the existing bounded RTC recovery operation. Registration closes before bootstrap and never reopens after successful admission, including failure; repeated/reentrant init cannot rerun NVS. Context/code use firmware lifetime in production, error/policy snapshots are copied, and no bootstrap callback storage is retained. This caller-context change and actual SDK/NVS timing require target validation.

Timers retain the legacy iteration accounting: the task delays 1 s after each pass, in addition to bounded SD polling and component calls. The 60-count inactivity threshold and 30-count ERROR blink are not hard wall-clock deadlines.

### Sleep decision
Authenticated `POST /api/v1/system/sleep` requests the same cooperative stop path from RUNNING or NOT_CONNECTED. The handler reserves a single slot, sends 202, then publishes only on SDK send success (otherwise cancels). Duplicates or unavailable states get 503 BUSY; no handler waits for teardown and no fixed grace delay is promised. Acceptance is not proof of client receipt or sleep. Generation-tagged reservations cannot carry a completion into a later wake cycle, and ERROR or independent automatic shutdown may supersede them. See `api.md` for strict request shape and admission details. Manual sleep is not permanent off: the existing timer wake/probe and live-bus resume below remain unchanged.

The thresholds below are defaults. Persisted `system.stay_awake_voltage_mv` and `system.low_battery_voltage_mv` replace them after reboot (9000..16000 mV inclusive, low strictly below awake). The settings are sampled once after successful synchronous bootstrap, before task publication; no per-iteration NVS reads are added. Failed bootstrap/invalid policy uses safe defaults. A valid snapshot is retained even if subsequent task creation fails; ERROR recovery still follows its independent backoff policy.

`system.sleep_enabled` defaults true. Setting it false prevents automatic inactivity sleep at any battery voltage, including low/unknown voltage, and can drain the car battery. Explicit `stop()` still requests normal ordered sleep, and ERROR still deep-sleeps with recovery backoff. This override does not force sleep or change durations. GET/PATCH settings use the stored next-boot policy; changing settings does not mutate the running core's snapshot.

| Input | HW | Signal | Threshold / timeout |
|---|---|---|---|
| CAN bus health | v1.2 | `CanDriver::isBusConnected()`: false when more than 3 consecutive ACK errors or stuff errors are seen, or the controller is bus-off | the bus must stay down for 60 s (`SLEEP_TRANSITION_TIMER_S`, one count per 1 s loop) |
| OBD port voltage | v1.2 | `battery_read()`: ADC1 channel 1 (GPIO 2 on the ESP32-S3), 12 dB attenuation, 12 bit, 16 sample average, divider 1 MΩ / 270 kΩ (ratio 4.704), rounded to 0.1 V. Returns NaN ("unknown") when the ADC is not initialised or any of the 16 conversions fails; the supervisor then logs it, never counts it as above 13.2 V, and sleeps for the normal 10 s. `/system` and the `0x03` frame report unknown as 0. Uses the raw count scaled to 3300 mV, no ADC calibration | above **13.2 V** (alternator running): stay awake and restart the 60 s count. At or below: the count runs |
| Battery voltage while asleep | v1.2 | same ADC | below configured low threshold (default **11.4 V**): sleep 60 s per cycle instead of 10 s |
| Ignition sense (OBD pin 1) | v2 | – | not built |

The default stay-awake threshold is 13.2 V. At exactly the threshold, inactivity counts; above it, the count resets. At exactly the low threshold, sleep uses 10 s, not 60 s. Unknown voltage uses inactivity counting and 10 s sleep unless automatic sleep is disabled.

### Sleep cycle (SLEEPING state)
1. Read the voltage. Sleep time is 10 s, or 60 s below the configured low threshold (default 11.4 V).
2. Release the PM lock, `esp_sleep_enable_timer_wakeup`, `battery_deinit()` (the ADC unit is released for the sleep), `vTaskDelay`. After the delay `battery_init()` again (a failure leaves the voltage "unknown", the device still sleeps normally) and re-acquire the lock.
3. Set the CAN transceiver RS pin low (high speed), run the registered application `setupCan()` adapter (re-reads the CAN config from NVS).
4. `quickCheckBus()`: sends one OBD request (`0x7DF`, mode 0x01 PID 0x00), waits 50 ms and returns true only if no ACK or stuff errors were counted.
5. Bus alive: application adapters `setupSdCard()` (the card was unmounted for the sleep; a failure only disables SD features), `setupObd()`, `setupWifi()`, `setupWebserver()`, state NOT_CONNECTED. Bus dead: the already-stopped web record is skipped, then deinit CAN, RS pin high (standby), stop the LED and sleep again. A cleanup failure goes through STOPPING.

The cycle is about `sleep time + 1 s` (supervisor loop delay) plus the wake work. Each wake transmits one CAN frame on the real bus, a standard OBD request. Sleep current and wake duration have not been measured (`plan/STATUS.md` → Metrics).

### Observed on a board (2026-10-10, HW v1.1, bench, `DEBUG_MODE 0`, nothing on the OBD port)
- `DEBUG_MODE` is only the default for the CAN `debug` setting. A stored `c_debug` in NVS wins, so a device that ever ran with debug on keeps the loopback and simulator until `PATCH /api/v1/settings` sets `{"can":{"debug":false}}` and the device reboots. Without that the bus is "connected" and the device never sleeps.
- Boot, then `Waiting for CAN bus connection...` every second (NOT_CONNECTED, Wi-Fi and web server up).
- 60 s after boot: `CAN disconnected for 60s (Voltage 1.00V), transitioning to SLEEP`, one second later `Shutting down modules for sleep...` (Wi-Fi goes down), then `Entering Light Sleep...`.
- The voltage read 0.9 to 1.0 V (no supply on the OBD pin), which is below 11.4 V, so every cycle is 60 s with `Battery low ... Sleeping aggressively...`. Cycles seen: sleep at 64.3 s, wake at 124.3 s (`CAN driver initialized`, `Bus is dead. Going back to sleep.`), sleep at 125.3 s, wake at 185.3 s, and again. So a cycle is the sleep time plus about 1 s, as written above. No panic, no reset loop in 220 s.
- While it sleeps the board has no Wi-Fi and no API, and the USB serial port does not answer `esptool` until the next wake window; reflashing needs several `--connect-attempts` (or a power cycle).
- Sleep current was not measured (software cannot measure it).

### Deinit and init order
Before sleep (STOPPING):
1. `stop_web_server()`: asks `WSDataStreamTask` to leave its loop and waits up to 1 s for it to exit by itself, stops `AsyncWebServer` (workers, httpd), then `button_deinit()` (removes the GPIO0 ISR handler, the `button` task exits, mutex and semaphore are freed, the pin goes back to its reset default; a task that does not leave within 500 ms returns `ESP_ERR_TIMEOUT` and the sequence retries), and only then deletes the task's semaphore, so the WebSocket handshake handler never sees a deleted semaphore. Waits at most 3 s for the workers; if one is still inside a handler it returns `ESP_ERR_TIMEOUT`, nothing further is stopped and the sequence retries on the next pass.
2. `OBD2::deinit()`: stops its three tasks, clears PID values, VIN, DTCs and all subscriber lists, deletes its queues and semaphores. PID definitions stay. If it times out (5 s), the sequence stops here and retries on the next pass.
3. `WIFI::deinit()` returns `esp_err_t`; a failure stops the sequence and retries. It closes scan admission, signals cancellation and waits up to 2.5 s for the scan worker before stopping/freeing the driver. No worker drain means no resource release. It unregisters Wi-Fi/IP handlers, stops/deletes the deferred-reconnect timer with a bounded callback drain, deinitializes the driver and drains queued events through a default-loop barrier before destroying netifs with `esp_netif_destroy_default_wifi` (also detaches their driver objects/default handlers). The scan snapshot is cleared; synchronization objects belong to the singleton and are reused on wake. It also frees mDNS (`mdns_free`) and clears the flag, so the next `start()` binds mDNS to the new netifs. These waits exclude additional bounded mutex waits and SDK-call latency. ERROR sleep makes only one ordered attempt, stopping at any failed barrier.
4. `CanDriver::deinit()`: stops the simulator, disables the node, then asks the TX and health tasks to exit (flag and notify, bounded 3 s wait, each task deletes itself) before deleting the TWAI node, queues, semaphore and TX ring. Safe to call twice. A task that does not leave in time returns `ESP_ERR_TIMEOUT` with nothing freed; an error stops the sequence and retries.
5. `SDCard::deinit()`: unmounts the card (flushes the FAT) and releases the card-detect pin. Runs only after the web server and OBD2 are down. A transfer holding the lease makes it time out after 5 s; the sequence stops and retries on the next pass. After it, the per-second `update_card_status()` poll does nothing.
6. RS pin high (transceiver standby), `LedStatus::deinit()` (LED off and worker released; its semaphore remains reusable). An LED stop error also retries instead of declaring sleep ready. Successful cleanup resets recovery history, then enters SLEEPING.

The ERROR state's cleanup unmounts the card only if earlier dependency barriers succeed. Components that never started or already stopped are skipped; partially initialized components remain eligible for cleanup. A failed cleanup retains the component's resources and records its error.

After wake: `LedStatus::init()`, CAN, then (bus alive) `SDCard::init()`, OBD (which re-subscribes the connected-callbacks and reloads the PID JSON from the SD card), Wi-Fi, web server (which re-subscribes the PID stream callback).

Additional NORMAL registrations join the dependency-ordered boot/live-bus wake plan and consumer-first shutdown plan without new policy code. They remain stopped on dead-bus wakes. Shutdown stops at the first failed cleanup, retaining every later provider; retry skips completed records. A normal extension's wake-init failure enters STOPPING like OBD/Wi-Fi/web, even if it is noncritical at boot. RETAINED extensions initialize only at boot and remain allocated across normal sleep. Explicit lifetime validation prevents retained or early wake-phase consumers depending on providers stopped during their required phase. The eight existing registrations preserve the orders above, battery placement, SD wake-failure tolerance, thresholds, iteration timers and recovery; SETTINGS is the separate retained bootstrap prefix. Neither NVS initialization nor supervisor policy loading repeats on normal sleep/wake. See architecture.md for descriptor bounds and ownership.

Setup/cleanup configuration and subscriptions now live in application wiring rather than SUPERVISOR setup/stop methods. The OBD callback uses `notify_disconnected()` without core access; settings section loaders use `recover_corrupt_settings()` only for the existing corrupt-partition branch. The latter retains the same RTC magic/count, three-attempt limit, factory-reset call, restart and failure return. No raw RTC state or broad friend access is exposed. CAN probe/battery/sleep policy and existing SD polling remain in supervisor platform integration; this is not a generic power-policy rewrite.

Not deinitialized before sleep, and have no `deinit()`: Settings (NVS), power management and the `www` LittleFS mount (see `decisions.md`, 2026-10-18 deinit exceptions). `battery_deinit()` runs around the light sleep (step 2 of the sleep cycle), not in STOPPING, because the SLEEPING state reads the voltage on every pass, including the dead-bus passes that skip STOPPING.

### Where it differs from a battery-safe design
- ERROR state: fixed by P2.9e, see States.
- Without an explicit stop request, the device stays awake while RUNNING, while automatic sleep is disabled, or while disconnected-bus voltage stays above the configured stay-awake threshold (default 13.2 V).
- `main()` keeps polling the supervisor state every 250 ms until it is RUNNING, including while the device sleeps.

### BOOT button (GPIO0) and sleep
- The button is not a wake source. The only wake source is the timer, and the web server (the only user of the button) is down while the device sleeps, so a press then has no effect and nothing is latched.
- It lives and dies with the web server: initialised at the end of `setup_web_server()`, released in `stop_web_server()`. Every wake that brings the web server back also re-initialises it, and "a press since boot" means a press since that init (`button_last_press_us()` is 0 after init).
- While idle the button costs nothing: no timer, no polling; the task blocks on a notification and the pin interrupt (both edges) is the only wake-up. It does not hold a PM lock, and the supervisor's `ESP_PM_NO_LIGHT_SLEEP` lock is held while awake anyway.
- GPIO0 is a strapping pin: holding it low at reset enters download mode. The driver only reads it (input, internal pull-up) and never drives it. A button that is already down when the driver starts (held through a reset or a wake) is ignored until it has been released once, so it can neither confirm a claim nor start a recovery hold.
- The hold gesture (10 s) is wired to PIN recovery by `auth_service` (`docs/api.md`, Authentication). `auth_service::stop()` runs in `stop_web_server()` just before `button_deinit()`: it clears the hook (waiting for a running recovery), forgets all sessions; the failed-login count stays in RTC memory and is restored by the next `auth_service::start()`. Its mutex is static and never freed, so a recovery that is still waiting for it cannot touch freed memory. The PIN record stays in NVS. So every sleep ends all sessions; after a wake the web server comes back and a login is needed again.
- Cost: two `esp_timer`-free static objects, one 4 KiB task (blocked), one mutex. A PIN derivation runs only on claim, login and PIN change (about 5000 HMAC iterations, a few hundred ms of CPU at most, never while idle).

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
