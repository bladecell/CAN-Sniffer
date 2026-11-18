# Bench CAN simulator

Run the simulator on a second CAN Sniffer PCB. It acts as the car on an isolated bench bus while the first board runs the normal sniffer firmware. Do not connect the simulator to a vehicle.

## Wire and flash

1. With both boards unpowered, connect CAN-H to CAN-H, CAN-L to CAN-L, and ground to ground. Use one 120 Ω termination at each end of the bus; count any termination already on the boards.
2. Power the boards separately. Identify the simulator's USB device before flashing or starting the host. Both boards may appear as `/dev/ttyACM*`.
3. Activate ESP-IDF 6.1 and run these commands from the repository root. In `menuconfig` under **Isolated CAN simulator**, select the simulator PCB revision and leave the GPIO overrides unset unless your wiring differs. See [Hardware](hardware.md) for the board pinout.

   ```sh
   . ~/.espressif/tools/activate_idf_v6.1.sh
   idf.py -C sw/simulator/firmware menuconfig
   idf.py -C sw/simulator/firmware -p /dev/ttyACM0 flash
   ```

4. Use the same CAN bitrate on both boards. Close `idf.py monitor` before starting the host so only one process reads the simulator's USB serial port.

## Start the host

```sh
cd sw/simulator/host
(cd ui && bun install --frozen-lockfile && bun run build)
SIM_PORT=/dev/ttyACM0 bun run start
```

Open [127.0.0.1:8787](http://127.0.0.1:8787/) after building the web UI. Select `example`, wait for the USB connection, and push the profile. The workbench shows live CAN traffic and lets you change PID values, apply ECU faults, or run a profile scenario. Omit `SIM_PORT` to open the workbench without a board; live controls need a connection.

For CLI use, check the connection and push the sample profile from a second terminal:

```sh
curl -fsS http://127.0.0.1:8787/status
curl -fsS -X POST http://127.0.0.1:8787/profiles/example/push
```

The host listens only on `127.0.0.1:8787` and has no login. Open that exact address rather than `localhost`. The browser UI uses same-origin requests; native HTTP and WebSocket clients can still connect without an `Origin` header. Keep it on the bench machine; do not expose it through a proxy or port forward. Optionally set `SIM_DEFAULT_PROFILE=example` to push that profile on each USB connection, or set `SIM_STREAM_MAX_PER_SEC` to change the frame stream limit (default 50).

| REST | Use |
|---|---|
| `GET /status` | Read serial connection state and simulator statistics |
| `GET /profiles` | List profiles |
| `GET`, `PUT`, `DELETE /profiles/:name` | Read, replace, or remove a profile JSON file |
| `POST /profiles/:name` | Import PID definitions with `{"action":"import-pids","ecu":"engine"}` |
| `POST /profiles/:name/push` | Send a profile to the ESP and save it |
| `POST /pid`, `/fault`, `/dtc`, `/vin`, `/ack` | Change a live setting with a JSON body; use the corresponding USB command fields without `cmd` or `seq` |
| `POST /scenario/start`, `/scenario/stop` | Start with `{"profile":"example"}` or stop the current scenario |

Connect to `ws://127.0.0.1:8787/ws` for events. Browser sockets are read-only; the workbench sends commands over same-origin HTTP with `X-Simulator-Intent: ui`. Native WebSocket clients without an `Origin` header can also send the REST action fields plus `action`, for example `{"action":"ack","ack":false}`. Use `scenario.start` and `scenario.stop` for scenario control.

## Profiles

Start from [`example.json`](../sw/simulator/host/profiles/example.json). It defines an engine ECU and a second ECU with DTCs and a VIN, plus a UDS DID, a cyclic frame, and a custom rule. A profile groups bus settings, ECUs, rules, cyclic frames, and an optional host scenario. The host pushes the configuration over USB; the ESP answers CAN requests without waiting for the host.

- Each ECU has physical request and response IDs. All configured OBD ECUs also respond to functional requests on `0x7DF`. PID support pages and PID `01` (MIL and stored DTC count) come from the ECU configuration.
- Set PID data as raw bytes, an engineering value, or a generator (`const`, `ramp`, `sine`, `noise`, `walk`). When a PID has a scale and offset, its engineering value is `raw * scale + offset`.
- The host can import the sniffer's [`pid_def.json`](pid-config.md); set `PID_DEF_PATH` to use a different file. Affine arithmetic formulas map engineering values to big-endian raw bytes. Unsupported formulas produce a warning and can only be written as raw bytes. For a live engineering write using an imported definition, include the profile name: `{"profile":"example","ecu":"engine","mode":1,"pid":12,"value":2100}`.
- Keep stored, pending, and permanent DTCs separate. The `clearable` setting controls whether OBD service `04` succeeds. `onUnknown` controls whether an unsupported request is ignored or receives a negative response.
- Use `silent`, `delayMs`, or `nrc` to exercise client error paths. NRC `0x78` sends response-pending before the normal reply, delayed by `pendingMs`. Disabling ACK puts the simulator into listen-only mode.
- A rule matches a CAN ID and mask, frame format, DLC, and optional masked byte pattern. Its actions can send raw frames or one ISO-TP payload; `$0` copies the first request byte. A cyclic entry sends a frame independently of requests.
- Scenario steps run on the host at `atMs` and apply a PID update, fault, or DTC change. The ESP does not replay scenario timing on boot.

## USB protocol

Send one JSON object per line, for example `{"cmd":"hello","seq":1}`. Each command produces a reply with `ok`, the supplied `seq`, and an `err` string on failure. The ESP interleaves replies with events carrying an `ev` field (`boot`, `frame`, or `drop`). A `frame` event includes direction, ID, frame format, data, and a microsecond timestamp; `unmatched: true` marks RX frames that produced no match. `stream` enables frame events and sets `maxPerSec`.

| Commands | Use |
|---|---|
| `hello`, `stats` | Identify the simulator and read runtime counters |
| `reset`, `save` | Erase saved configuration and reset to an empty 500 kbit/s, ACK-on model, or save the staged configuration for boot-time replay |
| `can.config` | Set `bitrate` and `ack` |
| `ecu.add`, `ecu.fault` | Create an ECU or configure its fault |
| `pid.add`, `pid.set`, `dtc.set`, `vin.set`, `did.set` | Configure ECU responses |
| `rule.add`, `rule.clear`, `cyclic.add` | Add a rule, clear rules and cyclic frames, or add a cyclic frame |
| `stream` | Control USB frame telemetry |

Use JSON integers for wire IDs, PIDs, and DIDs, and arrays of integers for bytes. DTCs are strings such as `P0301`.

## Limits

The simulator uses Classical CAN. The OBD-II and UDS presets use 11-bit IDs; custom rules can use 11- or 29-bit IDs. ISO-TP flow control uses the physical request ID for an ECU and the actual matched request ID for a custom rule. Payloads are limited to 256 bytes. CAN-side DTC clears and UDS sessions do not persist across reboot. HW v2 has no pin preset. The simulator is separate from the sniffer's built-in simulator and does not change it.
