# VAG Diagnostics (TP 2.0 / KWP / UDS) — Implementation Outline

Target: Golf VI (5K, PQ35). Companion docs: `~/dev/vcds/VCDS_AutoScan_Analysis.md` (VCDS.exe internals),
`~/dev/vcds/HEX_VII_Firmware_Analysis.md` (HEX-V2 firmware), TP 2.0 spec: https://jazdw.net/tp20

---

## 0. Scope decisions

- **Transport #1 (primary): VW TP 2.0** — engine (0x01), transmission, gateway (0x19), airbag (0x15),
  instrument cluster, comfort modules. Application layer on top: **KWP2000 (ISO 14230-3)**.
- **Transport #2 (secondary): UDS over ISO-TP (15765-2)** — newer/other modules; reuse the ISO-TP code
  already in `components/obd2` (`IsoTp` enum, `obd2_frame.cpp`, `raw_flow_control`).
- **No K-line** (CAN-only hardware). No TP 1.6 initially (Touareg/Phaeton only).
- Read-only by default: ident + fault codes + groups. `0x14` (clear DTCs) behind explicit user confirm.
- Everything surfaced through the existing web API / WebSocket, recorded via `recorder`.

## 1. New components

```
components/
  tp20/            VW TP 2.0 transport (channel state machine)
    include/tp20.hpp            public API
    tp20.cpp                    channel open/close, params, keep-alive, RX demux
    tp20_frames.cpp/hpp         frame codec (pure functions, unit-testable)
    tp20_rx.cpp                 RX pump + dispatch
  kwp/             KWP2000 application client (rides on tp20)
    kwp_client.cpp/hpp          session, ident, DTC, group reads
  vag_diag/        Scanner / orchestrator (the "AutoScan" engine)
    vag_diag.cpp/hpp            scan task, per-address strategy, results model
    vag_address_db.cpp/hpp      chassis -> address DB (AutoScan.txt format, LittleFS)
    vag_results.cpp             result records -> JSON/WS events
```

Naming/structure follows house style: singleton classes, `init()/deinit()` lifecycle,
FreeRTOS tasks pinned with `CORE_ID_CAN_TASKS`, hooks instead of callbacks where the
ISR/queue boundary matters, `-Wall -Wextra -Werror` clean.

## 2. TP 2.0 transport (components/tp20)

### 2.1 CAN-ID model (from jazdw.net/tp20, verified against VCDS behavior)
- Channel setup request: **CAN ID 0x200**, 7 bytes.
- Channel setup response: **CAN ID 0x200 + module_addr** (engine 0x01 -> 0x201).
- After setup, both sides switch to the **negotiated IDs** (ECU typically tells us TX=0x740;
  we request it to listen on 0x300–0x310).

### 2.2 Frame formats (classic CAN, 8-byte data)
```
Channel setup (7B): [dest][0xC0][RX_ID][V|RXPref][TX_ID][V|TXPref][app=0x01]
  response opcode 0xD0 positive, 0xD6..0xD8 negative
  "Pref" byte = prefix in high nibble + validity flag (0x1 = invalid)
Channel params (6B): [op][BS][T1][T2][T3][T4]
  0xA0 request (ECU->tester), 0xA1 response, 0xA3 test/keep-alive (1B), 0xA4 break, 0xA8 disconnect (1B)
  timing byte: units<<6 | scale   (units: 0=0.1ms, 1=1ms, 2=10ms, 3=100ms)
Data (2..8B): [op][seq][payload...]
  op 0x0 waiting-for-ACK + more, 0x1 waiting-for-ACK + last, 0x2/0x3 no-ACK variants
  op 0xB ACK ready-for-next, 0x9 ACK not-ready   (seq nibble in byte 1, wraps 0xF)
Broadcast (7B): [dest][0x23/0x24][kwp data...][resp-req 0x00/0x55/0xAA]
```

### 2.3 Channel state machine (per open channel)
```
IDLE --C0(0x200)--> WAIT_SETUP_RESP (0x200+addr, timeout ~250ms, retry x2)
  --D0--> NEGOTIATED --A1(params)--> OPEN
OPEN: send/recv data frames; ACK per BS block size; seq tracking (4-bit)
  keep-alive: A3 every ~T1*0.7 (task-timer), miss 2 -> channel dead -> IDLE
  teardown: A8 + wait A8 echo
```
One task (`tp20Task`) owns the channel: reads `can_driver.receive()` filtered by the
channel's two IDs (plus 0x200/0x200+addr during setup), sends with `transmitUrgent()`
(ACKs and flow-critical frames must preempt — same reasoning as the existing ISO-TP FC).
A small registry (max 2–3 concurrent channels; the Reddit report confirms multi-channel
works — one for gateway 0x1F/0x19, one for a module).

### 2.4 API sketch
```cpp
class Tp20 {
    esp_err_t openChannel(uint8_t vagAddr, uint32_t timeout_ms);   // full handshake incl. params
    esp_err_t sendData(const uint8_t* payload, size_t len, int timeout_ms);  // fragments + ACKs
    esp_err_t recvData(uint8_t* buf, size_t& len, int timeout_ms); // reassembles [len_hi len_lo data]
    esp_err_t keepAlive();          // A3
    esp_err_t closeChannel();       // A8
    bool      isOpen() const;
};
```
Note: the first 2 payload bytes of a KWP message are its 16-bit length (big-endian);
`recvData` reassembles multi-packet messages into one buffer before returning.

## 3. KWP2000 client (components/kwp)

Rides on one TP2.0 channel. Golf VI application layer (ISO 14230-3, target=module addr, source=0xF1):
```
0x10 89      startDiagnosticSession (normal session; try 0x85 if 0x89 rejected)
0x1A 9B      readEcuIdentification -> VAG part number
0x1A 89/8B/9C...  further ident records (validate set on the car; VCDS prints
                  "P/N / ASAM / GWSW / GWHW" — each maps to one 0x1A xx record)
0x21 NN      readDataByLocalIdentifier -> measuring-block group NN (0x01..)
0x18 02      readFaultCodes (DTC count + records)
0x14 FF 00   clearFaultCodes (gated behind user confirmation)
0x3E 00      testerPresent keep-alive at KWP level (in addition to TP2.0 A3)
```
Client exposes: `getIdent()`, `readFaultCodes()`, `readGroup(n)`, `clearFaults(confirm)`,
each returning parsed structs + raw hex for the UI.

## 4. UDS client (secondary)

Reuse `obd2_frame.cpp` ISO-TP + `raw_flow_control` patterns; factor what's generic into
`components/obd2` or copy small. Services: `10 03`, `22 F1 87/89/8A/90/9E`, `19 02 09`,
`14 FF FF FF` (gated), `3E 00`. Request IDs: `0x7E0–0x7E7` (+`0x7DF` functional).
Used for modules that answer ISO-TP on the Golf VI (some do) and for newer cars later.

## 5. Scanner engine (components/vag_diag) — the "AutoScan"

Per VAG address (default list 0x01..0x7F, from the address DB):
1. **TP2.0 probe**: channel setup C0 on 0x200 -> response on 0x200+addr within 250 ms?
   - `0xD0`  -> TP2.0 module: params, `0x10 89`, ident (`0x1A 9B`, …), fault codes (`0x18 02`), A8.
   - `0xD6-0xD8`/timeout -> UDS probe: `0x7Ex` TesterPresent / `10 03` -> ident via `22 F1 …`.
   - silence -> mark absent.
2. **Gateway pass**: open channel to 0x19, read its installation list (the PQ35 gateway
   publishes "module installed" records — capture once on the car to fix the exact KWP
   local-ID, then use it to skip absent addresses). Gateway also yields chassis code ->
   pick DB row (VCDS uses the same trick: gateway VIN/chassis sniffing).
3. **Result record** (one per address, mirroring VCDS's scan line):
   `{ addr, protocol: TP20|UDS|ABSENT, partNumber, swVer, hwVer, coding?, dtcCount, faults[] }`
4. Output: JSON array via REST, live progress via `/ws` (existing websocket), full log
   through `recorder` (SD card) for post-analysis.

Scan task runs at lower priority than TP20/ISO-TP RX handling; reuses the
`tryHoldDiagBus()`-style arbitration so the scanner never fights the OBD2 poller —
recommend: VAG scan and OBD2 polling are mutually exclusive (take over the same
"diagnostic bus owner" slot `OBD2` already manages).

## 6. Integration points (existing code)

| Existing | Used for |
|---|---|
| `CanDriver::transmitUrgent()` | TP2.0 ACKs, channel-setup retries, ISO-TP FC |
| `CanDriver::receive()` + RX queue | single RX pump in `tp20Task` (500k default already correct) |
| `CanDriver::setFrameTap()` | passive logging of all VAG traffic into `recorder` while scanning |
| `obd2_frame` `IsoTp` enum + FC responder | UDS client; keep OBD2's poller idle during VAG scans |
| `api_routes.hpp` X-macro | add `/api/v1/vag/scan`, `/api/v1/vag/module/{addr}`, `/api/v1/vag/groups/{addr}` |
| WebSocket `/ws` | scan progress + live group data events |
| `settings` (LittleFS/NVS) | address DB storage, scan options (bitrate 500k/250k, address list) |
| `obd2_simulator` pattern | `vag_simulator`: replay a recorded TP2.0 session for host tests |

## 7. Bench validation checklist (open items)

1. Channel setup on the Golf VI engine (0x01): confirm negotiated IDs (expect TX=0x740 pattern)
   and that params `A0 0F 8A FF 32 FF` are accepted.
2. KWP ident records: which `0x1A xx` local-IDs the engine/cluster actually implement.
3. Gateway (0x19): the installation-list read (local-ID TBD by capture), and whether the
   gateway's channel requires any security step. **Watch for scrambling**: the VCDS.exe
   reverse-engineering showed a 16-byte obfuscated frame format (XOR key + CPC) on its
   PC-interface link; if our captures show XOR'd bytes on the wire for gateway traffic,
   the gateway channel uses scrambling and we capture the key handshake then.
4. Timing: T1/T3 units per module (start from jazdw defaults `8A FF 32 FF`, adjust).
5. 250 kbit/s check: PQ35 comfort/infotainment buses are 500k; onlyif a gateway-routed
   sub-bus needs it.

## 8. Milestones

1. `tp20_frames` codec + unit tests (pure, no hardware) — frames from the jazdw example as fixtures.
2. `Tpx20` channel open/keep-alive/close against the bench car (engine first).
3. `kwp_client`: ident + fault codes on engine; JSON out.
4. Scan engine + REST/WS surface; address DB loader.
5. Gateway pass (installation list) after a capture session.
6. UDS client for the ISO-TP-capable subset; merge into scan strategy.
7. Polish: recorder integration, simulator, frontend page.

## 9. Safety rails

- Default read-only; `0x14` clear and any `0x2E`/coding writes require explicit per-action
  confirmation flag in the API.
- Always `A8`-disconnect + `0x10 8x` session exit in a destructor path; scan abort must
  not leave open channels (gateway keeps routing state).
- Never auto-clear DTCs as part of scanning.
