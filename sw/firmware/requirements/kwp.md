# KWP2000 Client Component — Requirements

Component to create: `components/kwp` (ESP-IDF C++, house conventions as `components/obd2`).

Purpose: KWP2000 (ISO 14230-3) application-layer client riding on the `tp20` transport
(`components/tp20`, see `requirements/tp20.md`). This is the diagnostic language spoken by VW PQ35
modules (Golf VI): engine, transmission, gateway, airbag, cluster, comfort.

RFC 2119 MUST/SHOULD/MAY. Each FR has a test requirement (TR).

---

## 1. Dependencies

| API | Use |
|---|---|
| `tp20::Tp20::openChannel / sendData / recvData / closeChannel` | transport; one channel per ECU |
| `tp20::ChannelIds` | negotiated IDs (not used above tp20, logged for diagnostics) |

This component MUST NOT talk to `CanDriver` directly and MUST NOT own tasks — it is a synchronous
client library called from the scanner (or an API handler with a worker). All timeouts are call
parameters, never infinite.

## 2. Protocol reference

### 2.1 Message format
A KWP2000 message over TP 2.0 is the reassembled payload delivered by `tp20::Tp20::recvData`:
```
[fmt? -- not used on TP2.0] [target][source][service][data...]
  target = ECU VAG address (0x01...), source = 0xF1 (tester)
```
Note: unlike KWP2000-on-K-line there is no format/length/checksum byte — TP 2.0 carries the length
prefix (added by tp20) and there is no checksum. Requests built by this component:
`[target][0xF1][service][params...]` with the length prefix added by tp20.

### 2.2 Services to implement
| Service | Request bytes (after tgt/src) | Meaning | Response |
|---|---|---|---|
| startDiagnosticSession | `10 89` (fallback `10 85`) | enter normal diagnostic session | `50 89 [P2 timing...]` |
| readEcuIdentification | `1A 9B` | VAG part number | `5A 9B [ascii part number...]` |
|  | `1A 96` / `1A 97` / `1A 9C` | SW version / SW number / system name (set MAY vary per ECU — probe list below) | `5A xx [ascii]` |
| readDataByLocalIdentifier | `21 NN` | measuring-block group NN (0x01..0xFF) | `61 NN [4 x 2-byte values + ...]` |
| readFaultCodes | `18 02` | DTC count + records | `58 02 [count] [records: 3 bytes + 2 status bytes each]` |
| clearDiagnosticInformation | `14 FF 00` | clear fault memory — **gated**, see FR-7 | `54` |
| testerPresent | `3E 00` | session keep-alive | `7E 00` |
| stopDiagnosticSession | `10 82` (best-effort) | leave session | `50 82` or negative (ignore) |

Ident record probing list (MAY be a build-time table, tried in order, failures logged not fatal):
`0x9B, 0x97, 0x96, 0x9C, 0x89, 0x8B, 0x9A, 0x9E`.

### 2.3 Responses
- Positive responses carry service + 0x40 in the high bit (e.g. `10` -> `50`).
- Negative: `[0x7F][service][NRC]`. NRC 0x78 (responsePending) is already handled by tp20
  (continues waiting); other NRCs => typed error `KwpNrc{service, nrc}` surfaced to the caller.
- A response whose target/source don't match the channel => drop + counter, never surface.

## 3. Functional requirements

- **FR-1 Message build/parse**: pure functions `kwp::codec` for request framing and response parsing
  of §2.2 — host-testable, no ESP-IDF includes, allocation-free (fixed buffers).
- **FR-2 Session**: `connect(addr)` = tp20 openChannel + `10 89` (retry once with `10 85` on NRC
  0x11/0x12 or negative) + parse the positive `50` response. Response form:
  `50 [session] [P2max:2B] [P3max:2B]` — store the raw P2/P3 bytes in the session record and expose
  them, but functional timeouts remain the call parameters (P2->recv window default 250 ms,
  P3->keep-alive default 2000 ms). Bench task: confirm whether PQ35 modules send P2/P3 and their
  units before trusting the values. `disconnect(addr)` = `10 82`
  best-effort + tp20 closeChannel. All calls take `addr`.
- **FR-3 getIdent**: `getIdent(addr, IdentOut&)` — probes the §2.2 record list, returns whatever
  answered: `{ partNumber, swVersion, swNumber, systemName, coding? }` as fixed-size char fields
  (no heap), plus a bitmask of which records answered.
- **FR-4 readFaultCodes**: returns count + up to 16 DTC records `{uint16 code, uint8 status1,
  uint8 status2}` raw (interpretation/DTC-text is NOT this component's job). Empty => count 0.
- **FR-5 readGroup**: `readGroup(addr, group, GroupOut&)` — `21 group` -> reassembled response;
  returns the raw value words (up to 4 groups of fields as returned) + raw hex. Formula/units
  interpretation is explicitly out of scope (frontend/tinyexpr layer later).
- **FR-6 Keep-alive**: while a session is open, `3E 00` every `keepalive_ms` (default 2000) IF no
  other request was sent in that window (piggyback rule). Uses the caller's worker/task context via
  an optional registered timer callback — MUST NOT create its own task.
- **FR-7 Safety gate**: `clearFaults(addr, confirm)` — returns `ESP_ERR_INVALID_ARG` unless
  `confirm == true` AND the API layer passes a per-request confirmation token (see vag_diag
  requirements). Component logs a WARNING on every actual `14` send.
- **FR-8 Concurrency**: one outstanding request per ECU address (mutex per session). Concurrent
  calls for different addresses MUST be possible (independent channels).
- **FR-9 Error mapping**: tp20 DEAD channel / timeouts => `ESP_ERR_TIMEOUT` + session marked
  unhealthy; next call re-runs `connect()` transparently at most once.

## 4. Public API (header sketch — implement exactly these)

```cpp
namespace kwp {
struct IdentOut {
    char partNumber[16]; char swVersion[16]; char swNumber[16]; char systemName[24];
    uint8_t answeredMask;                       // bit per probed record
};
struct DtcRecord { uint16_t code; uint8_t status1; uint8_t status2; };
struct FaultsOut { uint8_t count; DtcRecord records[16]; };
struct GroupOut  { uint8_t group; uint16_t raw[8]; uint8_t rawCount;
                   uint8_t hex[64]; uint8_t hexLen; };

class KwpClient {
public:
    static KwpClient& getInstance();
    esp_err_t init();  esp_err_t deinit();      // deinit disconnects all sessions

    esp_err_t connect(uint8_t addr, uint32_t timeout_ms);
    esp_err_t disconnect(uint8_t addr, uint32_t timeout_ms);
    bool      isConnected(uint8_t addr) const;

    esp_err_t getIdent(uint8_t addr, IdentOut& out, uint32_t timeout_ms);
    esp_err_t readFaultCodes(uint8_t addr, FaultsOut& out, uint32_t timeout_ms);
    esp_err_t readGroup(uint8_t addr, uint8_t group, GroupOut& out, uint32_t timeout_ms);
    esp_err_t clearFaults(uint8_t addr, bool confirm, uint32_t timeout_ms);

    struct Nrc { uint8_t service; uint8_t nrc; };       // last NRC per addr, for UI
    Nrc lastNrc(uint8_t addr) const;
};
} // namespace kwp
```

## 5. Non-functional requirements

- NRF-1 Fixed buffers everywhere; no heap after init.
- NRF-2 All protocol constants in one header (`kwp_protocol.hpp`) — no magic numbers at call sites.
- NRF-3 Codec unit-testable on host (`kwp_codec` has no ESP-IDF includes).
- NRF-4 Log tag `kwp`; requests/responses at DEBUG as hex; NRCs at INFO; failures at WARN.

## 6. Test requirements

- **TR-1 Host unit tests for the codec**: build every request byte-exact (`10 89`, `1A 9B`, `21 05`,
  `18 02`, `14 FF 00`, `3E 00` with tgt/src); parse positive responses incl. multi-record fault
  frames; parse NRC frames; drop mismatched tgt/src.
- **TR-2 Mock-transport tests**: a scripted `tp20` test double (interface seam: `KwpClient` takes an
  optional `ITransport*` for tests) covering: connect happy path incl. `10 85` fallback, ident probe
  list (some records negative), faults parse, group read, NRC 0x78 passthrough (tp20 handles it),
  session-retry on DEAD.
- **TR-3 Target integration test** (loopback, fake ECU per TR-2 script): full connect->ident->faults->
  group->disconnect over TWAI loopback with the tp20 component.

## 7. Acceptance criteria

1. TR-1..TR-3 pass in CI (host) / on target (loopback).
2. Against a real Golf VI engine over the bench: `connect` + `getIdent` returns the part number
   (matches the label on the ECU), `readGroup(1)` returns live values that change between reads.
3. `clearFaults` without `confirm == true` returns `ESP_ERR_INVALID_ARG` and sends nothing (assert in test).
4. Session left open > 60 s with only keep-alive traffic stays healthy; a bus loss marks unhealthy and
   the next call reconnects transparently.
