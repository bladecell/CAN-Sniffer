// obd2_frame.hpp: pure OBD-II / ISO-TP frame parsing. No hardware, no RTOS, so it builds for the linux host tests.
// Every length and index comes from the bus and is treated as hostile.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "esp_err.h"
#include "obd2_common.hpp"

namespace obd2frame
{

constexpr size_t   CAN_DATA_MAX     = 8;    // classic CAN
constexpr uint16_t MAX_ISOTP_LENGTH = 512;  // cap on a reassembled payload (VIN is 20, DTC lists are far smaller)

enum class IsoTp : uint8_t
{
    Single      = 0,
    First       = 1,
    Consecutive = 2,
    FlowControl = 3,
};

struct RawFrame
{
    uint8_t data[CAN_DATA_MAX];
    size_t  length;
};

struct SingleFrameInfo
{
    uint8_t  mode;           // response SID: 0x41, 0x62, ...
    uint16_t pid;            // 8-bit for 0x41 / 0x42, 16-bit for 0x62 / derived
    uint8_t  payloadOffset;  // index of the first data byte in the frame
    uint8_t  payloadLength;  // data bytes that belong to the message (padding excluded)
};

// ISO-TP frame type from the high nibble of byte 0. Needs at least one byte.
esp_err_t frameType(const uint8_t* data, size_t length, IsoTp& type);

// Validates a single frame (length 2..8, PCI length 1..7 and within the frame, known SID) and extracts mode/PID.
// Modes without a PID (e.g. 0x43, 0x7F) report pid 0 and payloadOffset 2.
esp_err_t parseSingleFrame(const uint8_t* data, size_t length, SingleFrameInfo& out);

// First frame: total payload length and number of consecutive frames still expected.
esp_err_t parseFirstFrame(const uint8_t* data, size_t length, uint16_t& totalLength, uint16_t& framesNeeded);

// True if this is a consecutive frame whose 4-bit sequence matches the 1-based index (wraps 15 -> 0).
bool consecutiveSequenceOk(const uint8_t* data, size_t length, uint16_t index);

// Mode 03/07/0A payload (0x43 / 0x47 / 0x4A, count byte NN, then NN 2-byte DTCs; layout assumed, see obd2_frame.cpp) to raw DTCs.
// Reads at most NN pairs and only those present; no count byte or a half pair gives none.
esp_err_t extractDtcs(const std::vector<uint8_t>& payload, std::vector<uint16_t>& rawDtcs);

// Mode 09 PID 02 payload (0x49 0x02 count, then 17 ASCII chars) to the VIN.
esp_err_t extractVin(const std::vector<uint8_t>& payload, char (&vin)[17]);

// ---- Request frames (8 data bytes, unused bytes 0) ----
// ISO 15765-2 single frame: byte 0 = 0x0 | N, N = number of bytes after the PCI byte (the SID plus its parameters, 1..7).
using RequestBytes = uint8_t[CAN_DATA_MAX];
void buildDtcRequest(uint8_t mode, RequestBytes& out);             // 01 mode          (03 / 07 / 0A, no parameters)
void buildVinRequest(RequestBytes& out);                           // 02 09 02
void buildClearDtcRequest(RequestBytes& out);                      // 01 04            (mode 04 has no parameters)
void buildDiagSessionRequest(uint8_t session, RequestBytes& out);  // 02 10 session
void buildCurrentDataRequest(uint8_t pid, RequestBytes& out);      // 02 01 pid
void buildFlowControl(RequestBytes& out);                          // 30 00 00         (continue to send, no block limit, no separation time)
// A configured PID. The PCI length comes from the mode, never from the caller: mode 01 -> `02 01 pid` (PID 1..0xFF),
// mode 0x22 -> `03 22 didHi didLo`. Any other mode (including derived) or a PID out of range builds nothing: returns false
// and zeroes `out`, so nothing wrong can reach the bus.
bool buildPidRequest(uint8_t mode, uint16_t pid, RequestBytes& out);

// The one rule for a PID definition (REST add, bulk JSON, model): mode 01 with PID 1..0xFF and length 2, mode 0x22 with
// length 3, derived (0x45) with length 0. Anything else would put an invalid single-frame PCI on the bus.
bool isValidPidSpec(uint8_t mode, uint16_t pid, uint8_t length);

// ---- TX-done and flow-control timing (used from the CAN TX-done ISR, so inline and branch-only) ----
// True if the frame that just left the controller is the running DTC / VIN request (mode 0x03/0x07/0x0A, or 0x09 for the
// VIN): functional ID and the exact bytes buildDtcRequest / buildVinRequest produce. Nothing else starts the response timer.
inline bool isDiagRequestFrame(uint8_t requestMode, uint32_t id, const uint8_t* data, size_t length)
{
    if (id != OBD2_FUNCTIONAL_ID || data == nullptr || length < 2)
    {
        return false;
    }
    if (requestMode == MODE_VEHICLE_INFO)  // 02 09 02
    {
        return length >= 3 && data[0] == 0x02 && data[1] == MODE_VEHICLE_INFO && data[2] == PID_VIN;
    }
    const bool dtcMode = requestMode == MODE_DTCS || requestMode == MODE_PENDING_DTCS || requestMode == MODE_PERMANENT_DTCS;
    return dtcMode && data[0] == 0x01 && data[1] == requestMode;  // 01 mode
}

// True for our ISO-TP flow control frame to a physical request ID 0x7E0-0x7E7 (buildFlowControl: first byte 0x30).
inline bool isFlowControlFrame(uint32_t id, const uint8_t* data, size_t length)
{
    return data != nullptr && length >= 1 && data[0] == 0x30 && id >= OBD2_RESPONSE_BASE_ID - RESPONSE_ID_OFFSET &&
           id <= OBD2_RESPONSE_BASE_ID - RESPONSE_ID_OFFSET + 7;
}

// The flow control frame must be on the bus within N_Ar of the first frame being received (wrap-safe, ms).
inline bool isoTpFlowControlLate(uint32_t firstFrameReceivedMs, uint32_t flowControlOnBusMs)
{
    return static_cast<uint32_t>(flowControlOnBusMs - firstFrameReceivedMs) > ISOTP_N_AR_MS;
}

// ---- Multi-ECU response collection (DTC modes 03/07/0A and VIN, mode 09/02) ----
// A functional request is answered by several ECUs at once; each reassembles in its own session keyed by the response
// CAN ID (11-bit 0x7E8-0x7EF). Hardware-free: the caller supplies frames and a millisecond clock (wrap-safe).

constexpr uint32_t DIAG_RESPONSE_ID_FIRST = 0x7E8;
constexpr uint32_t DIAG_RESPONSE_ID_LAST  = 0x7EF;
constexpr size_t   MAX_MERGED_DTCS        = 256;

enum class DiagKind : uint8_t
{
    None,
    Dtc,  // request mode 0x03 / 0x07 / 0x0A
    Vin,  // request mode 0x09, infotype 0x02
};

enum class DiagStatus : uint8_t
{
    Idle,      // no collection running
    Waiting,   // collecting
    Complete,  // at least one ECU answered, every session is finished and the bus went quiet
    Failed,    // nobody answered within the first-response wait
};

struct DiagTimings
{
    uint32_t firstResponseWaitMs;  // from markSent(): no responder by then means Failed
    uint32_t quietGapMs;           // no relevant frame and no active reassembly or pending ECU means Complete
    uint32_t nCrMs;                // per session, restarted by each valid frame
    uint32_t pendingMs;            // P2*: restarted by each valid NRC 0x78
    uint8_t  maxPending;           // consecutive 0x78 accepted per ECU before it is given up
    uint32_t totalCapMs;           // 0 = none
};

// The B33 OBD-II profile from obd2_common.hpp.
DiagTimings defaultDiagTimings();

struct DiagFrameResult
{
    bool      consumed;         // the frame belongs to the running collection; the caller must not parse it further
    bool      sendFlowControl;  // send ISO-TP flow control (continue, no block limit, no separation time) to flowControlId
    uint32_t  flowControlId;    // physical request ID of the ECU: response ID minus 8, never the functional ID
    esp_err_t err;              // ESP_OK, or why the frame was dropped (only that ECU's session is affected)
};

class DiagCollector
{
public:
    explicit DiagCollector(const DiagTimings& timings = defaultDiagTimings(), size_t maxSessions = ISOTP_MAX_SESSIONS);

    // Starts a collection for `requestMode` (0x03, 0x07, 0x0A, or 0x09 for the VIN) and clears the previous result.
    void begin(DiagKind kind, uint8_t requestMode, uint32_t nowMs);
    // The request went out on the bus: starts the first-response wait.
    void markSent(uint32_t nowMs);
    void cancel();

    DiagFrameResult onFrame(uint32_t canId, const uint8_t* data, size_t length, uint32_t nowMs);
    // Expires stale sessions and pending ECUs, then evaluates completion. Call regularly.
    DiagStatus poll(uint32_t nowMs);

    bool   active() const { return kind_ != DiagKind::None; }
    size_t activeSessions() const;
    size_t responders() const { return responders_; }
    // A session is open or some ECU has answered: the result is on its way (a stalled session is dropped after N_Cr).
    bool progressing() const { return activeSessions() > 0 || responders_ > 0; }
    // True while any ECU has an unexpired NRC 0x78 (response pending).
    bool anyPending() const;

    // Merged, de-duplicated raw DTCs (zero DTCs and padding are not stored).
    const uint16_t* dtcs() const { return dtcs_; }
    size_t          dtcCount() const { return dtcCount_; }
    // First valid VIN. False if no ECU sent one. conflictingVins() counts valid but different VINs that were ignored.
    bool   vin(char (&out)[17]) const;
    // What a completed collection means for the requester: ESP_OK, or for a VIN collection without any valid 17-character
    // alphanumeric VIN ESP_ERR_INVALID_RESPONSE (the ECUs answered, the content is unusable). A DTC collection is always OK.
    esp_err_t result() const;
    size_t conflictingVins() const { return vinConflicts_; }

private:
    struct Session
    {
        bool     used;
        uint32_t id;
        uint16_t total;
        uint16_t received;
        uint8_t  nextSeq;
        uint32_t lastMs;
        uint8_t  buf[MAX_ISOTP_LENGTH];
    };

    static bool inRange(uint32_t id) { return id >= DIAG_RESPONSE_ID_FIRST && id <= DIAG_RESPONSE_ID_LAST; }
    bool        matchesService(const uint8_t* data, size_t length) const;
    Session*    findSession(uint32_t id);
    void        dropSession(Session& s);
    void        finishPayload(uint32_t id, const uint8_t* payload, size_t length);
    void        addDtc(uint16_t raw);

    DiagTimings timings_;
    size_t      maxSessions_;
    DiagKind    kind_        = DiagKind::None;
    uint8_t     requestMode_ = 0;
    uint8_t     responseSid_ = 0;
    uint32_t    beginMs_     = 0;
    uint32_t    sentMs_      = 0;
    bool        sent_        = false;
    uint32_t    lastActivityMs_ = 0;
    size_t      responders_  = 0;
    uint16_t    respondedMask_ = 0;  // one bit per response ID: counted once per ECU
    uint8_t     pendingCount_[8]{};
    uint32_t    pendingSinceMs_[8]{};
    bool        pendingActive_[8]{};
    bool        gaveUp_[8]{};
    Session     sessions_[ISOTP_MAX_SESSIONS]{};
    uint16_t    dtcs_[MAX_MERGED_DTCS]{};
    size_t      dtcCount_ = 0;
    char        vin_[17]{};
    bool        haveVin_ = false;
    size_t      vinConflicts_ = 0;
};

// Requester deadline for a DTC / VIN request, all times in ms (wrap-safe). The base wait is `waitMs`; it stretches while an
// ISO-TP session is being assembled (up to reassemblyCapMs) or an ECU has an NRC 0x78 pending (up to pendingCapMs).
// "Reassembling" is DiagCollector::progressing(): a session is open or an ECU already answered, so the result follows within
// the quiet gap. A stalled session is dropped after N_Cr, so it only stays true while frames arrive.
struct DiagWaitState
{
    uint32_t nowMs;
    uint32_t startMs;
    uint32_t waitMs;
    uint32_t reassemblyCapMs;
    uint32_t pendingCapMs;
    bool     reassembling;
    bool     pending;
};
bool diagWaitExpired(const DiagWaitState& w);

// Recovery after the CAN bus (re)connects or the OBD health check clears the poll queue.
enum class ConnectPlan : uint8_t
{
    Discover,      // run supported-PID discovery (VIN differs, unknown or the VIN request failed)
    RestoreKnown,  // same VIN as the last discovery: keep the supported list and only refill the queue
};

// `vin` is the VIN just read (null or empty if the request failed); `knownVin` is the VIN of the last completed discovery.
ConnectPlan planOnConnect(const char* vin, const char* knownVin);

// True for a NUL-terminated string of exactly 17 ASCII uppercase letters or digits. Anything else is a failed VIN read.
bool isValidVin(const char* vin);

// A user-initiated VIN request succeeded, so the ECU answers. `pidsReady` is whether OBD is initialised.
enum class VinAction : uint8_t
{
    None,      // nothing to do (failed read, or initialised and same VIN)
    Adopt,     // initialised but no VIN on record: remember this one, keep the supported list
    Restore,   // not initialised, same VIN as the last discovery: refill from the known supported list
    Discover,  // not initialised with another or unknown VIN, or initialised with a different VIN: rediscover
};
VinAction planOnVinConfirmed(const char* vin, const char* knownVin, bool pidsReady);

// Health probe answered: refill the queue only if the user wants polling, the bus is up, discovery finished and the queue lost its recurring entries.
bool shouldRestoreQueue(bool autoPoll, bool busUp, bool pidsReady, bool queueHasRecurring);

constexpr uint32_t DISCOVERY_BACKOFF_FIRST_MS = 2000;
constexpr uint32_t DISCOVERY_BACKOFF_MAX_MS   = 30000;

// Delay before the next discovery retry: 2 s, doubling, capped at 30 s. Pass 0 for the first retry.
uint32_t nextDiscoveryBackoffMs(uint32_t currentMs);

}  // namespace obd2frame
