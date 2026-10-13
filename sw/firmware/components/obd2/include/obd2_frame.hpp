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

// Joins one single frame, or a first frame plus consecutive frames, into the ISO-TP payload (SID first).
// Checks frame types, sequence numbers, total length and that all frames are present. Returns the payload.
esp_err_t reassemble(const RawFrame* frames, size_t count, std::vector<uint8_t>& payload);

// Mode 03/07/0A payload (0x43 / 0x47 / 0x4A, then 2 bytes per DTC) to raw 16-bit DTCs. A trailing odd byte is dropped.
esp_err_t extractDtcs(const std::vector<uint8_t>& payload, std::vector<uint16_t>& rawDtcs);

// Mode 09 PID 02 payload (0x49 0x02 count, then 17 ASCII chars) to the VIN.
esp_err_t extractVin(const std::vector<uint8_t>& payload, char (&vin)[17]);

// Reassembles `count` frames and checks the response SID equals `mode` (0x43 / 0x47 / 0x4A), then extracts raw DTCs.
// ESP_ERR_INVALID_RESPONSE on a wrong SID. Clears `rawDtcs` on failure.
esp_err_t decodeDtcResponse(const RawFrame* frames, size_t count, uint8_t mode, std::vector<uint16_t>& rawDtcs);

// Reassembles `count` frames and checks the header is 0x49 0x02, then extracts the VIN.
// ESP_ERR_INVALID_RESPONSE on a wrong header.
esp_err_t decodeVinResponse(const RawFrame* frames, size_t count, char (&vin)[17]);

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
    // True while any ECU has an unexpired NRC 0x78 (response pending).
    bool anyPending() const;

    // Merged, de-duplicated raw DTCs (zero DTCs and padding are not stored).
    const uint16_t* dtcs() const { return dtcs_; }
    size_t          dtcCount() const { return dtcCount_; }
    // First valid VIN. False if no ECU sent one. conflictingVins() counts valid but different VINs that were ignored.
    bool   vin(char (&out)[17]) const;
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
