#include "obd2_frame.hpp"

#include <cstring>
#include <initializer_list>

#include "obd2_common.hpp"

namespace obd2frame
{

esp_err_t frameType(const uint8_t* data, size_t length, IsoTp& type)
{
    if (data == nullptr || length == 0 || length > CAN_DATA_MAX)
    {
        return ESP_ERR_INVALID_SIZE;
    }
    const uint8_t t = (data[0] >> 4) & 0x0F;
    if (t > static_cast<uint8_t>(IsoTp::FlowControl))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }
    type = static_cast<IsoTp>(t);
    return ESP_OK;
}

esp_err_t parseSingleFrame(const uint8_t* data, size_t length, SingleFrameInfo& out)
{
    IsoTp     type = IsoTp::Single;
    esp_err_t err  = frameType(data, length, type);
    if (err != ESP_OK)
    {
        return err;
    }
    if (type != IsoTp::Single || length < 2)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t pciLength = data[0] & 0x0F;
    if (pciLength < 1 || pciLength > 7 || pciLength > length - 1)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint8_t mode = data[1];
    uint8_t       offset;
    uint16_t      pid;
    switch (mode)
    {
        case RESPONSE_CURRENT_DATA:
        case RESPONSE_VEHICLE_INFO:
            offset = 3;
            pid    = data[2];
            break;
        case RESPONSE_READ_DATA_BY_IDENTIFIER:
        case RESPONSE_MODE_DERIVED_DATA:
            if (pciLength < 3)
            {
                return ESP_ERR_INVALID_SIZE;
            }
            offset = 4;
            pid    = static_cast<uint16_t>((data[2] << 8) | data[3]);
            break;
        default:
            offset = 2;
            pid    = 0;
            break;
    }

    // PCI length counts the SID, so message bytes end at index pciLength.
    const size_t end = static_cast<size_t>(pciLength) + 1;
    if (offset > end)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    out.mode          = mode;
    out.pid           = pid;
    out.payloadOffset = offset;
    out.payloadLength = static_cast<uint8_t>(end - offset);
    return ESP_OK;
}

esp_err_t parseFirstFrame(const uint8_t* data, size_t length, uint16_t& totalLength, uint16_t& framesNeeded)
{
    IsoTp     type = IsoTp::Single;
    esp_err_t err  = frameType(data, length, type);
    if (err != ESP_OK)
    {
        return err;
    }
    if (type != IsoTp::First || length < 3)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint16_t total = static_cast<uint16_t>(((data[0] & 0x0F) << 8) | data[1]);
    // A first frame carries 6 bytes, so anything up to 7 would have been a single frame.
    if (total < 8 || total > MAX_ISOTP_LENGTH)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    totalLength  = total;
    framesNeeded = static_cast<uint16_t>((total - 6 + 6) / 7);
    return ESP_OK;
}

bool consecutiveSequenceOk(const uint8_t* data, size_t length, uint16_t index)
{
    IsoTp type = IsoTp::Single;
    if (frameType(data, length, type) != ESP_OK || type != IsoTp::Consecutive)
    {
        return false;
    }
    return (data[0] & 0x0F) == (index & 0x0F);
}

// DTC response layout for CAN (assumed, user-supplied; secondary-source confirmation pending):
//   SID (0x43 / 0x47 / 0x4A), NN = number of DTCs in this ECU, then 2 bytes per DTC (high, low).
// Only whole pairs that are present are read, never more than NN, so padding after the list and a lying NN are harmless.
static size_t dtcPairsInPayload(const uint8_t* payload, size_t length)
{
    if (payload == nullptr || length < 2)
    {
        return 0;  // no count byte
    }
    const size_t present = (length - 2) / 2;
    return payload[1] < present ? payload[1] : present;
}

static void clearRequest(RequestBytes& out, std::initializer_list<uint8_t> bytes)
{
    memset(out, 0, sizeof(out));
    size_t i = 0;
    for (uint8_t b : bytes)
    {
        out[i++] = b;
    }
}

void buildDtcRequest(uint8_t mode, RequestBytes& out)
{
    clearRequest(out, {0x01, mode});
}

void buildVinRequest(RequestBytes& out)
{
    clearRequest(out, {0x02, MODE_VEHICLE_INFO, PID_VIN});
}

void buildClearDtcRequest(RequestBytes& out)
{
    clearRequest(out, {0x01, MODE_CLEAR_DTCS});
}

void buildDiagSessionRequest(uint8_t session, RequestBytes& out)
{
    clearRequest(out, {0x02, MODE_DIAGNOSTIC_SESSION_CONTROL, session});
}

void buildCurrentDataRequest(uint8_t pid, RequestBytes& out)
{
    clearRequest(out, {0x02, MODE_CURRENT_DATA, pid});
}

void buildFlowControl(RequestBytes& out)
{
    clearRequest(out, {0x30});
}

bool isValidPidSpec(uint8_t mode, uint16_t pid, uint8_t length)
{
    switch (mode)
    {
        case MODE_CURRENT_DATA:
            return pid >= 1 && pid <= 0xFF && length == 2;
        case MODE_READ_DATA_BY_IDENTIFIER:
            return length == 3;
        case MODE_DERIVED_DATA:
            return length == 0;
        default:
            return false;
    }
}

bool buildPidRequest(uint8_t mode, uint16_t pid, RequestBytes& out)
{
    if (mode == MODE_READ_DATA_BY_IDENTIFIER)
    {
        clearRequest(out, {0x03, mode, static_cast<uint8_t>(pid >> 8), static_cast<uint8_t>(pid & 0xFF)});
        return true;
    }
    if (mode == MODE_CURRENT_DATA && pid >= 1 && pid <= 0xFF)
    {
        clearRequest(out, {0x02, mode, static_cast<uint8_t>(pid)});
        return true;
    }
    clearRequest(out, {});
    return false;
}

esp_err_t extractDtcs(const std::vector<uint8_t>& payload, std::vector<uint16_t>& rawDtcs)
{
    rawDtcs.clear();
    if (payload.empty())
    {
        return ESP_ERR_INVALID_SIZE;
    }
    const size_t pairs = dtcPairsInPayload(payload.data(), payload.size());
    for (size_t i = 0; i < pairs; i++)
    {
        rawDtcs.push_back(static_cast<uint16_t>((payload[2 + 2 * i] << 8) | payload[3 + 2 * i]));
    }
    return ESP_OK;
}

esp_err_t extractVin(const std::vector<uint8_t>& payload, char (&vin)[17])
{
    constexpr size_t HEADER = 3;  // 0x49, 0x02, number of data items
    if (payload.size() < HEADER + sizeof(vin))
    {
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(vin, payload.data() + HEADER, sizeof(vin));
    return ESP_OK;
}

}  // namespace obd2frame

namespace obd2frame
{

bool isValidVin(const char* vin)
{
    if (vin == nullptr || strnlen(vin, 18) != 17)
    {
        return false;
    }
    for (int i = 0; i < 17; i++)
    {
        const char ch = vin[i];
        if (!((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z')))
        {
            return false;
        }
    }
    return true;
}

ConnectPlan planOnConnect(const char* vin, const char* knownVin)
{
    if (!isValidVin(vin) || !isValidVin(knownVin))
    {
        return ConnectPlan::Discover;
    }
    return strncmp(vin, knownVin, 17) == 0 ? ConnectPlan::RestoreKnown : ConnectPlan::Discover;
}

VinAction planOnVinConfirmed(const char* vin, const char* knownVin, bool pidsReady)
{
    if (!isValidVin(vin))
    {
        return VinAction::None;
    }
    const bool haveKnown = isValidVin(knownVin);
    if (!pidsReady)
    {
        return planOnConnect(vin, knownVin) == ConnectPlan::RestoreKnown ? VinAction::Restore : VinAction::Discover;
    }
    if (!haveKnown)
    {
        return VinAction::Adopt;
    }
    return strncmp(vin, knownVin, 17) == 0 ? VinAction::None : VinAction::Discover;
}

bool shouldRestoreQueue(bool autoPoll, bool busUp, bool pidsReady, bool queueHasRecurring)
{
    return autoPoll && busUp && pidsReady && !queueHasRecurring;
}

uint32_t nextDiscoveryBackoffMs(uint32_t currentMs)
{
    if (currentMs == 0)
    {
        return DISCOVERY_BACKOFF_FIRST_MS;
    }
    return currentMs >= DISCOVERY_BACKOFF_MAX_MS / 2 ? DISCOVERY_BACKOFF_MAX_MS : currentMs * 2;
}

}  // namespace obd2frame

namespace obd2frame
{

DiagTimings defaultDiagTimings()
{
    return {OBD2_FIRST_RESPONSE_WAIT_MS, OBD2_RESPONSE_QUIET_GAP_MS, ISOTP_N_CR_MS,
            OBD2_ECU_P2_STAR_MAX_MS,     OBD2_MAX_CONSECUTIVE_PENDING, OBD2_RESPONSE_TOTAL_CAP_MS};
}

namespace
{
constexpr uint8_t SID_NEGATIVE_RESPONSE = RESPONSE_NEGATIVE_RESPONSE_CODE;
constexpr uint8_t NRC_RESPONSE_PENDING  = 0x78;
constexpr uint8_t FLOW_CONTROL_CTS      = 0x30;

uint32_t elapsed(uint32_t now, uint32_t since)
{
    return static_cast<uint32_t>(now - since);
}
}  // namespace

DiagCollector::DiagCollector(const DiagTimings& timings, size_t maxSessions)
    : timings_(timings), maxSessions_(maxSessions < ISOTP_MAX_SESSIONS ? maxSessions : ISOTP_MAX_SESSIONS)
{
    cancel();
}

void DiagCollector::cancel()
{
    kind_ = DiagKind::None;
    for (Session& s : sessions_)
    {
        s.used = false;
    }
    memset(pendingCount_, 0, sizeof(pendingCount_));
    memset(pendingSinceMs_, 0, sizeof(pendingSinceMs_));
    memset(pendingActive_, 0, sizeof(pendingActive_));
    memset(gaveUp_, 0, sizeof(gaveUp_));
    memset(vin_, 0, sizeof(vin_));
    responders_    = 0;
    respondedMask_ = 0;
    dtcCount_      = 0;
    haveVin_       = false;
    vinConflicts_  = 0;
    sent_          = false;
}

void DiagCollector::begin(DiagKind kind, uint8_t requestMode, uint32_t nowMs)
{
    cancel();
    const bool dtcMode = requestMode == MODE_DTCS || requestMode == MODE_PENDING_DTCS || requestMode == MODE_PERMANENT_DTCS;
    if (kind == DiagKind::Dtc && dtcMode)
    {
        responseSid_ = static_cast<uint8_t>(requestMode | 0x40);
    }
    else if (kind == DiagKind::Vin && requestMode == MODE_VEHICLE_INFO)
    {
        responseSid_ = RESPONSE_VEHICLE_INFO;
    }
    else
    {
        return;  // not a collectable request: stay idle
    }
    kind_           = kind;
    requestMode_    = requestMode;
    beginMs_        = nowMs;
    lastActivityMs_ = nowMs;
}

void DiagCollector::markSent(uint32_t nowMs)
{
    sent_   = true;
    sentMs_ = nowMs;
}

size_t DiagCollector::activeSessions() const
{
    size_t n = 0;
    for (const Session& s : sessions_)
    {
        n += s.used ? 1 : 0;
    }
    return n;
}

bool diagWaitExpired(const DiagWaitState& w)
{
    uint32_t limit = w.waitMs;
    if (w.reassembling && w.reassemblyCapMs > limit)
    {
        limit = w.reassemblyCapMs;
    }
    if (w.pending && w.pendingCapMs > limit)
    {
        limit = w.pendingCapMs;
    }
    return static_cast<uint32_t>(w.nowMs - w.startMs) >= limit;
}

bool DiagCollector::anyPending() const
{
    for (bool p : pendingActive_)
    {
        if (p)
        {
            return true;
        }
    }
    return false;
}

bool DiagCollector::vin(char (&out)[17]) const
{
    if (!haveVin_)
    {
        return false;
    }
    memcpy(out, vin_, sizeof(out));
    return true;
}

bool DiagCollector::matchesService(const uint8_t* p, size_t n) const
{
    if (n == 0 || p[0] != responseSid_)
    {
        return false;
    }
    return kind_ != DiagKind::Vin || (n >= 2 && p[1] == PID_VIN);
}

DiagCollector::Session* DiagCollector::findSession(uint32_t id)
{
    for (size_t i = 0; i < maxSessions_; i++)
    {
        if (sessions_[i].used && sessions_[i].id == id)
        {
            return &sessions_[i];
        }
    }
    return nullptr;
}

void DiagCollector::dropSession(Session& s)
{
    s.used = false;
}

void DiagCollector::addDtc(uint16_t raw)
{
    if (raw == 0)
    {
        return;
    }
    for (size_t i = 0; i < dtcCount_; i++)
    {
        if (dtcs_[i] == raw)
        {
            return;
        }
    }
    if (dtcCount_ < MAX_MERGED_DTCS)
    {
        dtcs_[dtcCount_++] = raw;
    }
}

void DiagCollector::finishPayload(uint32_t id, const uint8_t* payload, size_t length)
{
    const uint32_t idx = id - DIAG_RESPONSE_ID_FIRST;
    pendingCount_[idx] = 0;
    pendingActive_[idx] = false;
    if ((respondedMask_ & (1u << idx)) == 0)
    {
        respondedMask_ = static_cast<uint16_t>(respondedMask_ | (1u << idx));
        responders_++;
    }

    if (kind_ == DiagKind::Dtc)
    {
        const size_t pairs = dtcPairsInPayload(payload, length);
        for (size_t i = 0; i < pairs; i++)
        {
            addDtc(static_cast<uint16_t>((payload[2 + 2 * i] << 8) | payload[3 + 2 * i]));
        }
        return;
    }

    constexpr size_t HEADER = 3;  // 0x49, 0x02, number of data items
    if (length < HEADER + 17)
    {
        return;
    }
    char candidate[18];
    memcpy(candidate, payload + HEADER, 17);
    candidate[17] = '\0';
    if (!isValidVin(candidate))
    {
        return;
    }
    if (!haveVin_)
    {
        memcpy(vin_, candidate, 17);
        haveVin_ = true;
    }
    else if (memcmp(vin_, candidate, 17) != 0)
    {
        vinConflicts_++;
    }
}

DiagFrameResult DiagCollector::onFrame(uint32_t canId, const uint8_t* data, size_t length, uint32_t nowMs)
{
    DiagFrameResult res = {false, false, 0, ESP_OK};
    IsoTp           type = IsoTp::Single;
    if (!active() || !inRange(canId) || frameType(data, length, type) != ESP_OK)
    {
        return res;
    }
    const uint32_t idx = canId - DIAG_RESPONSE_ID_FIRST;

    switch (type)
    {
        case IsoTp::Single:
        {
            const size_t pci = data[0] & 0x0F;
            if (pci < 1 || pci > 7 || pci > length - 1)
            {
                return res;
            }
            if (data[1] == SID_NEGATIVE_RESPONSE)
            {
                if (pci < 3 || data[2] != requestMode_)
                {
                    return res;
                }
                res.consumed = true;
                if (data[3] != NRC_RESPONSE_PENDING)
                {
                    pendingActive_[idx] = false;
                    lastActivityMs_     = nowMs;
                    return res;
                }
                if (gaveUp_[idx])
                {
                    return res;
                }
                if (++pendingCount_[idx] > timings_.maxPending)
                {
                    gaveUp_[idx]        = true;
                    pendingActive_[idx] = false;
                    return res;
                }
                pendingActive_[idx]  = true;
                pendingSinceMs_[idx] = nowMs;
                lastActivityMs_      = nowMs;
                return res;
            }
            if (!matchesService(data + 1, pci))
            {
                return res;
            }
            res.consumed    = true;
            lastActivityMs_ = nowMs;
            finishPayload(canId, data + 1, pci);
            return res;
        }
        case IsoTp::First:
        {
            if (length < 4 || !matchesService(data + 2, length - 2))
            {
                return res;
            }
            res.consumed = true;
            Session* existing = findSession(canId);
            if (existing != nullptr)
            {
                dropSession(*existing);  // a new first frame from the same ECU restarts its message
            }
            uint16_t total = 0, needed = 0;
            res.err = length == CAN_DATA_MAX ? parseFirstFrame(data, length, total, needed) : ESP_ERR_INVALID_SIZE;
            if (res.err != ESP_OK)
            {
                return res;
            }
            Session* slot = existing;
            for (size_t i = 0; slot == nullptr && i < maxSessions_; i++)
            {
                slot = sessions_[i].used ? nullptr : &sessions_[i];
            }
            if (slot == nullptr)
            {
                res.err = ESP_ERR_NO_MEM;
                return res;
            }
            slot->used     = true;
            slot->id       = canId;
            slot->total    = total;
            slot->received = 6;
            slot->nextSeq  = 1;
            slot->lastMs   = nowMs;
            memcpy(slot->buf, data + 2, 6);
            pendingActive_[idx] = false;
            lastActivityMs_     = nowMs;
            res.sendFlowControl = true;
            res.flowControlId   = canId - RESPONSE_ID_OFFSET;
            return res;
        }
        case IsoTp::Consecutive:
        {
            Session* s = findSession(canId);
            if (s == nullptr)
            {
                return res;
            }
            res.consumed = true;
            // total >= 8 (parseFirstFrame) and received starts at 6, so total - received never underflows.
            const size_t need = (s->total - s->received) < 7 ? (s->total - s->received) : 7;
            if ((data[0] & 0x0F) != s->nextSeq)
            {
                res.err = ESP_ERR_INVALID_RESPONSE;
            }
            else if (length - 1 < need)
            {
                res.err = ESP_ERR_INVALID_SIZE;
            }
            if (res.err != ESP_OK)
            {
                dropSession(*s);
                return res;
            }
            memcpy(s->buf + s->received, data + 1, need);
            s->received = static_cast<uint16_t>(s->received + need);
            s->nextSeq  = static_cast<uint8_t>((s->nextSeq + 1) & 0x0F);
            s->lastMs   = nowMs;
            lastActivityMs_ = nowMs;
            if (s->received >= s->total)
            {
                finishPayload(canId, s->buf, s->total);
                dropSession(*s);
            }
            return res;
        }
        case IsoTp::FlowControl:
            return res;
    }
    return res;
}

DiagStatus DiagCollector::poll(uint32_t nowMs)
{
    if (!active())
    {
        return DiagStatus::Idle;
    }
    bool busy = false;
    for (size_t i = 0; i < maxSessions_; i++)
    {
        if (sessions_[i].used && elapsed(nowMs, sessions_[i].lastMs) >= timings_.nCrMs)
        {
            dropSession(sessions_[i]);
        }
        busy = busy || sessions_[i].used;
    }
    for (size_t i = 0; i < 8; i++)
    {
        if (pendingActive_[i] && elapsed(nowMs, pendingSinceMs_[i]) >= timings_.pendingMs)
        {
            pendingActive_[i] = false;
        }
        busy = busy || pendingActive_[i];
    }

    if (timings_.totalCapMs != 0 && elapsed(nowMs, beginMs_) >= timings_.totalCapMs)
    {
        return responders_ > 0 ? DiagStatus::Complete : DiagStatus::Failed;
    }
    if (busy)
    {
        return DiagStatus::Waiting;
    }
    if (responders_ == 0)
    {
        return (sent_ && elapsed(nowMs, sentMs_) >= timings_.firstResponseWaitMs) ? DiagStatus::Failed : DiagStatus::Waiting;
    }
    return elapsed(nowMs, lastActivityMs_) >= timings_.quietGapMs ? DiagStatus::Complete : DiagStatus::Waiting;
}

}  // namespace obd2frame
