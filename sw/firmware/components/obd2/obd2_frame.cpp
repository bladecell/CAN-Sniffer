#include "obd2_frame.hpp"

#include <cstring>

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

esp_err_t reassemble(const RawFrame* frames, size_t count, std::vector<uint8_t>& payload)
{
    payload.clear();
    if (frames == nullptr || count == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    IsoTp     type = IsoTp::Single;
    esp_err_t err  = frameType(frames[0].data, frames[0].length, type);
    if (err != ESP_OK)
    {
        return err;
    }

    if (type == IsoTp::Single)
    {
        SingleFrameInfo info;
        err = parseSingleFrame(frames[0].data, frames[0].length, info);
        if (err != ESP_OK)
        {
            return err;
        }
        if (count != 1)
        {
            return ESP_ERR_INVALID_SIZE;
        }
        payload.assign(frames[0].data + 1, frames[0].data + 1 + (frames[0].data[0] & 0x0F));
        return ESP_OK;
    }

    uint16_t total = 0, needed = 0;
    err = parseFirstFrame(frames[0].data, frames[0].length, total, needed);
    if (err != ESP_OK)
    {
        return err;
    }
    if (count != static_cast<size_t>(needed) + 1)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    payload.reserve(total);
    payload.insert(payload.end(), frames[0].data + 2, frames[0].data + frames[0].length);

    for (size_t i = 1; i < count; i++)
    {
        if (!consecutiveSequenceOk(frames[i].data, frames[i].length, static_cast<uint16_t>(i)))
        {
            payload.clear();
            return ESP_ERR_INVALID_RESPONSE;
        }
        payload.insert(payload.end(), frames[i].data + 1, frames[i].data + frames[i].length);
    }

    if (payload.size() < total)
    {
        payload.clear();
        return ESP_ERR_INVALID_SIZE;
    }
    payload.resize(total);  // drop padding in the last frame
    return ESP_OK;
}

esp_err_t extractDtcs(const std::vector<uint8_t>& payload, std::vector<uint16_t>& rawDtcs)
{
    rawDtcs.clear();
    if (payload.empty())
    {
        return ESP_ERR_INVALID_SIZE;
    }
    for (size_t i = 1; i + 1 < payload.size(); i += 2)
    {
        rawDtcs.push_back(static_cast<uint16_t>((payload[i] << 8) | payload[i + 1]));
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
