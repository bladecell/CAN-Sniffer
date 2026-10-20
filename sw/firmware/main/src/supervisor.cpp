// supervisor.cpp

#include "supervisor.hpp"
#include "supervisor_sleep.hpp"

#include <sys/stat.h>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>

#include "application_components.hpp"
#include "atomic_file.hpp"
#include "battery.hpp"
#include "can_driver.hpp"
#include "error_policy.hpp"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "input_limits.hpp"
#include "json_budget.hpp"
#include "led_status.hpp"
#include "middleware.hpp"
#include "obd2.hpp"
#include "pid_json.hpp"
#include "reset_reason.hpp"
#include "sd_card.hpp"
#include "settings.hpp"
#include "utilities.h"

static const char* TAG = "SUPERVISOR";

// Consecutive ERROR sleeps and NVS factory resets. RTC_NOINIT keeps its content across deep sleep and every reset but
// is undefined after a power loss, so it is trusted only together with the magic word (error_policy::validCount).
static RTC_NOINIT_ATTR uint32_t s_errorMagic;
static RTC_NOINIT_ATTR uint32_t s_errorRestarts;

#define MOUNT_POINT "/sdcard"

namespace
{
constexpr TickType_t SD_OPERATION_TIMEOUT_TICKS = pdMS_TO_TICKS(5000);
supervisor::SleepTransaction sleepTransaction({
    nullptr,
    [](void*, uint64_t us) { return esp_sleep_enable_timer_wakeup(us); },
    [](void*) { return application_components::releasePowerLock(); },
    [](void*) { return application_components::acquirePowerLock(); }});

// SDCard accepts either leading-slash form for a relative path and one trailing
// slash.  Normalize those accepted spellings before comparing copy endpoints.
bool canonical_sd_relative_path(const char* path, std::string& canonical_path)
{
    if (!SDCard::is_path_under(path, MOUNT_POINT))
    {
        return false;
    }

    const char* relative_path = path + strlen(MOUNT_POINT);
    if (SDCard::validate_relative_path(relative_path, true) != ESP_OK)
    {
        return false;
    }

    if (*relative_path == '/')
    {
        ++relative_path;
    }

    canonical_path = relative_path;
    if (!canonical_path.empty() && canonical_path.back() == '/')
    {
        canonical_path.pop_back();
    }

    // FAT filename matching is case-insensitive for ASCII characters.  Do the
    // fold locally instead of using locale-sensitive ctype helpers or probing
    // the filesystem (which could race with the subsequent copy).
    for (char& character : canonical_path)
    {
        const unsigned char byte = static_cast<unsigned char>(character);
        if (byte >= static_cast<unsigned char>('a') && byte <= static_cast<unsigned char>('z'))
        {
            character = static_cast<char>(byte - ('a' - 'A'));
        }
    }

    return true;
}

bool is_confirmed_missing_errno(int error_number)
{
    return error_number == ENOENT || error_number == ENOTDIR;
}
}  // namespace

SUPERVISOR::SUPERVISOR() : core_(operations(this))
{
}

supervisor::Operations SUPERVISOR::operations(SUPERVISOR* self)
{
    return {self,
            [](void* p) { portENTER_CRITICAL(&static_cast<SUPERVISOR*>(p)->lifecycleMux_); },
            [](void* p) { portEXIT_CRITICAL(&static_cast<SUPERVISOR*>(p)->lifecycleMux_); },
            [](void* p) -> esp_err_t
            {
                auto* s = static_cast<SUPERVISOR*>(p);
                return xTaskCreatePinnedToCore(taskWrapper, "SupervisorTask", SUPERVISOR_TASK_STACK_SIZE, s,
                                               tskIDLE_PRIORITY + 1, &s->xTaskHandle, SUPERVISOR_TASK_CORE_ID) == pdPASS
                           ? ESP_OK
                           : ESP_ERR_NO_MEM;
            },
            [](void*) { return CanDriver::getInstance().isBusConnected(); },
            [](void*) { return CanDriver::getInstance().quickCheckBus(); },
            [](void*) { return battery_read(); },
            [](void*, uint64_t us) -> esp_err_t
            {
                ESP_LOGI(TAG, "Entering Light Sleep (%llu us)", static_cast<unsigned long long>(us));
                return sleepTransaction.prepare(us);
            },
            [](void*) { return sleepTransaction.finish(); },
            [](void*, uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); },
            [](void*, bool standby) -> esp_err_t
            {
                // Preload the latch before enabling output, including recovery
                // after failed bootstrap where CAN never configured this pin.
                const esp_err_t error = gpio_set_level(CAN_RS_GPIO, standby ? 1 : 0);
                return error == ESP_OK ? gpio_set_direction(CAN_RS_GPIO, GPIO_MODE_OUTPUT) : error;
            },
            [](void*) { return LedStatus::getInstance().init(LED_GPIO); },
            [](void*) { return LedStatus::getInstance().deinit(); },
            [](void*) { LedStatus::getInstance().blink(0xFFFFFFFF, 100, 100); },
            [](void*) { return error_policy::validCount(s_errorMagic, s_errorRestarts); },
            [](void*, uint32_t count)
            {
                s_errorMagic    = error_policy::kMagic;
                s_errorRestarts = count;
            },
            [](void*, uint32_t seconds)
            {
                ESP_LOGE(TAG, "ERROR: deep sleep %us, recovery count %u", static_cast<unsigned>(seconds),
                         static_cast<unsigned>(s_errorRestarts));
                esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(seconds) * 1000000ULL);
                esp_deep_sleep_start();
            },
            [](void*, const char* name, esp_err_t error, bool initializing)
            { ESP_LOGI(TAG, "%s %s: %s", initializing ? "Init" : "Deinit", name, esp_err_to_name(error)); },
            [](void* p) { return static_cast<SUPERVISOR*>(p)->_config.power; }};
}

esp_err_t SUPERVISOR::init(const supervisor::SettingsBootstrap& bootstrap)
{
    const esp_err_t error = core_.init(bootstrap);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "Supervisor init: %s", esp_err_to_name(error));
    }
    return error;
}

void SUPERVISOR::stop()
{
    (void)core_.requestStop();
}

uint32_t SUPERVISOR::reserve_sleep_request()
{
    return core_.reserveSleepRequest();
}

void SUPERVISOR::finish_sleep_request(uint32_t token, bool responseSent)
{
    (void)core_.finishSleepRequest(token, responseSent);
}

void SUPERVISOR::taskWrapper(void* param)
{
    SUPERVISOR* supervisor = static_cast<SUPERVISOR*>(param);
    supervisor->task();
}

void SUPERVISOR::task()
{
    while (1)
    {
        const State before = core_.state();
        core_.step();
        if (core_.state() != before)
        {
            ESP_LOGI(TAG, "State %d -> %d", static_cast<int>(before), static_cast<int>(core_.state()));
        }
        SDCard::getInstance().update_card_status();
        vTaskDelay(pdMS_TO_TICKS(supervisor::kLoopMs));
    }
}

esp_err_t SUPERVISOR::recover_corrupt_settings(const char* section)
{
    const uint32_t resets = error_policy::validCount(s_errorMagic, s_errorRestarts);
    if (!error_policy::nvsResetAllowed(resets))
    {
        ESP_LOGE(TAG, "NVS still corrupt after %u factory resets; failing %s setup (ERROR back-off)",
                 static_cast<unsigned>(resets), section);
        return ESP_FAIL;
    }
    s_errorMagic    = error_policy::kMagic;
    s_errorRestarts = resets + 1;
    ESP_LOGE(TAG, "NVS corruption detected! Factory resetting (%u of %u)...", static_cast<unsigned>(resets + 1),
             static_cast<unsigned>(error_policy::kMaxNvsResets));
    Settings::getInstance().factoryReset();
    restart_system();
    return ESP_FAIL;
}

uint32_t SUPERVISOR::get_uptime_seconds() const
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

static_assert(ESP_RST_UNKNOWN == 0 && ESP_RST_EXT == 2 && ESP_RST_SW == 3 && ESP_RST_DEEPSLEEP == 8 &&
                  ESP_RST_CPU_LOCKUP == 15,
              "reset_reason.hpp text table is indexed by esp_reset_reason_t");

std::string SUPERVISOR::get_restart_reason() const
{
    return reset_reason::text(static_cast<int>(esp_reset_reason()));
}

std::string SUPERVISOR::get_MAC_address() const
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char buf[18];
    (void)snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return std::string(buf);
}

void SUPERVISOR::restart_system()
{
    esp_restart();
}

float SUPERVISOR::get_battery_voltage() const
{
    return battery_read();
}

SUPERVISOR::PidDefinitionSaveResult SUPERVISOR::save_pid_def_to_json(const char* path)
{
    if (path == nullptr || !SDCard::is_path_under(path, "/sdcard"))
    {
        ESP_LOGE(TAG, "Refusing to write PID definitions outside /sdcard");
        return {ESP_ERR_INVALID_ARG, false};
    }

    auto&                          obd2 = OBD2::getInstance();
    std::vector<PIDDefinitionData> definitions;
    esp_err_t                      snapshotErr = obd2.getDefinitionSnapshot(definitions);
    if (snapshotErr != ESP_OK)
    {
        return {snapshotErr, false};
    }

    cJSON* rootArray = pidjson::serialize(definitions);
    if (rootArray == nullptr)
    {
        return {ESP_ERR_NO_MEM, false};
    }

    char* jsonString = cJSON_PrintUnformatted(rootArray);
    cJSON_Delete(rootArray);

    if (jsonString == nullptr)
    {
        return {ESP_ERR_NO_MEM, false};
    }

    const size_t len = strlen(jsonString);
    esp_err_t    ret = ESP_OK;

    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT_TICKS, true);
        if (!operation)
        {
            free(jsonString);
            return {operation.status(), operation.status() == ESP_ERR_TIMEOUT};
        }

        // Temp file, fsync, rename: a failed or interrupted save keeps the previous PID file (SUP-2).
        switch (atomic_file::write(path, jsonString, len))
        {
            case atomic_file::Result::Ok:
                break;
            case atomic_file::Result::PathTooLong:
                ret = ESP_ERR_INVALID_ARG;
                break;
            case atomic_file::Result::NoMem:
                ret = ESP_ERR_NO_MEM;
                break;
            default:
                ESP_LOGE(TAG, "Failed to save PID definitions to %s; previous file kept", path);
                ret = ESP_FAIL;
                break;
        }
    }

    free(jsonString);

    if (ret != ESP_OK)
    {
        return {ret, false};
    }

    ESP_LOGI(TAG, "Successfully saved PID definitions to %s", path);
    return {ESP_OK, false};
}

esp_err_t SUPERVISOR::load_pid_def_from_json(const char* path)
{
    if (path == nullptr || !SDCard::is_path_under(path, "/sdcard"))
    {
        ESP_LOGE(TAG, "Refusing to read PID definitions outside /sdcard");
        return ESP_ERR_INVALID_ARG;
    }

    char*  buffer     = nullptr;
    size_t bytes_read = 0;

    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT_TICKS, true);
        if (!operation)
        {
            return operation.status();
        }

        atomic_file::recover(path);  // finish a save that power loss interrupted

        struct stat st;
        if (stat(path, &st) != 0)
        {
            const int stat_errno = errno;
            if (is_confirmed_missing_errno(stat_errno))
            {
                ESP_LOGW(TAG, "JSON file not found (expected on first boot or missing): %s", path);
                return ESP_ERR_NOT_FOUND;
            }

            ESP_LOGE(TAG, "Failed to inspect JSON file %s (errno %d)", path, stat_errno);
            return ESP_FAIL;
        }

        if (st.st_size <= 0)
        {
            ESP_LOGW(TAG, "JSON file is empty or invalid: %s", path);
            return ESP_ERR_INVALID_ARG;
        }

        if (static_cast<uintmax_t>(st.st_size) > input_limits::PID_FILE_MAX)
        {
            ESP_LOGE(TAG, "JSON file is too large to load (%jd > %zu bytes): %s", static_cast<intmax_t>(st.st_size),
                     input_limits::PID_FILE_MAX, path);
            return ESP_ERR_INVALID_SIZE;
        }

        const size_t file_size = static_cast<size_t>(st.st_size);

        buffer = (char*)heap_caps_malloc(file_size + 1, MALLOC_CAP_SPIRAM);
        if (buffer == nullptr)
        {
            buffer = (char*)malloc(file_size + 1);
            if (buffer == nullptr)
            {
                ESP_LOGE(TAG, "Failed to allocate %zu bytes for JSON file", file_size + 1);
                return ESP_ERR_NO_MEM;
            }
        }

        FILE* f = fopen(path, "r");
        if (f == nullptr)
        {
            const int open_errno = errno;
            ESP_LOGE(TAG, "Failed to open file for reading: %s (errno %d)", path, open_errno);
            free(buffer);
            return is_confirmed_missing_errno(open_errno) ? ESP_ERR_NOT_FOUND : ESP_FAIL;
        }

        errno              = 0;
        bytes_read         = fread(buffer, 1, file_size, f);
        bool read_complete = bytes_read == file_size && ferror(f) == 0;

        // A successful fixed-size fread alone does not prove that the source
        // did not grow after stat().  Require EOF and an unchanged file size.
        if (read_complete)
        {
            errno = 0;
            if (fgetc(f) != EOF)
            {
                read_complete = false;
            }
        }
        if (ferror(f) != 0)
        {
            read_complete = false;
        }

        struct stat after_read;
        errno               = 0;
        const int fstat_ret = fstat(fileno(f), &after_read);
        if (fstat_ret != 0 || after_read.st_size != st.st_size)
        {
            read_complete = false;
        }

        const int close = fclose(f);

        if (!read_complete || close != 0)
        {
            ESP_LOGE(TAG, "Failed to read a complete, consistent JSON file from %s", path);
            free(buffer);
            return ESP_FAIL;
        }
    }

    buffer[bytes_read] = '\0';

    const json_budget::Verdict budget =
        json_budget::check(buffer, bytes_read, heap_caps_get_free_size(MALLOC_CAP_8BIT));
    if (budget != json_budget::Verdict::Ok)
    {
        ESP_LOGE(TAG, "PID file %s not parsed: %s", path, json_budget::reason(budget));
        free(buffer);
        return budget == json_budget::Verdict::NoMemory ? ESP_ERR_NO_MEM : ESP_ERR_INVALID_ARG;
    }

    const char* parse_end = nullptr;
    cJSON*      rootArray = cJSON_ParseWithLengthOpts(buffer, bytes_read, &parse_end, 0);

    if (rootArray != nullptr && parse_end != nullptr && parse_end >= buffer && parse_end <= buffer + bytes_read)
    {
        while (parse_end < buffer + bytes_read && std::isspace(static_cast<unsigned char>(*parse_end)))
        {
            ++parse_end;
        }

        if (parse_end != buffer + bytes_read)
        {
            cJSON_Delete(rootArray);
            rootArray = nullptr;
        }
    }
    else
    {
        cJSON_Delete(rootArray);
        rootArray = nullptr;
    }

    free(buffer);

    if (rootArray == nullptr || !cJSON_IsArray(rootArray))
    {
        ESP_LOGE(TAG, "Failed to parse complete JSON, or root is not an array");
        cJSON_Delete(rootArray);
        return ESP_ERR_INVALID_ARG;
    }

    int    http_status    = 200;
    cJSON* error_response = m_pid_def_put(rootArray, &http_status);
    bool   success        = error_response == nullptr && http_status == 204;
    cJSON_Delete(error_response);
    cJSON_Delete(rootArray);

    if (!success)
    {
        ESP_LOGW(TAG, "Failed to atomically load PID definitions from %s", path);
        if (http_status == 500)
        {
            return ESP_ERR_NO_MEM;
        }
        if (http_status == 503)
        {
            return ESP_ERR_TIMEOUT;
        }
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Successfully loaded PID definitions from %s", path);
    return ESP_OK;
}

esp_err_t SUPERVISOR::copy_file(const char* src_path, const char* dest_path)
{
    if (src_path == nullptr || dest_path == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    std::string canonical_source;
    std::string canonical_destination;
    if (!canonical_sd_relative_path(src_path, canonical_source) ||
        !canonical_sd_relative_path(dest_path, canonical_destination))
    {
        ESP_LOGE("FileOps", "Refusing file copy outside /sdcard: %s -> %s", src_path, dest_path);
        return ESP_ERR_INVALID_ARG;
    }

    if (canonical_source == canonical_destination)
    {
        ESP_LOGE("FileOps", "Refusing to copy a file onto itself: %s -> %s", src_path, dest_path);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_FAIL;
    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT_TICKS, true);
        if (!operation)
        {
            return operation.status();
        }

        // Temp file in the destination directory, then rename over it: a failed copy, a full card or a destination that
        // is the same file as the source under a FAT 8.3 alias leaves the old destination intact (identity cannot be
        // tested on FAT: st_ino is not unique there).
        switch (atomic_file::copy(src_path, dest_path))
        {
            case atomic_file::Result::Ok:
                ret = ESP_OK;
                break;
            case atomic_file::Result::PathTooLong:
                ret = ESP_ERR_INVALID_ARG;
                break;
            case atomic_file::Result::NoMem:
                ret = ESP_ERR_NO_MEM;
                break;
            default:
                ret = ESP_FAIL;
                break;
        }
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE("FileOps", "Copy %s -> %s failed: %s", src_path, dest_path, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("FileOps", "Successfully copied %s to %s", src_path, dest_path);
    return ESP_OK;
}

const char* SUPERVISOR::setup_function_to_string(SetupFunction func) const
{
    size_t index = static_cast<size_t>(func);
    if (index < get_setup_steps().size())
    {
        return get_setup_steps()[index].name;
    }
    return "UNKNOWN";
}

esp_err_t SUPERVISOR::get_setup_result(SetupFunction func) const
{
    size_t index = static_cast<size_t>(func);
    if (index < get_setup_steps().size())
    {
        return get_setup_steps()[index].result;
    }
    return ESP_ERR_INVALID_ARG;
}
