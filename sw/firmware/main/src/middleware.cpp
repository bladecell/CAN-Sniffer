#include "middleware.hpp"

#include <sys/stat.h>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <unordered_set>
#include <vector>

#include "cJSON.h"
#include "can_config_limits.hpp"
#include "confirm_token.hpp"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "obd2.hpp"
#include "pid_json.hpp"
#include "sd_card.hpp"
#include "settings.hpp"
#include "settings_validation.hpp"
#include "string"
#include "supervisor.hpp"
#include "utilities.h"

static const char* TAG = "MIDDLEWARE";

namespace
{

static MiddlewareJsonResult json_result(cJSON* body, int status = 200)
{
    return {body, status};
}

static cJSON* json_error_body(const char* reason)
{
    cJSON* root = cJSON_CreateObject();
    if (root != nullptr)
    {
        cJSON_AddStringToObject(root, "status", "error");
        if (reason != nullptr)
        {
            cJSON_AddStringToObject(root, "reason", reason);
        }
    }
    return root;
}

static int status_for_error(esp_err_t err, int invalid_arg_status = 400)
{
    if (err == ESP_ERR_INVALID_ARG)
    {
        return invalid_arg_status;
    }
    if (err == ESP_ERR_NOT_FOUND)
    {
        return 404;
    }
    if (err == ESP_ERR_TIMEOUT)
    {
        return 504;
    }
    if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_INVALID_SIZE)
    {
        return 503;
    }
    if (err == ESP_ERR_NO_MEM)
    {
        return 500;
    }
    return 500;
}

static MiddlewareJsonResult sd_operation_error(esp_err_t err)
{
    // ESP_ERR_TIMEOUT returned by acquire_operation() means the SD operation
    // mutex is contended. It is not an ECU/request timeout.
    if (err == ESP_ERR_TIMEOUT)
    {
        return json_result(json_error_body("SD card busy"), 503);
    }

    return json_result(json_error_body(esp_err_to_name(err)), status_for_error(err));
}

static bool is_confirmed_missing_errno(int error_number)
{
    return error_number == ENOENT || error_number == ENOTDIR;
}

static MiddlewareJsonResult dtc_db_io_error(cJSON* root, const std::string& reason, int http_status)
{
    cJSON_AddStringToObject(root, "status", "error");
    cJSON_AddStringToObject(root, "reason", reason.c_str());
    cJSON_AddNumberToObject(root, "dtc_count", 0);
    return json_result(root, http_status);
}

constexpr TickType_t SD_OPERATION_TIMEOUT = pdMS_TO_TICKS(1000);

}  // namespace

cJSON* single_pid_def_get(uint16_t pid, esp_err_t* operation_error)
{
    // Create a local struct to hold the snapshot
    PIDDefinitionData def;

    esp_err_t err = OBD2::getInstance().getDef(pid, def);
    if (operation_error != nullptr)
    {
        *operation_error = err;
    }
    if (err != ESP_OK)
    {
        return nullptr;
    }

    cJSON* item = cJSON_CreateObject();
    if (item == nullptr)
    {
        ESP_LOGE(TAG, "OOM building PID definition");
        if (operation_error != nullptr)
        {
            *operation_error = ESP_ERR_NO_MEM;
        }
        return nullptr;
    }

    // Use the struct data - no more mutex calls here
    cJSON_AddNumberToObject(item, "pid", def.pid);
    cJSON_AddNumberToObject(item, "mode", def.mode);
    cJSON_AddNumberToObject(item, "id", def.id);
    cJSON_AddNumberToObject(item, "length", def.len);
    cJSON_AddStringToObject(item, "name", def.name.c_str());
    cJSON_AddStringToObject(item, "unit", def.unit.c_str());
    cJSON_AddStringToObject(item, "description", def.description.c_str());
    cJSON_AddNumberToObject(item, "minValue", def.minValue);
    cJSON_AddNumberToObject(item, "maxValue", def.maxValue);
    cJSON_AddNumberToObject(item, "priority", def.priority);
    cJSON_AddNumberToObject(item, "update_interval_ms", def.updateInterval_ms);
    cJSON_AddNumberToObject(item, "color", def.color);
    cJSON_AddStringToObject(item, "icon", def.icon.c_str());
    cJSON_AddStringToObject(item, "formula", def.formula.c_str());

    return item;
}

cJSON* single_pid_data_get(uint16_t pid, esp_err_t* operation_error)
{
    PIDData_t data;

    esp_err_t err = OBD2::getInstance().getData(pid, data);
    if (operation_error != nullptr)
    {
        *operation_error = err;
    }
    if (err != ESP_OK)
    {
        return nullptr;
    }

    cJSON* item = cJSON_CreateObject();
    if (item == nullptr)
    {
        ESP_LOGE(TAG, "OOM building PID data");
        if (operation_error != nullptr)
        {
            *operation_error = ESP_ERR_NO_MEM;
        }
        return nullptr;
    }

    cJSON_AddNumberToObject(item, "id", data.id);
    cJSON_AddNumberToObject(item, "pid", pid);
    cJSON_AddNumberToObject(item, "value", data.value);
    cJSON_AddNumberToObject(item, "lastUpdated", data.lastUpdated);
    cJSON_AddBoolToObject(item, "isSupported", data.isSupported);
    cJSON_AddBoolToObject(item, "isValid", data.isValid);
    cJSON_AddNumberToObject(item, "update_interval_ms", data.updateInterval_ms);

    return item;
}

MiddlewareJsonResult m_pid_def_get(int filter_id)
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "OOM building PID def response");
        return json_result(nullptr, 500);
    }

    cJSON* data_array = cJSON_CreateArray();
    if (data_array == nullptr)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "OOM building PID def array");
        return json_result(nullptr, 500);
    }
    int count = 0;

    if (filter_id >= 0)
    {
        esp_err_t item_error = ESP_OK;
        cJSON*    item       = single_pid_def_get((uint16_t)filter_id, &item_error);
        if (item != nullptr)
        {
            cJSON_AddItemToArray(data_array, item);
            count++;
        }
        else
        {
            cJSON_Delete(data_array);
            cJSON_Delete(root);
            return json_result(nullptr, status_for_error(item_error, 404));
        }
    }
    else
    {
        std::vector<PIDDefinitionData> definitions;
        esp_err_t                      err = OBD2::getInstance().getDefinitionSnapshot(definitions);
        if (err != ESP_OK)
        {
            cJSON_Delete(data_array);
            cJSON_Delete(root);
            return json_result(nullptr, status_for_error(err, 503));
        }
        for (const auto& definition : definitions)
        {
            cJSON* item = cJSON_CreateObject();
            if (item != nullptr)
            {
                cJSON_AddNumberToObject(item, "pid", definition.pid);
                cJSON_AddNumberToObject(item, "mode", definition.mode);
                cJSON_AddNumberToObject(item, "id", definition.id);
                cJSON_AddNumberToObject(item, "length", definition.len);
                cJSON_AddStringToObject(item, "name", definition.name.c_str());
                cJSON_AddStringToObject(item, "unit", definition.unit.c_str());
                cJSON_AddStringToObject(item, "description", definition.description.c_str());
                cJSON_AddNumberToObject(item, "minValue", definition.minValue);
                cJSON_AddNumberToObject(item, "maxValue", definition.maxValue);
                cJSON_AddNumberToObject(item, "priority", definition.priority);
                cJSON_AddNumberToObject(item, "update_interval_ms", definition.updateInterval_ms);
                cJSON_AddNumberToObject(item, "color", definition.color);
                cJSON_AddStringToObject(item, "icon", definition.icon.c_str());
                cJSON_AddStringToObject(item, "formula", definition.formula.c_str());
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
            else
            {
                cJSON_Delete(data_array);
                cJSON_Delete(root);
                return json_result(nullptr, 500);
            }
        }
    }

    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "count", count);

    return json_result(root);
}

MiddlewareJsonResult m_pid_data_get(int filter_id)
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "OOM building PID data response");
        return json_result(nullptr, 500);
    }

    cJSON* data_array = cJSON_CreateArray();
    if (data_array == nullptr)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "OOM building PID data array");
        return json_result(nullptr, 500);
    }
    int count = 0;

    if (filter_id >= 0)
    {
        esp_err_t item_error = ESP_OK;
        cJSON*    item       = single_pid_data_get((uint16_t)filter_id, &item_error);
        if (item != nullptr)
        {
            cJSON_AddItemToArray(data_array, item);
            count++;
        }
        else
        {
            cJSON_Delete(data_array);
            cJSON_Delete(root);
            return json_result(nullptr, status_for_error(item_error, 404));
        }
    }
    else
    {
        std::vector<std::pair<uint16_t, PIDData_t>> data;
        esp_err_t                                   err = OBD2::getInstance().getDataSnapshot(data);
        if (err != ESP_OK)
        {
            cJSON_Delete(data_array);
            cJSON_Delete(root);
            return json_result(nullptr, status_for_error(err, 503));
        }
        for (const auto& [pid, snapshot] : data)
        {
            cJSON* item = cJSON_CreateObject();
            if (item != nullptr)
            {
                cJSON_AddNumberToObject(item, "id", snapshot.id);
                cJSON_AddNumberToObject(item, "pid", pid);
                cJSON_AddNumberToObject(item, "value", snapshot.value);
                cJSON_AddNumberToObject(item, "lastUpdated", snapshot.lastUpdated);
                cJSON_AddBoolToObject(item, "isSupported", snapshot.isSupported);
                cJSON_AddBoolToObject(item, "isValid", snapshot.isValid);
                cJSON_AddNumberToObject(item, "update_interval_ms", snapshot.updateInterval_ms);
                cJSON_AddItemToArray(data_array, item);
                count++;
            }
            else
            {
                cJSON_Delete(data_array);
                cJSON_Delete(root);
                return json_result(nullptr, 500);
            }
        }
    }

    cJSON_AddItemToObject(root, "data", data_array);
    cJSON_AddNumberToObject(root, "count", count);

    return json_result(root);
}

static void add_auto_poll_state(cJSON* root)
{
    cJSON_AddBoolToObject(root, "auto_poll", OBD2::getInstance().isAutoPoll());
    cJSON_AddBoolToObject(root, "polling", OBD2::getInstance().isPolling());
}

MiddlewareJsonResult m_auto_poll_set(bool on)
{
    const esp_err_t err = OBD2::getInstance().setAutoPoll(on);
    if (err != ESP_OK)
    {
        return json_result(json_error_body(esp_err_to_name(err)), 503);
    }
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }
    add_auto_poll_state(root);
    return json_result(root, 201);
}

MiddlewareJsonResult m_can_bus_get()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    const auto& nodeConfig = CanDriver::getInstance().getNodeConfig();
    const auto& config     = CanDriver::getInstance().getConfig();
    const auto& state      = CanDriver::getInstance().getState();

    const char* can_bus_state_name[] = {"not_initialized", "bus_off", "not_connected", "connected"};

    // Add CAN bus information to the JSON object
    cJSON_AddStringToObject(root, "state", can_bus_state_name[static_cast<int>(state)]);
    cJSON_AddNumberToObject(root, "bitrate", nodeConfig.bit_timing.bitrate);
    cJSON_AddStringToObject(root, "rs_mode",
                            config.rs_mode == CanDriver::RS_MODE::HIGH_SPEED ? "high_speed" : "slope_control");
    cJSON_AddBoolToObject(root, "debug_mode", config.debug);
    cJSON_AddBoolToObject(root, "initialized", CanDriver::getInstance().isInitialized());
    cJSON_AddBoolToObject(root, "bus_connected", CanDriver::getInstance().isBusConnected());

    cJSON* node_status = cJSON_CreateObject();

    auto        status            = CanDriver::getInstance().getStatus();
    const char* twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};

    cJSON_AddStringToObject(node_status, "twai_error_state", twai_state_name[status.state]);
    cJSON_AddNumberToObject(node_status, "tx_error_count", status.tx_error_count);
    cJSON_AddNumberToObject(node_status, "rx_error_count", status.rx_error_count);
    cJSON_AddItemToObject(root, "node_status", node_status);

    return json_result(root);
}

MiddlewareJsonResult m_obdii_get()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    // Add OBD-II information to the JSON object
    add_auto_poll_state(root);
    cJSON_AddBoolToObject(root, "pid_initialized", OBD2::getInstance().isPidInit());
    cJSON_AddNumberToObject(root, "pid_def_count", OBD2::getInstance().getPIDDEFSize());
    cJSON_AddNumberToObject(root, "pid_data_count", OBD2::getInstance().getPIDDataSize());
    cJSON_AddNumberToObject(root, "poll_task_utilization", OBD2::getInstance().getPollTaskUtilization());
    cJSON_AddStringToObject(root, "pid_def_path", SUPERVISOR::getInstance().get_pid_def_path().c_str());
    cJSON_AddStringToObject(root, "dtc_desc_path", SUPERVISOR::getInstance().get_dtc_desc_path().c_str());

    cJSON*               supported_pids = cJSON_CreateObject();
    supportedPIDsGroup_t supportedPIDsGroup;
    OBD2::getInstance().getSupportedPids(supportedPIDsGroup);
    cJSON_AddNumberToObject(supported_pids, "count", supportedPIDsGroup.numberOfSupportedPIDs);
    cJSON* groups = cJSON_CreateArray();
    for (int i = 0; i < SUPPORTED_PIDS_GROUP_COUNT; ++i)
    {
        cJSON_AddItemToArray(groups, cJSON_CreateNumber(supportedPIDsGroup.pidGroup[i]));
    }
    cJSON_AddItemToObject(supported_pids, "groups", groups);

    cJSON_AddItemToObject(root, "supported_pids", supported_pids);

    return json_result(root);
}

MiddlewareJsonResult m_system_get()
{
    // card_present() takes the SD mutex internally with portMAX_DELAY. Acquire
    // a bounded operation first so this endpoint cannot wait indefinitely for
    // another SD operation; the recursive acquisition in card_present() then
    // completes immediately while this lease is held.
    bool sd_card_detected = false;
    {
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT);
        if (!operation)
        {
            return sd_operation_error(operation.status());
        }

        sd_card_detected = SDCard::getInstance().card_present();
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    cJSON_AddStringToObject(root, "app_version", APP_VERSION_STRING);
    cJSON_AddNumberToObject(root, "uptime_s", SUPERVISOR::getInstance().get_uptime_seconds());
    cJSON_AddStringToObject(root, "restart_reason", SUPERVISOR::getInstance().get_restart_reason().c_str());
    cJSON_AddStringToObject(root, "mac", SUPERVISOR::getInstance().get_MAC_address().c_str());
    cJSON_AddNumberToObject(root, "state", static_cast<uint32_t>(SUPERVISOR::getInstance().get_state()));
    cJSON_AddNumberToObject(root, "battery_voltage", SUPERVISOR::getInstance().get_battery_voltage());
    cJSON_AddBoolToObject(root, "sd_card_detected", sd_card_detected);

    cJSON* component_status = cJSON_CreateArray();
    for (const auto& step : SUPERVISOR::getInstance().get_setup_steps())
    {
        cJSON* comp_obj = cJSON_CreateObject();
        cJSON_AddStringToObject(comp_obj, "name", step.name);
        cJSON_AddStringToObject(comp_obj, "status", esp_err_to_name(step.result));

        cJSON_AddItemToArray(component_status, comp_obj);
    }

    cJSON_AddItemToObject(root, "component_status", component_status);

    // Read-only diagnostics: heap and the unused stack of every task (shrinks under load, never grows back).
    cJSON_AddNumberToObject(root, "heap_free", esp_get_free_heap_size());
    cJSON_AddNumberToObject(root, "heap_min_free", esp_get_minimum_free_heap_size());
    const UBaseType_t               taskCapacity = uxTaskGetNumberOfTasks() + 4;
    std::unique_ptr<TaskStatus_t[]> taskStatus(new (std::nothrow) TaskStatus_t[taskCapacity]);
    cJSON*                          stacks = cJSON_CreateArray();
    if (taskStatus != nullptr && stacks != nullptr)
    {
        const UBaseType_t count = uxTaskGetSystemState(taskStatus.get(), taskCapacity, nullptr);
        for (UBaseType_t i = 0; i < count; i++)
        {
            cJSON* task = cJSON_CreateObject();
            if (task == nullptr)
            {
                break;
            }
            cJSON_AddStringToObject(task, "name", taskStatus[i].pcTaskName);
            cJSON_AddNumberToObject(task, "stack_free", taskStatus[i].usStackHighWaterMark);
            cJSON_AddItemToArray(stacks, task);
        }
    }
    if (stacks != nullptr)
    {
        cJSON_AddItemToObject(root, "tasks", stacks);
    }

    return json_result(root);
}

static void reboot_delayed_task(void* arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    SUPERVISOR::getInstance().restart_system();
}

MiddlewareJsonResult m_system_reboot()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "OOM building reboot response");
        return json_result(nullptr, 500);
    }

    cJSON_AddStringToObject(root, "status", "success");

    BaseType_t result = xTaskCreate(reboot_delayed_task, "sys_reboot", 2048, NULL, 5, NULL);
    if (result != pdPASS)
    {
        ESP_LOGW(TAG, "Failed to create reboot task");
        cJSON_ReplaceItemInObject(root, "status", cJSON_CreateString("error"));
        cJSON_AddStringToObject(root, "reason", "Failed to create reboot task");
        return json_result(root, 500);
    }

    return json_result(root);
}

MiddlewareJsonResult m_sdcard_info_get()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT);
    if (!operation)
    {
        cJSON_Delete(root);
        return sd_operation_error(operation.status());
    }

    SDCard::SDInfo sd_info;

    SDCard::getInstance().get_sd_info(sd_info);

    cJSON_AddStringToObject(root, "name", sd_info.name);
    cJSON_AddStringToObject(root, "mount_path", sd_info.mount_path);
    cJSON_AddNumberToObject(root, "capacity", sd_info.capacity_mb);
    cJSON_AddNumberToObject(root, "used_space_mb", sd_info.used_space_mb);
    cJSON_AddNumberToObject(root, "max_freq_mhz", sd_info.max_freq_mhz);
    cJSON_AddBoolToObject(root, "is_sdio", sd_info.is_sdio);
    cJSON_AddBoolToObject(root, "is_mmc", sd_info.is_mmc);
    cJSON_AddBoolToObject(root, "is_mounted", sd_info.is_mounted);
    cJSON_AddBoolToObject(root, "is_present", sd_info.is_present);

    return json_result(root);
}

MiddlewareJsonResult m_sdcard_format_post()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT);
    if (!operation)
    {
        cJSON_Delete(root);
        return sd_operation_error(operation.status());
    }

    esp_err_t ret = SDCard::getInstance().format_sdcard();

    if (ret == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(ret));
    }

    const int status = ret == ESP_OK ? 200
                                     : (ret == ESP_ERR_NOT_FOUND || ret == ESP_ERR_INVALID_STATE ? 503
                                        : ret == ESP_ERR_INVALID_ARG                             ? 400
                                                                                                 : 500);
    return json_result(root, status);
}

MiddlewareJsonResult m_sdcard_file_tree_get(const char* path)
{
    auto& sd        = SDCard::getInstance();
    auto  operation = sd.acquire_operation(SD_OPERATION_TIMEOUT, true);
    if (!operation)
    {
        return sd_operation_error(operation.status());
    }

    cJSON*    root       = nullptr;
    esp_err_t tree_error = sd.get_file_tree(path, 5, root);
    if (tree_error != ESP_OK)
    {
        cJSON_Delete(root);

        // Keep the directory-tree error distinct from its JSON body. A
        // missing path is a confirmed 404; acquisition failures were handled
        // above, while filesystem/resource/hot-removal failures are 500.
        const int status = tree_error == ESP_ERR_NOT_FOUND                                        ? 404
                           : tree_error == ESP_ERR_INVALID_ARG                                    ? 400
                           : tree_error == ESP_ERR_TIMEOUT || tree_error == ESP_ERR_INVALID_STATE ? 503
                                                                                                  : 500;
        return json_result(json_error_body(esp_err_to_name(tree_error)), status);
    }

    if (root == nullptr)
    {
        return json_result(json_error_body(esp_err_to_name(ESP_ERR_NO_MEM)), 500);
    }

    cJSON_AddStringToObject(root, "status", "success");
    return json_result(root);
}

MiddlewareJsonResult m_sdcard_file_delete_delete(const char* path)
{
    auto& sd        = SDCard::getInstance();
    auto  operation = sd.acquire_operation(SD_OPERATION_TIMEOUT, true);
    if (!operation)
    {
        return sd_operation_error(operation.status());
    }

    const size_t    path_len = strlen(path);
    const esp_err_t err = path_len > 0 && path[path_len - 1] == '/' ? sd.delete_directory(path) : sd.delete_file(path);

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return json_result(root, err == ESP_OK ? 200 : status_for_error(err, 400));
}

MiddlewareJsonResult m_vin_get()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    cJSON_AddStringToObject(root, "vin", OBD2::getInstance().getVIN().c_str());

    return json_result(root);
}

MiddlewareJsonResult m_dtc_get(int mode)
{
    cJSON* root  = cJSON_CreateObject();
    cJSON* items = cJSON_CreateArray();
    if (root == nullptr || items == nullptr)
    {
        cJSON_Delete(root);
        cJSON_Delete(items);
        return json_result(nullptr, 500);
    }
    int  global_count      = 0;
    bool allocation_failed = false;

    const bool valid_mode = mode == -1 || mode == MODE_DTCS || mode == MODE_PENDING_DTCS || mode == MODE_PERMANENT_DTCS;

    auto add_dtc_section = [&](int target_mode, const char* name)
    {
        if (mode == -1 || mode == target_mode)
        {
            global_count++;

            cJSON* item    = cJSON_CreateObject();
            cJSON* section = cJSON_CreateArray();
            if (item == nullptr || section == nullptr)
            {
                cJSON_Delete(item);
                cJSON_Delete(section);
                allocation_failed = true;
                return;
            }
            int dtc_count = 0;

            std::vector<std::string> dtc = OBD2::getInstance().getDTC(static_cast<uint8_t>(target_mode));

            for (const auto& d : dtc)
            {
                cJSON_AddItemToArray(section, cJSON_CreateString(d.c_str()));
                dtc_count++;
            }

            cJSON_AddNumberToObject(item, "mode", target_mode);
            cJSON_AddStringToObject(item, "type", name);
            cJSON_AddItemToObject(item, "dtc", section);
            cJSON_AddNumberToObject(item, "dtc_count", dtc_count);

            cJSON_AddItemToArray(items, item);
        }
    };

    add_dtc_section(MODE_DTCS, "confirmed_dtcs");
    add_dtc_section(MODE_PENDING_DTCS, "pending_dtcs");
    add_dtc_section(MODE_PERMANENT_DTCS, "permanent_dtcs");

    if (allocation_failed)
    {
        cJSON_Delete(items);
        cJSON_Delete(root);
        return json_result(nullptr, 500);
    }

    cJSON_AddItemToObject(root, "dtcs", items);
    cJSON_AddNumberToObject(root, "count", global_count);
    cJSON_AddStringToObject(root, "status", valid_mode ? "success" : "error");
    if (!valid_mode)
    {
        cJSON_AddStringToObject(root, "reason", "Unsupported DTC mode");
    }

    return json_result(root, valid_mode ? 200 : 422);
}

MiddlewareJsonResult m_dtc_description_get(const char* target_codes[], size_t count)
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "OOM building DTC description response");
        return json_result(nullptr, 500);
    }

    std::string dtc_desc_db_path = SUPERVISOR::getInstance().get_dtc_desc_path();

    // Retain this lease for the whole lookup, including every seek/read of the
    // shared VFS file. Do not add OBD or settings calls below this point.
    auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT, true);
    if (!operation)
    {
        cJSON_Delete(root);
        return sd_operation_error(operation.status());
    }

    std::unique_ptr<FILE, decltype(&fclose)> file(fopen(dtc_desc_db_path.c_str(), "rb"), fclose);

    if (!file)
    {
        const int         open_errno = errno;
        const std::string reason = is_confirmed_missing_errno(open_errno) ? "File " + dtc_desc_db_path + " not found"
                                                                          : "Failed to open file " + dtc_desc_db_path;
        return dtc_db_io_error(root, reason, is_confirmed_missing_errno(open_errno) ? 404 : 500);
    }

    errno = 0;
    if (fseek(file.get(), 0, SEEK_END) != 0)
    {
        return dtc_db_io_error(root, "Failed to seek file " + dtc_desc_db_path, 500);
    }

    errno                = 0;
    const long file_size = ftell(file.get());
    if (file_size < 0)
    {
        return dtc_db_io_error(root, "Failed to determine size of file " + dtc_desc_db_path, 500);
    }

    const long total_records = file_size / 128;

    if (total_records <= 0)
    {
        return dtc_db_io_error(root, "No records found in " + dtc_desc_db_path, 500);
    }

    cJSON* dtcs_array = cJSON_CreateArray();
    if (dtcs_array == nullptr)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "OOM building DTC description array");
        return json_result(nullptr, 500);
    }

    char read_code[6];
    char desc_buf[128];

    for (size_t i = 0; i < count; i++)
    {
        cJSON* item = cJSON_CreateObject();
        if (item == nullptr)
        {
            cJSON_Delete(dtcs_array);
            cJSON_Delete(root);
            ESP_LOGE(TAG, "OOM building DTC description item");
            return json_result(nullptr, 500);
        }

        cJSON_AddStringToObject(item, "dtc", target_codes[i]);

        long left = 0, right = total_records - 1;
        bool found = false;

        while (left <= right)
        {
            long mid = left + ((right - left) >> 1);

            errno = 0;
            if (fseek(file.get(), mid * 128, SEEK_SET) != 0)
            {
                cJSON_Delete(item);
                cJSON_Delete(dtcs_array);
                return dtc_db_io_error(root, "Failed to seek file " + dtc_desc_db_path, 500);
            }

            errno              = 0;
            const size_t bytes = fread(read_code, 1, 6, file.get());

            if (bytes != 6 || ferror(file.get()) != 0)
            {
                cJSON_Delete(item);
                cJSON_Delete(dtcs_array);
                return dtc_db_io_error(root, "Failed to read file " + dtc_desc_db_path, 500);
            }

            read_code[5] = '\0';
            int cmp      = strcmp(read_code, target_codes[i]);

            if (cmp == 0)
            {
                errno                          = 0;
                const size_t description_bytes = fread(desc_buf, 1, 122, file.get());
                if (description_bytes != 122 || ferror(file.get()) != 0)
                {
                    cJSON_Delete(item);
                    cJSON_Delete(dtcs_array);
                    return dtc_db_io_error(root, "Failed to read file " + dtc_desc_db_path, 500);
                }
                desc_buf[122] = '\0';

                cJSON_AddStringToObject(item, "description", desc_buf);
                found = true;
                break;
            }

            if (cmp < 0)
            {
                left = mid + 1;
            }
            else
            {
                right = mid - 1;
            }
        }

        if (!found)
        {
            cJSON_AddStringToObject(item, "description", "Description not found");
        }

        cJSON_AddItemToArray(dtcs_array, item);
    }

    FILE* raw_file = file.release();
    errno          = 0;
    if (fclose(raw_file) != 0)
    {
        cJSON_Delete(dtcs_array);
        return dtc_db_io_error(root, "Failed to close file " + dtc_desc_db_path, 500);
    }

    cJSON_AddStringToObject(root, "status", "success");
    cJSON_AddNumberToObject(root, "dtc_count", count);
    cJSON_AddItemToObject(root, "dtcs", dtcs_array);
    return json_result(root);
}

MiddlewareJsonResult m_vin_request()
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }
    esp_err_t err = OBD2::getInstance().requestVIN();

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return json_result(root, err == ESP_OK ? 200 : status_for_error(err, 503));
}

MiddlewareJsonResult m_dtc_request(int mode)
{
    const bool supported_mode = mode == MODE_DTCS || mode == MODE_PENDING_DTCS || mode == MODE_PERMANENT_DTCS;
    if (mode != -1 && !supported_mode)
    {
        cJSON* root = cJSON_CreateObject();
        if (root == nullptr)
        {
            return json_result(nullptr, 500);
        }

        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", "Unsupported DTC mode");
        return json_result(root, 422);
    }

    if (mode != -1)
    {
        const esp_err_t err = OBD2::getInstance().requestDTC(mode);

        cJSON* root = cJSON_CreateObject();
        if (root == nullptr)
        {
            return json_result(nullptr, 500);
        }
        if (err == ESP_OK)
        {
            cJSON_AddStringToObject(root, "status", "success");
            // add DTC data here
        }
        else
        {
            cJSON_AddStringToObject(root, "status", "error");
            cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
        }
        return json_result(root, err == ESP_OK ? 200 : status_for_error(err, 422));
    }

    struct DtcRequestResult
    {
        uint8_t     mode;
        const char* name;
        esp_err_t   error;
    } results[] = {{MODE_DTCS, "confirmed", ESP_OK},
                   {MODE_PENDING_DTCS, "pending", ESP_OK},
                   {MODE_PERMANENT_DTCS, "permanent", ESP_OK}};

    int       success_count = 0;
    esp_err_t first_error   = ESP_OK;
    for (auto& result : results)
    {
        result.error = OBD2::getInstance().requestDTC(result.mode);
        if (result.error == ESP_OK)
        {
            success_count++;
        }
        else if (first_error == ESP_OK)
        {
            first_error = result.error;
        }
    }

    cJSON* root             = cJSON_CreateObject();
    cJSON* successful_modes = cJSON_CreateArray();
    cJSON* failed_modes     = cJSON_CreateArray();
    if (root == nullptr || successful_modes == nullptr || failed_modes == nullptr)
    {
        cJSON_Delete(root);
        cJSON_Delete(successful_modes);
        cJSON_Delete(failed_modes);
        return json_result(nullptr, 500);
    }

    std::string failure_reason = "DTC request failed for:";
    bool        has_failures   = false;
    for (const auto& result : results)
    {
        if (result.error == ESP_OK)
        {
            cJSON_AddItemToArray(successful_modes, cJSON_CreateNumber(result.mode));
            continue;
        }

        cJSON* failed_mode = cJSON_CreateObject();
        if (failed_mode == nullptr)
        {
            cJSON_Delete(root);
            cJSON_Delete(successful_modes);
            cJSON_Delete(failed_modes);
            return json_result(nullptr, 500);
        }
        cJSON_AddNumberToObject(failed_mode, "mode", result.mode);
        cJSON_AddStringToObject(failed_mode, "error", esp_err_to_name(result.error));
        cJSON_AddItemToArray(failed_modes, failed_mode);

        failure_reason += has_failures ? ", " : " ";
        failure_reason += result.name;
        failure_reason += " (";
        failure_reason += esp_err_to_name(result.error);
        failure_reason += ')';
        has_failures = true;
    }

    cJSON_AddItemToObject(root, "successful_modes", successful_modes);
    cJSON_AddItemToObject(root, "failed_modes", failed_modes);

    if (success_count == static_cast<int>(sizeof(results) / sizeof(results[0])))
    {
        cJSON_AddStringToObject(root, "status", "success");
        return json_result(root, 200);
    }
    if (success_count > 0)
    {
        cJSON_AddStringToObject(root, "status", "partial_success");
        cJSON_AddStringToObject(root, "reason", failure_reason.c_str());
        return json_result(root, 200);
    }

    cJSON_AddStringToObject(root, "status", "error");
    cJSON_AddStringToObject(root, "reason", failure_reason.c_str());
    return json_result(root, status_for_error(first_error, 422));
}

static constexpr uint32_t kClearDtcTokenTtlMs = 10000;

static ConfirmToken& clear_dtc_token()
{
    static ConfirmToken t([]() -> uint64_t { return static_cast<uint64_t>(esp_timer_get_time() / 1000); },
                          []() -> uint32_t { return esp_random(); }, kClearDtcTokenTtlMs);
    return t;
}

MiddlewareJsonResult m_clear_dtc_request(const char* token)
{
    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    // Without a valid one-shot token nothing goes on the bus. A call with no token gets the pending
    // token back; a wrong, expired or used token gets a fresh one.
    if (!clear_dtc_token().consume(token))
    {
        char           fresh[ConfirmToken::kTokenSize];
        const uint32_t expiresMs = clear_dtc_token().challenge(token, fresh);
        cJSON_AddStringToObject(root, "status", "confirm_required");
        cJSON_AddStringToObject(root, "token", fresh);
        cJSON_AddNumberToObject(root, "expires_ms", expiresMs);
        return json_result(root, 409);
    }

    esp_err_t err = OBD2::getInstance().requestClearDTCs();

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return json_result(root, err == ESP_OK ? 200 : status_for_error(err, 503));
}

cJSON* m_static_pid_request()
{
    cJSON* root = cJSON_CreateObject();
    OBD2::getInstance().pollRequestStaticPids();

    cJSON_AddStringToObject(root, "status", "success");

    return root;
}

esp_err_t pid_stream_packet_get(uint16_t pid, uint8_t* out_packet)
{
    size_t offset = 0;

    // 8 bit message type
    out_packet[offset++] = MSG_TYPE_PID;

    // Length
    out_packet[offset++] = PID_STREAM_PACKET_SIZE - 2;

    // 32 bit pid
    out_packet[offset++] = (pid >> 0) & 0xFF;
    out_packet[offset++] = (pid >> 8) & 0xFF;
    out_packet[offset++] = (pid >> 16) & 0xFF;
    out_packet[offset++] = (pid >> 24) & 0xFF;

    // float value
    uint32_t float_bits;
    float    float_value = OBD2::getInstance().getValue(pid);
    memcpy(&float_bits, &float_value, sizeof(float_value));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // 32 bit lastUpdated
    uint32_t lastUpdated = OBD2::getInstance().getLastUpdated(pid);
    out_packet[offset++] = (lastUpdated >> 0) & 0xFF;
    out_packet[offset++] = (lastUpdated >> 8) & 0xFF;
    out_packet[offset++] = (lastUpdated >> 16) & 0xFF;
    out_packet[offset++] = (lastUpdated >> 24) & 0xFF;

    // 32 bit interval
    uint32_t updateInterval_ms = OBD2::getInstance().getUpdateInterval(pid);
    out_packet[offset++]       = (updateInterval_ms >> 0) & 0xFF;
    out_packet[offset++]       = (updateInterval_ms >> 8) & 0xFF;
    out_packet[offset++]       = (updateInterval_ms >> 16) & 0xFF;
    out_packet[offset++]       = (updateInterval_ms >> 24) & 0xFF;

    // 8 bit isSupported
    out_packet[offset++] = OBD2::getInstance().isSup(pid) ? 1 : 0;

    // 8 bit isValid
    out_packet[offset++] = OBD2::getInstance().isValid(pid) ? 1 : 0;

    return ESP_OK;
}

esp_err_t can_status_packet_get(uint8_t* out_packet)
{
    size_t offset = 0;

    // 8 bit message type
    out_packet[offset++] = MSG_TYPE_CAN_STATUS;

    // Length
    out_packet[offset++] = CAN_STATUS_PACKET_SIZE - 2;

    // 8 bit state
    const auto& state    = CanDriver::getInstance().getState();
    out_packet[offset++] = static_cast<uint8_t>(state);

    // float utilization
    float    utilization = OBD2::getInstance().getPollTaskUtilization();
    uint32_t float_bits;
    memcpy(&float_bits, &utilization, sizeof(utilization));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // float battery voltage
    float battery_voltage = SUPERVISOR::getInstance().get_battery_voltage();
    memcpy(&float_bits, &battery_voltage, sizeof(battery_voltage));
    out_packet[offset++] = (float_bits >> 0) & 0xFF;
    out_packet[offset++] = (float_bits >> 8) & 0xFF;
    out_packet[offset++] = (float_bits >> 16) & 0xFF;
    out_packet[offset++] = (float_bits >> 24) & 0xFF;

    // 8 bit isBusConnected
    out_packet[offset++] = CanDriver::getInstance().isBusConnected() ? 1 : 0;

    return ESP_OK;
}

MiddlewareJsonResult m_pid_def_delete(int filter_id)
{
    cJSON*    root = cJSON_CreateObject();
    esp_err_t err  = OBD2::getInstance().removePID(filter_id);
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    return json_result(root, err == ESP_OK ? 200 : status_for_error(err, 404));
}

static cJSON* pid_put_error(const char* reason)
{
    cJSON* root = cJSON_CreateObject();
    if (root != nullptr)
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", reason);
    }
    return root;
}

cJSON* m_pid_def_put(cJSON* data, int* http_status)
{
    if (http_status != nullptr)
    {
        *http_status = 204;
    }

    std::vector<PIDDefinitionData> definitions;
    const pidjson::ParseResult     parsed = pidjson::parse(data, definitions);
    if (!parsed.ok)
    {
        if (http_status != nullptr)
        {
            *http_status = parsed.httpStatus;
        }
        return pid_put_error(parsed.reason);
    }

    esp_err_t result = OBD2::getInstance().replacePIDDefinitions(definitions);
    if (result != ESP_OK)
    {
        const int status = result == ESP_ERR_INVALID_ARG                                 ? 422
                           : result == ESP_ERR_INVALID_SIZE || result == ESP_ERR_TIMEOUT ? 503
                                                                                         : 500;
        if (http_status != nullptr)
        {
            *http_status = status;
        }
        return pid_put_error(esp_err_to_name(result));
    }

    // There is deliberately no success JSON tree. The model commit is the
    // final fallible operation performed by this path.
    if (http_status != nullptr)
    {
        *http_status = 204;
    }
    return nullptr;
}

MiddlewareJsonResult m_pid_def_post(cJSON* data)
{
    if (data == nullptr || !cJSON_IsArray(data))
    {
        return json_result(json_error_body("Payload must be a JSON array"), 400);
    }

    int success_count  = 0;
    int error_count    = 0;
    int failure_status = 422;

    cJSON* added_array  = cJSON_CreateArray();
    cJSON* failed_array = cJSON_CreateArray();
    if (added_array == nullptr || failed_array == nullptr)
    {
        cJSON_Delete(added_array);
        cJSON_Delete(failed_array);
        return json_result(nullptr, 500);
    }

    cJSON* item = nullptr;

    cJSON_ArrayForEach(item, data)
    {
        if (!cJSON_IsObject(item))
        {
            cJSON* fail_obj = cJSON_CreateObject();
            cJSON_AddNullToObject(fail_obj, "pid");
            cJSON_AddStringToObject(fail_obj, "error", esp_err_to_name(ESP_ERR_INVALID_ARG));
            cJSON_AddItemToArray(failed_array, fail_obj);

            error_count++;
            continue;
        }

        cJSON* pid      = cJSON_GetObjectItem(item, "pid");
        cJSON* id       = cJSON_GetObjectItem(item, "id");
        cJSON* mode     = cJSON_GetObjectItem(item, "mode");
        cJSON* name     = cJSON_GetObjectItem(item, "name");
        cJSON* formula  = cJSON_GetObjectItem(item, "formula");
        cJSON* interval = cJSON_GetObjectItem(item, "interval");
        cJSON* unit     = cJSON_GetObjectItem(item, "unit");
        cJSON* desc     = cJSON_GetObjectItem(item, "desc");
        cJSON* minV     = cJSON_GetObjectItem(item, "minV");
        cJSON* maxV     = cJSON_GetObjectItem(item, "maxV");
        cJSON* priority = cJSON_GetObjectItem(item, "priority");
        cJSON* color    = cJSON_GetObjectItem(item, "color");
        cJSON* icon     = cJSON_GetObjectItem(item, "icon");

        // Every number is range-checked against the schema (docs/pid-config.md) before any cast: nothing wraps.
        // Optional fields that are absent or null keep their default; present but invalid rejects the item.
        uint32_t   parsed_id = 0, parsed_mode = 0, parsed_pid32 = 0, parsed_interval = 0;
        uint32_t   parsed_priority32 = 0, parsed_color = 0x4EB31B;
        float      parsed_minV = NAN, parsed_maxV = NAN;
        const auto optionalPresent = [](const cJSON* v) { return v != nullptr && !cJSON_IsNull(v); };

        bool valid = pidjson::integerFromJson(pid, 0, std::numeric_limits<uint16_t>::max(), parsed_pid32) &&
                     pidjson::integerFromJson(id, 0, 0x7FF, parsed_id) &&
                     pidjson::integerFromJson(mode, 0, std::numeric_limits<uint8_t>::max(), parsed_mode) &&
                     pidjson::integerFromJson(interval, 0, std::numeric_limits<uint16_t>::max(), parsed_interval) &&
                     cJSON_IsString(name) && name->valuestring != nullptr && cJSON_IsString(formula) &&
                     formula->valuestring != nullptr;
        if (valid && optionalPresent(priority))
        {
            valid = pidjson::integerFromJson(priority, 0, std::numeric_limits<uint8_t>::max(), parsed_priority32);
        }
        if (valid && optionalPresent(color))
        {
            valid = pidjson::integerFromJson(color, 0, 0xFFFFFF, parsed_color);
        }
        if (valid && optionalPresent(minV))
        {
            valid = pidjson::floatFromJson(minV, parsed_minV);
        }
        if (valid && optionalPresent(maxV))
        {
            valid = pidjson::floatFromJson(maxV, parsed_maxV);
        }
        if (valid && std::isfinite(parsed_minV) && std::isfinite(parsed_maxV) && parsed_minV > parsed_maxV)
        {
            valid = false;
        }
        uint8_t parsed_len =
            parsed_mode == MODE_READ_DATA_BY_IDENTIFIER ? 3 : (parsed_mode == MODE_DERIVED_DATA ? 0 : 2);
        if (valid)
        {
            valid = pidjson::lengthFromJson(item, parsed_len, false);
        }
        if (!valid)
        {
            cJSON* fail_obj = cJSON_CreateObject();
            if (cJSON_IsNumber(pid) && std::isfinite(pid->valuedouble) && pid->valuedouble >= 0 &&
                pid->valuedouble <= std::numeric_limits<uint16_t>::max() &&
                std::floor(pid->valuedouble) == pid->valuedouble)
            {
                cJSON_AddNumberToObject(fail_obj, "pid", pid->valuedouble);
            }
            else
            {
                cJSON_AddNullToObject(fail_obj, "pid");
            }
            cJSON_AddStringToObject(fail_obj, "error", esp_err_to_name(ESP_ERR_INVALID_ARG));
            cJSON_AddItemToArray(failed_array, fail_obj);
            error_count++;
            continue;
        }

        const uint16_t parsed_pid  = static_cast<uint16_t>(parsed_pid32);
        std::string    parsed_unit = cJSON_IsString(unit) && unit->valuestring != nullptr ? unit->valuestring : "";
        std::string    parsed_desc = cJSON_IsString(desc) && desc->valuestring != nullptr ? desc->valuestring : "";
        std::string    parsed_icon = cJSON_IsString(icon) && icon->valuestring != nullptr ? icon->valuestring : "";

        esp_err_t err = OBD2::getInstance().addPID(
            parsed_id, static_cast<uint8_t>(parsed_mode), parsed_pid, parsed_len, std::string(name->valuestring),
            parsed_unit, parsed_desc, std::string(formula->valuestring), parsed_minV, parsed_maxV,
            static_cast<uint8_t>(parsed_priority32), static_cast<uint16_t>(parsed_interval), parsed_color, parsed_icon);

        if (err == ESP_OK)
        {
            cJSON_AddItemToArray(added_array, cJSON_CreateNumber(parsed_pid));
            success_count++;
        }
        else
        {
            cJSON* fail_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(fail_obj, "pid", parsed_pid);
            cJSON_AddStringToObject(fail_obj, "error", esp_err_to_name(err));
            cJSON_AddItemToArray(failed_array, fail_obj);
            error_count++;
            const int item_status = status_for_error(err, 422);
            if (item_status == 503 || item_status == 500)
            {
                failure_status = item_status;
            }
        }
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        cJSON_Delete(added_array);
        cJSON_Delete(failed_array);
        return json_result(nullptr, 500);
    }

    cJSON_AddItemToObject(root, "added", added_array);
    cJSON_AddItemToObject(root, "failed", failed_array);

    if (error_count > 0 && success_count == 0)
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", "All items failed validation or hardware addition.");
    }
    else if (error_count > 0)
    {
        cJSON_AddStringToObject(root, "status", "partial_success");
        cJSON_AddStringToObject(root, "reason", "Some items were added, but others encountered errors.");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "success");
    }

    return json_result(root, (error_count > 0 && success_count == 0) ? failure_status : 200);
}

MiddlewareJsonResult m_pid_def_save(cJSON* payload)
{
    SUPERVISOR::PidDefinitionSaveResult result = {ESP_OK, false};

    if (payload == nullptr)
    {
        result = SUPERVISOR::getInstance().save_pid_def_to_json(SUPERVISOR::getInstance().get_pid_def_path().c_str());
    }
    else
    {
        if (!cJSON_IsObject(payload))
        {
            return json_result(json_error_body("Payload must be a JSON object"), 400);
        }

        cJSON* obj = cJSON_GetObjectItemCaseSensitive(payload, "pid_def_path");
        if (cJSON_IsString(obj) && (obj->valuestring != NULL))
        {
            if (!SDCard::is_path_under(obj->valuestring, "/sdcard"))
            {
                return json_result(json_error_body("pid_def_path must be under /sdcard"), 400);
            }

            result = SUPERVISOR::getInstance().save_pid_def_to_json(obj->valuestring);
        }
        else
        {
            return json_result(json_error_body("Missing or invalid 'pid_def_path' in payload"), 400);
        }
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    if (result.error == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", result.sd_card_busy ? "SD card busy" : esp_err_to_name(result.error));
    }

    const int status = result.error == ESP_OK ? 200
                                              : (result.sd_card_busy || result.error == ESP_ERR_INVALID_STATE
                                                     ? 503
                                                     : status_for_error(result.error, 400));
    return json_result(root, status);
}

MiddlewareJsonResult m_pid_def_load(cJSON* payload)
{
    esp_err_t ret = ESP_OK;

    if (payload == nullptr)
    {
        ret = SUPERVISOR::getInstance().load_pid_def_from_json(SUPERVISOR::getInstance().get_pid_def_path().c_str());
    }
    else
    {
        if (!cJSON_IsObject(payload))
        {
            return json_result(json_error_body("Payload must be a JSON object"), 400);
        }

        cJSON* obj = cJSON_GetObjectItemCaseSensitive(payload, "pid_def_path");
        if (cJSON_IsString(obj) && (obj->valuestring != NULL))
        {
            if (!SDCard::is_path_under(obj->valuestring, "/sdcard"))
            {
                return json_result(json_error_body("pid_def_path must be under /sdcard"), 400);
            }

            ret = SUPERVISOR::getInstance().load_pid_def_from_json(obj->valuestring);
        }
        else
        {
            return json_result(json_error_body("Missing or invalid 'pid_def_path' in payload"), 400);
        }
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    if (ret == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(ret));
    }

    const int status = ret == ESP_OK ? 200
                                     : (ret == ESP_ERR_NOT_FOUND                                 ? 404
                                        : ret == ESP_ERR_INVALID_ARG                             ? 400
                                        : ret == ESP_ERR_TIMEOUT || ret == ESP_ERR_INVALID_STATE ? 503
                                                                                                 : 500);
    return json_result(root, status);
}

MiddlewareJsonResult m_system_copy_file(cJSON* payload)
{
    if (payload == nullptr || !cJSON_IsObject(payload))
    {
        return json_result(json_error_body("Payload must be a JSON object"), 400);
    }

    cJSON* src_node  = cJSON_GetObjectItem(payload, "source_path");
    cJSON* dest_node = cJSON_GetObjectItem(payload, "destination_path");

    if (!cJSON_IsString(src_node) || !cJSON_IsString(dest_node))
    {
        return json_result(json_error_body("Missing or invalid 'source_path' or 'destination_path'"), 400);
    }

    const char* src_path  = src_node->valuestring;
    const char* dest_path = dest_node->valuestring;

    if (!SDCard::is_path_under(src_path, "/sdcard") || !SDCard::is_path_under(dest_path, "/sdcard"))
    {
        return json_result(json_error_body("source_path and destination_path must be under /sdcard"), 400);
    }

    esp_err_t err = ESP_OK;
    {
        // Keep this bounded lease through both the precheck and copy. The
        // supervisor's recursive SD operation is consequently guaranteed not
        // to turn a later SD-lock contention into a generic 504 timeout.
        auto operation = SDCard::getInstance().acquire_operation(SD_OPERATION_TIMEOUT, true);
        if (!operation)
        {
            return sd_operation_error(operation.status());
        }

        struct stat source_stat;
        if (stat(src_path, &source_stat) != 0)
        {
            const int stat_errno = errno;
            if (is_confirmed_missing_errno(stat_errno))
            {
                return json_result(json_error_body("Source file not found"), 404);
            }
            return json_result(json_error_body("Failed to inspect source file"), 500);
        }

        err = SUPERVISOR::getInstance().copy_file(src_path, dest_path);
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    if (err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", esp_err_to_name(err));
    }

    const int status = err == ESP_OK ? 200 : (err == ESP_ERR_INVALID_ARG ? 400 : status_for_error(err, 500));
    return json_result(root, status);
}

MiddlewareJsonResult m_settings_get()
{
    cJSON* root = cJSON_CreateArray();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }

    esp_err_t read_error = ESP_OK;

    // 1. Wifi Settings
    WIFI::Config wifi_cfg;
    esp_err_t    wifi_error = Settings::getInstance().getWifiConfig(wifi_cfg);
    if (wifi_error == ESP_OK)
    {
        cJSON* wifi_item = cJSON_CreateObject();
        cJSON_AddStringToObject(wifi_item, "name", "wifi");

        cJSON* wifi_settings = cJSON_CreateObject();
        cJSON_AddStringToObject(wifi_settings, "ssid", wifi_cfg.ssid.c_str());
        // cJSON_AddStringToObject(wifi_settings, "password", wifi_cfg.password.c_str());
        cJSON_AddNumberToObject(wifi_settings, "channel", wifi_cfg.channel);
        cJSON_AddNumberToObject(wifi_settings, "max_connections", wifi_cfg.max_connections);
        cJSON_AddNumberToObject(wifi_settings, "auth_mode", static_cast<int>(wifi_cfg.auth_mode));
        cJSON_AddBoolToObject(wifi_settings, "ssid_hidden", wifi_cfg.ssid_hidden);
        cJSON_AddBoolToObject(wifi_settings, "pmf_required", wifi_cfg.pmf_required);
        cJSON_AddNumberToObject(wifi_settings, "gtk_rekey_interval", wifi_cfg.gtk_rekey_interval);
        cJSON_AddStringToObject(wifi_settings, "sta_ssid", wifi_cfg.sta_ssid.c_str());
        // cJSON_AddStringToObject(wifi_settings, "sta_password", wifi_cfg.sta_password.c_str());
        cJSON_AddNumberToObject(wifi_settings, "sta_auth_mode", static_cast<int>(wifi_cfg.sta_auth_mode));
        cJSON_AddNumberToObject(wifi_settings, "mode", static_cast<int>(wifi_cfg.mode));
        cJSON_AddNumberToObject(wifi_settings, "sta_max_retry", wifi_cfg.sta_max_retry);

        cJSON_AddItemToObject(wifi_item, "settings", wifi_settings);
        cJSON_AddItemToArray(root, wifi_item);
    }
    else
    {
        read_error = wifi_error;
    }

    // 2. CAN Settings
    CanDriver::Config can_cfg;
    esp_err_t         can_error = Settings::getInstance().getCanConfig(can_cfg);
    if (can_error == ESP_OK)
    {
        cJSON* can_item = cJSON_CreateObject();
        cJSON_AddStringToObject(can_item, "name", "can");

        cJSON* can_settings = cJSON_CreateObject();
        cJSON_AddNumberToObject(can_settings, "bitrate", static_cast<uint32_t>(can_cfg.bitrate));
        cJSON_AddNumberToObject(can_settings, "tx_pin", can_cfg.tx_pin);
        cJSON_AddNumberToObject(can_settings, "rx_pin", can_cfg.rx_pin);
        cJSON_AddNumberToObject(can_settings, "lbk_pin", can_cfg.lbk_pin);
        cJSON_AddNumberToObject(can_settings, "rs_pin", can_cfg.rs_pin);
        cJSON_AddBoolToObject(can_settings, "debug", can_cfg.debug);
        cJSON_AddNumberToObject(can_settings, "rs_mode", static_cast<uint8_t>(can_cfg.rs_mode));
        cJSON_AddNumberToObject(can_settings, "tx_queue_depth", can_cfg.tx_queue_depth);
        cJSON_AddNumberToObject(can_settings, "rx_queue_size", can_cfg.rx_queue_size);
        cJSON_AddBoolToObject(can_settings, "filter", can_cfg.filter);

        cJSON* mfilter_json = cJSON_CreateObject();
        cJSON_AddNumberToObject(mfilter_json, "id", can_cfg.mfilter_cfg.id);
        cJSON_AddNumberToObject(mfilter_json, "mask", can_cfg.mfilter_cfg.mask);
        cJSON_AddBoolToObject(mfilter_json, "is_ext", can_cfg.mfilter_cfg.is_ext);
        cJSON_AddItemToObject(can_settings, "mfilter_cfg", mfilter_json);

        cJSON_AddItemToObject(can_item, "settings", can_settings);
        cJSON_AddItemToArray(root, can_item);
    }
    else if (read_error == ESP_OK)
    {
        read_error = can_error;
    }

    // 3. System Settings
    SUPERVISOR::Config supervisor_cfg;
    esp_err_t          supervisor_error = Settings::getInstance().getSupervisorConfig(supervisor_cfg);
    if (supervisor_error == ESP_OK)
    {
        cJSON* sup_item = cJSON_CreateObject();
        cJSON_AddStringToObject(sup_item, "name", "system");

        cJSON* sup_settings = cJSON_CreateObject();
        cJSON_AddStringToObject(sup_settings, "pid_def_path", supervisor_cfg.pid_def_path.c_str());
        cJSON_AddStringToObject(sup_settings, "dtc_desc_path", supervisor_cfg.dtc_desc_path.c_str());

        cJSON_AddItemToObject(sup_item, "settings", sup_settings);
        cJSON_AddItemToArray(root, sup_item);
    }
    else if (read_error == ESP_OK)
    {
        read_error = supervisor_error;
    }

    return json_result(root, read_error == ESP_OK ? 200 : 500);
}

static bool cj_str(cJSON* obj, const char* key, std::string& out)
{
    cJSON* n = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(n) && n->valuestring)
    {
        out = n->valuestring;
        return true;
    }
    return false;
}

template <typename T>
static bool cj_bool(cJSON* obj, const char* key, T& out)
{
    cJSON* n = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsBool(n))
    {
        out = static_cast<T>(cJSON_IsTrue(n));
        return true;
    }
    return false;
}

// Reads the fields of one settings section. A number must be an integer inside [lo, hi] and pass `valid` before it is
// cast: the double -> integer/enum cast of NaN, inf or an out-of-range value is undefined behaviour. The first bad field
// sets `err`; later fields are skipped.
struct SettingsFields
{
    cJSON*      obj;
    size_t      n = 0;
    std::string err;

    explicit SettingsFields(cJSON* o) : obj(o) {}

    template <typename T>
    void num(const char* key, double lo, double hi, bool (*valid)(int64_t), T& dst)
    {
        cJSON* v = err.empty() ? cJSON_GetObjectItemCaseSensitive(obj, key) : nullptr;
        if (v == nullptr)
        {
            return;
        }
        int64_t x = 0;
        if (!cJSON_IsNumber(v) || !settings_validation::checkedInt(v->valuedouble, lo, hi, x))
        {
            err = std::string(key) + " must be an integer in range";
        }
        else if (valid != nullptr && !valid(x))
        {
            err = std::string(key) + " has an unsupported value";
        }
        else
        {
            dst = static_cast<T>(x);
            n++;
        }
    }

    void str(const char* key, std::string& dst) { n += err.empty() && cj_str(obj, key, dst); }

    template <typename T>
    void flag(const char* key, T& dst) { n += err.empty() && cj_bool(obj, key, dst); }
};

constexpr double kU8Max  = 255;
constexpr double kU32Max = 4294967295.0;

static SettingsFields apply_wifi_settings(cJSON* s, WIFI::Config& c)
{
    namespace sv = settings_validation;
    SettingsFields f(s);
    f.str("ssid", c.ssid);
    f.str("password", c.password);
    f.str("sta_ssid", c.sta_ssid);
    f.str("sta_password", c.sta_password);
    f.num("channel", 0, kU8Max, sv::channelValid, c.channel);
    f.num("max_connections", 0, kU8Max, sv::maxConnValid, c.max_connections);
    f.num("auth_mode", 0, kU8Max, sv::wifiAuthValid, c.auth_mode);
    f.flag("ssid_hidden", c.ssid_hidden);
    f.flag("pmf_required", c.pmf_required);
    f.num("gtk_rekey_interval", 0, kU32Max, nullptr, c.gtk_rekey_interval);
    f.num("sta_auth_mode", 0, kU8Max, sv::wifiAuthValid, c.sta_auth_mode);
    f.num("mode", 0, kU8Max, sv::wifiModeValid, c.mode);
    f.num("sta_max_retry", 0, kU8Max, nullptr, c.sta_max_retry);
    return f;
}

static SettingsFields apply_can_settings(cJSON* s, CanDriver::Config& c)
{
    namespace sv = settings_validation;
    SettingsFields f(s);
    f.num("bitrate", 0, kU32Max, sv::bitrateValid, c.bitrate);
    f.num("tx_pin", -1000, 1000, sv::gpioUsable, c.tx_pin);
    f.num("rx_pin", -1000, 1000, sv::gpioUsable, c.rx_pin);
    f.num("lbk_pin", -1000, 1000, sv::gpioUsable, c.lbk_pin);
    f.num("rs_pin", -1000, 1000, sv::gpioUsable, c.rs_pin);
    f.flag("debug", c.debug);
    f.num("rs_mode", 0, kU8Max, sv::rsModeValid, c.rs_mode);
    f.num("tx_queue_depth", 0, kU32Max, [](int64_t v) { return can_limits::txQueueDepthValid(static_cast<double>(v)); }, c.tx_queue_depth);
    f.num("rx_queue_size", 0, kU32Max, [](int64_t v) { return can_limits::rxQueueSizeValid(static_cast<double>(v)); }, c.rx_queue_size);
    f.flag("filter", c.filter);

    cJSON* m = cJSON_GetObjectItemCaseSensitive(s, "mfilter_cfg");
    if (cJSON_IsObject(m))
    {
        SettingsFields mf(m);
        mf.num("id", 0, kU32Max, nullptr, c.mfilter_cfg.id);
        mf.num("mask", 0, kU32Max, nullptr, c.mfilter_cfg.mask);
        bool is_ext = c.mfilter_cfg.is_ext;
        mf.flag("is_ext", is_ext);
        c.mfilter_cfg.is_ext = is_ext;
        f.n += mf.n;
        if (f.err.empty())
        {
            f.err = mf.err;
        }
    }
    return f;
}

static SettingsFields apply_system_settings(cJSON* s, SUPERVISOR::Config& c)
{
    SettingsFields f(s);
    f.str("pid_def_path", c.pid_def_path);
    f.str("dtc_desc_path", c.dtc_desc_path);
    return f;
}

// Rejects the request (400, nothing saved for this section) on a bad field or an invalid merged section.
template <typename Cfg>
static bool section_ok(const SettingsFields& f, const Cfg& cfg, const char* (*check)(const Cfg&), esp_err_t& overall_err, std::string& reason)
{
    const char* why = !f.err.empty() ? f.err.c_str() : (f.n != 0 ? check(cfg) : nullptr);
    if (why == nullptr)
    {
        return true;
    }
    overall_err = ESP_ERR_INVALID_ARG;
    reason      = why;
    return false;
}

static void process_single_setting_item(cJSON* item, esp_err_t& overall_err, std::string& reason)
{
    if (!cJSON_IsObject(item))
    {
        overall_err = ESP_ERR_INVALID_ARG;
        reason      = "settings item must be a JSON object";
        return;
    }

    cJSON* name_node = cJSON_GetObjectItemCaseSensitive(item, "name");
    if (!cJSON_IsString(name_node) || name_node->valuestring == nullptr)
    {
        overall_err = ESP_ERR_INVALID_ARG;
        reason      = "missing or invalid 'name'";
        return;
    }

    std::string name = name_node->valuestring;

    cJSON* settings_node = cJSON_GetObjectItemCaseSensitive(item, "settings");
    if (settings_node == nullptr || !cJSON_IsObject(settings_node))
    {
        settings_node = item;
    }

    esp_err_t res       = ESP_OK;
    bool      no_fields = false;

    if (name == "wifi")
    {
        WIFI::Config wifi_cfg;
        res = Settings::getInstance().getWifiConfig(wifi_cfg);
        if (res != ESP_OK)
        {
            overall_err = res;
            reason      = esp_err_to_name(res);
            return;
        }
        const SettingsFields f = apply_wifi_settings(settings_node, wifi_cfg);
        if (!section_ok(f, wifi_cfg, Settings::checkWifiConfig, overall_err, reason))
        {
            return;
        }
        no_fields = (f.n == 0);
        if (!no_fields)
        {
            res = Settings::getInstance().setWifiConfig(wifi_cfg);
        }
    }
    else if (name == "can")
    {
        CanDriver::Config can_cfg;
        res = Settings::getInstance().getCanConfig(can_cfg);
        if (res != ESP_OK)
        {
            overall_err = res;
            reason      = esp_err_to_name(res);
            return;
        }
        const SettingsFields f = apply_can_settings(settings_node, can_cfg);
        if (!section_ok(f, can_cfg, Settings::checkCanConfig, overall_err, reason))
        {
            return;
        }
        no_fields = (f.n == 0);
        if (!no_fields)
        {
            res = Settings::getInstance().setCanConfig(can_cfg);
        }
    }
    else if (name == "system")
    {
        SUPERVISOR::Config sup_cfg;
        res = Settings::getInstance().getSupervisorConfig(sup_cfg);
        if (res != ESP_OK)
        {
            overall_err = res;
            reason      = esp_err_to_name(res);
            return;
        }
        const SettingsFields f = apply_system_settings(settings_node, sup_cfg);
        if (!section_ok(f, sup_cfg, Settings::checkSupervisorConfig, overall_err, reason))
        {
            return;
        }
        no_fields = (f.n == 0);
        if (!no_fields)
        {
            res = Settings::getInstance().setSupervisorConfig(sup_cfg);
        }
    }
    else
    {
        overall_err = ESP_ERR_NOT_FOUND;
        reason      = "unknown settings section";
        return;
    }

    if (no_fields)
    {
        overall_err = ESP_ERR_INVALID_ARG;
        reason      = "no valid settings fields";
        return;
    }

    if (res != ESP_OK)
    {
        overall_err = res;
        reason      = esp_err_to_name(res);
    }
}

MiddlewareJsonResult m_settings_set(cJSON* payload)
{
    if (payload == nullptr)
    {
        return json_result(json_error_body("Payload cannot be null"), 400);
    }

    esp_err_t   overall_err = ESP_OK;
    std::string reason;

    if (cJSON_IsArray(payload))
    {
        int size = cJSON_GetArraySize(payload);
        for (int i = 0; i < size; i++)
        {
            cJSON* item = cJSON_GetArrayItem(payload, i);
            process_single_setting_item(item, overall_err, reason);
        }
    }
    else if (cJSON_IsObject(payload))
    {
        process_single_setting_item(payload, overall_err, reason);
    }
    else
    {
        return json_result(json_error_body("Payload must be a JSON array or object"), 400);
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        return json_result(nullptr, 500);
    }
    if (overall_err == ESP_OK)
    {
        cJSON_AddStringToObject(root, "status", "success");
    }
    else
    {
        cJSON_AddStringToObject(root, "status", "error");
        cJSON_AddStringToObject(root, "reason", reason.empty() ? esp_err_to_name(overall_err) : reason.c_str());
    }

    const int status = overall_err == ESP_OK                ? 200
                       : overall_err == ESP_ERR_INVALID_ARG ? 400
                       : overall_err == ESP_ERR_NOT_FOUND   ? 422
                                                            : 500;
    return json_result(root, status);
}
