#include "sd_card.hpp"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <cstddef>
#include <memory>
#include <new>
#include <utility>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "delete_tree.hpp"
#include "sd_path.hpp"
#include "esp_vfs_fat.h"

static const char* TAG = "SD_CARD";

namespace
{

// SD-2: the supervisor polls the card detect pin every second; it must not queue behind a long transfer.
constexpr TickType_t STATUS_LEASE_TIMEOUT = pdMS_TO_TICKS(250);
constexpr TickType_t DEINIT_LEASE_TIMEOUT = pdMS_TO_TICKS(5000);
// Recursive delete holds the lease the whole time.
constexpr int64_t DELETE_DEADLINE_US = 30LL * 1000 * 1000;

esp_err_t directory_errno_to_error(int error)
{
    switch (error)
    {
    case ENOENT:
    case ENOTDIR:
        return ESP_ERR_NOT_FOUND;
    case ENOMEM:
    case EMFILE:
    case ENFILE:
        return ESP_ERR_NO_MEM;
    default:
        // In particular, do not turn EIO, ENODEV, or descriptor exhaustion
        // into a valid (but incomplete) directory tree.
        return ESP_FAIL;
    }
}

}  // namespace

esp_err_t SDCard::validate_relative_path(const char* path, bool allow_root)
{
    return sd_path::validRelative(path, allow_root) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

SDCard::SDCard() : card(nullptr), mount_path(nullptr)
{
    ensure_operation_mutex();
}

SDCard::~SDCard()
{
    {
        Operation operation = acquire_operation(portMAX_DELAY);
        if (operation && card != nullptr && mount_path != nullptr)
        {
            ESP_LOGI(TAG, "Cleaning up and unmounting filesystem from %s...", mount_path);
            esp_err_t ret = unmount_sdcard();
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "Filesystem successfully unmounted during destruction.");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to unmount filesystem during destruction: %s", esp_err_to_name(ret));
            }
            mount_path = nullptr;
        }
    }

    if (operation_mutex != nullptr)
    {
        vSemaphoreDelete(operation_mutex);
        operation_mutex = nullptr;
    }
}

SDCard::Operation::Operation(SDCard* owner, TaskHandle_t owner_task, esp_err_t status, bool mounted) :
    owner_(owner), owner_task_(owner_task), status_(status), mounted_(mounted)
{
}

SDCard::Operation::Operation(Operation&& other) noexcept
{
    if (other.owned_by_current_task())
    {
        owner_            = other.owner_;
        owner_task_       = other.owner_task_;
        status_           = other.status_;
        mounted_          = other.mounted_;
        other.owner_      = nullptr;
        other.owner_task_ = nullptr;
        other.status_     = ESP_ERR_INVALID_STATE;
        other.mounted_    = false;
    }
    else
    {
        status_ = (other.status_ == ESP_OK) ? ESP_ERR_INVALID_STATE : other.status_;
    }
}

SDCard::Operation& SDCard::Operation::operator=(Operation&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    const TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

    // Moving an operation from another task must not release this operation
    // before rejecting the source. Either object may represent one recursive
    // mutex level, so leave both untouched when task affinity is violated.
    if ((owner_ != nullptr && owner_task_ != current_task) ||
        (other.owner_ != nullptr && other.owner_task_ != current_task))
    {
        ESP_LOGE(TAG, "Refusing to move-assign SD operation across tasks");
        return *this;
    }

    release();
    owner_            = other.owner_;
    owner_task_       = other.owner_task_;
    status_           = other.status_;
    mounted_          = other.mounted_;
    other.owner_      = nullptr;
    other.owner_task_ = nullptr;
    other.status_     = ESP_ERR_INVALID_STATE;
    other.mounted_    = false;

    return *this;
}

SDCard::Operation::~Operation()
{
    release();
}

bool SDCard::Operation::acquired() const
{
    return owned_by_current_task();
}

SDCard::Operation::operator bool() const
{
    return acquired();
}

esp_err_t SDCard::Operation::status() const
{
    return (owner_ != nullptr && !owned_by_current_task()) ? ESP_ERR_INVALID_STATE : status_;
}

bool SDCard::Operation::mounted() const
{
    return acquired() && mounted_;
}

bool SDCard::Operation::owned_by_current_task() const
{
    return owner_ != nullptr && status_ == ESP_OK && owner_task_ == xTaskGetCurrentTaskHandle();
}

void SDCard::Operation::release()
{
    if (owner_ != nullptr)
    {
        if (owned_by_current_task())
        {
            xSemaphoreGiveRecursive(owner_->operation_mutex);
        }
        else
        {
            ESP_LOGE(TAG, "Refusing to release SD operation from a non-owner task");
        }
        owner_      = nullptr;
        owner_task_ = nullptr;
        status_     = ESP_ERR_INVALID_STATE;
        mounted_    = false;
    }
}

bool SDCard::ensure_operation_mutex()
{
    if (operation_mutex == nullptr)
    {
        operation_mutex = xSemaphoreCreateRecursiveMutex();
    }
    return operation_mutex != nullptr;
}

SDCard::Operation SDCard::acquire_operation(TickType_t timeout, bool require_mounted)
{
    if (!ensure_operation_mutex())
    {
        return Operation(nullptr, nullptr, ESP_ERR_NO_MEM, false);
    }

    if (xSemaphoreTakeRecursive(operation_mutex, timeout) != pdTRUE)
    {
        return Operation(nullptr, nullptr, ESP_ERR_TIMEOUT, false);
    }

    const bool mounted = card != nullptr;
    if (require_mounted && !mounted)
    {
        xSemaphoreGiveRecursive(operation_mutex);
        return Operation(nullptr, nullptr, ESP_ERR_INVALID_STATE, false);
    }

    return Operation(this, xTaskGetCurrentTaskHandle(), ESP_OK, mounted);
}

/* SD Card Management */

esp_err_t SDCard::init(const SDCard::Config& config)
{
    if (!ensure_operation_mutex())
    {
        return ESP_ERR_NO_MEM;
    }

    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    mount_path = config.base_path;

    // Mount Settings
    mount_config.format_if_mount_failed   = config.format_if_mount_failed;
    mount_config.max_files                = config.max_files;
    mount_config.allocation_unit_size     = 16 * 1024;
    mount_config.disk_status_check_enable = false;

    cd_pin = config.cd_pin;

    gpio_reset_pin(cd_pin);
    gpio_set_direction(cd_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(cd_pin, GPIO_PULLUP_ONLY);

    // Native Host Configuration
    host              = SDMMC_HOST_DEFAULT();
    host.slot         = config.slot;
    host.max_freq_khz = MAX_FREQUENCY_KHZ;

    // Slot Configuration
    slot_config.clk = config.sclk_pin;
    slot_config.cmd = config.mosi_pin;
    slot_config.d0  = config.miso_pin;

    slot_config.d1 = GPIO_NUM_NC;
    slot_config.d2 = GPIO_NUM_NC;
    slot_config.d3 = GPIO_NUM_NC;

    slot_config.gpio_cd = GPIO_NUM_NC;
    slot_config.gpio_wp = GPIO_NUM_NC;
    slot_config.width   = 1;
    slot_config.flags   = 0;

    last_stable_state = !card_present();

    update_card_status();

    if (is_mounted())
    {
        print_card_status();
    }

    ESP_LOGI(TAG, "SDCard driver initialized");
    return ESP_OK;
}

esp_err_t SDCard::deinit()
{
    if (!ensure_operation_mutex())
    {
        return ESP_ERR_NO_MEM;
    }

    Operation operation = acquire_operation(DEINIT_LEASE_TIMEOUT);
    if (!operation)
    {
        return operation.status();
    }

    if (cd_pin == GPIO_NUM_NC && card == nullptr)
    {
        return ESP_OK;
    }

    esp_err_t ret = unmount_sdcard();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SD deinit: unmount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (cd_pin != GPIO_NUM_NC)
    {
        gpio_reset_pin(cd_pin);
        cd_pin = GPIO_NUM_NC;
    }
    last_stable_state = false;
    return ESP_OK;
}

bool SDCard::card_present()
{
    // A GPIO read needs no lease; taking it made the supervisor wait behind every file transfer (SD-2).
    return cd_pin != GPIO_NUM_NC && gpio_get_level(cd_pin) == 0;
}

void SDCard::update_card_status()
{
    // Bounded: while a download, upload or delete holds the lease a card is in use, so skip this round.
    Operation operation = acquire_operation(STATUS_LEASE_TIMEOUT);
    if (!operation || cd_pin == GPIO_NUM_NC)
    {
        return;
    }

    bool current_raw = card_present();

    if (current_raw != last_stable_state)
    {
        // Small debounce delay
        vTaskDelay(pdMS_TO_TICKS(100));

        if (card_present() == current_raw)
        {
            last_stable_state = current_raw;

            if (last_stable_state)
            {
                ESP_LOGI(TAG, "SD Card detected, mounting...");
                mount_sdcard();
            }
            else
            {
                ESP_LOGI(TAG, "SD Card removed, unmounting...");
                unmount_sdcard();
            }
        }
    }
}

esp_err_t SDCard::mount_sdcard()
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    if (card != nullptr)
    {
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_path, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        card = nullptr;
        return ret;
    }

    if (mount_callback)
    {
        mount_callback();
    }

    return ret;
}

esp_err_t SDCard::unmount_sdcard()
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    if (card == nullptr)
    {
        return ESP_OK;
    }

    esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_path, card);

    if (ret != ESP_OK)
    {
        return ret;
    }

    card = nullptr;

    if (unmount_callback)
    {
        unmount_callback();
    }

    return ESP_OK;
}

esp_err_t SDCard::format_sdcard()
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    esp_err_t ret = mount_sdcard();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Cannot format: Mount failed (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Formatting SD card...");
    ret = esp_vfs_fat_sdcard_format(mount_path, card);

    if (ret == ESP_OK)
    {
        unmount_sdcard();
        ret = mount_sdcard();
    }

    return ret;
}

bool SDCard::is_mounted()
{
    // One pointer read, no lease: callers that need the mount to stay put hold their own lease.
    return card != nullptr;
}

void SDCard::get_sd_info(SDCard::SDInfo& sd_info)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return;
    }

    sd_info.is_mounted = is_mounted();
    sd_info.is_present = card_present();

    uint64_t total_bytes = 0;
    uint64_t free_bytes  = 0;

    if (mount_path != nullptr)
    {
        esp_vfs_fat_info(mount_path, &total_bytes, &free_bytes);
        strncpy(sd_info.mount_path, mount_path, sizeof(sd_info.mount_path) - 1);
        sd_info.mount_path[sizeof(sd_info.mount_path) - 1] = '\0';
    }
    else
    {
        sd_info.mount_path[0] = '\0';
    }

    sd_info.used_space_mb = (total_bytes >= free_bytes) ? ((total_bytes - free_bytes) / (1024ULL * 1024)) : 0;

    if (card != nullptr)
    {
        strncpy(sd_info.name, card->cid.name, sizeof(sd_info.name) - 1);
        sd_info.name[sizeof(sd_info.name) - 1] = '\0';

        sd_info.capacity_mb  = ((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024ULL * 1024);
        sd_info.max_freq_mhz = card->max_freq_khz / 1000;
        sd_info.is_sdio      = card->is_sdio;
        sd_info.is_mmc       = card->is_mmc;
    }
    else
    {
        sd_info.name[0]      = '\0';
        sd_info.capacity_mb  = 0;
        sd_info.max_freq_mhz = 0;
        sd_info.is_sdio      = 0;
        sd_info.is_mmc       = 0;
    }
}

void SDCard::print_card_status()
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return;
    }

    SDCard::SDInfo sd_info;

    get_sd_info(sd_info);

    ESP_LOGI(TAG, "--- SD Card Info ---");
    ESP_LOGI(TAG, "Present: %s", sd_info.is_present ? "true" : "false");
    ESP_LOGI(TAG, "Mounted: %s", sd_info.is_mounted ? "true" : "false");
    if (sd_info.is_mounted)
    {
        ESP_LOGI(TAG, "Name: %s", sd_info.name);
        ESP_LOGI(TAG, "Type: %s", (sd_info.is_sdio) ? "SDIO" : (sd_info.is_mmc) ? "MMC" : "SDSC/SDHC/SDXC");
        ESP_LOGI(TAG, "Capacity: %llu MB", sd_info.capacity_mb);
        ESP_LOGI(TAG, "Used Space: %llu MB", sd_info.used_space_mb);  // Changed to %llu
        ESP_LOGI(TAG, "Speed: %u MHz", (unsigned int)sd_info.max_freq_mhz);
    }
}

void SDCard::on_mount(Callback cb)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return;
    }

    mount_callback = std::move(cb);
}

void SDCard::on_unmount(Callback cb)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return;
    }

    unmount_callback = std::move(cb);
}

/* Filesystem Management */

void SDCard::get_stat(const char* path, struct stat* st)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return;
    }

    stat(path, st);
}

bool SDCard::exists(const char* path)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return false;
    }

    struct stat st;
    return (stat(path, &st) == 0);
}

bool SDCard::is_file(struct stat* st)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return false;
    }

    if (st == nullptr)
    {
        return false;
    }
    return S_ISREG(st->st_mode);
}

bool SDCard::is_directory(struct stat* st)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return false;
    }

    if (st == nullptr)
    {
        return false;
    }
    return S_ISDIR(st->st_mode);
}

esp_err_t SDCard::create_file(const char* relative_path)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    struct stat st;
    if (stat(filepath.get(), &st) == 0 && S_ISREG(st.st_mode))
    {
        return ESP_OK;
    }

    FILE* f = fopen(filepath.get(), "w");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create file %s: %s", filepath.get(), strerror(errno));
        return ESP_FAIL;
    }
    if (fclose(f) != 0)
    {
        ESP_LOGE(TAG, "Failed to close new file %s: %s", filepath.get(), strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SDCard::create_directory(const char* relative_path)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    struct stat st;

    if (stat(filepath.get(), &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
        {
            return ESP_OK;  // Already exists
        }
    }

    if (mkdir(filepath.get(), 0777) != 0)
    {
        ESP_LOGE(TAG, "Failed to create directory %s: %s", filepath.get(), strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SDCard::delete_file(const char* relative_path)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    struct stat st;
    if (stat(filepath.get(), &st) != 0)
    {
        const int stat_errno = errno;
        if (stat_errno == ENOENT || stat_errno == ENOTDIR)
        {
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGE(TAG, "Failed to stat file %s (errno: %d)", filepath.get(), stat_errno);
        return ESP_FAIL;
    }

    if (unlink(filepath.get()) != 0)
    {
        ESP_LOGE(TAG, "Failed to delete file %s: %s", filepath.get(), strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SDCard::delete_directory(const char* relative_path)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    // Deleting the mount point is never a valid directory operation. This
    // guard is deliberately below the REST layer as a final safety boundary.
    if (validate_relative_path(relative_path, false) != ESP_OK)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // One heap path buffer shared by every level of the walk (sd_tree::deleteTree keeps no per-level path on the stack).
    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]);
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    struct stat st;

    if (stat(filepath.get(), &st) != 0)
    {
        const int stat_errno = errno;
        if (stat_errno == ENOENT || stat_errno == ENOTDIR)
        {
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGE(TAG, "Failed to stat directory %s (errno: %d)", filepath.get(), stat_errno);
        return ESP_FAIL;
    }

    if (!S_ISDIR(st.st_mode))
    {
        return ESP_ERR_NOT_FOUND;
    }

    size_t len = strlen(filepath.get());
    while (len > 1 && filepath[len - 1] == '/')
    {
        filepath[--len] = '\0';
    }

    const sd_tree::Deadline deadline = {esp_timer_get_time, esp_timer_get_time() + DELETE_DEADLINE_US};
    switch (sd_tree::deleteTree(filepath.get(), buffer_size, len, 0, &deadline))
    {
        case sd_tree::Result::Ok:
            return ESP_OK;
        case sd_tree::Result::TimedOut:
            ESP_LOGE(TAG, "Deleting %s took longer than %d s, stopped", relative_path, static_cast<int>(DELETE_DEADLINE_US / 1000000));
            return ESP_ERR_TIMEOUT;
        case sd_tree::Result::TooDeep:
            ESP_LOGE(TAG, "Directory %s nests deeper than %d levels", relative_path, sd_tree::kMaxDepth);
            return ESP_ERR_INVALID_SIZE;
        case sd_tree::Result::PathTooLong:
            ESP_LOGE(TAG, "Path under %s is too long to delete", relative_path);
            return ESP_ERR_INVALID_SIZE;
        default:
            ESP_LOGE(TAG, "Failed to delete directory %s", relative_path);
            return ESP_FAIL;
    }
}

esp_err_t SDCard::write_file(const char* relative_path, const void* data, size_t size, bool append)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    if (data == nullptr || size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const char* mode = append ? "a" : "w";

    FILE* f = fopen(filepath.get(), mode);
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file %s in mode '%s'", filepath.get(), mode);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, size, f);

    bool flushed = (fflush(f) == 0);
    if (!flushed)
    {
        ESP_LOGE(TAG, "Flush failed for %s: %s", filepath.get(), strerror(errno));
    }
    if (fsync(fileno(f)) != 0)
    {
        ESP_LOGE(TAG, "Hardware sync failed for %s!", filepath.get());
        flushed = false;
    }
    const bool closed = (fclose(f) == 0);
    if (!closed)
    {
        ESP_LOGE(TAG, "Close failed for %s: %s", filepath.get(), strerror(errno));
    }

    if (written != size)
    {
        ESP_LOGE(TAG, "Write incomplete! Wrote %zu/%zu bytes to %s", written, size, filepath.get());
        return ESP_FAIL;
    }
    if (!flushed || !closed)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Successfully %s %zu bytes to %s", append ? "appended" : "wrote", size, filepath.get());
    return ESP_OK;
}

esp_err_t SDCard::read_file(const char* relative_path, void* buffer, size_t max_size, size_t* bytes_read)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    if (buffer == nullptr || max_size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FILE* f = fopen(filepath.get(), "r");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", filepath.get());
        return ESP_FAIL;
    }

    size_t     read_len = fread(buffer, 1, max_size, f);
    const bool read_err = ferror(f) != 0;

    (void)fclose(f);  // read-only handle, nothing to lose on close failure

    if (read_err)
    {
        ESP_LOGE(TAG, "Read error on %s", filepath.get());
        return ESP_FAIL;
    }

    if (bytes_read != nullptr)
    {
        *bytes_read = read_len;
    }

    ESP_LOGI(TAG, "Successfully read %zu bytes from %s", read_len, filepath.get());
    return ESP_OK;
}

esp_err_t SDCard::get_file_tree(const char* relative_path, const sd_tree::Limits& limits, cJSON*& tree)
{
    tree = nullptr;
    if (relative_path == nullptr || !limits.valid())
    {
        return ESP_ERR_INVALID_ARG;
    }

    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    sd_tree::Budget budget(limits);
    const esp_err_t err = scan_directory_locked(relative_path, 0, budget, tree);
    if (err != ESP_OK)
    {
        return err;
    }
    if (cJSON_AddBoolToObject(tree, "truncated", budget.truncated()) == nullptr)
    {
        cJSON_Delete(tree);
        tree = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

// `level` counts the directories between this one and the requested one (0 = the requested directory).
esp_err_t SDCard::scan_directory_locked(const char* relative_path, int level, sd_tree::Budget& budget, cJSON*& tree)
{
    tree = nullptr;

    if (level > SCAN_DEPTH_LIMIT)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    struct ScanBuffers
    {
        char full_path[PATH_MAX];
        char next_rel[PATH_MAX];
        char next_full[PATH_MAX];
    };

    std::unique_ptr<ScanBuffers> bufs(new (std::nothrow) ScanBuffers);
    if (!bufs)
    {
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t path_error = get_absolute_path(relative_path, bufs->full_path, sizeof(bufs->full_path));
    if (path_error != ESP_OK)
    {
        return path_error;
    }

    struct stat directory_stat;
    if (stat(bufs->full_path, &directory_stat) != 0)
    {
        const int stat_errno = errno;
        ESP_LOGD(TAG, "Failed to stat directory %s: %s", bufs->full_path, strerror(stat_errno));
        return directory_errno_to_error(stat_errno);
    }

    if (!S_ISDIR(directory_stat.st_mode))
    {
        ESP_LOGD(TAG, "Path is not a directory: %s", bufs->full_path);
        return ESP_ERR_NOT_FOUND;
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "OOM building directory scan");
        return ESP_ERR_NO_MEM;
    }
    if (cJSON_AddStringToObject(root, "path", (relative_path[0] == '\0') ? "/" : relative_path) == nullptr)
    {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON* children = cJSON_CreateArray();
    if (children == nullptr)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "OOM building directory scan children");
        return ESP_ERR_NO_MEM;
    }
    cJSON_AddItemToObject(root, "children", children);

    DIR* dir = opendir(bufs->full_path);
    if (!dir)
    {
        const int open_errno = errno;
        ESP_LOGD(TAG, "Failed to open directory %s: %s", bufs->full_path, strerror(open_errno));
        cJSON_Delete(root);
        return directory_errno_to_error(open_errno);
    }

    struct dirent* entry;
    int read_errno = 0;
    while (true)
    {
        errno = 0;
        entry = readdir(dir);
        if (entry == nullptr)
        {
            read_errno = errno;
            break;
        }

        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        const sd_tree::Budget::Verdict verdict = budget.next(level);
        if (verdict == sd_tree::Budget::Verdict::Skip)
        {
            continue;
        }
        if (verdict == sd_tree::Budget::Verdict::Stop)
        {
            break;  // limit reached: keep what is already in the tree
        }

        const size_t relative_len = strlen(relative_path);
        const char*  separator = (relative_len > 0 && relative_path[relative_len - 1] == '/') ? "" : "/";
        int next_rel_len = snprintf(bufs->next_rel, sizeof(bufs->next_rel), "%s%s%s", relative_path, separator,
                                    entry->d_name);
        if (next_rel_len < 0 || (size_t)next_rel_len >= sizeof(bufs->next_rel))
        {
            continue;
        }

        if (get_absolute_path(bufs->next_rel, bufs->next_full, sizeof(bufs->next_full)) != ESP_OK)
        {
            continue;
        }

        struct stat st;
        if (stat(bufs->next_full, &st) != 0)
        {
            const int stat_errno = errno;
            ESP_LOGD(TAG, "Failed to stat tree entry %s: %s", bufs->next_full, strerror(stat_errno));
            closedir(dir);
            cJSON_Delete(root);
            return directory_errno_to_error(stat_errno);
        }

        if (S_ISDIR(st.st_mode))
        {
            cJSON* sub_dir = nullptr;
            if (!budget.mayExpand(level))
            {
                // Listed but not opened: the client asks again with this path to see inside.
                budget.markTruncated();
                sub_dir = cJSON_CreateObject();
                if (sub_dir == nullptr || cJSON_AddStringToObject(sub_dir, "path", bufs->next_rel) == nullptr ||
                    cJSON_AddArrayToObject(sub_dir, "children") == nullptr ||
                    cJSON_AddTrueToObject(sub_dir, "truncated") == nullptr)
                {
                    cJSON_Delete(sub_dir);
                    closedir(dir);
                    cJSON_Delete(root);
                    return ESP_ERR_NO_MEM;
                }
                cJSON_AddItemToArray(children, sub_dir);
                continue;
            }
            esp_err_t err = scan_directory_locked(bufs->next_rel, level + 1, budget, sub_dir);
            if (err != ESP_OK)
            {
                closedir(dir);
                cJSON_Delete(root);
                return err;
            }
            cJSON_AddItemToArray(children, sub_dir);
        }
        else
        {
            cJSON* file = cJSON_CreateObject();
            if (file == nullptr)
            {
                ESP_LOGE(TAG, "OOM building directory scan entry");
                closedir(dir);
                cJSON_Delete(root);
                return ESP_ERR_NO_MEM;
            }
            if (cJSON_AddStringToObject(file, "name", entry->d_name) == nullptr ||
                cJSON_AddStringToObject(file, "type", "file") == nullptr ||
                cJSON_AddNumberToObject(file, "size", (double)st.st_size) == nullptr)
            {
                cJSON_Delete(file);
                closedir(dir);
                cJSON_Delete(root);
                return ESP_ERR_NO_MEM;
            }
            cJSON_AddItemToArray(children, file);
        }
    }

    const int close_result = closedir(dir);
    const int close_errno = errno;

    if (read_errno != 0)
    {
        ESP_LOGD(TAG, "Failed to read directory %s: %s", bufs->full_path, strerror(read_errno));
        cJSON_Delete(root);
        return directory_errno_to_error(read_errno);
    }

    if (close_result != 0)
    {
        ESP_LOGD(TAG, "Failed to close directory %s: %s", bufs->full_path, strerror(close_errno));
        cJSON_Delete(root);
        return directory_errno_to_error(close_errno);
    }

    tree = root;
    return ESP_OK;
}

esp_err_t SDCard::get_absolute_path(const char* relative_path, char* out_buf, size_t out_size)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    if (relative_path == nullptr || mount_path == nullptr || out_buf == nullptr || out_size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (validate_relative_path(relative_path, true) != ESP_OK)
    {
        ESP_LOGE(TAG, "Rejected unsafe relative path: %s", relative_path);
        return ESP_ERR_INVALID_ARG;
    }

    int written;
    if (relative_path[0] == '\0' || strcmp(relative_path, "/") == 0)
    {
        written = snprintf(out_buf, out_size, "%s", mount_path);
    }
    else
    {
        const char* p = (relative_path[0] == '/') ? relative_path + 1 : relative_path;
        written       = snprintf(out_buf, out_size, "%s/%s", mount_path, p);
    }

    if (written < 0 || (size_t)written >= out_size)
    {
        // Never expose a truncated path to a caller, even when it correctly
        // checks the returned error.
        out_buf[0] = '\0';
        ESP_LOGE(TAG, "Path is too long to fit in buffer!");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool SDCard::is_path_under(const char* path, const char* root)
{
    return sd_path::isUnder(path, root);
}

esp_err_t SDCard::open_file(const char* relative_path, const char* mode, FILE*& fd)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    if (mode == nullptr || relative_path == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_mounted())
    {
        ESP_LOGE(TAG, "SD Card not mounted");
        return ESP_FAIL;
    }

    if (strlen(relative_path) <= 1)
    {
        return ESP_FAIL;
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    fd = fopen(filepath.get(), mode);

    if (!fd)
    {
        ESP_LOGE(TAG, "Failed to open file '%s' in mode '%s'", filepath.get(), mode);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t SDCard::close_file(FILE* fd)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    if (fd != nullptr)
    {
        return fclose(fd) == 0 ? ESP_OK : ESP_FAIL;
    }
    return ESP_FAIL;
}

esp_err_t SDCard::file_write_chunk(FILE* fd, const char* chunk, size_t len)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    if (fd == nullptr || (chunk == nullptr && len != 0))
    {
        return ESP_FAIL;
    }

    if (fwrite(chunk, 1, len, fd) != len)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

size_t SDCard::file_read_chunk(FILE* fd, char* chunk, size_t max_len)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return 0;
    }

    if (fd == nullptr)
    {
        return 0;
    }

    return fread(chunk, 1, max_len, fd);
}

esp_err_t SDCard::get_file_stat(const char* relative_path, struct stat* st)
{
    Operation operation = acquire_operation(portMAX_DELAY);
    if (!operation)
    {
        return operation.status();
    }

    const size_t buffer_size = PATH_MAX;
    std::unique_ptr<char[]> filepath(new (std::nothrow) char[buffer_size]());
    if (!filepath)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    if (filepath.get() == nullptr || st == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (stat(filepath.get(), st) != 0)
    {
        if (errno == ENOENT || errno == ENOTDIR)
        {
            ESP_LOGD(TAG, "File does not exist: %s", filepath.get());
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGE(TAG, "Failed to stat file %s (errno: %d)", filepath.get(), errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}
