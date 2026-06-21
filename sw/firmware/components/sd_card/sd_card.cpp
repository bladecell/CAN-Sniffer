#include "sd_card.hpp"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

static const char* TAG = "SD_CARD";

SDCard::SDCard() : card(nullptr), mount_path(nullptr)
{
}

SDCard::~SDCard()
{
    if (card != nullptr && mount_path != nullptr)
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

/* SD Card Management */

esp_err_t SDCard::init(const SDCard::Config& config)
{
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

bool SDCard::card_present()
{
    return (gpio_get_level(cd_pin) == 0);
}

void SDCard::update_card_status()
{
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

    mount_callback();

    return ret;
}

esp_err_t SDCard::unmount_sdcard()
{
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

    unmount_callback();

    return ESP_OK;
}

esp_err_t SDCard::format_sdcard()
{
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
    return card != nullptr;
}

void SDCard::get_sd_info(SDCard::SDInfo& sd_info)
{
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

    sd_info.used_space_mb = (total_bytes >= free_bytes) ? ((total_bytes - free_bytes) / (1024 * 1024)) : 0;

    if (card != nullptr)
    {
        strncpy(sd_info.name, card->cid.name, sizeof(sd_info.name) - 1);
        sd_info.name[sizeof(sd_info.name) - 1] = '\0';

        sd_info.capacity_mb  = ((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024 * 1024);
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

/* Filesystem Management */

void SDCard::get_stat(const char* path, struct stat* st)
{
    stat(path, st);
}

bool SDCard::exists(const char* path)
{
    struct stat st;
    return (stat(path, &st) == 0);
}

bool SDCard::is_file(struct stat* st)
{
    if (st == nullptr)
    {
        return false;
    }
    return S_ISREG(st->st_mode);
}

bool SDCard::is_directory(struct stat* st)
{
    if (st == nullptr)
    {
        return false;
    }
    return S_ISDIR(st->st_mode);
}

esp_err_t SDCard::create_file(const char* path)
{
    struct stat st;
    if (stat(path, &st) == 0 && S_ISREG(st.st_mode))
    {
        return ESP_OK;
    }

    FILE* f = fopen(path, "w");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create file %s: %s", path, strerror(errno));
        return ESP_FAIL;
    }
    fclose(f);
    return ESP_OK;
}

esp_err_t SDCard::create_directory(const char* path)
{
    struct stat st;

    if (stat(path, &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
            return ESP_OK;  // Already exists
    }

    if (mkdir(path, 0777) != 0)
    {
        ESP_LOGE(TAG, "Failed to create directory %s: %s", path, strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SDCard::delete_file(const char* path)
{
    if (!exists(path))
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (unlink(path) != 0)
    {
        ESP_LOGE(TAG, "Failed to delete file %s: %s", path, strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SDCard::delete_directory(const char* path)
{
    struct stat st;

    if (stat(path, &st) != 0 || !S_ISDIR(st.st_mode))
    {
        return ESP_ERR_NOT_FOUND;
    }

    if (rmdir(path) != 0)
    {
        ESP_LOGE(TAG, "Failed to delete directory %s: %s", path, strerror(errno));
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t SDCard::write_file(const char* filename, const void* data, size_t size, bool append)
{
    if (data == nullptr || size == 0)
        return ESP_ERR_INVALID_ARG;

    const char* mode = append ? "a" : "w";

    FILE* f = fopen(filename, mode);
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file %s in mode '%s'", filename, mode);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, size, f);

    fflush(f);
    if (fsync(fileno(f)) != 0)
    {
        ESP_LOGE(TAG, "Hardware sync failed for %s!", filename);
    }

    fclose(f);

    if (written != size)
    {
        ESP_LOGE(TAG, "Write incomplete! Wrote %zu/%zu bytes to %s", written, size, filename);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Successfully %s %zu bytes to %s", append ? "appended" : "wrote", size, filename);
    return ESP_OK;
}

esp_err_t SDCard::read_file(const char* filename, void* buffer, size_t max_size, size_t* bytes_read)
{
    if (buffer == nullptr || max_size == 0)
        return ESP_ERR_INVALID_ARG;

    FILE* f = fopen(filename, "r");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", filename);
        return ESP_FAIL;
    }

    size_t read_len = fread(buffer, 1, max_size, f);

    fclose(f);

    if (bytes_read != nullptr)
    {
        *bytes_read = read_len;
    }

    ESP_LOGI(TAG, "Successfully read %zu bytes from %s", read_len, filename);
    return ESP_OK;
}

cJSON* SDCard::scan_directory(const char* relative_path, int depth)
{
    if (depth > SCAN_DEPTH_LIMIT)
        return NULL;

    char full_path[PATH_MAX];
    get_absolute_path(relative_path, full_path, sizeof(full_path));

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "path", (relative_path[0] == '\0') ? "/" : relative_path);

    cJSON* children = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "children", children);

    DIR* dir = opendir(full_path);
    if (!dir)
        return root;

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        char next_rel[PATH_MAX];
        snprintf(next_rel, sizeof(next_rel), "%s/%s", (relative_path[0] == '\0') ? "" : relative_path, entry->d_name);

        char next_full[PATH_MAX];
        get_absolute_path(next_rel, next_full, sizeof(next_full));

        struct stat st;
        if (stat(next_full, &st) == 0 && S_ISDIR(st.st_mode))
        {
            cJSON* sub_dir = scan_directory(next_rel, depth + 1);
            if (sub_dir)
                cJSON_AddItemToArray(children, sub_dir);
        }
        else
        {
            cJSON* file = cJSON_CreateObject();
            cJSON_AddStringToObject(file, "name", entry->d_name);
            cJSON_AddStringToObject(file, "type", "file");
            cJSON_AddNumberToObject(file, "size", (double)st.st_size);
            cJSON_AddItemToArray(children, file);
        }
    }
    closedir(dir);
    return root;
}

void SDCard::get_absolute_path(const char* relative_path, char* out_buf, size_t out_size)
{
    if (relative_path == nullptr || relative_path[0] == '\0' || strcmp(relative_path, "/") == 0)
    {
        snprintf(out_buf, out_size, "%s", this->mount_path);
    }
    else
    {
        const char* p = (relative_path[0] == '/') ? relative_path + 1 : relative_path;
        snprintf(out_buf, out_size, "%s/%s", this->mount_path, p);
    }
}