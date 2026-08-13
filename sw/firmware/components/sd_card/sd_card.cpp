#include "sd_card.hpp"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <cstddef>
#include <memory>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

static const char* TAG = "SD_CARD";

static bool path_has_traversal(const char* path)
{
    if (path == nullptr)
        return true;

    while (*path != '\0')
    {
        while (*path == '/')
            path++;

        const char* start = path;
        while (*path != '\0' && *path != '/')
            path++;

        size_t len = (size_t)(path - start);
        if (len == 2 && start[0] == '.' && start[1] == '.')
            return true;
    }

    return false;
}

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

    if (mount_callback)
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

    if (unmount_callback)
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

esp_err_t SDCard::create_file(const char* relative_path)
{
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

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
    fclose(f);
    return ESP_OK;
}

esp_err_t SDCard::create_directory(const char* relative_path)
{
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    struct stat st;

    if (stat(filepath.get(), &st) == 0)
    {
        if (S_ISDIR(st.st_mode))
            return ESP_OK;  // Already exists
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
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    if (!exists(filepath.get()))
    {
        return ESP_ERR_NOT_FOUND;
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
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    struct stat st;

    if (stat(filepath.get(), &st) != 0 || !S_ISDIR(st.st_mode))
    {
        return ESP_ERR_NOT_FOUND;
    }

    DIR* dir = opendir(filepath.get());
    if (!dir)
    {
        return ESP_FAIL;
    }

    const char* base_rel = (relative_path == nullptr || relative_path[0] == '\0' || strcmp(relative_path, "/") == 0) ? "" : relative_path;
    if (base_rel[0] == '/' && base_rel[1] == '\0') base_rel = ""; // edge case for "/"

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        char child_abs[PATH_MAX];
        snprintf(child_abs, sizeof(child_abs), "%s/%s", filepath.get(), entry->d_name);
        
        char child_rel[PATH_MAX];
        snprintf(child_rel, sizeof(child_rel), "%s/%s", base_rel, entry->d_name);

        struct stat child_st;
        if (stat(child_abs, &child_st) == 0)
        {
            if (S_ISDIR(child_st.st_mode))
            {
                if (delete_directory(child_rel) != ESP_OK)
                {
                    closedir(dir);
                    return ESP_FAIL;
                }
            }
            else
            {
                if (unlink(child_abs) != 0)
                {
                    ESP_LOGE(TAG, "Failed to delete file %s: %s", child_abs, strerror(errno));
                    closedir(dir);
                    return ESP_FAIL;
                }
            }
        }
    }
    closedir(dir);

    if (rmdir(filepath.get()) != 0)
    {
        ESP_LOGE(TAG, "Failed to delete directory %s: %s", filepath.get(), strerror(errno));
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t SDCard::write_file(const char* relative_path, const void* data, size_t size, bool append)
{
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    if (data == nullptr || size == 0)
        return ESP_ERR_INVALID_ARG;

    const char* mode = append ? "a" : "w";

    FILE* f = fopen(filepath.get(), mode);
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file %s in mode '%s'", filepath.get(), mode);
        return ESP_FAIL;
    }

    size_t written = fwrite(data, 1, size, f);

    fflush(f);
    if (fsync(fileno(f)) != 0)
    {
        ESP_LOGE(TAG, "Hardware sync failed for %s!", filepath.get());
    }

    fclose(f);

    if (written != size)
    {
        ESP_LOGE(TAG, "Write incomplete! Wrote %zu/%zu bytes to %s", written, size, filepath.get());
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Successfully %s %zu bytes to %s", append ? "appended" : "wrote", size, filepath.get());
    return ESP_OK;
}

esp_err_t SDCard::read_file(const char* relative_path, void* buffer, size_t max_size, size_t* bytes_read)
{
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

    esp_err_t err = get_absolute_path(relative_path, filepath.get(), buffer_size);

    if (err != ESP_OK)
    {
        return err;
    }

    if (buffer == nullptr || max_size == 0)
        return ESP_ERR_INVALID_ARG;

    FILE* f = fopen(filepath.get(), "r");
    if (f == nullptr)
    {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", filepath.get());
        return ESP_FAIL;
    }

    size_t read_len = fread(buffer, 1, max_size, f);

    fclose(f);

    if (bytes_read != nullptr)
    {
        *bytes_read = read_len;
    }

    ESP_LOGI(TAG, "Successfully read %zu bytes from %s", read_len, filepath.get());
    return ESP_OK;
}

cJSON* SDCard::scan_directory(const char* relative_path, int depth)
{
    if (depth > SCAN_DEPTH_LIMIT)
        return NULL;

    struct ScanBuffers
    {
        char full_path[PATH_MAX];
        char next_rel[PATH_MAX];
        char next_full[PATH_MAX];
    };

    std::unique_ptr<ScanBuffers> bufs(new ScanBuffers);

    if (get_absolute_path(relative_path, bufs->full_path, sizeof(bufs->full_path)) != ESP_OK)
    {
        return NULL;
    }

    cJSON* root = cJSON_CreateObject();
    if (root == nullptr)
    {
        ESP_LOGE(TAG, "OOM building directory scan");
        return NULL;
    }
    cJSON_AddStringToObject(root, "path", (relative_path[0] == '\0') ? "/" : relative_path);

    cJSON* children = cJSON_CreateArray();
    if (children == nullptr)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "OOM building directory scan children");
        return NULL;
    }
    cJSON_AddItemToObject(root, "children", children);

    DIR* dir = opendir(bufs->full_path);
    if (!dir)
    {
        return root;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        snprintf(bufs->next_rel, sizeof(bufs->next_rel), "%s/%s", (relative_path[0] == '\0') ? "" : relative_path,
                 entry->d_name);

        if (get_absolute_path(bufs->next_rel, bufs->next_full, sizeof(bufs->next_full)) != ESP_OK)
            continue;

        struct stat st;
        if (stat(bufs->next_full, &st) == 0 && S_ISDIR(st.st_mode))
        {
            cJSON* sub_dir = scan_directory(bufs->next_rel, depth + 1);
            if (sub_dir)
                cJSON_AddItemToArray(children, sub_dir);
        }
        else
        {
            cJSON* file = cJSON_CreateObject();
            if (file == nullptr)
            {
                ESP_LOGE(TAG, "OOM building directory scan entry");
                continue;
            }
            cJSON_AddStringToObject(file, "name", entry->d_name);
            cJSON_AddStringToObject(file, "type", "file");
            cJSON_AddNumberToObject(file, "size", (double)st.st_size);
            cJSON_AddItemToArray(children, file);
        }
    }

    closedir(dir);

    return root;
}

esp_err_t SDCard::get_absolute_path(const char* relative_path, char* out_buf, size_t out_size)
{
    if (relative_path == nullptr)
        return ESP_ERR_INVALID_ARG;

    if (path_has_traversal(relative_path))
    {
        ESP_LOGE(TAG, "Rejected path containing traversal component: %s", relative_path);
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(mount_path) + strlen(relative_path) >= out_size)
    {
        ESP_LOGE(TAG, "Path is too long to fit in buffer!");
        return ESP_ERR_NO_MEM;
    }

    if (relative_path[0] == '\0' || strcmp(relative_path, "/") == 0)
    {
        snprintf(out_buf, out_size, "%s", mount_path);
    }
    else
    {
        const char* p = (relative_path[0] == '/') ? relative_path + 1 : relative_path;
        snprintf(out_buf, out_size, "%s/%s", mount_path, p);
    }

    return ESP_OK;
}

bool SDCard::is_path_under(const char* path, const char* root)
{
    if (path == nullptr || root == nullptr)
        return false;

    if (path_has_traversal(path))
        return false;

    size_t root_len = strlen(root);

    if (strncmp(path, root, root_len) != 0)
        return false;

    return (path[root_len] == '\0' || path[root_len] == '/');
}

esp_err_t SDCard::open_file(const char* relative_path, const char* mode, FILE*& fd)
{
    if (mode == nullptr || relative_path == nullptr)
        return ESP_ERR_INVALID_ARG;

    if (!is_mounted())
    {
        ESP_LOGE(TAG, "SD Card not mounted");
        return ESP_FAIL;
    }

    if (strstr(relative_path, "..") || strlen(relative_path) <= 1)
    {
        return ESP_FAIL;
    }

    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

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
    if (fd != nullptr)
    {
        fclose(fd);
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t SDCard::file_write_chunk(FILE* fd, const char* chunk, size_t len)
{
    if (fd == nullptr)
        return ESP_FAIL;

    if (fwrite(chunk, 1, len, fd) != len)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

size_t SDCard::file_read_chunk(FILE* fd, char* chunk, size_t max_len)
{
    if (fd == nullptr)
        return 0;

    return fread(chunk, 1, max_len, fd);
}

esp_err_t SDCard::get_file_stat(const char* relative_path, struct stat* st)
{
    const size_t buffer_size = PATH_MAX;
    auto         filepath    = std::make_unique<char[]>(buffer_size);

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
        if (errno == ENOENT)
        {
            ESP_LOGD(TAG, "File does not exist: %s", filepath.get());
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGE(TAG, "Failed to stat file %s (errno: %d)", filepath.get(), errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}