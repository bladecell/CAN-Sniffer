#pragma once

#include <functional>

#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "soc/gpio_num.h"

#define MAX_FREQUENCY_KHZ 20000
#define SCAN_DEPTH_LIMIT 10

class SDCard
{
public:
    struct Config
    {
        gpio_num_t  miso_pin;  // DAT0
        gpio_num_t  mosi_pin;  // CMD
        gpio_num_t  sclk_pin;  // CLK
        gpio_num_t  cd_pin;    // Card Detect
        const char* base_path;
        int         slot;
        int         max_files;
        bool        format_if_mount_failed;
    };

    struct SDInfo
    {
        char     name[32];
        char     mount_path[32];
        uint64_t capacity_mb;
        uint64_t used_space_mb;
        uint32_t max_freq_mhz;
        bool     is_sdio;
        bool     is_mmc;
        bool     is_mounted;
        bool     is_present;
    };

    esp_err_t init(const SDCard::Config& config);
    esp_err_t mount_sdcard();
    esp_err_t unmount_sdcard();
    esp_err_t format_sdcard();

    void get_sd_info(SDCard::SDInfo& sd_info);
    bool is_mounted();

    void print_card_status();

    void get_stat(const char* path, struct stat* st);

    bool exists(const char* path);
    bool is_file(struct stat* st);
    bool is_directory(struct stat* st);
    bool card_present();
    void update_card_status();

    esp_err_t create_file(const char* path);
    esp_err_t create_directory(const char* path);
    esp_err_t delete_file(const char* path);
    esp_err_t delete_directory(const char* path);
    esp_err_t write_file(const char* filename, const void* data, size_t size, bool append);
    esp_err_t read_file(const char* filename, void* buffer, size_t max_size, size_t* bytes_read);
    cJSON*    scan_directory(const char* path, int depth);

    typedef std::function<void()> Callback;

    void on_mount(Callback cb)
    {
        mount_callback = cb;
    }
    void on_unmount(Callback cb)
    {
        unmount_callback = cb;
    }

    SDCard();
    ~SDCard();

    static SDCard& getInstance()
    {
        static SDCard instance;
        return instance;
    }

private:
    SDCard(const SDCard&)            = delete;
    SDCard& operator=(const SDCard&) = delete;
    SDCard(SDCard&&)                 = delete;
    SDCard& operator=(SDCard&&)      = delete;

    bool       last_stable_state = false;
    gpio_num_t cd_pin            = GPIO_NUM_NC;

    sdmmc_card_t*              card         = nullptr;
    sdmmc_host_t               host         = {};
    esp_vfs_fat_mount_config_t mount_config = {};
    sdmmc_slot_config_t        slot_config  = {};
    const char*                mount_path   = nullptr;

    Callback mount_callback;
    Callback unmount_callback;
};