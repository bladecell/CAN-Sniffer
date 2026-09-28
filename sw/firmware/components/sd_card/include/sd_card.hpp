#pragma once

#include <sys/stat.h>

#include <functional>

#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"
#include "soc/gpio_num.h"

#define MAX_FREQUENCY_KHZ 20000
#define SCAN_DEPTH_LIMIT 10

class SDCard
{
public:
    // Owns one recursive SD operation. A lease is task-affine: it must be
    // acquired, used, moved, and destroyed by the same FreeRTOS task. A
    // cross-task lease is invalid and its destructor deliberately does not
    // release the recursive mutex. Acquire this before any non-SD lock;
    // callbacks run while it is held.
    class Operation
    {
    public:
        Operation(const Operation&)            = delete;
        Operation& operator=(const Operation&) = delete;
        Operation(Operation&& other) noexcept;
        Operation& operator=(Operation&& other) noexcept;
        ~Operation();

        // True only while this operation owns the SD mutex in this task.
        bool acquired() const;
        explicit operator bool() const;
        // ESP_OK, ESP_ERR_TIMEOUT, or the reason acquisition/use was refused.
        esp_err_t status() const;
        // Mounted-state snapshot taken while the operation was acquired.
        bool mounted() const;

    private:
        friend class SDCard;

        Operation(SDCard* owner, TaskHandle_t owner_task, esp_err_t status, bool mounted);
        bool owned_by_current_task() const;
        void release();

        SDCard*      owner_      = nullptr;
        TaskHandle_t owner_task_ = nullptr;
        esp_err_t    status_     = ESP_ERR_INVALID_STATE;
        bool         mounted_    = false;
    };

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

    // Acquires the SD mutex for a caller-specified time. If require_mounted is
    // true, a successful operation also guarantees mounted() is true.
    Operation acquire_operation(TickType_t timeout, bool require_mounted = false);

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

    esp_err_t create_file(const char* relative_path);
    esp_err_t create_directory(const char* relative_path);
    esp_err_t delete_file(const char* relative_path);
    esp_err_t delete_directory(const char* relative_path);
    esp_err_t write_file(const char* relative_path, const void* data, size_t size, bool append);
    esp_err_t read_file(const char* relative_path, void* buffer, size_t max_size, size_t* bytes_read);
    esp_err_t open_file(const char* relative_path, const char* mode, FILE*& fd);
    esp_err_t get_file_stat(const char* relative_path, struct stat* st);
    esp_err_t close_file(FILE* fd);
    esp_err_t file_write_chunk(FILE* fd, const char* chunk, size_t len);
    size_t    file_read_chunk(FILE* fd, char* chunk, size_t max_len);
    // Builds a directory tree while retaining the caller's SD operation. The
    // output is only populated on success; filesystem failures are returned
    // to the caller instead of being represented as an empty tree.
    esp_err_t get_file_tree(const char* relative_path, int depth, cJSON*& tree);
    cJSON*    scan_directory(const char* relative_path, int depth);
    esp_err_t get_absolute_path(const char* relative_path, char* out_buf, size_t out_size);

    // Validates a decoded, slash-separated path relative to the SD mount point.
    static esp_err_t validate_relative_path(const char* path, bool allow_root = true);
    static bool is_path_under(const char* path, const char* root);

    typedef std::function<void()> Callback;

    void on_mount(Callback cb);
    void on_unmount(Callback cb);

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

    bool ensure_operation_mutex();
    esp_err_t scan_directory_locked(const char* relative_path, int depth, cJSON*& tree);

    bool       last_stable_state = false;
    gpio_num_t cd_pin            = GPIO_NUM_NC;

    sdmmc_card_t*              card         = nullptr;
    sdmmc_host_t               host         = {};
    esp_vfs_fat_mount_config_t mount_config = {};
    sdmmc_slot_config_t        slot_config  = {};
    const char*                mount_path   = nullptr;

    Callback mount_callback;
    Callback unmount_callback;
    SemaphoreHandle_t operation_mutex = nullptr;
};
