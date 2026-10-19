/**
 * @file wifi.hpp
 * @author bladecell (github@bum.anonaddy.com)
 * @brief
 * @version 0.1
 * @date 2025-12-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "wifi_scan_policy.hpp"

class WIFI
{
public:
    struct Config
    {
        // AP settings
        std::string      ssid;
        std::string      password;
        uint8_t          channel            = 6;
        uint8_t          max_connections    = 4;
        wifi_auth_mode_t auth_mode          = WIFI_AUTH_WPA2_PSK;
        bool             ssid_hidden        = false;
        bool             pmf_required       = true;
        uint32_t         gtk_rekey_interval = 86400;

        // STA settings
        std::string      sta_ssid      = "";
        std::string      sta_password  = "";
        wifi_auth_mode_t sta_auth_mode = WIFI_AUTH_WPA2_PSK;

        // Mode Selection
        wifi_mode_t mode          = WIFI_MODE_AP;  // WIFI_MODE_AP, WIFI_MODE_STA, or WIFI_MODE_APSTA
        uint8_t     sta_max_retry = 5;

        // mDNS (empty hostname or instance keeps the compile-time default)
        std::string mdns_hostname;
        std::string mdns_instance;
        uint16_t    mdns_http_port = 80;
    };

    enum class State
    {
        UNINITIALIZED,
        INITIALIZED,
        STARTING,
        RUNNING,
        STOPPING,
        ERROR,
        STA_CONNECTING,
        STA_DISCONNECTED
    };

    struct ClientInfo
    {
        uint8_t  mac[6];
        uint8_t  aid;
        uint32_t connect_time;
    };

    using ClientConnectCallback    = std::function<void(const uint8_t* mac, uint8_t aid)>;
    using ClientDisconnectCallback = std::function<void(const uint8_t* mac, uint8_t aid)>;
    using APStartCallback          = std::function<void()>;
    using StaConnectedCallback     = std::function<void(std::string ip)>;
    using StaDisconnectedCallback  = std::function<void()>;

    WIFI();
    ~WIFI();

    static WIFI& getInstance()
    {
        static WIFI instance;
        return instance;
    }

    // Lifecycle
    esp_err_t init(const Config& config);
    esp_err_t start();
    esp_err_t stop();
    esp_err_t switchToAP();
    esp_err_t deinit();

    struct ScanNetwork
    {
        char ssid[33] = {};
        char ssidHex[65] = {};
        int8_t rssi = 0;
        uint8_t channel = 0;
        wifi_auth_mode_t authMode = WIFI_AUTH_OPEN;
    };
    enum class ScanState { Idle, Scanning, Complete, Failed };
    struct ScanSnapshot
    {
        ScanState state = ScanState::Idle;
        uint32_t id = 0;
        esp_err_t error = ESP_OK;
        int64_t completedUs = 0;
        uint16_t count = 0;
        bool truncated = false;
        ScanNetwork networks[wifi_scan::kMaxResults] = {};
    };
    enum class ScanStartResult { Started, Busy, Down, Cooldown, NoMemory };
    ScanStartResult startScan(uint32_t& id);
    esp_err_t getScan(ScanSnapshot& snapshot);

    // Status
    WIFI::State getState() const
    {
        return m_state.load();
    }
    bool isRunning() const
    {
        return m_state == State::RUNNING;
    }

    std::string getApIP() const;
    std::string getStaIP() const;
    // True when this device is serving an access point with no password (mode AP or AP+STA, auth OPEN). Read from the
    // applied config, so the AP fallback after a failed STA connect counts as well.
    bool isApOpen() const
    {
        return (m_config.mode == WIFI_MODE_AP || m_config.mode == WIFI_MODE_APSTA) && m_config.auth_mode == WIFI_AUTH_OPEN;
    }
    std::string getSSID() const
    {
        return m_config.ssid;
    }

    // Client management
    uint8_t                 getConnectedClientCount() const;
    std::vector<ClientInfo> getConnectedClients() const;
    esp_err_t               disconnectClient(const uint8_t* mac);

    // Event callbacks
    void onClientConnected(const ClientConnectCallback& callback);
    void onClientDisconnected(const ClientDisconnectCallback& callback);
    void onAPStarted(const APStartCallback& callback);
    void onStaConnected(const StaConnectedCallback& callback);
    void onStaDisconnected(const StaDisconnectedCallback& callback);

private:
    struct LifecycleGuard
    {
        SemaphoreHandle_t mutex;
        bool locked;
        explicit LifecycleGuard(SemaphoreHandle_t handle, TickType_t wait = pdMS_TO_TICKS(1000))
            : mutex(handle), locked(handle && xSemaphoreTakeRecursive(handle, wait) == pdTRUE) {}
        ~LifecycleGuard() { if (locked) { xSemaphoreGiveRecursive(mutex); } }
    };
    static void scanTask(void* arg);
    void runScan();
    void scanDone(const wifi_event_sta_scan_done_t* event);
    esp_err_t cancelScan();
    void resumeConnection();
    void deferConnection();
    static void reconnectTimer(void* arg);
    esp_timer_handle_t m_reconnectTimer = nullptr;
    SemaphoreHandle_t m_lifecycleMutex = nullptr;
    SemaphoreHandle_t m_shutdownMutex = nullptr;
    SemaphoreHandle_t m_scanExited = nullptr;
    SemaphoreHandle_t m_scanSignal = nullptr;
    ScanSnapshot m_scan;
    int64_t m_lastScanStartUs = -1;
    bool m_scanActive = false;
    bool m_scanCancel = false;
    std::atomic<bool> m_scanDone{false};
    wifi_scan::CompletionGate m_scanEvents;
    std::atomic<bool> m_scanFault{false};
    bool m_scanTemporarySta = false;
    std::atomic<uint32_t> m_scanStatus{0};
    int64_t m_scanDoneUs = 0;  // Published by the event handler before m_scanDone's release store.
    std::atomic<bool> m_closing{true};
    std::atomic<bool> m_deinitializing{false};
    std::atomic<bool> m_reconnectPending{false};
    WIFI(const WIFI&)            = delete;
    WIFI& operator=(const WIFI&) = delete;
    // Event handler
    static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    // Internal handlers
    void        handleClientConnected(wifi_event_ap_staconnected_t* event);
    void        handleClientDisconnected(wifi_event_ap_stadisconnected_t* event);
    void        handleAPStart();
    void        handleAPStop();
    void        handleStaConnected();
    void        handleStaDisconnected(wifi_event_sta_disconnected_t* event);
    void        handleStaGotIP(ip_event_got_ip_t* event);
    static void internal_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

    esp_err_t start_mdns_service();
    void      stop_mdns_service();
    bool      m_mdns_up = false;  // mdns_init() done and not yet freed; cleared by deinit() so wake re-binds the new netifs

    // State management
    void setState(State state);

    // Configuration
    Config m_config;

    esp_err_t setAPConfig();
    esp_err_t setSTAConfig();

    // State
    std::atomic<State> m_state;

    // Network interface
    esp_netif_t* m_netif_ap;
    esp_netif_t* m_netif_sta;

    // Event handlers
    esp_event_handler_instance_t m_wifi_event_handler;
    esp_event_handler_instance_t m_ip_event_handler;
    esp_event_handler_instance_t m_internal_event_handler;

    // Callbacks
    std::vector<ClientConnectCallback>    m_connect_callbacks;
    std::vector<ClientDisconnectCallback> m_disconnect_callbacks;
    std::vector<APStartCallback>          m_start_callbacks;
    std::vector<StaConnectedCallback>     m_sta_connected_callbacks;
    std::vector<StaDisconnectedCallback>  m_sta_disconnected_callbacks;

    // Client tracking
    std::vector<ClientInfo> m_clients;

    // Synchronization
    mutable SemaphoreHandle_t m_mutex;

    uint8_t m_retry_count = 0;

    // Callback vectors are touched by the event task and by on*()/deinit(): copy or push under m_mutex only (WIFI-2).
    template <typename V>
    V snapshotCallbacks(const V& v) const
    {
        if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) { return V{}; }
        V copy = v;
        xSemaphoreGive(m_mutex);
        return copy;
    }
    template <typename V, typename F>
    void pushCallback(V& v, const F& cb)
    {
        if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) { return; }
        v.push_back(cb);
        xSemaphoreGive(m_mutex);
    }

    // Helper methods
    void        addClient(const uint8_t* mac, uint8_t aid);
    void        removeClient(const uint8_t* mac);
    ClientInfo* findClient(const uint8_t* mac);
    const char* get_wifi_reason_str(uint8_t reason);
};
