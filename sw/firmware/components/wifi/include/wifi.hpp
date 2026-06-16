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

#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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
    void      deinit();

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
    std::string getSSID() const
    {
        return m_config.ssid;
    }

    // Client management
    uint8_t                 getConnectedClientCount() const;
    std::vector<ClientInfo> getConnectedClients() const;
    esp_err_t               disconnectClient(const uint8_t* mac);

    // Event callbacks
    void onClientConnected(ClientConnectCallback callback);
    void onClientDisconnected(ClientDisconnectCallback callback);
    void onAPStarted(APStartCallback callback);
    void onStaConnected(StaConnectedCallback callback);
    void onStaDisconnected(StaDisconnectedCallback callback);

private:
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

    // Helper methods
    void        addClient(const uint8_t* mac, uint8_t aid);
    void        removeClient(const uint8_t* mac);
    ClientInfo* findClient(const uint8_t* mac);
    const char* get_wifi_reason_str(uint8_t reason);
};
