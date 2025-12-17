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

#include <vector>
#include <algorithm>
#include <functional>
#include <cstring>
#include <string>

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

class WIFI
{
public:
    using ClientConnectCallback = std::function<void(const uint8_t *mac, uint8_t aid)>;
    using ClientDisconnectCallback = std::function<void(const uint8_t *mac, uint8_t aid)>;
    using APStartCallback = std::function<void()>;

    struct Config
    {
        std::string ssid;
        std::string password;
        uint8_t channel = 6;
        uint8_t max_connections = 4;
        wifi_auth_mode_t auth_mode = WIFI_AUTH_WPA2_PSK;
        bool ssid_hidden = false;
        bool pmf_required = true;
        uint32_t gtk_rekey_interval = 86400;
    };

    enum class State
    {
        UNINITIALIZED,
        INITIALIZED,
        STARTING,
        RUNNING,
        STOPPING,
        ERROR
    };

    struct ClientInfo
    {
        uint8_t mac[6];
        uint8_t aid;
        uint32_t connect_time;
    };

    WIFI();
    ~WIFI();

    static WIFI &getInstance()
    {
        static WIFI instance;
        return instance;
    }

    // Lifecycle
    esp_err_t init(const Config &config);
    esp_err_t start();
    esp_err_t stop();
    void deinit();

    // Status
    State getState() const { return m_state; }
    bool isRunning() const { return m_state == State::RUNNING; }
    std::string getIP() const;
    std::string getSSID() const { return m_config.ssid; }

    // Client management
    uint8_t getConnectedClientCount() const;
    std::vector<ClientInfo> getConnectedClients() const;
    esp_err_t disconnectClient(const uint8_t *mac);

    // Event callbacks
    void onClientConnected(ClientConnectCallback callback);
    void onClientDisconnected(ClientDisconnectCallback callback);
    void onAPStarted(APStartCallback callback);

private:
    WIFI(const WIFI &) = delete;
    WIFI &operator=(const WIFI &) = delete;
    // Event handler
    static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data);

    // Internal handlers
    void handleClientConnected(wifi_event_ap_staconnected_t *event);
    void handleClientDisconnected(wifi_event_ap_stadisconnected_t *event);
    void handleAPStart();
    void handleAPStop();

    // State management
    void setState(State state);

    // Configuration
    Config m_config;

    // State
    State m_state;

    // Network interface
    esp_netif_t *m_netif;

    // Event handlers
    esp_event_handler_instance_t m_wifi_event_handler;

    // Callbacks
    std::vector<ClientConnectCallback> m_connect_callbacks;
    std::vector<ClientDisconnectCallback> m_disconnect_callbacks;
    std::vector<APStartCallback> m_start_callbacks;

    // Client tracking
    std::vector<ClientInfo> m_clients;

    // Synchronization
    mutable SemaphoreHandle_t m_mutex;

    // Helper methods
    void addClient(const uint8_t *mac, uint8_t aid);
    void removeClient(const uint8_t *mac);
    ClientInfo *findClient(const uint8_t *mac);
};
