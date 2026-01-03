/**
 * @file wifi.cpp
 * @author bladecell (github@bum.anonaddy.com)
 * @brief
 * @version 0.1
 * @date 2025-12-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "wifi.hpp"

static const char *TAG = "WIFI";

WIFI::WIFI()
    : m_state(State::UNINITIALIZED),
      m_netif_ap(nullptr),
      m_netif_sta(nullptr),
      m_wifi_event_handler(nullptr),
      m_ip_event_handler(nullptr),
      m_mutex(xSemaphoreCreateMutex())
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

WIFI::~WIFI()
{
    deinit();
    if (m_mutex)
    {
        vSemaphoreDelete(m_mutex);
    }
}

esp_err_t WIFI::init(const Config &config)
{
    if (m_state != State::UNINITIALIZED)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    m_config = config;

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "Event loop already exists (skipping creation)");
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }

    if (m_config.mode == WIFI_MODE_AP || m_config.mode == WIFI_MODE_APSTA)
    {
        m_netif_ap = esp_netif_create_default_wifi_ap();
    }

    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        m_netif_sta = esp_netif_create_default_wifi_sta();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        setState(State::ERROR);
        return ret;
    }

    ret = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &wifi_event_handler,
                                              this,
                                              &m_wifi_event_handler);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Event handler registration failed");
        setState(State::ERROR);
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &ip_event_handler,
                                              this,
                                              &m_ip_event_handler);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Event handler registration failed");
        setState(State::ERROR);
        return ret;
    }

    setState(State::INITIALIZED);
    ESP_LOGI(TAG, "WiFi AP Manager initialized");
    return ESP_OK;
}

esp_err_t WIFI::start()
{
    if (m_state != State::INITIALIZED)
    {
        ESP_LOGW(TAG, "Invalid state for start");
        return ESP_ERR_INVALID_STATE;
    }

    setState(State::STARTING);

    ESP_ERROR_CHECK(esp_wifi_set_mode(m_config.mode));

    // Configure AP
    if (m_config.mode == WIFI_MODE_AP || m_config.mode == WIFI_MODE_APSTA)
    {
        wifi_config_t ap_config = {};
        strncpy((char *)ap_config.ap.ssid, m_config.ssid.c_str(), sizeof(ap_config.ap.ssid));
        strncpy((char *)ap_config.ap.password, m_config.password.c_str(), sizeof(ap_config.ap.password));
        ap_config.ap.ssid_len = m_config.ssid.length();
        ap_config.ap.channel = m_config.channel;
        ap_config.ap.authmode = m_config.auth_mode;
        ap_config.ap.ssid_hidden = m_config.ssid_hidden ? 1 : 0;
        ap_config.ap.max_connection = m_config.max_connections;
        ap_config.ap.pmf_cfg.required = m_config.pmf_required;
        ap_config.ap.gtk_rekey_interval = m_config.gtk_rekey_interval;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
        ESP_LOGI(TAG, "Configured AP Mode: %s", m_config.ssid.c_str());
    }

    // Configure STA
    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        wifi_config_t sta_config = {};
        strncpy((char *)sta_config.sta.ssid, m_config.sta_ssid.c_str(), sizeof(sta_config.sta.ssid));
        strncpy((char *)sta_config.sta.password, m_config.sta_password.c_str(), sizeof(sta_config.sta.password));
        sta_config.sta.threshold.authmode = m_config.sta_auth_mode;

        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_LOGI(TAG, "Configured STA Mode: %s", m_config.sta_ssid.c_str());
    }

    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi start failed");
        setState(State::ERROR);
        return ret;
    }

    // Connect if STA mode is active
    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        setState(State::STA_CONNECTING);
        esp_wifi_connect();
    }

    // Only AP mode sets running immediately, STA waits for IP event
    if (m_config.mode == WIFI_MODE_AP)
    {
        setState(State::RUNNING);
        ESP_LOGI(TAG, "WiFi AP starting - SSID: %s, Channel: %d",
                 m_config.ssid.c_str(), m_config.channel);
    }

    ret = start_mdns_service();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mDNS failed: %s", esp_err_to_name(ret));
    }

    return ESP_OK;
}

esp_err_t WIFI::start_mdns_service()
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK)
        return err;

    mdns_hostname_set(HOSTNAME);
    mdns_instance_name_set(MDNS_INSTANCE);
    return err;
}

esp_err_t WIFI::stop()
{
    if (m_state == State::UNINITIALIZED)
        return ESP_ERR_INVALID_STATE;

    setState(State::STOPPING);

    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        esp_wifi_disconnect();
    }

    esp_err_t ret = esp_wifi_stop();
    setState(State::INITIALIZED);
    return ret;
}

void WIFI::deinit()
{
    if (m_state == State::RUNNING)
        stop();

    if (m_state != State::UNINITIALIZED)
    {
        if (m_wifi_event_handler)
        {
            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, m_wifi_event_handler);
        }
        if (m_ip_event_handler)
        {
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, m_ip_event_handler);
        }

        esp_wifi_deinit();
        if (m_netif_ap)
        {
            esp_netif_destroy(m_netif_ap);
            m_netif_ap = nullptr;
        }
        if (m_netif_sta)
        {
            esp_netif_destroy(m_netif_sta);
            m_netif_sta = nullptr;
        }

        setState(State::UNINITIALIZED);
    }
}

void WIFI::wifi_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    WIFI *manager = static_cast<WIFI *>(arg);

    switch (event_id)
    {
    case WIFI_EVENT_AP_STACONNECTED:
        manager->handleClientConnected((wifi_event_ap_staconnected_t *)event_data);
        break;
    case WIFI_EVENT_AP_STADISCONNECTED:
        manager->handleClientDisconnected((wifi_event_ap_stadisconnected_t *)event_data);
        break;
    case WIFI_EVENT_AP_START:
        manager->handleAPStart();
        break;
    case WIFI_EVENT_AP_STOP:
        manager->handleAPStop();
        break;

    // --- STA Events ---
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "STA Started");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        manager->handleStaConnected();
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        manager->handleStaDisconnected();
        break;
    default:
        break;
    }
}

void WIFI::ip_event_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    WIFI *manager = static_cast<WIFI *>(arg);
    if (event_id == IP_EVENT_STA_GOT_IP)
    {
        manager->handleStaGotIP((ip_event_got_ip_t *)event_data);
    }
}

void WIFI::handleClientConnected(wifi_event_ap_staconnected_t *event)
{
    ESP_LOGI(TAG, "AP Client connected - MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
    addClient(event->mac, event->aid);
    for (auto &callback : m_connect_callbacks)
        callback(event->mac, event->aid);
}

void WIFI::handleClientDisconnected(wifi_event_ap_stadisconnected_t *event)
{
    ESP_LOGI(TAG, "AP Client disconnected - MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
    removeClient(event->mac);
    for (auto &callback : m_disconnect_callbacks)
        callback(event->mac, event->aid);
}

void WIFI::handleAPStart()
{
    ESP_LOGI(TAG, "AP Interface Started");
    for (auto &callback : m_start_callbacks)
        callback();
}

void WIFI::handleAPStop()
{
    ESP_LOGI(TAG, "AP Interface Stopped");
}

void WIFI::handleStaConnected()
{
    ESP_LOGI(TAG, "STA Connected to AP (waiting for IP...)");
    // We don't consider this "Running" yet, we wait for IP_EVENT
}

void WIFI::handleStaGotIP(ip_event_got_ip_t *event)
{
    m_retry_count = 0; // Reset retry count on success
    setState(State::RUNNING);

    char ip_str[16];
    esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "STA Got IP: %s", ip_str);

    for (auto &cb : m_sta_connected_callbacks)
    {
        cb(std::string(ip_str));
    }
}

void WIFI::handleStaDisconnected()
{
    ESP_LOGI(TAG, "STA Disconnected");
    setState(State::STA_DISCONNECTED);

    for (auto &cb : m_sta_disconnected_callbacks)
    {
        cb();
    }

    // Auto-retry logic
    if (m_retry_count < m_config.sta_max_retry)
    {
        m_retry_count++;
        ESP_LOGI(TAG, "Retrying connection to AP (%d/%d)...", m_retry_count, m_config.sta_max_retry);
        setState(State::STA_CONNECTING);
        esp_wifi_connect();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to connect to AP after max retries, falling back to AP mode");
        if (m_config.mode == WIFI_MODE_APSTA)
        {
            setState(State::RUNNING);
        }
        else
        {
            switchToAP();
        }
    }
}

esp_err_t WIFI::switchToAP()
{
    ESP_LOGI(TAG, "Switching to AP Mode...");

    if (m_state == State::RUNNING || m_state == State::STA_CONNECTING)
    {
        esp_wifi_stop();
    }

    if (m_netif_ap == NULL)
    {
        m_netif_ap = esp_netif_create_default_wifi_ap();
    }

    m_config.mode = WIFI_MODE_AP;
    if (esp_err_t ret = esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    wifi_config_t ap_config = {};
    strncpy((char *)ap_config.ap.ssid, m_config.ssid.c_str(), sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = m_config.ssid.length();
    ap_config.ap.channel = m_config.channel;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;
    ap_config.ap.max_connection = m_config.max_connections;

    if (esp_err_t ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_err_t ret = start();

    if (ret == ESP_OK)
    {
        setState(State::RUNNING);
    }

    return ret;
}

void WIFI::onClientConnected(ClientConnectCallback callback) { m_connect_callbacks.push_back(callback); }
void WIFI::onClientDisconnected(ClientDisconnectCallback callback) { m_disconnect_callbacks.push_back(callback); }
void WIFI::onAPStarted(APStartCallback callback) { m_start_callbacks.push_back(callback); }
void WIFI::onStaConnected(StaConnectedCallback callback) { m_sta_connected_callbacks.push_back(callback); }
void WIFI::onStaDisconnected(StaDisconnectedCallback callback) { m_sta_disconnected_callbacks.push_back(callback); }

uint8_t WIFI::getConnectedClientCount() const
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    uint8_t count = m_clients.size();
    xSemaphoreGive(m_mutex);
    return count;
}

std::vector<WIFI::ClientInfo> WIFI::getConnectedClients() const
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    std::vector<ClientInfo> clients = m_clients;
    xSemaphoreGive(m_mutex);
    return clients;
}

void WIFI::addClient(const uint8_t *mac, uint8_t aid)
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    ClientInfo info;
    memcpy(info.mac, mac, 6);
    info.aid = aid;
    info.connect_time = esp_log_timestamp();
    m_clients.push_back(info);
    xSemaphoreGive(m_mutex);
}

void WIFI::removeClient(const uint8_t *mac)
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_clients.erase(
        std::remove_if(m_clients.begin(), m_clients.end(),
                       [mac](const ClientInfo &info)
                       {
                           return memcmp(info.mac, mac, 6) == 0;
                       }),
        m_clients.end());
    xSemaphoreGive(m_mutex);
}

std::string WIFI::getApIP() const
{
    if (m_netif_ap)
    {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(m_netif_ap, &ip_info);
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        return std::string(ip_str);
    }
    return "";
}

std::string WIFI::getStaIP() const
{
    if (m_netif_sta)
    {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(m_netif_sta, &ip_info);
        char ip_str[16];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        return std::string(ip_str);
    }
    return "";
}

void WIFI::setState(State state)
{
    m_state = state;
}