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
    : m_state(State::UNINITIALIZED), m_netif(nullptr), m_wifi_event_handler(nullptr), m_mutex(xSemaphoreCreateMutex())
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
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    m_netif = esp_netif_create_default_wifi_ap();

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

    // Configure WiFi
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, m_config.ssid.c_str(), sizeof(wifi_config.ap.ssid));
    strncpy((char *)wifi_config.ap.password, m_config.password.c_str(), sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = m_config.ssid.length();
    wifi_config.ap.channel = m_config.channel;
    wifi_config.ap.authmode = m_config.auth_mode;
    wifi_config.ap.ssid_hidden = m_config.ssid_hidden ? 1 : 0;
    wifi_config.ap.max_connection = m_config.max_connections;
    wifi_config.ap.pmf_cfg.required = m_config.pmf_required;
    wifi_config.ap.gtk_rekey_interval = m_config.gtk_rekey_interval;
    wifi_config.ap.transition_disable = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    esp_err_t ret = esp_wifi_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi start failed");
        setState(State::ERROR);
        return ret;
    }

    ESP_LOGI(TAG, "WiFi AP starting - SSID: %s, Channel: %d",
             m_config.ssid.c_str(), m_config.channel);

    return ESP_OK;
}

esp_err_t WIFI::stop()
{
    if (m_state != State::RUNNING)
    {
        return ESP_ERR_INVALID_STATE;
    }

    setState(State::STOPPING);
    esp_err_t ret = esp_wifi_stop();
    setState(State::INITIALIZED);
    return ret;
}

void WIFI::deinit()
{
    if (m_state == State::RUNNING)
    {
        stop();
    }

    if (m_state != State::UNINITIALIZED)
    {
        if (m_wifi_event_handler)
        {
            esp_event_handler_instance_unregister(WIFI_EVENT,
                                                  ESP_EVENT_ANY_ID,
                                                  m_wifi_event_handler);
        }

        esp_wifi_deinit();

        if (m_netif)
        {
            esp_netif_destroy(m_netif);
            m_netif = nullptr;
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

    default:
        break;
    }
}

void WIFI::handleClientConnected(wifi_event_ap_staconnected_t *event)
{
    ESP_LOGI(TAG, "Client connected - MAC: " MACSTR ", AID: %d",
             MAC2STR(event->mac), event->aid);

    addClient(event->mac, event->aid);

    // Notify callbacks
    for (auto &callback : m_connect_callbacks)
    {
        callback(event->mac, event->aid);
    }
}

void WIFI::handleClientDisconnected(wifi_event_ap_stadisconnected_t *event)
{
    ESP_LOGI(TAG, "Client disconnected - MAC: " MACSTR ", AID: %d",
             MAC2STR(event->mac), event->aid);

    removeClient(event->mac);

    // Notify callbacks
    for (auto &callback : m_disconnect_callbacks)
    {
        callback(event->mac, event->aid);
    }
}

void WIFI::handleAPStart()
{
    setState(State::RUNNING);
    ESP_LOGI(TAG, "AP Started");

    for (auto &callback : m_start_callbacks)
    {
        callback();
    }
}

void WIFI::handleAPStop()
{
    ESP_LOGI(TAG, "AP Stopped");
}

void WIFI::onClientConnected(ClientConnectCallback callback)
{
    m_connect_callbacks.push_back(callback);
}

void WIFI::onClientDisconnected(ClientDisconnectCallback callback)
{
    m_disconnect_callbacks.push_back(callback);
}

void WIFI::onAPStarted(APStartCallback callback)
{
    m_start_callbacks.push_back(callback);
}

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

std::string WIFI::getIP() const
{
    if (m_netif)
    {
        esp_netif_ip_info_t ip_info;
        esp_netif_get_ip_info(m_netif, &ip_info);
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