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

#include <algorithm>
#include <cstring>

#include "esp_log.h"
#include "esp_mac.h"
#include "mdns.h"
#include "nvs_flash.h"

#define HOSTNAME "can-sniffer"
#define MDNS_INSTANCE "ESP32 CAN Sniffer"

// The ERROR_CHECK Macro
#define ERROR_CHECK(x, str, action, ...)                                                                           \
    do                                                                                                             \
    {                                                                                                              \
        esp_err_t err_rc_ = (x);                                                                                   \
        if (unlikely(err_rc_ != ESP_OK))                                                                           \
        {                                                                                                          \
            ESP_LOGE(TAG, "%s(%d): " str ": %s", __FUNCTION__, __LINE__, ##__VA_ARGS__, esp_err_to_name(err_rc_)); \
            action;                                                                                                \
        }                                                                                                          \
    } while (0)

static const char* TAG = "WIFI";

ESP_EVENT_DEFINE_BASE(WIFI_INTERNAL_EVENT);

enum
{
    WIFI_INT_EVENT_FALLBACK_AP,
    WIFI_INT_EVENT_RETRY_CONNECTION
};

WIFI::WIFI()
    : m_state(State::UNINITIALIZED),
      m_netif_ap(nullptr),
      m_netif_sta(nullptr),
      m_wifi_event_handler(nullptr),
      m_ip_event_handler(nullptr),
      m_mutex(xSemaphoreCreateMutex()),
      m_retry_count(0)
{
    if (m_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create mutex!");
        m_state.store(State::ERROR, std::memory_order_release);
        return;
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ERROR_CHECK(nvs_flash_erase(), "Failed to erase NVS flash", setState(State::ERROR));
        ret = nvs_flash_init();
    }
    ERROR_CHECK(ret, "Failed to initialize NVS", setState(State::ERROR));
}

WIFI::~WIFI()
{
    deinit();
    if (m_mutex)
    {
        vSemaphoreDelete(m_mutex);
    }
}

esp_err_t WIFI::init(const Config& config)
{
    if (getState() != State::UNINITIALIZED)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    m_config = config;

    ERROR_CHECK(esp_netif_init(), "Failed to initialize netif", return ESP_ERR_INVALID_STATE);
    esp_err_t ret = esp_event_loop_create_default();
    if (ret == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "Event loop already exists (skipping creation)");
    }
    else
    {
        ERROR_CHECK(ret, "Failed to create event loop", return ESP_ERR_INVALID_STATE);
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
    ret                    = esp_wifi_init(&cfg);

    ERROR_CHECK(ret, "WiFi init failed", goto err);

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, this,
                                              &m_wifi_event_handler);

    ERROR_CHECK(ret, "Failed to register WIFI event handlers", goto err);

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, this,
                                              &m_ip_event_handler);

    ERROR_CHECK(ret, "Failed to register IP event handlers", goto err);

    ret =
        esp_event_handler_instance_register(WIFI_INTERNAL_EVENT, ESP_EVENT_ANY_ID, &internal_event_handler, this, NULL);

    ERROR_CHECK(ret, "Failed to register internal event handlers", goto err);

    setState(State::INITIALIZED);
    ESP_LOGI(TAG, "WiFi AP Manager initialized");
    esp_wifi_set_ps(WIFI_PS_NONE);
    return ESP_OK;

err:
    setState(State::ERROR);
    return ret;
}

esp_err_t WIFI::setAPConfig()
{
    wifi_config_t ap_config = {};
    strlcpy((char*)ap_config.ap.ssid, m_config.ssid.c_str(), sizeof(ap_config.ap.ssid));
    strlcpy((char*)ap_config.ap.password, m_config.password.c_str(), sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len           = m_config.ssid.length();
    ap_config.ap.channel            = m_config.channel;
    ap_config.ap.authmode           = m_config.auth_mode;
    ap_config.ap.ssid_hidden        = m_config.ssid_hidden ? 1 : 0;
    ap_config.ap.max_connection     = m_config.max_connections;
    ap_config.ap.pmf_cfg.required   = m_config.pmf_required;
    ap_config.ap.gtk_rekey_interval = m_config.gtk_rekey_interval;
    return esp_wifi_set_config(WIFI_IF_AP, &ap_config);
}

esp_err_t WIFI::setSTAConfig()
{
    wifi_config_t sta_config = {};
    strlcpy((char*)sta_config.sta.ssid, m_config.sta_ssid.c_str(), sizeof(sta_config.sta.ssid));
    strlcpy((char*)sta_config.sta.password, m_config.sta_password.c_str(), sizeof(sta_config.sta.password));
    sta_config.sta.threshold.authmode = m_config.sta_auth_mode;
    sta_config.sta.pmf_cfg.capable    = true;
    sta_config.sta.pmf_cfg.required   = false;
    return esp_wifi_set_config(WIFI_IF_STA, &sta_config);
}

esp_err_t WIFI::start()
{
    if (getState() != State::INITIALIZED && getState() != State::STA_DISCONNECTED && getState() != State::STARTING)
    {
        ESP_LOGE(TAG, "Invalid state for start: %d", (int)getState());
        return ESP_ERR_INVALID_STATE;
    }

    setState(State::STARTING);

    ERROR_CHECK(esp_wifi_set_mode(m_config.mode), "Failed to set WiFi mode", goto err);

    // Configure AP
    if (m_config.mode == WIFI_MODE_AP || m_config.mode == WIFI_MODE_APSTA)
    {
        ERROR_CHECK(setAPConfig(), "Failed to set AP config", goto err);
        ESP_LOGI(TAG, "Configured AP Mode: %s", m_config.ssid.c_str());
    }

    // Configure STA
    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        ERROR_CHECK(setSTAConfig(), "Failed to set STA config", goto err);
        ESP_LOGI(TAG, "Configured STA Mode: %s", m_config.sta_ssid.c_str());
    }

    ERROR_CHECK(esp_wifi_start(), "Failed to start WiFi", goto err);

    // Connect if STA mode is active
    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        setState(State::STA_CONNECTING);
        ERROR_CHECK(esp_wifi_connect(), "Failed to connect to WiFi", goto err);
    }

    // Only AP mode sets running immediately, STA waits for IP event
    if (m_config.mode == WIFI_MODE_AP)
    {
        setState(State::RUNNING);
        ESP_LOGI(TAG, "WiFi AP starting - SSID: %s, Channel: %d", m_config.ssid.c_str(), m_config.channel);
    }

    ERROR_CHECK(start_mdns_service(), "mDNS failed", goto err);

    return ESP_OK;
err:
    setState(State::ERROR);
    return ESP_FAIL;
}

esp_err_t WIFI::start_mdns_service()
{
    esp_err_t err = mdns_init();
    if (err == ESP_ERR_INVALID_STATE)
    {
        return ESP_OK;
    }
    if (err != ESP_OK)
        return err;

    mdns_hostname_set(HOSTNAME);
    mdns_instance_name_set(MDNS_INSTANCE);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    return ESP_OK;
}

esp_err_t WIFI::stop()
{
    if (getState() == State::UNINITIALIZED)
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
    if (getState() == State::RUNNING)
        stop();

    if (getState() != State::UNINITIALIZED)
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
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_connect_callbacks.clear();
    m_disconnect_callbacks.clear();
    m_sta_connected_callbacks.clear();
    m_sta_disconnected_callbacks.clear();
    m_clients.clear();
    xSemaphoreGive(m_mutex);
}

void WIFI::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WIFI* manager = static_cast<WIFI*>(arg);

    switch (event_id)
    {
        case WIFI_EVENT_AP_STACONNECTED:
            manager->handleClientConnected((wifi_event_ap_staconnected_t*)event_data);
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            manager->handleClientDisconnected((wifi_event_ap_stadisconnected_t*)event_data);
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
            manager->handleStaDisconnected((wifi_event_sta_disconnected_t*)event_data);
            break;
        default:
            break;
    }
}

void WIFI::ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WIFI* manager = static_cast<WIFI*>(arg);
    if (event_id == IP_EVENT_STA_GOT_IP)
    {
        manager->handleStaGotIP((ip_event_got_ip_t*)event_data);
    }
}

void WIFI::handleClientConnected(wifi_event_ap_staconnected_t* event)
{
    ESP_LOGI(TAG, "AP Client connected - MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
    addClient(event->mac, event->aid);

    auto callbacks = m_connect_callbacks;
    for (auto& cb : callbacks)
        cb(event->mac, event->aid);
}

void WIFI::handleClientDisconnected(wifi_event_ap_stadisconnected_t* event)
{
    ESP_LOGI(TAG, "AP Client disconnected - MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
    removeClient(event->mac);

    auto callbacks = m_disconnect_callbacks;
    for (auto& cb : callbacks)
        cb(event->mac, event->aid);
}

void WIFI::handleAPStart()
{
    ESP_LOGI(TAG, "AP Interface Started");
    auto callbacks = m_start_callbacks;
    for (auto& cb : callbacks)
        cb();
}

void WIFI::handleAPStop()
{
    ESP_LOGI(TAG, "AP Interface Stopped");
}

void WIFI::handleStaConnected()
{
    ESP_LOGI(TAG, "STA Connected to AP (waiting for IP...)");
}

void WIFI::handleStaGotIP(ip_event_got_ip_t* event)
{
    m_retry_count = 0;
    setState(State::RUNNING);

    char ip_str[16];
    esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "STA Got IP: %s", ip_str);

    xSemaphoreTake(m_mutex, portMAX_DELAY);
    auto callbacks = m_sta_connected_callbacks;
    xSemaphoreGive(m_mutex);

    for (auto& cb : callbacks)
        cb(std::string(ip_str));
}

void WIFI::handleStaDisconnected(wifi_event_sta_disconnected_t* event)
{
    ESP_LOGW("WIFI", "Disconnected! Reason: %s (%d)", get_wifi_reason_str(event->reason), event->reason);
    setState(State::STA_DISCONNECTED);

    for (auto& cb : m_sta_disconnected_callbacks)
        cb();

    if (m_retry_count < m_config.sta_max_retry)
    {
        m_retry_count++;

        esp_event_post(WIFI_INTERNAL_EVENT, WIFI_INT_EVENT_RETRY_CONNECTION, NULL, 0, portMAX_DELAY);
    }
    else
    {
        ESP_LOGE(TAG, "Max retries reached. Posting AP Fallback event...");

        esp_event_post(WIFI_INTERNAL_EVENT, WIFI_INT_EVENT_FALLBACK_AP, NULL, 0, portMAX_DELAY);
    }
}

void WIFI::internal_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WIFI* manager = static_cast<WIFI*>(arg);

    if (event_id == WIFI_INT_EVENT_FALLBACK_AP)
    {
        ESP_LOGI(TAG, "Internal Event: Executing AP Fallback");
        manager->switchToAP();
    }
    else if (event_id == WIFI_INT_EVENT_RETRY_CONNECTION)
    {
        ESP_LOGI(TAG, "Internal Event: Retrying STA Connection");
        esp_wifi_connect();
    }
}

esp_err_t WIFI::switchToAP()
{
    ESP_LOGI(TAG, "Switching to AP Mode...");

    esp_wifi_stop();

    if (m_netif_ap == NULL)
    {
        m_netif_ap = esp_netif_create_default_wifi_ap();
    }

    m_config.mode = WIFI_MODE_AP;

    ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP), "Failed to set WiFi mode", goto err);

    ERROR_CHECK(setAPConfig(), "Failed to set AP config", goto err);

    setState(State::INITIALIZED);
    ERROR_CHECK(start(), "Failed to start AP", goto err);

    return ESP_OK;
err:
    setState(State::ERROR);
    return ESP_FAIL;
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
void WIFI::onStaConnected(StaConnectedCallback callback)
{
    m_sta_connected_callbacks.push_back(callback);
}
void WIFI::onStaDisconnected(StaDisconnectedCallback callback)
{
    m_sta_disconnected_callbacks.push_back(callback);
}

uint8_t WIFI::getConnectedClientCount() const
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Mutex timeout");
        return 0;
    }
    uint8_t count = m_clients.size();
    xSemaphoreGive(m_mutex);
    return count;
}

std::vector<WIFI::ClientInfo> WIFI::getConnectedClients() const
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Mutex timeout");
        return {};
    }
    std::vector<ClientInfo> clients = m_clients;
    xSemaphoreGive(m_mutex);
    return clients;
}

void WIFI::addClient(const uint8_t* mac, uint8_t aid)
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Mutex timeout");
        return;
    }
    ClientInfo info;
    memcpy(info.mac, mac, 6);
    info.aid          = aid;
    info.connect_time = esp_log_timestamp();
    m_clients.push_back(info);
    xSemaphoreGive(m_mutex);
}

void WIFI::removeClient(const uint8_t* mac)
{
    if (xSemaphoreTake(m_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Mutex timeout");
        return;
    }
    m_clients.erase(std::remove_if(m_clients.begin(), m_clients.end(),
                                   [mac](const ClientInfo& info) { return memcmp(info.mac, mac, 6) == 0; }),
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
    m_state.store(state);
}

const char* WIFI::get_wifi_reason_str(uint8_t reason)
{
    switch (reason)
    {
        case WIFI_REASON_UNSPECIFIED:
            return "UNSPECIFIED";
        case WIFI_REASON_AUTH_EXPIRE:
            return "AUTH_EXPIRE";
        case WIFI_REASON_AUTH_LEAVE:
            return "AUTH_LEAVE";
        case WIFI_REASON_ASSOC_EXPIRE:
            return "ASSOC_EXPIRE";
        case WIFI_REASON_ASSOC_TOOMANY:
            return "ASSOC_TOOMANY";
        case WIFI_REASON_NOT_AUTHED:
            return "NOT_AUTHED";
        case WIFI_REASON_NOT_ASSOCED:
            return "NOT_ASSOCED";
        case WIFI_REASON_ASSOC_LEAVE:
            return "ASSOC_LEAVE";
        case WIFI_REASON_ASSOC_NOT_AUTHED:
            return "ASSOC_NOT_AUTHED";
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
            return "4WAY_HANDSHAKE_TIMEOUT";
        case WIFI_REASON_BEACON_TIMEOUT:
            return "BEACON_TIMEOUT";
        case WIFI_REASON_NO_AP_FOUND:
            return "NO_AP_FOUND";
        case WIFI_REASON_AUTH_FAIL:
            return "AUTH_FAIL";
        case WIFI_REASON_ASSOC_FAIL:
            return "ASSOC_FAIL";
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
            return "HANDSHAKE_TIMEOUT";
        case WIFI_REASON_CONNECTION_FAIL:
            return "CONNECTION_FAIL";
        default:
            return "UNKNOWN_REASON";
    }
}