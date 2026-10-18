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
#include "esp_wifi_default.h"
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
    WIFI_INT_EVENT_RESUME,
    WIFI_INT_EVENT_DRAIN
};

WIFI::WIFI()
    : m_state(State::UNINITIALIZED),
      m_netif_ap(nullptr),
      m_netif_sta(nullptr),
      m_wifi_event_handler(nullptr),
      m_ip_event_handler(nullptr),
      m_internal_event_handler(nullptr),
      m_mutex(xSemaphoreCreateMutex()),
      m_retry_count(0)
{
    m_lifecycleMutex = xSemaphoreCreateRecursiveMutex();
    m_shutdownMutex = xSemaphoreCreateRecursiveMutex();
    m_scanExited = xSemaphoreCreateBinary();
    m_scanSignal = xSemaphoreCreateBinary();
    if (!m_mutex || !m_lifecycleMutex || !m_shutdownMutex || !m_scanExited || !m_scanSignal)
    {
        ESP_LOGE(TAG, "Failed to create mutex!");
        m_state.store(State::ERROR, std::memory_order_release);
        return;
    }

    // Settings owns NVS and its recovery (erase on a bad partition). This call only covers a WIFI built before Settings
    // ran: it is a no-op when NVS is up, and it must never erase settings, credentials or the auth PIN (WIFI-4).
    ERROR_CHECK(nvs_flash_init(), "Failed to initialize NVS", setState(State::ERROR));
}

WIFI::~WIFI()
{
    if (deinit() != ESP_OK) { return; }
    if (m_scanExited) { vSemaphoreDelete(m_scanExited); }
    if (m_scanSignal) { vSemaphoreDelete(m_scanSignal); }
    if (m_shutdownMutex) { vSemaphoreDelete(m_shutdownMutex); }
    if (m_lifecycleMutex) { vSemaphoreDelete(m_lifecycleMutex); }
    if (m_mutex)
    {
        vSemaphoreDelete(m_mutex);
    }
}

esp_err_t WIFI::init(const Config& config)
{
    LifecycleGuard guard(m_lifecycleMutex);
    if (!guard.locked || m_deinitializing.load()) { return ESP_ERR_INVALID_STATE; }
    if (getState() != State::UNINITIALIZED)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    m_config = config;
    m_scan = {};
    m_scanFault = false;
    m_reconnectPending = false;

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

    {
        esp_timer_create_args_t timer = {};
        timer.callback = reconnectTimer;
        timer.arg = this;
        timer.name = "wifi_retry";
        ret = esp_timer_create(&timer, &m_reconnectTimer);
        ERROR_CHECK(ret, "Failed to create reconnect timer", goto err);
    }

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, this,
                                              &m_wifi_event_handler);

    ERROR_CHECK(ret, "Failed to register WIFI event handlers", goto err);

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, this,
                                              &m_ip_event_handler);

    ERROR_CHECK(ret, "Failed to register IP event handlers", goto err);

    ret =
        esp_event_handler_instance_register(WIFI_INTERNAL_EVENT, ESP_EVENT_ANY_ID, &internal_event_handler, this, &m_internal_event_handler);

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
    ap_config.ap.ssid_len           = static_cast<uint8_t>(std::min(m_config.ssid.length(), sizeof(ap_config.ap.ssid) - 1));
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
    LifecycleGuard guard(m_lifecycleMutex);
    if (!guard.locked || m_deinitializing.load()) { return ESP_ERR_INVALID_STATE; }
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
    m_closing = false;

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
    if (m_mdns_up)
    {
        return ESP_OK;
    }

    esp_err_t err = mdns_init();
    if (err == ESP_ERR_INVALID_STATE)
    {
        m_mdns_up = true;
        return ESP_OK;
    }
    if (err != ESP_OK)
    {
        return err;
    }

    mdns_hostname_set(HOSTNAME);
    mdns_instance_name_set(MDNS_INSTANCE);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    m_mdns_up = true;
    return ESP_OK;
}

void WIFI::stop_mdns_service()
{
    if (m_mdns_up)
    {
        mdns_free();
        m_mdns_up = false;
    }
}

esp_err_t WIFI::stop()
{
    LifecycleGuard shutdown(m_shutdownMutex);
    if (!shutdown.locked) { return ESP_ERR_TIMEOUT; }
    esp_err_t cancel = cancelScan();
    if (cancel != ESP_OK) { return cancel; }
    LifecycleGuard guard(m_lifecycleMutex);
    if (!guard.locked) { return ESP_ERR_TIMEOUT; }
    if (getState() == State::UNINITIALIZED)
    {
        return ESP_ERR_INVALID_STATE;
    }

    setState(State::STOPPING);

    if (m_config.mode == WIFI_MODE_STA || m_config.mode == WIFI_MODE_APSTA)
    {
        esp_wifi_disconnect();
    }

    esp_err_t ret = esp_wifi_stop();
    if (ret == ESP_ERR_WIFI_NOT_INIT || ret == ESP_ERR_WIFI_NOT_STARTED) { ret = ESP_OK; }
    if (ret == ESP_OK && m_scanTemporarySta)
    {
        ret = esp_wifi_set_mode(m_config.mode);
        if (ret == ESP_OK) { m_scanTemporarySta = false; }
    }
    if (ret == ESP_OK) { setState(State::INITIALIZED); }
    return ret;
}

esp_err_t WIFI::deinit()
{
    if (!m_mutex) { return ESP_ERR_NO_MEM; }
    LifecycleGuard shutdown(m_shutdownMutex);
    if (!shutdown.locked) { return ESP_ERR_TIMEOUT; }
    m_deinitializing = true;
    esp_err_t cancel = cancelScan();
    if (cancel != ESP_OK) { return cancel; }
    // mDNS is bound to the netifs destroyed below; leaving it up would keep it on dead netifs after the next init (WIFI-1).
    stop_mdns_service();

    if (getState() != State::UNINITIALIZED && getState() != State::INITIALIZED)
    {
        esp_err_t ret = stop();
        if (ret != ESP_OK) { return ret; }
    }

    if (getState() != State::UNINITIALIZED)
    {
        if (m_wifi_event_handler)
        {
            esp_err_t ret = esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, m_wifi_event_handler);
            if (ret != ESP_OK) { return ret; }
            m_wifi_event_handler = nullptr;
        }
        if (m_ip_event_handler)
        {
            esp_err_t ret = esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, m_ip_event_handler);
            if (ret != ESP_OK) { return ret; }
            m_ip_event_handler = nullptr;
        }
        if (m_reconnectTimer)
        {
            esp_err_t ret = esp_timer_stop_blocking(m_reconnectTimer, pdMS_TO_TICKS(500));
            if (ret != ESP_OK) { return ret; }
            ret = esp_timer_delete(m_reconnectTimer);
            if (ret != ESP_OK) { return ret; }
            m_reconnectTimer = nullptr;
        }
        esp_err_t ret = esp_wifi_deinit();
        if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT) { return ret; }
        // Driver task is gone. Retire its queued events before a new registration can observe them on next init.
        if (m_internal_event_handler)
        {
            xSemaphoreTake(m_scanSignal, 0);
            ret = esp_event_post(WIFI_INTERNAL_EVENT, WIFI_INT_EVENT_DRAIN, nullptr, 0, pdMS_TO_TICKS(100));
            if (ret != ESP_OK || xSemaphoreTake(m_scanSignal, pdMS_TO_TICKS(1000)) != pdTRUE)
            {
                return ESP_ERR_TIMEOUT;
            }
            ret = esp_event_handler_instance_unregister(WIFI_INTERNAL_EVENT, ESP_EVENT_ANY_ID, m_internal_event_handler);
            if (ret != ESP_OK) { return ret; }
            m_internal_event_handler = nullptr;
        }
        m_wifi_event_handler = nullptr;
        m_ip_event_handler = nullptr;
        if (m_netif_ap)
        {
            esp_netif_destroy_default_wifi(m_netif_ap);
            m_netif_ap = nullptr;
        }
        if (m_netif_sta)
        {
            esp_netif_destroy_default_wifi(m_netif_sta);
            m_netif_sta = nullptr;
        }

        setState(State::UNINITIALIZED);
    }
    xSemaphoreTake(m_mutex, portMAX_DELAY);
    m_connect_callbacks.clear();
    m_disconnect_callbacks.clear();
    m_sta_connected_callbacks.clear();
    m_sta_disconnected_callbacks.clear();
    m_start_callbacks.clear();
    m_clients.clear();
    xSemaphoreGive(m_mutex);
    m_scan = {};
    m_reconnectPending = false;
    m_deinitializing = false;
    return ESP_OK;
}

void WIFI::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WIFI* manager = static_cast<WIFI*>(arg);

    switch (event_id)
    {
        case WIFI_EVENT_SCAN_DONE:
            manager->scanDone(static_cast<wifi_event_sta_scan_done_t*>(event_data));
            break;
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

    auto callbacks = snapshotCallbacks(m_connect_callbacks);
    for (auto& cb : callbacks)
    {
        cb(event->mac, event->aid);
    }
}

void WIFI::handleClientDisconnected(wifi_event_ap_stadisconnected_t* event)
{
    ESP_LOGI(TAG, "AP Client disconnected - MAC: " MACSTR ", AID: %d", MAC2STR(event->mac), event->aid);
    removeClient(event->mac);

    auto callbacks = snapshotCallbacks(m_disconnect_callbacks);
    for (auto& cb : callbacks)
    {
        cb(event->mac, event->aid);
    }
}

void WIFI::handleAPStart()
{
    ESP_LOGI(TAG, "AP Interface Started");
    auto callbacks = snapshotCallbacks(m_start_callbacks);
    for (auto& cb : callbacks)
    {
        cb();
    }
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
    if (m_closing.load()) { return; }
    m_retry_count = 0;
    setState(State::RUNNING);

    char ip_str[16];
    esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "STA Got IP: %s", ip_str);

    auto callbacks = snapshotCallbacks(m_sta_connected_callbacks);

    for (auto& cb : callbacks)
    {
        cb(std::string(ip_str));
    }
}

void WIFI::handleStaDisconnected(wifi_event_sta_disconnected_t* event)
{
    LifecycleGuard guard(m_lifecycleMutex, 0);
    if (m_closing.load()) { return; }
    if (!guard.locked || m_scanActive)
    {
        deferConnection();
        return;
    }
    if (!wifi_scan::mayReconnect(m_config.mode == WIFI_MODE_AP, m_closing.load(), m_scanActive)) { return; }
    ESP_LOGW("WIFI", "Disconnected! Reason: %s (%d)", get_wifi_reason_str(event->reason), event->reason);
    setState(State::STA_DISCONNECTED);

    for (auto& cb : snapshotCallbacks(m_sta_disconnected_callbacks))
    {
        cb();
    }

    m_reconnectPending = true;
    resumeConnection();
}

void WIFI::internal_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    WIFI* manager = static_cast<WIFI*>(arg);

    if (event_id == WIFI_INT_EVENT_DRAIN)
    {
        xSemaphoreGive(manager->m_scanSignal);
        return;
    }

    LifecycleGuard guard(manager->m_lifecycleMutex, 0);
    if (manager->m_closing.load()) { return; }
    if (!guard.locked || manager->m_scanActive)
    {
        manager->deferConnection();
        return;
    }

    if (!wifi_scan::mayReconnect(manager->m_config.mode == WIFI_MODE_AP, manager->m_closing.load(), manager->m_scanActive))
    {
        manager->m_reconnectPending = false;
        return;
    }

    if (event_id != WIFI_INT_EVENT_RESUME || !manager->m_reconnectPending.exchange(false)) { return; }
    if (manager->m_retry_count >= manager->m_config.sta_max_retry)
    {
        ESP_LOGI(TAG, "Internal Event: Executing AP Fallback");
        manager->switchToAP();
    }
    else
    {
        ESP_LOGI(TAG, "Internal Event: Retrying STA Connection");
        ++manager->m_retry_count;
        manager->setState(State::STA_CONNECTING);
        if (esp_wifi_connect() != ESP_OK) { manager->deferConnection(); }
    }
}

esp_err_t WIFI::switchToAP()
{
    LifecycleGuard guard(m_lifecycleMutex, 0);
    if (!guard.locked || m_scanActive || m_closing.load() || m_deinitializing.load())
    {
        m_reconnectPending = !m_closing.load();
        return ESP_ERR_INVALID_STATE;
    }
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

void WIFI::resumeConnection()
{
    // Called with the lifecycle lock, never from the HTTP task's request filter.
    if (m_closing.load() || !m_reconnectPending.load()) { return; }
    if (esp_event_post(WIFI_INTERNAL_EVENT, WIFI_INT_EVENT_RESUME, nullptr, 0, 0) != ESP_OK) { deferConnection(); }
}

void WIFI::deferConnection()
{
    m_reconnectPending = true;
    if (!m_closing.load() && m_reconnectTimer)
    {
        // INVALID_STATE means a retry is already scheduled; no idle polling timer is kept running.
        const esp_err_t ret = esp_timer_start_once(m_reconnectTimer, 100000);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { ESP_LOGW(TAG, "Reconnect scheduling failed: %s", esp_err_to_name(ret)); }
    }
}

void WIFI::reconnectTimer(void* arg)
{
    auto* manager = static_cast<WIFI*>(arg);
    if (!manager->m_closing.load() && manager->m_reconnectPending.load())
    {
        if (esp_event_post(WIFI_INTERNAL_EVENT, WIFI_INT_EVENT_RESUME, nullptr, 0, 0) != ESP_OK)
        {
            manager->deferConnection();
        }
    }
}

void WIFI::onClientConnected(const ClientConnectCallback& callback)
{
    pushCallback(m_connect_callbacks, callback);
}
void WIFI::onClientDisconnected(const ClientDisconnectCallback& callback)
{
    pushCallback(m_disconnect_callbacks, callback);
}
void WIFI::onAPStarted(const APStartCallback& callback)
{
    pushCallback(m_start_callbacks, callback);
}
void WIFI::onStaConnected(const StaConnectedCallback& callback)
{
    pushCallback(m_sta_connected_callbacks, callback);
}
void WIFI::onStaDisconnected(const StaDisconnectedCallback& callback)
{
    pushCallback(m_sta_disconnected_callbacks, callback);
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
    // One read of the pointer: deinit() clears it. It destroys the netif only after the web server (the only caller) has
    // stopped (docs/power.md deinit order), so the handle is never used after destroy.
    esp_netif_t* const netif = m_netif_ap;
    if (netif)
    {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
        {
            return "";
        }
        char ip_str[16];
        (void)snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        return std::string(ip_str);
    }
    return "";
}

std::string WIFI::getStaIP() const
{
    // One read of the pointer: deinit() clears it. It destroys the netif only after the web server (the only caller) has
    // stopped (docs/power.md deinit order), so the handle is never used after destroy.
    esp_netif_t* const netif = m_netif_sta;
    if (netif)
    {
        esp_netif_ip_info_t ip_info = {};
        if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK)
        {
            return "";
        }
        char ip_str[16];
        (void)snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
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
        case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:
            return "ASSOC_EXPIRE";
        case WIFI_REASON_ASSOC_TOOMANY:
            return "ASSOC_TOOMANY";
        case WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA:
            return "NOT_AUTHED";
        case WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA:
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
