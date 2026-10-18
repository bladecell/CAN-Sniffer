#include "wifi.hpp"

#include "esp_timer.h"

WIFI::ScanStartResult WIFI::startScan(uint32_t& id)
{
    LifecycleGuard guard(m_lifecycleMutex, 0);
    if (!guard.locked) { return ScanStartResult::Busy; }
    const int64_t now = esp_timer_get_time();
    const auto admission = wifi_scan::admit(!m_closing.load() && !m_deinitializing.load() &&
                                               getState() == State::RUNNING,
                                           m_scanActive, m_scanFault, now, m_lastScanStartUs);
    if (admission == wifi_scan::Admission::Busy) { return ScanStartResult::Busy; }
    if (admission == wifi_scan::Admission::Down) { return ScanStartResult::Down; }
    if (admission == wifi_scan::Admission::Cooldown) { return ScanStartResult::Cooldown; }

    // The task holds the only SDK result buffer. No allocation scales with the number of visible APs.
    xSemaphoreTake(m_scanExited, 0);
    xSemaphoreTake(m_scanSignal, 0);
    const uint32_t next = m_scan.id + 1;
    m_scan = {};
    m_scan.id = next ? next : 1;
    m_scan.state = ScanState::Scanning;
    m_scanActive = true;
    m_scanCancel = false;
    m_scanDone = false;
    m_scanTemporarySta = false;
    m_scanStatus = 1;
    if (xTaskCreate(scanTask, "wifi_scan", 6144, this, 2, nullptr) != pdPASS)
    {
        m_scanActive = false;
        m_scan.state = ScanState::Failed;
        m_scan.error = ESP_ERR_NO_MEM;
        m_scan.completedUs = now;
        return ScanStartResult::NoMemory;
    }
    m_lastScanStartUs = now;
    id = m_scan.id;
    return ScanStartResult::Started;
}

esp_err_t WIFI::getScan(ScanSnapshot& snapshot)
{
    LifecycleGuard guard(m_lifecycleMutex, 0);
    if (!guard.locked) { return ESP_ERR_TIMEOUT; }
    if (m_closing.load() || m_deinitializing.load()) { return ESP_ERR_INVALID_STATE; }
    if (m_scanActive && wifi_scan::deadlineReached(esp_timer_get_time(), m_lastScanStartUs))
    {
        return ESP_ERR_TIMEOUT;
    }
    if ((m_scan.state == ScanState::Complete || m_scan.state == ScanState::Failed) &&
        wifi_scan::expired(esp_timer_get_time(), m_scan.completedUs))
    {
        const uint32_t id = m_scan.id;
        m_scan = {};
        m_scan.id = id;
    }
    snapshot = m_scan;
    resumeConnection();
    return ESP_OK;
}

void WIFI::scanDone(const wifi_event_sta_scan_done_t* event)
{
    // Semaphore lifetime exceeds handler registration. Never wait for the lifecycle lock on the default event loop.
    if (event && m_scanEvents.accept())
    {
        m_scanStatus.store(event->status);
        m_scanDoneUs = esp_timer_get_time();
        m_scanDone.store(true);
        xSemaphoreGive(m_scanSignal);
    }
}

esp_err_t WIFI::cancelScan()
{
    {
        LifecycleGuard guard(m_lifecycleMutex);
        if (!guard.locked) { return ESP_ERR_TIMEOUT; }
        m_closing = true;
        if (!m_scanActive) { return ESP_OK; }
        m_scanCancel = true;
        xSemaphoreGive(m_scanSignal);
    }
    // Cancellation wakes the task immediately; it has a separate 500 ms completion drain bound.
    return xSemaphoreTake(m_scanExited, pdMS_TO_TICKS(2500)) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

void WIFI::scanTask(void* arg)
{
    static_cast<WIFI*>(arg)->runScan();
    vTaskDelete(nullptr);
}

void WIFI::runScan()
{
    esp_err_t result = ESP_OK;
    bool started = false;
    bool canClean = true;
    bool listReleased = false;
    {
        LifecycleGuard guard(m_lifecycleMutex);
        if (!guard.locked || wifi_scan::deadlineReached(esp_timer_get_time(), m_lastScanStartUs)) { result = ESP_ERR_TIMEOUT; }
        else if (m_scanCancel || m_closing.load()) { result = ESP_ERR_INVALID_STATE; }
        else
        {
            wifi_mode_t mode;
            result = esp_wifi_get_mode(&mode);
            if (result == ESP_OK && mode == WIFI_MODE_AP)
            {
                // Adding the STA interface does not stop/restart the AP and never writes m_config or NVS.
                result = esp_wifi_set_mode(WIFI_MODE_APSTA);
                m_scanTemporarySta = result == ESP_OK;
            }
            if (result == ESP_OK)
            {
                wifi_scan_config_t config = {};
                config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
                config.scan_time.active.max = 120;
                config.home_chan_dwell_time = 30;
                m_scanEvents.arm();
                result = esp_wifi_scan_start(&config, false);
                started = result == ESP_OK;
                if (!started) { m_scanEvents.close(); }
            }
        }
    }

    if (started)
    {
        while (!m_scanDone.load())
        {
            bool cancel;
            {
                LifecycleGuard guard(m_lifecycleMutex);
                cancel = !guard.locked || m_scanCancel;
            }
            const int64_t remaining = wifi_scan::kDeadlineUs - (esp_timer_get_time() - m_lastScanStartUs);
            if (cancel || remaining <= 0)
            {
                result = cancel ? ESP_ERR_INVALID_STATE : ESP_ERR_TIMEOUT;
                break;
            }
            xSemaphoreTake(m_scanSignal, pdMS_TO_TICKS(static_cast<uint32_t>((remaining + 999) / 1000)));
        }
        if (m_scanDone.load() && wifi_scan::deadlineReached(m_scanDoneUs, m_lastScanStartUs)) { result = ESP_ERR_TIMEOUT; }
        if (result != ESP_OK && !m_scanDone.load())
        {
            const esp_err_t stopped = esp_wifi_scan_stop();
            canClean = stopped == ESP_OK;
            if (!m_scanDone.load()) { xSemaphoreTake(m_scanSignal, pdMS_TO_TICKS(500)); }
            // No subsequent scan may reuse a completion that has not been observed. A full deinit/init retires it.
            if (stopped != ESP_OK || !m_scanDone.load()) { m_scanFault = true; }
        }
    }

    LifecycleGuard guard(m_lifecycleMutex);
    if (!guard.locked)
    {
        // Retain admission and resources on a drain failure; stop/deinit will time out rather than free under us.
        return;
    }
    m_scanEvents.close();
    if (m_scanCancel) { result = ESP_ERR_INVALID_STATE; }
    if (started && result == ESP_OK && m_scanStatus.load() != 0) { result = ESP_FAIL; }
    if (started && result == ESP_OK)
    {
        uint16_t total = 0;
        result = esp_wifi_scan_get_ap_num(&total);
        if (result == ESP_OK && total != 0)
        {
            wifi_ap_record_t records[wifi_scan::kMaxResults] = {};
            uint16_t count = wifi_scan::kMaxResults;
            result = esp_wifi_scan_get_ap_records(&count, records);
            if (result == ESP_OK)
            {
                listReleased = true;
                const uint16_t copied = count > wifi_scan::kMaxResults ? wifi_scan::kMaxResults : count;
                m_scan.count = copied;
                m_scan.truncated = total > copied || count > copied;
                for (uint16_t i = 0; i < copied; ++i)
                {
                    auto& item = m_scan.networks[i];
                    wifi_scan::encodeSsid(records[i].ssid, sizeof(records[i].ssid), item.ssid, item.ssidHex);
                    item.rssi = records[i].rssi;
                    item.channel = records[i].primary;
                    item.authMode = records[i].authmode;
                }
            }
        }
    }
    if (started && canClean && !listReleased)
    {
        const esp_err_t cleared = esp_wifi_clear_ap_list();
        if (cleared != ESP_OK) { m_scanFault = true; if (result == ESP_OK) { result = cleared; } }
    }
    if (m_scanTemporarySta && canClean)
    {
        const esp_err_t restored = esp_wifi_set_mode(WIFI_MODE_AP);
        if (restored != ESP_OK) { m_scanFault = true; result = restored; }
        else { m_scanTemporarySta = false; }
    }
    if (result != ESP_OK) { m_scan.count = 0; m_scan.truncated = false; }
    m_scan.error = result;
    m_scan.state = result == ESP_OK ? ScanState::Complete : ScanState::Failed;
    m_scan.completedUs = esp_timer_get_time();
    m_scanActive = false;
    resumeConnection();
    xSemaphoreGive(m_scanExited);
}
