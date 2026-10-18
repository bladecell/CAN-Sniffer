#include "auth.hpp"

#include "auth_dev.hpp"

#include "button.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "mbedtls/md.h"
#include "mbedtls/private/pkcs5.h"
#include "nvs.h"
#include "psa/crypto.h"

static const char* TAG = "AUTH";

namespace
{
constexpr const char* kNvsNamespace = "auth";
constexpr const char* kNvsKey       = "pin";
constexpr TickType_t  kOpWait       = pdMS_TO_TICKS(3000);

// Failed-login count. RTC_NOINIT survives a crash and a sleep/wake but is undefined after a power loss, so it counts
// only next to the magic word. A reboot cannot be used to shed the penalty.
RTC_NOINIT_ATTR uint32_t s_rtcMagic;
RTC_NOINIT_ATTR uint32_t s_rtcFailures;
constexpr uint32_t       kRtcMagic = 0xA07F1A11u;

portMUX_TYPE      s_spin = portMUX_INITIALIZER_UNLOCKED;
StaticSemaphore_t s_opBuf;           // static, never deleted: a late recovery on the button task can still take it
SemaphoreHandle_t s_op = nullptr;    // serialises claim, login, change PIN and recovery
bool              s_up   = false;
bool              s_buttonOk = false;  // the hold hook is registered, so the button driver is running

void lockFast()
{
  portENTER_CRITICAL(&s_spin);
}
void unlockFast()
{
  portEXIT_CRITICAL(&s_spin);
}

void fillRandom(uint8_t* buf, size_t len)
{
  esp_fill_random(buf, len);
}

bool deriveKey(const uint8_t* pin, size_t pinLen, const uint8_t* salt, size_t saltLen, uint32_t iterations, uint8_t* out)
{
  if (psa_crypto_init() != PSA_SUCCESS)
  {
    return false;
  }
  const int64_t t0 = esp_timer_get_time();
  const int     rc = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256, pin, pinLen, salt, saltLen, iterations,
                                                   auth::kHashLen, out);
  ESP_LOGI(TAG, "PIN derivation: %u iterations, %d ms, %s", static_cast<unsigned>(iterations),
           static_cast<int>((esp_timer_get_time() - t0) / 1000), rc == 0 ? "ok" : "failed");
  return rc == 0;
}

bool saveRecord(const uint8_t* rec, size_t len)
{
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK)
  {
    return false;
  }
  esp_err_t err = nvs_set_blob(h, kNvsKey, rec, len);
  if (err == ESP_OK)
  {
    err = nvs_commit(h);
  }
  nvs_close(h);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "storing the PIN record failed: %s", esp_err_to_name(err));
  }
  return err == ESP_OK;
}

bool eraseRecord()
{
  nvs_handle_t h;
  if (nvs_open(kNvsNamespace, NVS_READWRITE, &h) != ESP_OK)
  {
    return false;
  }
  esp_err_t err = nvs_erase_key(h, kNvsKey);
  if (err == ESP_ERR_NVS_NOT_FOUND)
  {
    err = ESP_OK;
  }
  if (err == ESP_OK)
  {
    err = nvs_commit(h);
  }
  nvs_close(h);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "erasing the PIN record failed: %s", esp_err_to_name(err));
  }
  return err == ESP_OK;
}

int64_t nowUs()
{
  return esp_timer_get_time();
}

uint32_t loadFailures()
{
  return (s_rtcMagic == kRtcMagic && s_rtcFailures <= 32) ? s_rtcFailures : 0;
}

void storeFailures(uint32_t n)
{
  s_rtcMagic    = kRtcMagic;
  s_rtcFailures = n;
}

const auth::Platform kPlatform{deriveKey, fillRandom, saveRecord, eraseRecord, lockFast, unlockFast,
                               auth_service::kPbkdf2Iterations, nowUs, loadFailures, storeFailures};
auth::AuthCore       s_core(kPlatform);

// Reads the stored record. A missing namespace or key is a device nobody has claimed; anything unreadable or malformed
// is Unavailable, which fails closed until the BOOT-hold recovery clears it.
auth::State loadRecord(auth::PinRecord* rec)
{
  nvs_handle_t h;
  esp_err_t    err = nvs_open(kNvsNamespace, NVS_READONLY, &h);
  if (err == ESP_ERR_NVS_NOT_FOUND)
  {
    return auth::State::Unclaimed;
  }
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "NVS open failed: %s; gated routes closed", esp_err_to_name(err));
    return auth::State::Unavailable;
  }
  uint8_t blob[auth::kRecordSize];
  size_t  size = 0;
  err          = nvs_get_blob(h, kNvsKey, nullptr, &size);
  if (err == ESP_OK && size == sizeof(blob))
  {
    err = nvs_get_blob(h, kNvsKey, blob, &size);
  }
  else if (err == ESP_OK)
  {
    err = ESP_ERR_NVS_INVALID_LENGTH;
  }
  nvs_close(h);
  if (err == ESP_ERR_NVS_NOT_FOUND)
  {
    return auth::State::Unclaimed;
  }
  if (err != ESP_OK || !auth::decodeRecord(blob, size, rec))
  {
    ESP_LOGE(TAG, "PIN record unreadable (%s); gated routes closed until BOOT is held for %d s", esp_err_to_name(err),
             static_cast<int>(button_hold_us() / 1000000));
    return auth::State::Unavailable;
  }
  return auth::State::Claimed;
}

struct OpLock
{
  bool ok;
  OpLock() : ok(s_op != nullptr && xSemaphoreTake(s_op, kOpWait) == pdTRUE) {}
  ~OpLock()
  {
    if (ok)
    {
      xSemaphoreGive(s_op);
    }
  }
  OpLock(const OpLock&)            = delete;
  OpLock& operator=(const OpLock&) = delete;
};

// Runs on the button task after BOOT was held for button_hold_us().
void onHold(void*)
{
  OpLock lock;
  if (!lock.ok)
  {
    ESP_LOGW(TAG, "BOOT hold ignored: another auth operation is running, hold again");
    return;
  }
  if (!s_up)
  {
    return;  // the web server is stopping
  }
  if (s_core.recover() == auth::Result::Ok)
  {
    ESP_LOGW(TAG, "PIN cleared by BOOT hold; the device can be claimed again");
  }
}
}  // namespace

namespace auth_service
{
esp_err_t start()
{
  if (s_up)
  {
    return ESP_OK;
  }
  if (s_op == nullptr)
  {
    s_op = xSemaphoreCreateMutexStatic(&s_opBuf);
  }
  if (s_op == nullptr)
  {
    s_core.load(auth::State::Unavailable, nullptr);
    return ESP_ERR_NO_MEM;
  }
  auth::PinRecord rec{};
  const auth::State st = loadRecord(&rec);
  s_core.load(st, &rec);
  const esp_err_t hook = button_set_hold_hook(onHold, nullptr);
  s_buttonOk = hook == ESP_OK;
  if (!s_buttonOk)
  {
    ESP_LOGW(TAG, "no BOOT button (%s): claim and hold recovery unavailable", esp_err_to_name(hook));
  }
  s_up = true;
#if defined(CONFIG_SNIFFER_AUTH_DEV_BYPASS)
  ESP_LOGW(TAG, "DEV BYPASS ACTIVE: gated routes need NO login. Development build only, never ship it");
#endif
  ESP_LOGI(TAG, "access control up: %s", st == auth::State::Claimed ? "claimed"
                                         : st == auth::State::Unclaimed ? "unclaimed" : "UNAVAILABLE");
  return ESP_OK;
}

void stop()
{
  if (!s_up)
  {
    return;
  }
  s_up = false;  // first: a recovery that is still waiting for the lock sees it and leaves
  (void)button_set_hold_hook(nullptr, nullptr);  // waits for a running recovery
  s_buttonOk = false;
  s_core.load(auth::State::Unavailable, nullptr);  // drops sessions; the failure count stays in RTC memory
}

auth::Result requireAdmin(const char* bearer)
{
#if defined(CONFIG_SNIFFER_AUTH_DEV_BYPASS)
    // Rate limited to one line a minute: the dashboard polls, so a line per request would bury the log.
    static int64_t s_lastWarnUs = -60LL * 1000000;
    if (esp_timer_get_time() - s_lastWarnUs >= 60LL * 1000000)
    {
        s_lastWarnUs = esp_timer_get_time();
        ESP_LOGW(TAG, "DEV BYPASS ACTIVE: gated routes need NO login. Development build only, never ship it");
    }
    (void)bearer;
    return auth::Result::Ok;
#endif
  if (!s_up)
  {
    return auth::Result::Unavailable;
  }
  return s_core.check(bearer, nowUs());
}

StatusView status(const char* bearer)
{
  const bool authed = auth_dev::kBypass || (s_up && s_core.check(bearer, nowUs()) == auth::Result::Ok);
  // An unclaimed device may have a claim waiting for its press: looking at it commits it, so serialise like a claim.
  OpLock lock;
  if (!lock.ok && s_core.state() == auth::State::Unclaimed)
  {
    return {auth::State::Unclaimed, authed, 0, false, 0};
  }
  const auth::AuthCore::Status s = s_core.status(button_last_press_us(), nowUs());
  return {s.state, authed, s.retryAfterS, s.pending, s.pendingExpiresS};
}

auth::Result claimPost(const char* pin, size_t len, char* idOut)
{
  OpLock lock;
  if (!lock.ok || !s_up)
  {
    return auth::Result::Unavailable;
  }
  if (!s_buttonOk)
  {
    return auth::Result::ButtonRequired;
  }
  const auth::Result r = s_core.claimPost(pin, len, button_last_press_us(), nowUs(), idOut);
  if (r == auth::Result::Ok)
  {
    ESP_LOGI(TAG, "claim parked, waiting for a BOOT press");
  }
  else if (r == auth::Result::ClaimConflict)
  {
    ESP_LOGW(TAG, "two claims at once: both cancelled");
  }
  return r;
}

auth::ClaimView claimStatus(const char* id)
{
  OpLock lock;
  if (!lock.ok || !s_up)
  {
    return {auth::ClaimStatus::Unknown, 0, false, {}};
  }
  const auth::ClaimView v = s_core.claimStatus(id, button_last_press_us(), nowUs());
  if (v.status == auth::ClaimStatus::Committed && v.hasToken)
  {
    ESP_LOGI(TAG, "device claimed");
  }
  return v;
}

auth::Result login(const char* pin, size_t len, char* tokenOut, uint32_t* retryS)
{
  *retryS = 0;
  OpLock lock;
  if (!lock.ok || !s_up)
  {
    return auth::Result::Unavailable;
  }
  const auth::Result r = s_core.login(pin, len, nowUs(), tokenOut, retryS, button_last_press_us());
  if (r == auth::Result::BadPin)
  {
    ESP_LOGW(TAG, "login failed, next try in %us", static_cast<unsigned>(*retryS));
  }
  return r;
}

auth::Result logout(const char* bearer)
{
  return s_up ? s_core.logout(bearer) : auth::Result::Unavailable;
}

auth::Result changePin(const char* oldPin, size_t oldLen, const char* newPin, size_t newLen, uint32_t* retryS)
{
  *retryS = 0;
  OpLock lock;
  if (!lock.ok || !s_up)
  {
    return auth::Result::Unavailable;
  }
  const auth::Result r = s_core.changePin(oldPin, oldLen, newPin, newLen, nowUs(), retryS, button_last_press_us());
  if (r == auth::Result::Ok)
  {
    ESP_LOGI(TAG, "PIN changed, all sessions ended");
  }
  return r;
}
}  // namespace auth_service
