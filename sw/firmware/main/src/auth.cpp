#include "auth.hpp"

#include "button.h"
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

portMUX_TYPE      s_spin = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t s_op   = nullptr;  // serialises claim, login, change PIN and recovery
bool              s_up   = false;

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

const auth::Platform kPlatform{deriveKey, fillRandom, saveRecord, eraseRecord, lockFast, unlockFast,
                               auth_service::kPbkdf2Iterations};
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

int64_t nowUs()
{
  return esp_timer_get_time();
}

// Runs on the button task after BOOT was held for button_hold_us().
void onHold(void*)
{
  OpLock lock;
  if (!lock.ok)
  {
    return;
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
  s_op = xSemaphoreCreateMutex();
  if (s_op == nullptr)
  {
    s_core.load(auth::State::Unavailable, nullptr);
    return ESP_ERR_NO_MEM;
  }
  auth::PinRecord rec{};
  const auth::State st = loadRecord(&rec);
  s_core.load(st, &rec);
  const esp_err_t hook = button_set_hold_hook(onHold, nullptr);
  if (hook != ESP_OK)
  {
    ESP_LOGW(TAG, "no BOOT hold recovery (%s)", esp_err_to_name(hook));
  }
  s_up = true;
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
  (void)button_set_hold_hook(nullptr, nullptr);  // waits for a running recovery
  s_up = false;
  s_core.load(auth::State::Unavailable, nullptr);  // drops sessions
  SemaphoreHandle_t op = s_op;
  s_op                 = nullptr;
  vSemaphoreDelete(op);
}

auth::Result requireAdmin(const char* bearer)
{
  if (!s_up)
  {
    return auth::Result::Unavailable;
  }
  return s_core.check(bearer, nowUs());
}

StatusView status(const char* bearer)
{
  const auth::AuthCore::Status s = s_core.status(button_last_press_us(), nowUs());
  return {s.state, s_up && s_core.check(bearer, nowUs()) == auth::Result::Ok, s.claimWindowS, s.retryAfterS};
}

auth::Result claim(const char* pin, size_t len, char* tokenOut)
{
  OpLock lock;
  if (!lock.ok || !s_up)
  {
    return auth::Result::Unavailable;
  }
  const auth::Result r = s_core.claim(pin, len, button_last_press_us(), nowUs(), tokenOut);
  if (r == auth::Result::Ok)
  {
    ESP_LOGI(TAG, "device claimed");
  }
  return r;
}

auth::Result login(const char* pin, size_t len, char* tokenOut, uint32_t* retryS)
{
  *retryS = 0;
  OpLock lock;
  if (!lock.ok || !s_up)
  {
    return auth::Result::Unavailable;
  }
  const auth::Result r = s_core.login(pin, len, nowUs(), tokenOut, retryS);
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
  const auth::Result r = s_core.changePin(oldPin, oldLen, newPin, newLen, nowUs(), retryS);
  if (r == auth::Result::Ok)
  {
    ESP_LOGI(TAG, "PIN changed, all sessions ended");
  }
  return r;
}
}  // namespace auth_service
