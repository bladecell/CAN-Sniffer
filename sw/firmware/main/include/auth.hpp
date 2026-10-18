#pragma once

// Device side of access control (P2.10c): PBKDF2 via mbedTLS, hardware RNG, NVS record, the BOOT-hold recovery hook and
// the locks around auth::AuthCore (the rules live there, host-tested). Thread-safe. Nothing here logs a PIN or token.

#include <cstddef>
#include <cstdint>

#include "auth_core.hpp"
#include "esp_err.h"

namespace auth_service
{
// PBKDF2-HMAC-SHA256 cost for new PINs. Chosen for a login of roughly 100 to 300 ms on the ESP32-S3; the first
// derivation logs its measured time ("PIN derivation"), adjust this from that number. Stored with the record, so a
// changed value only applies to PINs set afterwards.
constexpr uint32_t kPbkdf2Iterations = 5000;

// Loads the record from NVS, registers the BOOT-hold recovery hook (when the button driver is up). A storage problem is
// not an error here: the state becomes Unavailable and gated routes answer 503. Call after button_init().
esp_err_t start();
// Clears the hook, forgets sessions and releases the lock. Call before button_deinit().
void stop();

// The one admin check. `bearer` is the token from the Authorization header, or null.
auth::Result requireAdmin(const char* bearer);

struct StatusView
{
  auth::State state;
  bool        authenticated;
  uint32_t    claimWindowS;
  uint32_t    retryAfterS;
};
StatusView status(const char* bearer);

auth::Result claim(const char* pin, size_t len, char* tokenOut);
auth::Result login(const char* pin, size_t len, char* tokenOut, uint32_t* retryS);
auth::Result logout(const char* bearer);
auth::Result changePin(const char* oldPin, size_t oldLen, const char* newPin, size_t newLen, uint32_t* retryS);
}  // namespace auth_service
