#pragma once

// Text for esp_reset_reason_t (P2.9 follow-up). Indexed by the enum value so host tests can check it without ESP-IDF;
// supervisor.cpp static_asserts that the indices still match the enum.

namespace reset_reason
{
inline const char* text(int reason)
{
  static const char* const kText[] = {"Unknown",         "Power on",        "External reset", "Software reset",
                                      "Panic reset",     "Interrupt watchdog", "Task watchdog", "Watchdog reset",
                                      "Deep sleep",      "Brownout reset",  "SDIO reset",     "USB reset",
                                      "JTAG reset",      "Efuse reset",     "Power glitch",   "CPU lockup"};
  constexpr int kCount = static_cast<int>(sizeof(kText) / sizeof(kText[0]));
  return reason >= 0 && reason < kCount ? kText[reason] : "invalid reason";
}
}  // namespace reset_reason
