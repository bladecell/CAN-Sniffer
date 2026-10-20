#pragma once

#include "application_components.hpp"
#include "supervisor_registry.hpp"
#include <array>

// The production component table. Header-only so host tests can register it with the real Registry and Core; the
// callbacks are only addressed here, so a host build supplies stubs for them.
namespace production_components
{
inline std::array<supervisor::Descriptor, 8> descriptors()
{
    using namespace supervisor;
    return {{
        {Id::POWER_MANAGEMENT, "POWER_MANAGEMENT", application_components::setupPowerManagement, nullptr, nullptr, true,
         Participation::RETAINED, Role::POWER_MANAGEMENT, 1},
        {Id::FLASH_FILESYSTEM, "FLASH_FILESYSTEM", application_components::setupFlashFilesystem, nullptr, nullptr, true,
         Participation::RETAINED, Role::FLASH_FILESYSTEM, 2},
        {Id::SD_CARD, "SD_CARD", application_components::setupSdCard, application_components::stopSdCard, nullptr,
         false, Participation::NORMAL, Role::SD_CARD, 3, 4},
        {Id::BATTERY, "BATTERY", application_components::setupBattery, application_components::stopBattery, nullptr,
         false, Participation::BATTERY, Role::BATTERY, 4},
        {Id::CAN, "CAN", application_components::setupCan, application_components::stopCan, nullptr, false,
         Participation::PROBE, Role::CAN, 5, 3},
        {Id::OBD,
         "OBD",
         application_components::setupObd,
         application_components::stopObd,
         nullptr,
         false,
         Participation::NORMAL,
         Role::OBD,
         6,
         1,
         {{{Id::CAN}, {Id::SD_CARD}}},
         2,
         true},
        {Id::WIFI, "WIFI", application_components::setupWifi, application_components::stopWifi, nullptr, false,
         Participation::NORMAL, Role::WIFI, 7, 2, {}, 0, false},
        {Id::WEBSERVER,
         "WEBSERVER",
         application_components::setupWebserver,
         application_components::stopWebserver,
         nullptr,
         false,
         Participation::NORMAL,
         Role::WEBSERVER,
         8,
         0,
         {{{Id::OBD}, {Id::WIFI}, {Id::FLASH_FILESYSTEM}, {Id::SD_CARD}}},
         4,
         false},
    }};
}
}  // namespace production_components
