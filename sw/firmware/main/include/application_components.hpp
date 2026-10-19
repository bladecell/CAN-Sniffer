#pragma once

#include "esp_err.h"

// Application wiring selected by main's descriptors, not component base classes.
// Callbacks ignore their context; code, singleton targets and PM state have
// firmware lifetime. Nothing initializes hardware before registry admission.
namespace application_components
{
// Explicit infrastructure stage on main's init caller, not a registration.
esp_err_t bootstrapSettings(void*);
esp_err_t setupPowerManagement(void*);
esp_err_t setupFlashFilesystem(void*);
esp_err_t setupWifi(void*);
esp_err_t setupCan(void*);
esp_err_t setupObd(void*);
esp_err_t setupBattery(void*);
esp_err_t setupSdCard(void*);
esp_err_t setupWebserver(void*);
esp_err_t stopSdCard(void*);
esp_err_t stopBattery(void*);
esp_err_t stopCan(void*);
esp_err_t stopObd(void*);
esp_err_t stopWifi(void*);
esp_err_t stopWebserver(void*);

// Sleep-platform bridge: the handle remains private to application wiring.
esp_err_t releasePowerLock();
esp_err_t acquirePowerLock();
}
