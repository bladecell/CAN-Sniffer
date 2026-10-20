#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace supervisor
{
constexpr size_t kComponentCapacity  = 16;
constexpr size_t kNameCapacity       = 32;
constexpr size_t kDependencyCapacity = 4;
constexpr size_t kLegacyCount        = 9;
// SETTINGS is a reserved infrastructure/status prefix, never a registry slot.
enum class Id : uint16_t
{
    SETTINGS         = 0,
    POWER_MANAGEMENT = 1,
    FLASH_FILESYSTEM = 2,
    SD_CARD          = 3,
    BATTERY          = 4,
    CAN              = 5,
    OBD              = 6,
    WIFI             = 7,
    WEBSERVER        = 8,
    COUNT            = 9,
    INVALID          = UINT16_MAX
};
enum class Role : uint8_t
{
    SETTINGS,
    POWER_MANAGEMENT,
    FLASH_FILESYSTEM,
    SD_CARD,
    BATTERY,
    CAN,
    OBD,
    WIFI,
    WEBSERVER,
    NONE
};
enum class Participation : uint8_t
{
    RETAINED,
    NORMAL,
    BATTERY,
    PROBE
};
enum class Phase
{
    NEVER_STARTED,
    STARTING,
    ACTIVE,
    PARTIAL,
    STOPPING,
    STOPPED
};

struct Lifecycle
{
    Phase     phase       = Phase::NEVER_STARTED;
    esp_err_t initError   = ESP_FAIL;
    esp_err_t deinitError = ESP_OK;
};

struct Dependency
{
    Id id = Id::INVALID;
    // Ordering/lifetime alone preserves legacy optional-feature degradation.
    bool requireSuccess = false;
};

struct Descriptor
{
    Id          id             = Id::INVALID;
    const char* name           = nullptr;
    esp_err_t (*init)(void*)   = nullptr;
    esp_err_t (*deinit)(void*) = nullptr;
    // Metadata is copied, but callback code and context must outlive the firmware.
    void*                                       context          = nullptr;
    bool                                        critical         = false;
    Participation                               participation    = Participation::NORMAL;
    Role                                        role             = Role::NONE;
    uint16_t                                    bootPriority     = 100;
    uint16_t                                    shutdownPriority = 0;
    std::array<Dependency, kDependencyCapacity> dependencies{};
    size_t                                      dependencyCount = 0;
};

struct Component
{
    Descriptor                      descriptor{};
    std::array<char, kNameCapacity> name{};
    Lifecycle                       lifecycle{};
};

struct SetupStep
{
    const char* name;
    // Initial boot only, including ESP_FAIL for unattempted steps.
    std::atomic<esp_err_t> result{ESP_FAIL};
};

// Core serializes registration/admission. Planning runs with registration closed,
// outside the ESP interrupt mask; metadata is immutable before task publication.
class Registry
{
public:
    Registry()                                  = default;
    Registry(const Registry&)                   = delete;
    Registry&        operator=(const Registry&) = delete;
    esp_err_t        add(const Descriptor& descriptor);
    esp_err_t        freeze();
    Component*       find(Id id);
    const Component* find(Id id) const;
    Id               role(Role role) const;
    const Component& at(size_t slot) const
    {
        return components_[slot];
    }
    size_t size() const
    {
        return count_;
    }
    size_t bootSlot(size_t position) const
    {
        return boot_[position];
    }
    size_t shutdownSlot(size_t position) const
    {
        return shutdown_[position];
    }
    size_t shutdownCount() const
    {
        return shutdownCount_;
    }
    static const char* legacyName(size_t role);

private:
    esp_err_t                                 plan(bool shutdown);
    std::array<Component, kComponentCapacity> components_{};
    std::array<size_t, kComponentCapacity>    boot_{};
    std::array<size_t, kComponentCapacity>    shutdown_{};
    size_t                                    count_         = 0;
    size_t                                    shutdownCount_ = 0;
    bool                                      frozen_        = false;
};
}  // namespace supervisor
