#include "supervisor_registry.hpp"

#include <cstring>

namespace supervisor
{
const char* Registry::legacyName(size_t role)
{
  static constexpr const char* names[] = {"SETTINGS", "POWER_MANAGEMENT", "FLASH_FILESYSTEM", "SD_CARD", "BATTERY",
                                         "CAN", "OBD", "WIFI", "WEBSERVER"};
  return role < kLegacyCount ? names[role] : "UNKNOWN";
}

Component* Registry::find(Id id)
{
  for (size_t i = 0; i < count_; ++i)
  {
    if (components_[i].descriptor.id == id)
    {
      return &components_[i];
    }
  }
  return nullptr;
}

const Component* Registry::find(Id id) const
{
  for (size_t i = 0; i < count_; ++i)
  {
    if (components_[i].descriptor.id == id)
    {
      return &components_[i];
    }
  }
  return nullptr;
}

Id Registry::role(Role role) const
{
  for (size_t i = 0; i < count_; ++i)
  {
    if (components_[i].descriptor.role == role)
    {
      return components_[i].descriptor.id;
    }
  }
  return Id::INVALID;
}

esp_err_t Registry::add(const Descriptor& d)
{
  if (frozen_)
  {
    return ESP_ERR_INVALID_STATE;
  }
  if (!d.name || !d.init || d.id == Id::INVALID || d.id == Id::SETTINGS || d.role == Role::SETTINGS ||
      d.dependencyCount > kDependencyCapacity ||
      static_cast<unsigned>(d.role) > static_cast<unsigned>(Role::NONE) ||
      static_cast<unsigned>(d.participation) > static_cast<unsigned>(Participation::PROBE))
  {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t length = strnlen(d.name, kNameCapacity);
  if (length == 0 || length == kNameCapacity || std::strcmp(d.name, legacyName(0)) == 0)
  {
    return ESP_ERR_INVALID_ARG;
  }
  const bool retained = d.participation == Participation::RETAINED;
  if (retained ? d.deinit != nullptr : d.deinit == nullptr)
  {
    return ESP_ERR_INVALID_ARG;
  }
  const size_t r = static_cast<size_t>(d.role);
  if (r < kLegacyCount)
  {
    const Participation expected = r < 3 ? Participation::RETAINED :
      d.role == Role::BATTERY ? Participation::BATTERY : d.role == Role::CAN ? Participation::PROBE : Participation::NORMAL;
    if (static_cast<size_t>(d.id) != r || std::strcmp(d.name, legacyName(r)) != 0 ||
        d.critical != (r < 3 || r > 6) || d.participation != expected || d.wakeCritical != (r >= 6))
    {
      return ESP_ERR_INVALID_ARG;
    }
  }
  else if (static_cast<size_t>(d.id) < kLegacyCount || d.participation == Participation::BATTERY ||
           d.participation == Participation::PROBE)
  {
    return ESP_ERR_INVALID_ARG;
  }
  for (size_t i = 0; i < count_; ++i)
  {
    if (components_[i].descriptor.id == d.id || std::strcmp(components_[i].name.data(), d.name) == 0 ||
        (d.role != Role::NONE && components_[i].descriptor.role == d.role))
    {
      return ESP_ERR_INVALID_STATE;
    }
  }
  for (size_t i = 0; i < d.dependencyCount; ++i)
  {
    if (d.dependencies[i].id == d.id || d.dependencies[i].id == Id::INVALID)
    {
      return ESP_ERR_INVALID_ARG;
    }
    for (size_t j = 0; j < i; ++j)
    {
      if (d.dependencies[i].id == d.dependencies[j].id)
      {
        return ESP_ERR_INVALID_ARG;
      }
    }
  }
  if (count_ == kComponentCapacity)
  {
    return ESP_ERR_NO_MEM;
  }
  Component& c = components_[count_++];
  c.descriptor = d;
  std::memcpy(c.name.data(), d.name, length + 1);
  c.descriptor.name = c.name.data();
  return ESP_OK;
}

esp_err_t Registry::plan(bool shutdown)
{
  std::array<bool, kComponentCapacity> done{};
  size_t remaining = count_;
  for (size_t i = 0; i < count_; ++i)
  {
    const auto mode = components_[i].descriptor.participation;
    if (shutdown && (mode == Participation::RETAINED || mode == Participation::BATTERY))
    {
      done[i] = true;
      --remaining;
    }
  }
  size_t position = 0;
  while (remaining)
  {
    size_t best = count_;
    for (size_t i = 0; i < count_; ++i)
    {
      if (done[i])
      {
        continue;
      }
      bool ready = true;
      for (size_t j = 0; j < count_; ++j)
      {
        if (done[j])
        {
          continue;
        }
        const auto& consumer = components_[shutdown ? j : i].descriptor;
        const Id provider = components_[shutdown ? i : j].descriptor.id;
        for (size_t k = 0; k < consumer.dependencyCount; ++k)
        {
          if (consumer.dependencies[k].id == provider)
          {
            ready = false;
          }
        }
      }
      if (!ready)
      {
        continue;
      }
      const auto& d = components_[i].descriptor;
      if (best == count_)
      {
        best = i;
      }
      else
      {
        const auto& b = components_[best].descriptor;
        const auto priority = shutdown ? d.shutdownPriority : d.bootPriority;
        const auto bestPriority = shutdown ? b.shutdownPriority : b.bootPriority;
        if (priority < bestPriority || (priority == bestPriority && d.id < b.id))
        {
          best = i;
        }
      }
    }
    if (best == count_)
    {
      return ESP_ERR_INVALID_STATE;
    }
    (shutdown ? shutdown_ : boot_)[position++] = best;
    done[best] = true;
    --remaining;
  }
  if (shutdown)
  {
    shutdownCount_ = position;
  }
  return ESP_OK;
}

esp_err_t Registry::freeze()
{
  if (frozen_)
  {
    return ESP_ERR_INVALID_STATE;
  }
  for (size_t i = 1; i < kLegacyCount; ++i)
  {
    if (role(static_cast<Role>(i)) == Id::INVALID)
    {
      return ESP_ERR_INVALID_STATE;
    }
  }
  for (size_t i = 0; i < count_; ++i)
  {
    const auto& d = components_[i].descriptor;
    for (size_t j = 0; j < d.dependencyCount; ++j)
    {
      // An implicit retained provider: admission guarantees bootstrap completes
      // before any registered consumer runs, and normal sleep never releases it.
      if (d.dependencies[j].id == Id::SETTINGS)
      {
        continue;
      }
      const Component* provider = find(d.dependencies[j].id);
      if (!provider)
      {
        return ESP_ERR_INVALID_STATE;
      }
      const auto p = provider->descriptor.participation;
      if ((d.participation == Participation::RETAINED && p != Participation::RETAINED) ||
          ((d.participation == Participation::BATTERY || d.participation == Participation::PROBE) &&
           p == Participation::NORMAL) ||
          (d.participation == Participation::BATTERY && p == Participation::PROBE))
      {
        return ESP_ERR_INVALID_STATE;
      }
    }
  }
  if (plan(false) != ESP_OK || plan(true) != ESP_OK)
  {
    return ESP_ERR_INVALID_STATE;
  }
  frozen_ = true;
  return ESP_OK;
}
}
