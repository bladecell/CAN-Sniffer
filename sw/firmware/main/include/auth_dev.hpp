#pragma once

// Development-only bypass of the login for gated routes (P2.10f). Compile time only: a Kconfig bool that is off in every
// committed config. There is no runtime switch (no header, query, NVS key). With the option off, the code that honours
// it is not compiled at all; host test_auth_core.cpp pins kBypass == false for the default configuration.

#include "sdkconfig.h"

#if defined(CONFIG_SNIFFER_AUTH_DEV_BYPASS) && defined(CONFIG_SNIFFER_RELEASE_BUILD)
#error "CONFIG_SNIFFER_AUTH_DEV_BYPASS must not be combined with CONFIG_SNIFFER_RELEASE_BUILD"
#endif

namespace auth_dev
{
#if defined(CONFIG_SNIFFER_AUTH_DEV_BYPASS)
constexpr bool kBypass = true;
#else
constexpr bool kBypass = false;
#endif
}  // namespace auth_dev
