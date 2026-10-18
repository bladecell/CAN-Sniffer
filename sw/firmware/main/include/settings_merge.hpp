// SET-3: recovery when a settings section reports a missing required key. Pure, builds for the linux host tests.
#pragma once
#include "esp_err.h"

namespace settings_merge
{
// `get` fills `cfg` key by key and leaves a missing key untouched; it returns `notFound` when any required key was
// missing. Start from the defaults so a missing key keeps its default, save the merge so valid stored keys (credentials,
// pins) survive, and fall back to writing the plain defaults only when the merge cannot be saved. Returns get()'s result
// for the final stored state.
template <class Cfg, class Get, class Store, class SetDefault>
esp_err_t recoverMissing(Cfg& cfg, const Cfg& defaults, esp_err_t notFound, Get get, Store store, SetDefault setDefault)
{
    cfg = defaults;
    const bool merged = get(cfg) == notFound && store(cfg) == ESP_OK;
    if (!merged)
    {
        setDefault();
    }
    cfg = defaults;
    return get(cfg);
}
}  // namespace settings_merge
