import { canStore } from "$lib/canStore.svelte";
import type { PidDataMetrics } from "$lib/types";
import { safeHexColor } from "$lib/color";
/**
 * Reactive selector that extracts both static configurations and runtime 
 * stream data for a given PID from the global canStore.
 */
export function usePidData(getPid: () => number): PidDataMetrics {
  // Invoke getPid() inside the derived macro so it registers as a dependency
  const definition = $derived(
    canStore.pidDefinitions?.find((c) => c.pid === getPid())
  );

  const formattedColor = $derived(() => safeHexColor(definition?.color));

  // Track the runtime stream dynamically using the evaluated PID
  const runtime = $derived(canStore.pids.get(getPid()));

  return {
    get label() { return definition?.name ?? "Metric"; },
    get id() { return String(definition?.id ?? ""); },
    get description() { return definition?.description ?? ""; },
    get unit() { return definition?.unit ?? "%"; },
    get icon() { return definition?.icon ?? "gear"; },
    get color() { return formattedColor(); },
    get min() { return definition?.minValue ?? 0; },
    get max() { return definition?.maxValue ?? 100; },
    get updateInterval() { return definition?.update_interval_ms ?? 1000; },
    get formula() { return definition?.formula ?? "NaN"; },

    // Live data channels
    get currentValue() { return runtime?.value ?? 0; },
    get isValid() { return runtime?.isValid ?? false; },
    get isSupported() { return runtime?.isSupported ?? false; },
    get displayValue() { return runtime?.isSupported ? runtime.value.toFixed(1) : "···"; }
  };
}

export { getModeLabel } from "./modeLabel";
