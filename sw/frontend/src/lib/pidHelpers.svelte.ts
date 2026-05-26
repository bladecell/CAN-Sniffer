import { canStore } from "$lib/canStore.svelte";

// Type definitions matching your canStore structure
export interface PidDefinition {
    pid: number | string;
    name: string;
    description?: string;
    unit?: string;
    icon?: string;
    color: number | string;
    minValue?: number;
    maxValue?: number;
}

export interface PidRuntimeData {
    value: number;
    valid: boolean;
    supported: boolean;
}

/**
 * Reactive selector that extracts both static configurations and runtime 
 * stream data for a given PID from the global canStore.
 */
export function usePidData(getPid: () => number | string) {
  // Invoke getPid() inside the derived macro so it registers as a dependency
  const definition = $derived(
    canStore.pidDefinitions?.find((c: any) => c.pid === getPid())
  );

  const formattedColor = $derived(() => {
    if (!definition?.color) return "#10b981";
    if (typeof definition.color === "number") {
      return `#${definition.color.toString(16).padStart(6, "0")}`;
    }
    return definition.color;
  });

  // Track the runtime stream dynamically using the evaluated PID
  const runtime = $derived(canStore.pids.get(getPid()));

  return {
    get label() { return definition?.name ?? "Metric"; },
    get description() { return definition?.description ?? ""; },
    get unit() { return definition?.unit ?? "%"; },
    get icon() { return definition?.icon ?? "gear"; },
    get color() { return formattedColor(); },
    get min() { return definition?.minValue ?? 0; },
    get max() { return definition?.maxValue ?? 100; },
    
    // Live data channels
    get currentValue() { return runtime?.value ?? 0; },
    get isValid() { return runtime?.valid ?? false; },
    get supported() { return runtime?.supported ?? false; },
    get displayValue() { return runtime?.supported ? runtime.value.toFixed(1) : "···"; }
  };
}