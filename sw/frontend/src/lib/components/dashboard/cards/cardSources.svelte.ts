// Lifetimes for the data a card reads. Called from a card's setup; the cleanup runs when the card unmounts.
import { untrack } from "svelte";
import { connectionStore } from "$lib/connectionStore.svelte";
import { onConnectionOpen } from "$lib/connectionEvents";
import { vehicleStore } from "$lib/vehicleStore.svelte";

/** Runs `load` on mount and again after every device connection. Reads inside `load` are not tracked. */
export function loadWhileMounted(load: () => unknown): void {
  $effect(() => {
    untrack(() => void load());
    return onConnectionOpen(() => untrack(() => void load()));
  });
}

/**
 * Sources for a card that shows PID values: the live stream stays on while the card is mounted,
 * the PID definitions (names, colours, ranges) and the OBD2 status (for "Polling off") are loaded.
 * The stream is not OBD polling: the dashboard never starts that.
 */
export function pidSources(): void {
  $effect(() => connectionStore.acquireStream());
  loadWhileMounted(() => {
    if (vehicleStore.pidDefinitions.length === 0) void vehicleStore.loadDefinitions();
    void vehicleStore.getObd2Status();
  });
  // A failed load (the PID map busy while OBD2 starts) used to wait for the next reconnect, so the card kept showing "PID n".
  // Retry while mounted until the list is in. Untracked, so a loaded list (even an empty one) ends the loop.
  $effect(() => {
    let stopped = false;
    let timer: ReturnType<typeof setTimeout> | undefined;
    const attempt = async (): Promise<void> => {
      if (stopped || (await vehicleStore.loadDefinitions())) return;
      timer = setTimeout(() => void attempt(), PID_RETRY_MS);
    };
    untrack(() => {
      if (vehicleStore.pidDefinitions.length === 0) void attempt();
    });
    return () => {
      stopped = true;
      clearTimeout(timer);
    };
  });
}
const PID_RETRY_MS = 5000;
