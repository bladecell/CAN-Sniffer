// Lifetimes for the data a card reads. Called from a card's setup; the cleanup runs when the card unmounts.
import { untrack } from "svelte";
import { connectionStore } from "$lib/connectionStore.svelte";
import { vehicleStore } from "$lib/vehicleStore.svelte";
import { createPidRetry } from "$lib/pidRetry";

/**
 * Runs `load` on mount. Reads inside `load` are not tracked.
 * It does not run again on reconnect: vehicleStore reloads the shared data once per connection, staggered.
 */
export function loadWhileMounted(load: () => unknown): void {
  $effect(() => {
    untrack(() => void load());
  });
}

// A failed load (the PID map busy while OBD2 starts) used to wait for the next reconnect, so cards kept showing "PID n".
// One loop for the page, however many cards are mounted. A loaded list (even an empty one) ends it.
const pidRetry = createPidRetry({
  load: () => vehicleStore.loadDefinitions(),
  isLoaded: () => vehicleStore.pidDefinitions.length > 0,
});

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
  $effect(() => untrack(() => pidRetry.acquire()));
}
