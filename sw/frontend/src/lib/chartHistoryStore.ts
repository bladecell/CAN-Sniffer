import { createChartHistory } from "./chartHistory";
import { connectionStore } from "./connectionStore.svelte";
import { vehicleStore } from "./vehicleStore.svelte";

/** Time window the PID card chart shows. Chart history is capped by count, see chartBuffer.ts. */
export const MAX_HISTORY_SEC = 60;

export const chartHistoryStore = createChartHistory({
  subscribeMessages: (cb) => connectionStore.subscribe(cb),
  definitions: () => vehicleStore.pidDefinitions,
});
