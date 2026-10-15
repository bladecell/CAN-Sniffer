import { canStore } from "./canStore.svelte";
import { createChartHistory } from "./chartHistory";

/** Time window the PID card chart shows. Chart history is capped by count, see chartBuffer.ts. */
export const MAX_HISTORY_SEC = 60;

export const chartHistoryStore = createChartHistory({
  subscribeMessages: (cb) => canStore.subscribe(cb),
  definitions: () => canStore.pidDefinitions,
});
