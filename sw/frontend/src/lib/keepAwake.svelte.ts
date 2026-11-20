import { alertStore } from "./alertStore.svelte";
import { parseJsonResponse } from "./api";
import { firmwareApi } from "./firmwareApi.svelte";
import { createKeepAwakeWatch } from "./keepAwakeWatch";
import type { SystemStatus } from "./types";

/** One watch for the page, so the alert fires wherever the user is, not only on the Sleep tab. */
export const keepAwakeWatch = createKeepAwakeWatch({
  read: async () => (await parseJsonResponse<SystemStatus>(await firmwareApi.system())).keep_awake === true,
  notify: () => alertStore.add("Keep awake ended. The device can sleep again.", "info"),
});
