import { auth } from "./auth.svelte";
import { createFirmwareApi } from "./firmwareApi";

export const firmwareApi = createFirmwareApi({
  fetch: (url, init) => fetch(url, init),
  gatedFetch: (url, init) => auth.gatedFetch(url, init),
});
