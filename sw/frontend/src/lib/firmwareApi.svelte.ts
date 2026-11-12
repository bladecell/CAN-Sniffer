import { auth } from "./auth.svelte";
import { timedFetch } from "./fetchTimeout";
import { createFirmwareApi } from "./firmwareApi";

export const firmwareApi = createFirmwareApi({
  fetch: (url, init) => timedFetch(fetch, url, init),
  gatedFetch: (url, init) => auth.gatedFetch(url, init),
});
