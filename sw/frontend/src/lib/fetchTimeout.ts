export const API_TIMEOUT_MS = 10_000;
export const SD_TIMEOUT_MS = 30_000;

/** Per-call timeout for a firmware URL. Null means none: file transfers and a format run as long as the card needs. */
export function timeoutFor(url: string): number | null {
  const path = url.split("?")[0];
  if (path.includes("/sd_card/file/") || path.endsWith("/sd_card/format")) return null;
  if (/\/(sd_card|recorder\/trips|dtc|req)\b/.test(path)) return SD_TIMEOUT_MS;
  return API_TIMEOUT_MS;
}

type FetchFn = (url: string, init?: RequestInit) => Promise<Response>;

/** Aborts a stalled request so its single-flight slot is released. A caller-supplied signal wins. */
export function timedFetch(fetchFn: FetchFn, url: string, init?: RequestInit, makeSignal: (ms: number) => AbortSignal = AbortSignal.timeout): Promise<Response> {
  const ms = init?.signal ? null : timeoutFor(url);
  return fetchFn(url, ms === null ? init : { ...init, signal: makeSignal(ms) });
}
