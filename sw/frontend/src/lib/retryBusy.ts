export const RETRY_CAP_MS = 10_000;
const DEFAULT_WAIT_MS = 1_000;

export interface RetryOptions {
  /** Total tries, first included. */
  attempts?: number;
  capMs?: number;
  sleep?(ms: number): Promise<void>;
}

/** Milliseconds from a `Retry-After` seconds header, clamped to the cap. Missing or unusable means one second. */
export function retryWaitMs(header: string | null, capMs = RETRY_CAP_MS): number {
  const seconds = header === null || header.trim() === "" ? NaN : Number(header);
  return Math.min(capMs, Number.isFinite(seconds) && seconds > 0 ? seconds * 1000 : DEFAULT_WAIT_MS);
}

/**
 * Calls again while the device answers 429 or 503, waiting its Retry-After (capped).
 * After the last try the busy response is returned, so the caller handles it like any other error.
 */
export async function retryBusy(call: () => Promise<Response>, opts: RetryOptions = {}): Promise<Response> {
  const attempts = opts.attempts ?? 3;
  const sleep = opts.sleep ?? ((ms: number) => new Promise<void>((r) => setTimeout(r, ms)));
  for (let n = 1; ; n++) {
    const res = await call();
    if ((res.status !== 429 && res.status !== 503) || n >= attempts) return res;
    await sleep(retryWaitMs(res.headers.get("Retry-After"), opts.capMs));
  }
}
