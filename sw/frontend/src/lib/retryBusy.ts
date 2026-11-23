export const RETRY_CAP_MS = 10_000;
const DEFAULT_WAIT_MS = 1_000;

export interface RetryOptions {
  /** Total tries, first included. */
  attempts?: number;
  sleep?(ms: number): Promise<void>;
}

/**
 * Wait before retry number `retry` (1 for the first). The Retry-After seconds are the base and double on each retry, clamped to the cap.
 * A missing or unusable header uses one second as the base.
 */
export function retryWaitMs(header: string | null, retry: number, capMs = RETRY_CAP_MS): number {
  const seconds = header === null || header.trim() === "" ? NaN : Number(header);
  const base = Number.isFinite(seconds) && seconds > 0 ? seconds * 1000 : DEFAULT_WAIT_MS;
  return Math.min(capMs, base * 2 ** (retry - 1));
}

/**
 * Calls again while the device answers 503 (busy, low memory), waiting its Retry-After with capped exponential backoff.
 * After the last try the busy response is returned, so the caller handles it like any other error.
 * Callers must only pass idempotent requests.
 */
export async function retryBusy<R extends Response>(call: () => Promise<R>, opts: RetryOptions = {}): Promise<R> {
  const attempts = opts.attempts ?? 3;
  const sleep = opts.sleep ?? ((ms: number) => new Promise<void>((r) => setTimeout(r, ms)));
  for (let n = 1; ; n++) {
    const res = await call();
    if (res.status !== 503 || n >= attempts) return res;
    await sleep(retryWaitMs(res.headers.get("Retry-After"), n));
  }
}
