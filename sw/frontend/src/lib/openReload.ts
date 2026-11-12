/** At most `limit` calls run at once; the rest wait in order. Every read the page makes goes through one, so mount-time loads and the open reload share the budget. */
export function createLimiter(limit: number): <T>(fn: () => Promise<T>) => Promise<T> {
  let active = 0;
  const waiting: Array<() => void> = [];
  const release = () => {
    active--;
    waiting.shift()?.();
  };
  return async (fn) => {
    if (active >= limit) await new Promise<void>((resume) => waiting.push(resume));
    active++;
    try {
      return await fn();
    } finally {
      release();
    }
  };
}

/** The last uptime seen and when this page read it. */
export interface UptimeSample {
  uptimeS: number;
  at: number;
}

const REBOOT_SLACK_S = 10;

/** True when the device's uptime is below what the last sample predicts: it restarted since, so its definitions may differ. */
export function rebooted(prev: UptimeSample | null, uptimeS: number, now: number): boolean {
  if (prev === null) return true;
  return uptimeS + REBOOT_SLACK_S < prev.uptimeS + (now - prev.at) / 1000;
}
