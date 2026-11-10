export const PID_RETRY_MS = 5000;

export interface PidRetryOptions {
  /** One load attempt. True when the definitions are in. */
  load(): Promise<boolean>;
  isLoaded(): boolean;
  setTimeout?(fn: () => void, ms: number): unknown;
  clearTimeout?(handle: unknown): void;
}

/**
 * One retry loop for the whole page, however many cards want the PID definitions.
 * Runs from the first acquire to the last release, and ends as soon as a load succeeds.
 */
export function createPidRetry(opts: PidRetryOptions) {
  const setT = opts.setTimeout ?? ((fn, ms) => setTimeout(fn, ms));
  const clearT = opts.clearTimeout ?? ((h) => clearTimeout(h as ReturnType<typeof setTimeout>));
  let users = 0;
  let timer: unknown = null;
  let running = false;

  const attempt = async (): Promise<void> => {
    timer = null;
    running = true;
    const ok = await opts.load();
    running = false;
    if (users > 0 && !ok) timer = setT(() => void attempt(), PID_RETRY_MS);
  };

  return {
    acquire(): () => void {
      users++;
      if (users === 1 && !running && !opts.isLoaded()) void attempt();
      let released = false;
      return () => {
        if (released) return;
        released = true;
        if (--users === 0 && timer !== null) {
          clearT(timer);
          timer = null;
        }
      };
    },
  };
}
