// Tells the user when the keep-awake hold ends. The firmware reports only a boolean (docs/api.md, POST /system/keep_awake): the hold
// lasts 30 minutes and also ends when the device sleeps or restarts, so the page watches for it instead of reading a countdown.

export const KEEP_AWAKE_MS = 30 * 60 * 1000;
/** Margin after the 30 minutes, so the firmware has dropped the hold when the check reads it. */
const MARGIN_MS = 5000;

export interface KeepAwakeDeps {
  /** True while the device still holds. Rejects when it cannot be reached. */
  read: () => Promise<boolean>;
  notify: () => void;
  setTimer?: (fn: () => void, ms: number) => unknown;
  clearTimer?: (handle: unknown) => void;
}

export function createKeepAwakeWatch({ read, notify, setTimer = (fn, ms) => setTimeout(fn, ms), clearTimer = (h) => clearTimeout(h as ReturnType<typeof setTimeout>) }: KeepAwakeDeps) {
  let armed = false;
  let handle: unknown = null;

  const stop = () => {
    if (handle !== null) clearTimer(handle);
    handle = null;
    armed = false;
  };

  const end = () => {
    stop();
    notify();
  };

  function arm() {
    stop();
    armed = true;
    handle = setTimer(() => {
      handle = null;
      // Unreachable at the deadline: the device is asleep or restarting, which also ended the hold.
      read().then((active) => (active ? arm() : end()), end);
    }, KEEP_AWAKE_MS + MARGIN_MS);
  }

  return {
    arm,
    /** The user turned it off themselves: no alert. */
    disarm: stop,
    /** Feed every status read. Starts watching a hold found already active, and ends the watch with an alert when it drops. */
    seen(active: boolean) {
      if (active && !armed) arm();
      else if (!active && armed) end();
    },
    get armed() {
      return armed;
    },
  };
}
