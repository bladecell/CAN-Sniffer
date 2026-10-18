// Liveness check for the WebSocket. One ping at a time: while a pong is awaited no new ping is sent,
// so a slow server cannot stack pings. Timeout is well above the measured pong latency (0.5 to 5 s with
// several WebSocket clients), so a slow answer is not mistaken for a dead link.
export const PING_INTERVAL_MS = 3000;
export const PONG_TIMEOUT_MS = 6000;
/** A socket that is not open after this long is abandoned and retried. */
export const CONNECT_TIMEOUT_MS = 5000;

export interface HeartbeatTimers {
  setInterval(fn: () => void, ms: number): unknown;
  clearInterval(handle: unknown): void;
  setTimeout(fn: () => void, ms: number): unknown;
  clearTimeout(handle: unknown): void;
}

export interface HeartbeatOptions {
  /** Sends a ping. Returns false when the socket is not open; nothing is awaited then. */
  ping(): boolean;
  /** No pong within PONG_TIMEOUT_MS. */
  onTimeout(): void;
  /** Any pong arrived. Pongs are broadcast to every client, so one from another client also counts. */
  onPong(): void;
  timers?: HeartbeatTimers;
}

export interface Heartbeat {
  start(): void;
  stop(): void;
  pong(): void;
}

export function createHeartbeat(opts: HeartbeatOptions): Heartbeat {
  const t: HeartbeatTimers = opts.timers ?? globalThis;
  let interval: unknown = null;
  let timeout: unknown = null;
  let awaiting = false;

  function clearTimeoutHandle() {
    if (timeout !== null) t.clearTimeout(timeout);
    timeout = null;
  }

  return {
    start() {
      if (interval !== null) return;
      interval = t.setInterval(() => {
        if (awaiting || !opts.ping()) return;
        awaiting = true;
        timeout = t.setTimeout(() => {
          timeout = null;
          opts.onTimeout();
        }, PONG_TIMEOUT_MS);
      }, PING_INTERVAL_MS);
    },
    stop() {
      if (interval !== null) t.clearInterval(interval);
      interval = null;
      clearTimeoutHandle();
      awaiting = false;
    },
    pong() {
      awaiting = false;
      clearTimeoutHandle();
      opts.onPong();
    },
  };
}
