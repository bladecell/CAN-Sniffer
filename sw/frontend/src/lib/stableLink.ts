/** A link must stay up this long before the reconnect backoff resets, so a socket that opens and dies keeps backing off. */
export const STABLE_LINK_MS = 10_000;

export const backoffMayReset = (upSince: number | null, now: number): boolean =>
  upSince !== null && now - upSince >= STABLE_LINK_MS;
