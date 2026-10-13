/** Returns a function that is true at most once per `intervalMs`. */
export function createThrottle(intervalMs: number, now: () => number = Date.now) {
  let last = -Infinity;
  return () => {
    const t = now();
    if (t - last < intervalMs) return false;
    last = t;
    return true;
  };
}
