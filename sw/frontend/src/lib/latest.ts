/** Latest-call-wins guard for overlapping async work. */
export function createLatest() {
  let seq = 0;
  return {
    /** Returns a check that is true only until the next `begin()`. */
    begin() {
      const mine = ++seq;
      return () => mine === seq;
    },
  };
}
