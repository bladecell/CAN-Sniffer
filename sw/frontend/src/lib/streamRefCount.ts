/** `start` runs on 0->1, `stop` on 1->0. Each acquire returns a release that works once. */
export function createStreamRefCount(start: () => void, stop: () => void) {
  let n = 0;
  return {
    count: () => n,
    acquire() {
      if (++n === 1) start();
      let released = false;
      return () => {
        if (released) return;
        released = true;
        if (--n === 0) stop();
      };
    },
  };
}
