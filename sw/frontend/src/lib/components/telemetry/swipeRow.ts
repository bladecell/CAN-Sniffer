/** Phone swipe on a list card. Attach to the sliding element (`.slide`); its parent is the `.row`. */

/** Past this distance on release, the gesture commits. Beyond it the card resists. */
const THRESHOLD = 112;
/** Movement before a swipe is decided. Vertical movement wins and leaves scrolling alone. */
const LOCK = 8;

export interface SwipeActions {
  left: () => void;
  right: () => void;
}

/** Touch and pen only, so a mouse click on the card is never a swipe. The card snaps back unless released past the threshold. */
export function swipeRow(actions: SwipeActions) {
  return (slide: HTMLElement) => {
    const row = slide.parentElement;
    if (!row) return;

    let id = -1;
    let startX = 0;
    let startY = 0;
    let width = 1;
    let active = false;
    let dx = 0;
    let justSwiped = false;

    const show = (x: number) => {
      dx = x;
      if (x === 0) delete row.dataset.dir;
      else row.dataset.dir = x < 0 ? "left" : "right";
      row.classList.toggle("armed", Math.abs(x) >= THRESHOLD);
      slide.style.transform = x === 0 ? "" : `translateX(${x}px)`;
    };

    const onDown = (e: PointerEvent) => {
      if (e.pointerType === "mouse" || e.button !== 0) return;
      id = e.pointerId;
      startX = e.clientX;
      startY = e.clientY;
      width = row.offsetWidth;
      active = false;
    };

    const onMove = (e: PointerEvent) => {
      if (e.pointerId !== id) return;
      const d = e.clientX - startX;
      const dy = e.clientY - startY;
      if (!active) {
        if (Math.abs(d) < LOCK && Math.abs(dy) < LOCK) return;
        if (Math.abs(d) <= Math.abs(dy) * 1.5) {
          id = -1;
          return;
        }
        active = true;
        slide.classList.add("drag");
        slide.setPointerCapture(e.pointerId);
      }
      const a = Math.abs(d);
      const eased = a > THRESHOLD ? THRESHOLD + (a - THRESHOLD) * 0.4 : a;
      show(Math.sign(d) * Math.min(eased, width * 0.6));
    };

    const onUp = (e: PointerEvent) => {
      if (e.pointerId !== id) return;
      const wasActive = active;
      const released = dx;
      id = -1;
      active = false;
      if (!wasActive) return;
      slide.classList.remove("drag");
      justSwiped = true;
      setTimeout(() => (justSwiped = false), 50);
      show(0);
      if (e.type === "pointerup" && Math.abs(released) >= THRESHOLD) {
        if (released < 0) actions.left();
        else actions.right();
      }
    };

    const onClick = (e: MouseEvent) => {
      if (!justSwiped) return;
      e.preventDefault();
      e.stopPropagation();
    };

    slide.addEventListener("pointerdown", onDown);
    slide.addEventListener("pointermove", onMove);
    slide.addEventListener("pointerup", onUp);
    slide.addEventListener("pointercancel", onUp);
    slide.addEventListener("click", onClick, true);
    return () => {
      slide.removeEventListener("pointerdown", onDown);
      slide.removeEventListener("pointermove", onMove);
      slide.removeEventListener("pointerup", onUp);
      slide.removeEventListener("pointercancel", onUp);
      slide.removeEventListener("click", onClick, true);
    };
  };
}
