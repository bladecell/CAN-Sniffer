import { describe, expect, test } from "bun:test";
import { returnFocusTarget, trapTarget } from "./overlay";

describe("trapTarget", () => {
  test("returns -1 when the dialog has nothing to focus", () => {
    expect(trapTarget(0, -1, false)).toBe(-1);
    expect(trapTarget(0, -1, true)).toBe(-1);
  });

  test("lets the browser move focus between inner items", () => {
    expect(trapTarget(4, 1, false)).toBe(-1);
    expect(trapTarget(4, 2, true)).toBe(-1);
  });

  test("Tab on the last item wraps to the first", () => {
    expect(trapTarget(4, 3, false)).toBe(0);
  });

  test("Shift+Tab on the first item wraps to the last", () => {
    expect(trapTarget(4, 0, true)).toBe(3);
  });

  test("a focus outside the dialog is pulled back in from the matching end", () => {
    expect(trapTarget(4, -1, false)).toBe(0);
    expect(trapTarget(4, -1, true)).toBe(3);
  });

  test("a single item keeps focus on itself in both directions", () => {
    expect(trapTarget(1, 0, false)).toBe(0);
    expect(trapTarget(1, 0, true)).toBe(0);
  });
});

describe("returnFocusTarget", () => {
  const node = (connected: boolean) => ({ connected });
  const isConnected = (n: { connected: boolean }) => n.connected;

  test("returns the opener while it is still on the page", () => {
    const opener = node(true);
    const main = node(true);
    expect(returnFocusTarget(opener, isConnected, main)).toBe(opener);
  });

  test("falls back when the opener was removed while the dialog was open", () => {
    const main = node(true);
    expect(returnFocusTarget(node(false), isConnected, main)).toBe(main);
  });

  test("falls back when there was no opener", () => {
    const main = node(true);
    expect(returnFocusTarget(null, isConnected, main)).toBe(main);
  });

  test("returns null when there is neither an opener nor a fallback", () => {
    expect(returnFocusTarget(null, isConnected, null)).toBeNull();
  });
});
