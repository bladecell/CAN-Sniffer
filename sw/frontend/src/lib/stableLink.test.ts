import { describe, expect, test } from "bun:test";
import { backoffMayReset, STABLE_LINK_MS } from "./stableLink";

describe("backoffMayReset", () => {
  test("not before the link has been up STABLE_LINK_MS", () => {
    expect(backoffMayReset(1000, 1000 + STABLE_LINK_MS - 1)).toBe(false);
  });
  test("once it has", () => {
    expect(backoffMayReset(1000, 1000 + STABLE_LINK_MS)).toBe(true);
  });
  test("never when the link is not up", () => {
    expect(backoffMayReset(null, 99_999)).toBe(false);
  });
});
