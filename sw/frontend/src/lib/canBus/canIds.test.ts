import { describe, expect, test } from "bun:test";
import { idColorIndex, idKey } from "./canIds";

describe("idColorIndex", () => {
  test("is stable and within 1..5", () => {
    for (const k of [0, 1, 0x316, 0x7ff, idKey(0x18daf110, true)]) {
      const i = idColorIndex(k);
      expect(i).toBe(idColorIndex(k));
      expect(i).toBeGreaterThanOrEqual(1);
      expect(i).toBeLessThanOrEqual(5);
    }
  });
  test("neighbouring ids get different colours", () => {
    expect(idColorIndex(0x316)).not.toBe(idColorIndex(0x317));
  });
});
