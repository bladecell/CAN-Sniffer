import { expect, test } from "bun:test";
import { jitter, reconnectDelay } from "./backoff";

test("jitter stays within 75..125% of the delay", () => {
  expect(jitter(4000, 0)).toBe(3000);
  expect(jitter(4000, 0.5)).toBe(4000);
  expect(jitter(4000, 0.999999)).toBe(5000);
  for (let n = 0; n < 12; n++) for (const r of [0, 0.3, 0.99]) {
    const base = reconnectDelay(n);
    expect(jitter(base, r)).toBeGreaterThanOrEqual(base * 0.75);
    expect(jitter(base, r)).toBeLessThanOrEqual(base * 1.25);
  }
});
