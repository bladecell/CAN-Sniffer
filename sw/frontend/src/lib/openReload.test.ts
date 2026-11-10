import { describe, expect, test } from "bun:test";
import { runLimited } from "./openReload";

describe("runLimited", () => {
  test("never runs more than the limit at once, and runs every task", async () => {
    let active = 0;
    let peak = 0;
    let done = 0;
    const task = async () => {
      peak = Math.max(peak, ++active);
      await new Promise((r) => setTimeout(r, 1));
      active--;
      done++;
    };
    await runLimited(Array.from({ length: 7 }, () => task), 2);
    expect(peak).toBe(2);
    expect(done).toBe(7);
  });

  test("a failing task does not stop the rest", async () => {
    let done = 0;
    await runLimited([async () => { throw new Error("x"); }, async () => void done++, async () => void done++], 2);
    expect(done).toBe(2);
  });
});
