import { describe, expect, test } from "bun:test";
import {
  PRIORITY_LABELS,
  brief,
  intervalError,
  intervalToPos,
  levelToPriority,
  pageDown,
  pageUp,
  posToInterval,
  priorityEnabled,
  priorityToLevel,
  savePriority,
  stepDown,
  stepUp,
} from "./pollRate";

describe("interval slider position", () => {
  test("Static sits at the far left, 16 ms just after the stop, 10 s at the right end", () => {
    expect(intervalToPos(0)).toBe(0);
    expect(intervalToPos(16)).toBeCloseTo(0.12, 6);
    expect(intervalToPos(10000)).toBeCloseTo(1, 6);
  });

  test("the scale is logarithmic: 1 s sits near the middle", () => {
    expect(intervalToPos(1000)).toBeCloseTo(0.685, 2);
  });

  test("values above 10 s pin to the right end instead of leaving the track", () => {
    expect(intervalToPos(20000)).toBe(1);
  });
});

describe("interval slider pointer value", () => {
  test("the far left is Static, the stop is 16 ms", () => {
    expect(posToInterval(0)).toBe(0);
    expect(posToInterval(0.03)).toBe(0);
    expect(posToInterval(0.12)).toBe(16);
  });

  test("the right end is 10 s", () => {
    expect(posToInterval(1)).toBe(10000);
  });

  test("common values survive a round trip through the track", () => {
    for (const ms of [50, 100, 250, 500, 1000, 5000, 10000]) {
      expect(posToInterval(intervalToPos(ms))).toBe(ms);
    }
  });
});

describe("interval keyboard steps", () => {
  test("up from Static returns the last polled interval", () => {
    expect(stepUp(0, 250)).toBe(250);
    expect(stepUp(0, 512)).toBe(512);
  });

  test("steps use a finer grain at short intervals", () => {
    expect(stepUp(16, 512)).toBe(20);
    expect(stepUp(500, 512)).toBe(550);
    expect(stepDown(500)).toBe(450);
  });

  test("stepping down below 16 ms lands on Static", () => {
    expect(stepDown(16)).toBe(0);
    expect(stepDown(20)).toBe(0);
    expect(stepDown(0)).toBe(0);
  });

  test("stepping up from a loaded 1 to 15 ms lands on the 16 ms floor", () => {
    expect(stepUp(1, 512)).toBe(16);
    expect(stepUp(5, 512)).toBe(16);
  });

  test("stepping up stops at 10 s", () => {
    expect(stepUp(10000, 512)).toBe(10000);
  });

  test("page keys jump along 50, 100, 250, 500, 1000, 2000, 5000, 10000", () => {
    expect(pageUp(250)).toBe(500);
    expect(pageUp(10000)).toBe(10000);
    expect(pageDown(500)).toBe(250);
    expect(pageDown(50)).toBe(0);
    expect(pageDown(0)).toBe(0);
    expect(pageUp(0)).toBe(50);
  });
});

describe("interval readout", () => {
  test("brief shows Static, milliseconds, then seconds", () => {
    expect(brief(0)).toBe("Static");
    expect(brief(250)).toBe("250 ms");
    expect(brief(1000)).toBe("1 s");
    expect(brief(1500)).toBe("1.5 s");
    expect(brief(10000)).toBe("10 s");
  });
});

describe("interval error", () => {
  test("a loaded 1 to 15 ms interval is an error, Static and 16 ms and up are not", () => {
    expect(intervalError(5)).not.toBe("");
    expect(intervalError(15)).not.toBe("");
    expect(intervalError(0)).toBe("");
    expect(intervalError(16)).toBe("");
    expect(intervalError(20000)).toBe("");
  });

  test("a missing interval asks for one", () => {
    expect(intervalError(undefined)).not.toBe("");
  });
});

describe("priority levels", () => {
  test("stored 0, 1 and 2 all show the highest level", () => {
    expect([0, 1, 2].map(priorityToLevel)).toEqual([0, 0, 0]);
  });
  test("stored 3 to 6 show levels 2 to 5, and above 6 shows the lowest", () => {
    expect([3, 4, 5, 6, 7, 200].map(priorityToLevel)).toEqual([1, 2, 3, 4, 4, 4]);
  });
  test("a missing priority shows Normal", () => {
    expect(PRIORITY_LABELS[priorityToLevel(undefined)]).toBe("Normal");
  });
  test("levels save as 2 to 6", () => {
    expect([0, 1, 2, 3, 4].map(levelToPriority)).toEqual([2, 3, 4, 5, 6]);
  });
  test("save lifts a stored 0 or 1 to 2 and keeps everything else", () => {
    expect([0, 1, 2, 6, 200].map(savePriority)).toEqual([2, 2, 2, 6, 200]);
  });
  test("Static disables priority", () => {
    expect(priorityEnabled(0)).toBe(false);
    expect(priorityEnabled(16)).toBe(true);
  });
});
