import { describe, expect, test } from "bun:test";
import {
  arcPath,
  arcPoint,
  sweepPath,
  chartPaths,
  controlReason,
  emptyNote,
  fitFont,
  formatReading,
  levelOf,
  readState,
  uptimeLabel,
  wifiModeLabel,
  windowOf,
  zoneStops,
  worstLevel,
} from "./dashboardCardView";
import type { PidValue } from "./types";

const live = (value: number, over: Partial<PidValue> = {}): PidValue => ({
  value,
  lastUpdated: 1,
  rate: 1,
  isValid: true,
  isSupported: true,
  ...over,
});

describe("levelOf", () => {
  test("high is bad when warn is below critical", () => {
    const t = { warn: 100, critical: 118 };
    expect(levelOf(91, t)).toBe("ok");
    expect(levelOf(100, t)).toBe("warn");
    expect(levelOf(117.9, t)).toBe("warn");
    expect(levelOf(118, t)).toBe("crit");
  });

  test("low is bad when warn is above critical", () => {
    const t = { warn: 20, critical: 10 };
    expect(levelOf(64, t)).toBe("ok");
    expect(levelOf(20, t)).toBe("warn");
    expect(levelOf(10, t)).toBe("crit");
    expect(levelOf(5, t)).toBe("crit");
  });

  test("no level without thresholds or a value", () => {
    expect(levelOf(91, undefined)).toBeNull();
    expect(levelOf(null, { warn: 1, critical: 2 })).toBeNull();
  });
});

describe("readState", () => {
  test("a valid supported reading is a value", () => {
    expect(readState(live(91), true)).toBe("value");
    expect(readState(live(91), null)).toBe("value");
  });

  test("no reading, or an invalid one, is no data while polling", () => {
    expect(readState(undefined, true)).toBe("nodata");
    expect(readState(live(0, { isValid: false }), true)).toBe("nodata");
    expect(readState(live(0, { isSupported: false }), true)).toBe("nodata");
  });

  test("polling off wins over any stored reading", () => {
    expect(readState(live(91), false)).toBe("off");
    expect(readState(undefined, false)).toBe("off");
  });
});

describe("controlReason", () => {
  const ok = { linked: true, busUp: true, sdPresent: true, needsLogin: false, loggedIn: false };

  test("no reason when the control can run", () => {
    expect(controlReason(ok)).toBeNull();
    expect(controlReason({ ...ok, needsLogin: true, loggedIn: true })).toBeNull();
  });

  test("the reasons come in the order the card shows them", () => {
    expect(controlReason({ ...ok, linked: false })).toBe("Device not connected");
    expect(controlReason({ ...ok, busUp: false })).toBe("Bus disconnected");
    expect(controlReason({ ...ok, sdPresent: false })).toBe("No SD card");
    expect(controlReason({ ...ok, needsLogin: true })).toBe("Log in to control");
  });

  test("an unknown SD state does not block the control", () => {
    expect(controlReason({ ...ok, sdPresent: null })).toBeNull();
  });
});

describe("formatReading", () => {
  test("no value is a dash", () => {
    expect(formatReading(null)).toBe("—");
    expect(formatReading(Number.NaN)).toBe("—");
  });

  test("large values drop the decimals, small ones keep one", () => {
    expect(formatReading(2340)).toBe("2340");
    expect(formatReading(91)).toBe("91.0");
    expect(formatReading(14.1)).toBe("14.1");
  });
});

const sweepSpec = (start: number, sweep: number) => ({ start, sweep });

describe("gauge geometry", () => {
  // The arc runs clockwise from 135 degrees (bottom left) through 270 degrees to 45 degrees (bottom right).
  test("fraction 0 is the bottom left end and 1 the bottom right end", () => {
    const start = arcPoint(50, 50, 40, 0);
    expect(start.x).toBeCloseTo(50 - 40 * Math.SQRT1_2, 6);
    expect(start.y).toBeCloseTo(50 + 40 * Math.SQRT1_2, 6);
    const end = arcPoint(50, 50, 40, 1);
    expect(end.x).toBeCloseTo(50 + 40 * Math.SQRT1_2, 6);
    expect(end.y).toBeCloseTo(50 + 40 * Math.SQRT1_2, 6);
  });

  test("half the fraction is the top of the dial", () => {
    const mid = arcPoint(50, 50, 40, 0.5);
    expect(mid.x).toBeCloseTo(50, 6);
    expect(mid.y).toBeCloseTo(10, 6);
  });

  test("the arc path starts at the arc start and uses a large arc above half", () => {
    expect(arcPath(50, 50, 40, 0.25)).toMatch(/^M[\d.-]+,[\d.-]+ A40,40 0 0 1 /);
    expect(arcPath(50, 50, 40, 0.75)).toMatch(/ A40,40 0 1 1 /);
  });

  // 0.6 of 270 degrees is 162 degrees: a small arc. The flag used to switch at 0.5.
  test("the large-arc flag switches at 180 degrees of sweep, not at half the fraction", () => {
    expect(arcPath(50, 50, 40, 0.6)).toMatch(/ A40,40 0 0 1 /);
    expect(arcPath(50, 50, 40, 0.7)).toMatch(/ A40,40 0 1 1 /);
  });

  test("sweepPath draws a top semicircle from 180 to 360 degrees", () => {
    const start = sweepSpec(180, 180);
    const [, x0, y0, flag, x1, y1] = sweepPath(65, 62, 46, start, 1).match(/^M([-\d.]+),([-\d.]+) A46,46 0 (\d) 1 ([-\d.]+),([-\d.]+)$/) ?? [];
    expect([Number(x0), Number(y0), Number(x1), Number(y1)].map((n) => Math.round(n))).toEqual([19, 62, 111, 62]);
    expect(flag).toBe("0");
  });
});

describe("chartPaths flat scale", () => {
  test("constant values draw a mid-height line without NaN", () => {
    const { line } = chartPaths({ t: [0, 500, 1000], v: [5, 5, 5] }, { w: 100, h: 40, from: 0, to: 1000, lo: 5, hi: 5 });
    expect(line).not.toContain("NaN");
    expect(line).toContain(",20");
  });
});

describe("chartPaths", () => {
  // Box 100 x 50, time 0..1500 ms, values 0..20. Value 10 at t=500 is y 25, value 20 is y 0.
  const box = { w: 100, h: 50, from: 0, to: 1500, lo: 0, hi: 20 };

  test("line breaks at a gap and area closes each run on its own", () => {
    const { line, area } = chartPaths({ t: [0, 500, 1000, 1500], v: [0, 10, null, 20] }, box);
    expect(line.match(/M/g)?.length).toBe(2);
    expect(line.startsWith("M0.0,50.0")).toBe(true);
    expect(area.match(/Z/g)?.length).toBe(1);
    expect(area).not.toContain("100.0,0.0L");
  });

  test("values outside the scale clamp to the edges", () => {
    const { line } = chartPaths({ t: [0, 1500], v: [0, 30] }, box);
    expect(line).toContain("0.0,0.0");
  });

  test("no samples draw nothing", () => {
    expect(chartPaths({ t: [], v: [] }, box)).toEqual({ line: "", area: "" });
  });
});

describe("worstLevel", () => {
  test("crit beats warn beats ok, and no levels give null", () => {
    expect(worstLevel(["ok", "warn", null])).toBe("warn");
    expect(worstLevel(["warn", "crit", "ok"])).toBe("crit");
    expect(worstLevel([null, "ok"])).toBe("ok");
    expect(worstLevel([null])).toBeNull();
    expect(worstLevel([])).toBeNull();
  });
});

describe("status labels", () => {
  test("Wi-Fi mode numbers follow the settings enum (1 STA, 2 AP, 3 AP+STA)", () => {
    expect(wifiModeLabel(1)).toBe("Station");
    expect(wifiModeLabel(2)).toBe("Access point");
    expect(wifiModeLabel(3)).toBe("Access point and station");
    expect(wifiModeLabel(9)).toBe("Unknown");
  });

  test("uptime reads in seconds, minutes, then hours", () => {
    expect(uptimeLabel(45)).toBe("45 s");
    expect(uptimeLabel(600)).toBe("10 min");
    expect(uptimeLabel(3 * 3600 + 12 * 60 + 5)).toBe("3 h 12 min");
    expect(uptimeLabel(-1)).toBe("0 s");
  });
});

describe("windowOf", () => {
  test("keeps the samples within the span before the newest one", () => {
    const s = { t: [0, 30_000, 60_000, 90_000], v: [1, 2, null, 4] };
    expect(windowOf(s, 60_000)).toEqual({ t: [30_000, 60_000, 90_000], v: [2, null, 4] });
  });

  test("no samples is an empty window", () => {
    expect(windowOf({ t: [], v: [] }, 60_000)).toEqual({ t: [], v: [] });
  });
});

describe("zoneStops", () => {
  test("high values are bad when warn is below critical", () => {
    expect(zoneStops({ min: 0, max: 100 }, { warn: 80, critical: 90 })).toEqual({ warn: 0.8, critical: 0.9, lowBad: false });
  });

  test("low values are bad when warn is above critical", () => {
    expect(zoneStops({ min: 0, max: 100 }, { warn: 20, critical: 10 })).toEqual({ warn: 0.2, critical: 0.1, lowBad: true });
  });

  test("positions are fractions of the PID range, clamped to it", () => {
    expect(zoneStops({ min: 50, max: 150 }, { warn: 100, critical: 140 })).toEqual({ warn: 0.5, critical: 0.9, lowBad: false });
    expect(zoneStops({ min: 0, max: 100 }, { warn: -50, critical: 500 })).toEqual({ warn: 0, critical: 1, lowBad: false });
  });
});

describe("fitFont", () => {
  test("keeps the base size when the number fits", () => {
    expect(fitFont(23, 4, 76)).toBe(23);
  });
  test("shrinks a long number to the width", () => {
    const size = fitFont(25, 5, 64);
    expect(size).toBeLessThan(25);
    expect(size * 0.7 * 5).toBeCloseTo(64);
  });
  test("ignores an empty text or a bad width", () => {
    expect(fitFont(23, 0, 76)).toBe(23);
    expect(fitFont(23, 5, 0)).toBe(23);
    expect(fitFont(23, 5, Number.NaN)).toBe(23);
  });
});

describe("emptyNote", () => {
  test("says why a card has no reading, and nothing for a value", () => {
    expect(emptyNote("nodata")).toBe("No reading yet");
    expect(emptyNote("off")).toBe("Polling is off");
    expect(emptyNote("value")).toBe("");
  });
});
