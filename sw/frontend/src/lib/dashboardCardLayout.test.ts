import { describe, expect, test } from "bun:test";
import { batteryBands, dtcVisible, faultChip, statusVisible } from "./dashboardCardLayout";

describe("statusVisible", () => {
  test("chip grid fits every item at 6 columns by 3 rows", () => {
    expect(statusVisible(7, 6, 3, "chips")).toEqual({ shown: 7, more: 0 });
  });
  test("chip grid at 4 columns by 3 rows keeps one cell for the more line", () => {
    expect(statusVisible(7, 4, 3, "chips")).toEqual({ shown: 5, more: 2 });
  });
  test("list at 4 by 5 shows all seven items", () => {
    expect(statusVisible(7, 4, 5, "list")).toEqual({ shown: 7, more: 0 });
  });
  test("list at 4 by 3 never draws past the card", () => {
    const r = statusVisible(7, 4, 3, "list");
    expect(r.shown + r.more).toBe(7);
    expect(r.more).toBeGreaterThan(0);
  });
});

describe("dtcVisible", () => {
  test("a 108 px list fits four rows: three codes and a more line for nine", () => {
    expect(dtcVisible(9, 108)).toEqual({ shown: 3, more: 6 });
  });
  test("a tall list fits nine codes", () => {
    expect(dtcVisible(9, 250)).toEqual({ shown: 9, more: 0 });
  });
  test("a list too short for one row still shows a more line", () => {
    expect(dtcVisible(9, 10)).toEqual({ shown: 0, more: 9 });
  });
  test("no codes shows none", () => {
    expect(dtcVisible(0, 100)).toEqual({ shown: 0, more: 0 });
  });
  test("a measured row pitch replaces the default", () => {
    expect(dtcVisible(9, 108, 36)).toEqual({ shown: 2, more: 7 });
    expect(dtcVisible(9, 108, 20)).toEqual({ shown: 4, more: 5 });
  });
  test("the shown rows leave room for the whole more line", () => {
    // 5 rows of 27.5 px pitch (24.5 + 3 gap) and a 22 px more line need 159.5 px; 150 px fits 4 rows plus the line.
    const v = dtcVisible(26, 150, 27.5, 22, 3);
    expect(v).toEqual({ shown: 4, more: 22 });
    expect(v.shown * 27.5 + 22).toBeLessThanOrEqual(150);
  });
  test("every code is shown when the rows alone fit, with no more line", () => {
    expect(dtcVisible(4, 4 * 27.5 - 3, 27.5, 22, 3)).toEqual({ shown: 4, more: 0 });
  });
  test("a bad measured pitch falls back to the default", () => {
    expect(dtcVisible(9, 108, 0)).toEqual(dtcVisible(9, 108));
    expect(dtcVisible(9, 108, Number.NaN)).toEqual(dtcVisible(9, 108));
  });
});

describe("faultChip", () => {
  test("zero faults reads No issues", () => {
    expect(faultChip(0)).toEqual({ text: "No issues", tone: "ok" });
  });
  test("faults read as a count with a warning tone", () => {
    expect(faultChip(3)).toEqual({ text: "3 faults", tone: "warn" });
    expect(faultChip(1)).toEqual({ text: "1 fault", tone: "warn" });
  });
  test("unread reads No data", () => {
    expect(faultChip(null)).toEqual({ text: "No data", tone: "muted" });
  });
});

describe("batteryBands", () => {
  test("five bands cover the scale from 0 to 1 with no gaps", () => {
    const b = batteryBands();
    expect(b).toHaveLength(5);
    expect(b[0].from).toBe(0);
    expect(b[4].to).toBe(1);
    for (let i = 1; i < 5; i++) expect(b[i].from).toBeCloseTo(b[i - 1].to, 10);
  });
  test("the low edge sits at 10.5 V on a 9.5 to 15.5 V scale", () => {
    expect(batteryBands()[1].from).toBeCloseTo(1 / 6, 10);
  });
});
