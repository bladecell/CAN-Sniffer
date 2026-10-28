import { describe, expect, test } from "bun:test";
import { UNITS, unitsWith } from "./units";

const REQUIRED = ["%", "kPa", "Pa", "rpm", "km/h", "ratio", "count", "km", "V", "s", "min", "g/s", "°", "°C", "L/h", "L"];

describe("UNITS", () => {
  test("offers every unit from the owner's list", () => {
    for (const unit of REQUIRED) expect(UNITS).toContain(unit);
  });

  test("keeps the units that were already there, and no unit first", () => {
    for (const unit of ["mph", "°F", "A", "bar"]) expect(UNITS).toContain(unit);
    expect(UNITS[0]).toBe("");
  });

  test("drops the units the firmware no longer uses", () => {
    expect(UNITS).not.toContain("grams/sec");
    expect(UNITS).not.toContain("° before TDC");
    expect(UNITS).not.toContain("seconds");
    expect(UNITS).not.toContain("minutes");
  });

  test("values are unique", () => {
    expect(new Set(UNITS).size).toBe(UNITS.length);
  });
});

describe("unitsWith", () => {
  test("a unit in the list, or none, adds nothing", () => {
    expect(unitsWith("rpm")).toBe(UNITS);
    expect(unitsWith("")).toBe(UNITS);
  });

  test("a saved unit outside the list is added at the end", () => {
    expect(unitsWith("ohm")).toEqual([...UNITS, "ohm"]);
  });
});
