import { describe, expect, test } from "bun:test";
import { contourSvg } from "./contourBg";
import { contourOptions, todaySeed } from "./background.svelte";

const opts = (w: number, h: number, v: number) => contourOptions(w, h, v, 55, 0.17, true);

describe("contour background", () => {
  test("same inputs give the same SVG", () => {
    expect(contourSvg(opts(1920, 1080, 12))).toBe(contourSvg(opts(1920, 1080, 12)));
  });
  test("the variation slider changes the pattern", () => {
    expect(contourSvg(opts(800, 600, 10))).not.toBe(contourSvg(opts(800, 600, 60)));
  });
  test("output is a well-formed SVG with finite coordinates", () => {
    const svg = contourSvg(opts(800, 600, 30));
    expect(svg.startsWith("<svg")).toBe(true);
    expect(svg).toContain("<path d=\"M");
    expect(svg).not.toMatch(/NaN|Infinity/);
  });
  test("line count follows screen width within bounds", () => {
    expect(opts(320, 600, 0).levels).toBe(4);
    expect(opts(1920, 1080, 0).levels).toBe(11);
    expect(opts(7680, 4320, 0).levels).toBe(11);
  });
  test("slider 0 is Random: seeded from the date, stable within a day, different across days", () => {
    const at = (d: string) => contourOptions(1920, 1080, 0, 55, 0.17, true, new Date(d));
    expect(todaySeed(new Date(2026, 10, 23))).toBe(20261123);
    expect(at("2026-11-23T10:00:00").seed).toBe(at("2026-11-23T18:00:00").seed);
    expect(contourSvg(at("2026-11-23T10:00:00"))).not.toBe(contourSvg(at("2026-11-24T10:00:00")));
    expect(opts(1920, 1080, 1).seed).toBe(7);
  });
});
