import { describe, expect, test } from "bun:test";
import { contourSvg } from "./contourBg";
import { contourOptions, circuitOptions, hexOptions, layersOptions, polygonOptions, meteorOptions, overlayOptions, shinyOptions, todaySeed, waveOptions } from "./background.svelte";
import { soundWaveSvg } from "./soundWave";
import { polygonNetSvg } from "./polygonNet";
import { hexGridSvg } from "./hexGrid";
import { waveLayersSvg } from "./layerWaves";
import { circuitBoardSvg, traces } from "./circuitBoard";
import { shinyOverlaySvg } from "./shinyOverlay";
import { meteorSvg } from "./meteor";
import { simpleShinySvg } from "./simpleShiny";

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
    expect(opts(1920, 1080, 0).levels).toBe(10);
    expect(opts(7680, 4320, 0).levels).toBe(10);
  });
  test("slider 0 is Random: seeded from the date, stable within a day, different across days", () => {
    const at = (d: string) => contourOptions(1920, 1080, 0, 55, 0.17, true, new Date(d));
    expect(todaySeed(new Date(2026, 10, 23))).toBe(20261123);
    expect(at("2026-11-23T10:00:00").seed).toBe(at("2026-11-23T18:00:00").seed);
    expect(contourSvg(at("2026-11-23T10:00:00"))).not.toBe(contourSvg(at("2026-11-24T10:00:00")));
    expect(opts(1920, 1080, 1).seed).toBe(7);
  });

  test("sound wave: deterministic, morphs with the slider, random follows the date", () => {
    const w = (v: number, d = "2026-11-23T10:00:00") => soundWaveSvg(waveOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(w(20)).toBe(w(20));
    expect(w(20)).not.toBe(w(60));
    expect(w(0)).not.toBe(w(0, "2026-11-24T10:00:00"));
    expect(w(30)).not.toMatch(/NaN|Infinity/);
    expect(w(30)).toContain("<path d=\"M");
  });

  test("simple shiny: deterministic, morphs, random follows the date, no NaN", () => {
    const sh = (v: number, d = "2026-11-23T10:00:00") => simpleShinySvg(shinyOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(sh(20)).toBe(sh(20));
    expect(sh(20)).not.toBe(sh(60));
    expect(sh(0)).not.toBe(sh(0, "2026-11-24T10:00:00"));
    expect(sh(30)).not.toMatch(/NaN|Infinity/);
    expect(sh(20)).not.toBe(sh(20.5));
    expect(sh(30).match(/<path /g)?.length).toBe(2);
  });

  test("meteor: deterministic, glides with the slider, random follows the date, no NaN", () => {
    const m = (v: number, d = "2026-11-23T10:00:00") => meteorSvg(meteorOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(m(20)).toBe(m(20));
    expect(m(20)).not.toBe(m(20.5));
    expect(m(0)).not.toBe(m(0, "2026-11-24T10:00:00"));
    expect(m(30)).not.toMatch(/NaN|Infinity/);
    expect(m(30).match(/<path /g)?.length).toBeGreaterThan(20);
  });

  test("shiny overlay: eight facets, deterministic, reshapes with the slider, random follows the date", () => {
    const so = (v: number, d = "2026-11-23T10:00:00") => shinyOverlaySvg(overlayOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(so(20)).toBe(so(20));
    expect(so(20)).not.toBe(so(60));
    expect(so(0)).not.toBe(so(0, "2026-11-24T10:00:00"));
    expect(so(30)).not.toMatch(/NaN|Infinity/);
    expect(so(30).match(/<path /g)?.length).toBe(8);
  });

  test("circuit board: deterministic, each slider value is a different board, random follows the date", () => {
    const cb = (v: number, d = "2026-11-23T10:00:00") => circuitBoardSvg(circuitOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(cb(20)).toBe(cb(20));
    expect(cb(20)).not.toBe(cb(21));
    expect(cb(0)).not.toBe(cb(0, "2026-11-24T10:00:00"));
    expect(cb(30)).not.toMatch(/NaN|Infinity/);
    expect(cb(30).match(/<path /g)?.length).toBe(2);
  });
  test("circuit traces stay on the grid, use each cell once and move one step at a time", () => {
    const seen = new Set<string>();
    for (const t of traces(40, 25, 5)) {
      expect(t.length).toBeGreaterThanOrEqual(3);
      expect(t.length).toBeLessThanOrEqual(20);
      t.forEach(([x, y], i) => {
        expect(x >= 0 && x < 40 && y >= 0 && y < 25).toBe(true);
        expect(seen.has(`${x},${y}`)).toBe(false);
        seen.add(`${x},${y}`);
        if (i > 0) expect(Math.max(Math.abs(x - t[i - 1][0]), Math.abs(y - t[i - 1][1]))).toBe(1);
      });
    }
    expect(seen.size).toBeGreaterThan(100);
  });
  test("circuit board survives a tiny or degenerate screen", () => {
    expect(circuitBoardSvg(circuitOptions(1, 1, 5, 55, 0.17, true))).toContain("<svg");
    expect(circuitBoardSvg(circuitOptions(0, 0, 5, 55, 0.17, false))).not.toMatch(/NaN|Infinity/);
  });

  test("wave layers: three layers, deterministic, glides with the slider, random follows the date", () => {
    const wl = (v: number, d = "2026-11-23T10:00:00") => waveLayersSvg(layersOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(wl(20)).toBe(wl(20));
    expect(wl(20)).not.toBe(wl(20.5));
    expect(wl(0)).not.toBe(wl(0, "2026-11-24T10:00:00"));
    expect(wl(30)).not.toMatch(/NaN|Infinity/);
    expect(wl(30).match(/<path /g)?.length).toBe(3);
    expect(waveLayersSvg(layersOptions(1, 1, 5, 55, 0.17, true))).not.toMatch(/NaN|Infinity/);
  });

  test("hexagon: deterministic, cells change with the slider, random follows the date, no NaN", () => {
    const hx = (v: number, d = "2026-11-23T10:00:00") => hexGridSvg(hexOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(hx(20)).toBe(hx(20));
    expect(hx(20)).not.toBe(hx(40));
    expect(hx(0)).not.toBe(hx(0, "2026-11-24T10:00:00"));
    expect(hx(30)).not.toMatch(/NaN|Infinity/);
    expect(hx(30).match(/<path /g)?.length).toBe(3);
    expect(hexGridSvg(hexOptions(1, 1, 5, 55, 0.17, true))).not.toMatch(/NaN|Infinity/);
  });

  test("polygon net: deterministic, glides with the slider, random follows the date, no NaN", () => {
    const pn = (v: number, d = "2026-11-23T10:00:00") => polygonNetSvg(polygonOptions(1920, 1080, v, 55, 0.17, true, new Date(d)));
    expect(pn(20)).toBe(pn(20));
    expect(pn(20)).not.toBe(pn(20.5));
    expect(pn(0)).not.toBe(pn(0, "2026-11-24T10:00:00"));
    expect(pn(30)).not.toMatch(/NaN|Infinity/);
    expect(pn(30).match(/<path /g)?.length).toBe(2);
    expect(polygonNetSvg(polygonOptions(1, 1, 5, 55, 0.17, true))).not.toMatch(/NaN|Infinity/);
    expect(polygonNetSvg(polygonOptions(0, 0, 5, 55, 0.17, false))).toContain("<svg");
  });
});
