import { describe, expect, test } from "bun:test";
import { parseStoredLayout } from "./layoutStorage";
import type { BatteryGridItem, PidGridItem } from "./types";

const base = { id: "b1", cardType: "battery", w: 12, h: 8 };

describe("parseStoredLayout: car battery cards", () => {
  test("an old placeholder battery item (displayMode default) loads as the glyph mode", () => {
    const [item] = parseStoredLayout(JSON.stringify([{ ...base, displayMode: "default" }]));
    expect(item).toMatchObject({ cardType: "battery", displayMode: "glyph" });
  });

  test("a saved gauge mode keeps its gauge style", () => {
    const [item] = parseStoredLayout(
      JSON.stringify([{ ...base, displayMode: "gauge", gaugeStyle: "half" }]),
    );
    expect((item as BatteryGridItem).gaugeStyle).toBe("half");
    expect(item.displayMode).toBe("gauge");
  });

  test("an unknown gauge style is dropped, the card is kept", () => {
    const items = parseStoredLayout(
      JSON.stringify([{ ...base, displayMode: "gauge", gaugeStyle: "hologram" }]),
    );
    expect(items).toHaveLength(1);
    expect((items[0] as BatteryGridItem).gaugeStyle).toBeUndefined();
  });

  test("a bar card keeps its bar style and an unknown one is dropped", () => {
    const [solid] = parseStoredLayout(JSON.stringify([{ ...base, displayMode: "bar", barStyle: "solid" }]));
    expect((solid as BatteryGridItem).barStyle).toBe("solid");
    const [bad] = parseStoredLayout(JSON.stringify([{ ...base, id: "b2", displayMode: "bar", barStyle: 3 }]));
    expect((bad as BatteryGridItem).barStyle).toBeUndefined();
  });

  test("an unknown mode falls back to glyph", () => {
    const [item] = parseStoredLayout(JSON.stringify([{ ...base, displayMode: "pie" }]));
    expect(item.displayMode).toBe("glyph");
  });

  test("existing PID, DTC and overview cards still load", () => {
    const items = parseStoredLayout(
      JSON.stringify([
        { id: "p", cardType: "pid", pid: 12, displayMode: "gauge", w: 12, h: 12 },
        { id: "d", cardType: "dtcs", displayMode: "compact", w: 14, h: 8 },
        { id: "o", cardType: "overview", displayMode: "default", w: 17, h: 11, pids: [1, 2, 3], color: "#123456" },
      ]),
    );
    expect(items.map((i) => i.id)).toEqual(["p", "d", "o"]);
  });

  test("an old PID bar (no barStyle) loads with no style, which means solid", () => {
    const [item] = parseStoredLayout(JSON.stringify([{ id: "p", cardType: "pid", pid: 12, displayMode: "bar", w: 10, h: 8 }]));
    expect((item as PidGridItem).barStyle).toBeUndefined();
  });

  test("a PID bar keeps zones, and an unknown bar style is dropped without losing the card", () => {
    const items = parseStoredLayout(
      JSON.stringify([
        { id: "p1", cardType: "pid", pid: 12, displayMode: "bar", barStyle: "zones", w: 10, h: 8 },
        { id: "p2", cardType: "pid", pid: 12, displayMode: "bar", barStyle: "neon", w: 10, h: 8 },
      ]),
    );
    expect(items).toHaveLength(2);
    expect((items[0] as PidGridItem).barStyle).toBe("zones");
    expect((items[1] as PidGridItem).barStyle).toBeUndefined();
  });
});
