import { describe, expect, test } from "bun:test";
import { EDITOR_MODES } from "./pidModes";

describe("EDITOR_MODES", () => {
  test("offers the three modes of the original editor, in order", () => {
    expect(EDITOR_MODES).toEqual([
      { label: "Current Data", value: "0x01" },
      { label: "Read Data By Identifier", value: "0x22" },
      { label: "Derived Data", value: "0x45" },
    ]);
  });

  test("does not offer 0x09 (Vehicle Info)", () => {
    expect(EDITOR_MODES.map((m) => m.value)).not.toContain("0x09");
  });
});
