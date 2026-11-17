import { describe, expect, test } from "bun:test";
import { captureEndedText, formatBytes, validCaptureName } from "./canCapture";

describe("capture names", () => {
  test("only cap_NNNNNN.csv passes", () => {
    expect(validCaptureName("cap_000001.csv")).toBe(true);
    expect(validCaptureName("cap_1.csv")).toBe(false);
    expect(validCaptureName("../cap_000001.csv")).toBe(false);
    expect(validCaptureName("cap_000001.csv?x")).toBe(false);
    expect(validCaptureName("cap_000001.csv/")).toBe(false);
  });
});

describe("labels", () => {
  test("sizes", () => {
    expect(formatBytes(512)).toBe("512 B");
    expect(formatBytes(12.4 * 1024 * 1024)).toBe("12.4 MB");
    expect(formatBytes(2048)).toBe("2.0 KB");
  });

  test("ended and card errors read as text; an unknown value is shown as is", () => {
    expect(captureEndedText("limit", undefined)).toBe("Reached the 8 MB limit");
    expect(captureEndedText(undefined, "NO_SD")).toBe("No SD card");
    expect(captureEndedText("novel", undefined)).toBe("novel");
    expect(captureEndedText(undefined, undefined)).toBe("");
  });
});
