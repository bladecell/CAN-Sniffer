import { describe, expect, test } from "bun:test";
import {
  cooldownSeconds,
  formatBytes,
  isTripCsvName,
  retryAfterSeconds,
  sidecarName,
  tripFileName,
  tripRows,
  validTrips,
} from "./recorderView";

describe("retryAfterSeconds", () => {
  test("uses the whole seconds the firmware sent", () => {
    expect(retryAfterSeconds("7")).toBe(7);
    expect(retryAfterSeconds("1")).toBe(1);
  });

  test("clamps to the 1..10 s window", () => {
    expect(retryAfterSeconds("0")).toBe(1);
    expect(retryAfterSeconds("-4")).toBe(1);
    expect(retryAfterSeconds("99")).toBe(10);
  });

  test("falls back to the full window when the header is missing or not a number", () => {
    expect(retryAfterSeconds(null)).toBe(10);
    expect(retryAfterSeconds("")).toBe(10);
    expect(retryAfterSeconds("Wed, 21 Oct 2026 07:28:00 GMT")).toBe(10);
  });
});

describe("cooldownSeconds", () => {
  test("rounds up while the window is open", () => {
    expect(cooldownSeconds(10_000, 0)).toBe(10);
    expect(cooldownSeconds(10_000, 1)).toBe(10);
    expect(cooldownSeconds(10_000, 9_001)).toBe(1);
  });

  test("is zero once the window has closed", () => {
    expect(cooldownSeconds(10_000, 10_000)).toBe(0);
    expect(cooldownSeconds(10_000, 25_000)).toBe(0);
  });
});

describe("file names", () => {
  test("pads the trip number to six digits", () => {
    expect(tripFileName(46)).toBe("trip_000046.csv");
    expect(tripFileName(999_999)).toBe("trip_999999.csv");
  });

  test("the sidecar shares the CSV stem", () => {
    expect(sidecarName("trip_000046.csv")).toBe("trip_000046.json");
  });
});

describe("formatBytes", () => {
  test("kilobytes below one million bytes, never zero", () => {
    expect(formatBytes(0)).toBe("1 kB");
    expect(formatBytes(2744)).toBe("3 kB");
    expect(formatBytes(999_000)).toBe("999 kB");
  });

  test("megabytes with one decimal", () => {
    expect(formatBytes(4_812_340)).toBe("4.8 MB");
  });
});

describe("tripRows", () => {
  const listed = [
    { index: 46, name: "trip_000046.csv", size: 4_000, sidecar: true },
    { index: 45, name: "trip_000045.csv", size: 1_000, sidecar: false },
  ];

  test("keeps the API order and marks nothing live when idle", () => {
    const rows = tripRows(listed, { state: "idle", trip: 46, fileBytes: 0 });
    expect(rows.map((r) => r.name)).toEqual(["trip_000046.csv", "trip_000045.csv"]);
    expect(rows.some((r) => r.live)).toBe(false);
  });

  test("adds the open trip first when it is not listed yet", () => {
    const rows = tripRows(listed, { state: "recording", trip: 47, fileBytes: 900 });
    expect(rows[0]).toEqual({ key: "trip_000047.csv", name: "trip_000047.csv", size: 900, sidecar: false, live: true });
    expect(rows).toHaveLength(3);
  });

  test("marks the listed open trip live and uses the live size", () => {
    const rows = tripRows(listed, { state: "stopping", trip: 46, fileBytes: 5_000 });
    expect(rows).toHaveLength(2);
    expect(rows[0]).toMatchObject({ name: "trip_000046.csv", size: 5_000, live: true });
    expect(rows[1].live).toBe(false);
  });

  test("an error state does not invent a live row", () => {
    const rows = tripRows(listed, { state: "error", trip: 47, fileBytes: 0 });
    expect(rows).toHaveLength(2);
  });
});

describe("isTripCsvName", () => {
  test("accepts the recorder's own names", () => {
    expect(isTripCsvName("trip_000046.csv")).toBe(true);
  });

  test("refuses traversal, separators, encoding, case and wrong extensions", () => {
    for (const name of [
      "../x",
      "../trip_000046.csv",
      "a/b",
      "trip_000046.csv/../../x",
      "%2e%2e%2fx",
      "trip_%30%30%30%30%34%36.csv",
      "trip_000046.json",
      "trip_000046.CSV",
      "trip_000046.csv ",
      "trip_00046.csv",
      "trip_0000460.csv",
      "",
    ]) {
      expect(isTripCsvName(name)).toBe(false);
    }
  });
});

describe("validTrips", () => {
  test("keeps only recorder trip names, in order", () => {
    const kept = validTrips([
      { name: "../x", size: 1 },
      { name: "trip_000046.csv", size: 2 },
      { name: "a/b", size: 3 },
      { name: "trip_000045.csv", size: 4 },
    ]);
    expect(kept.map((t) => t.name)).toEqual(["trip_000046.csv", "trip_000045.csv"]);
  });
});
