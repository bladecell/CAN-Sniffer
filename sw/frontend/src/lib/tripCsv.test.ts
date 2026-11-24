import { describe, expect, test } from "bun:test";
import { parseTripCsv, readTextLimited, TRIP_MAX_BYTES } from "./tripCsv";

const H = "t_ms,pid,value,valid\n";

describe("parseTripCsv", () => {
  test("groups rows by PID with the trip span", () => {
    const r = parseTripCsv(H + "1000,12,800.5,1\n1000,13,40,1\n2000,12,950,1\n");
    expect(r.ok).toBe(true);
    if (!r.ok) return;
    expect(r.startMs).toBe(1000);
    expect(r.endMs).toBe(2000);
    expect(r.rows).toBe(3);
    expect(r.series.map((s) => s.pid)).toEqual([12, 13]);
    expect(r.series[0].t).toEqual([1000, 2000]);
    expect(r.series[0].v).toEqual([800.5, 950]);
  });

  test("invalid rows and empty values are gaps, repeated gaps collapse", () => {
    const r = parseTripCsv(H + "1000,12,5,1\n2000,12,,0\n3000,12,,0\n4000,12,,1\n5000,12,7,1\n");
    if (!r.ok) throw new Error("not ok");
    expect(r.series[0].v).toEqual([5, null, 7]);
    expect(r.series[0].t).toEqual([1000, 2000, 5000]);
  });

  test("accepts CRLF, a BOM and exponent values", () => {
    const r = parseTripCsv("﻿t_ms,pid,value,valid\r\n1,12,1.2345678e+08,1\r\n2,12,-3e-05,1\r\n");
    if (!r.ok) throw new Error("not ok");
    expect(r.series[0].v).toEqual([123456780, -0.00003]);
  });

  test("skips malformed rows and counts them", () => {
    const r = parseTripCsv(H + "x,12,1,1\n1,99999,1,1\n1,12,abc,1\n1,12,1,2\n1,12,1\n,,,\n1,12,1,1,9\n2,12,Infinity,1\n3,12,4,1\n4,12,5,1\n4,1");
    if (!r.ok) throw new Error("not ok");
    expect(r.series[0].v).toEqual([4, 5]);
    expect(r.series[0].t).toEqual([3, 4]);
    expect(r.skipped).toBe(9);
  });

  test("out-of-order and negative times are skipped per PID", () => {
    const r = parseTripCsv(H + "5000,12,1,1\n4000,12,2,1\n-5,13,1,1\n6000,12,3,1\n6000,13,1,1\n");
    if (!r.ok) throw new Error("not ok");
    expect(r.series[0].t).toEqual([5000, 6000]);
    expect(r.series[1].t).toEqual([6000]);
    expect(r.skipped).toBe(2);
  });

  test("a wrong header or no data rows is an error", () => {
    expect(parseTripCsv("")).toEqual({ ok: false, error: "empty" });
    expect(parseTripCsv("a,b,c\n1,2,3\n")).toEqual({ ok: false, error: "bad-header" });
    expect(parseTripCsv(H)).toEqual({ ok: false, error: "no-data" });
    expect(parseTripCsv(H + "junk\n")).toEqual({ ok: false, error: "no-data" });
  });

  test("row and PID limits are enforced and reported", () => {
    const rows = Array.from({ length: 10 }, (_, i) => `${i},12,${i},1`).join("\n");
    const r = parseTripCsv(H + rows, { maxRows: 4, maxPids: 2 });
    if (!r.ok) throw new Error("not ok");
    expect(r.rows).toBe(4);
    expect(r.truncated).toBe(true);
    const p = parseTripCsv(H + "1,1,1,1\n1,2,1,1\n1,3,1,1\n2,1,2,1\n", { maxPids: 2 });
    if (!p.ok) throw new Error("not ok");
    expect(p.series.map((s) => s.pid)).toEqual([1, 2]);
    expect(p.truncated).toBe(true);
    expect(p.skipped).toBe(1);
  });
});

describe("readTextLimited", () => {
  test("reads a body within the limit", async () => {
    expect(await readTextLimited(new Response("abc"), 10)).toBe("abc");
  });

  test("refuses a body over the limit, from the header or while streaming", async () => {
    expect(readTextLimited(new Response("abcdef"), 5)).rejects.toThrow("too large");
    const streamed = new Response(new ReadableStream({ start(c) { c.enqueue(new TextEncoder().encode("abc")); c.enqueue(new TextEncoder().encode("def")); c.close(); } }));
    expect(readTextLimited(streamed, 5)).rejects.toThrow("too large");
    const lying = new Response("abcdef", { headers: { "Content-Length": "999999999" } });
    expect(readTextLimited(lying, 100)).rejects.toThrow("too large");
  });

  test("the default limit holds a full 8 MiB trip file", () => {
    expect(TRIP_MAX_BYTES).toBeGreaterThan(8 * 1024 * 1024);
  });
});
