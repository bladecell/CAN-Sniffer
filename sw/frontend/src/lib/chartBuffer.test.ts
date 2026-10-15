import { describe, expect, test } from "bun:test";
import { CHART_CAPACITY, createChartBuffer } from "./chartBuffer";

describe("chart buffer", () => {
  test("default capacity is the documented 10 000 samples per PID", () => {
    expect(CHART_CAPACITY).toBe(10_000);
    const b = createChartBuffer();
    for (let i = 0; i < 10_001; i++) b.push(i, i);
    expect(b.capacity).toBe(10_000);
    expect(b.size).toBe(10_000);
    expect(b.snapshot().t[0]).toBe(1);
  });

  test("keeps only the newest capacity samples, dropping the oldest", () => {
    const b = createChartBuffer(4);
    for (let i = 0; i < 10; i++) b.push(i * 100, i);
    expect(b.size).toBe(4);
    expect(b.snapshot().t).toEqual([600, 700, 800, 900]);
    expect(b.snapshot().v).toEqual([6, 7, 8, 9]);
  });

  test("snapshots are oldest first before and after the ring wraps", () => {
    const b = createChartBuffer(4);
    for (let i = 0; i < 3; i++) b.push(i * 100, i);
    expect(b.snapshot().t).toEqual([0, 100, 200]);
    b.push(300, 3);
    b.push(400, 4);
    expect(b.snapshot().t).toEqual([100, 200, 300, 400]);
    expect(b.lastTime()).toBe(400);
  });

  test("window query returns samples at or after fromT, inclusive, across a wrap", () => {
    const b = createChartBuffer(4);
    for (let i = 0; i < 10; i++) b.push(i * 100, i); // holds 600..900
    expect(b.snapshot(800).t).toEqual([800, 900]);
    expect(b.snapshot(800).v).toEqual([8, 9]);
    expect(b.snapshot(850).t).toEqual([900]);
    expect(b.snapshot(0).t).toEqual([600, 700, 800, 900]);
    expect(b.snapshot(1000).t).toEqual([]);
  });

  test("window query on a buffer that has not wrapped", () => {
    const b = createChartBuffer(8);
    for (let i = 0; i < 3; i++) b.push(i * 100, i);
    expect(b.snapshot(100).t).toEqual([100, 200]);
  });

  test("rejects out-of-order and non-finite timestamps", () => {
    const b = createChartBuffer(4);
    expect(b.push(100, 1)).toBe(true);
    expect(b.push(50, 2)).toBe(false);
    expect(b.push(Number.NaN, 3)).toBe(false);
    expect(b.push(Number.POSITIVE_INFINITY, 4)).toBe(false);
    expect(b.size).toBe(1);
  });

  test("equal timestamps are kept in arrival order", () => {
    const b = createChartBuffer(4);
    b.push(100, 1);
    b.push(100, 2);
    expect(b.snapshot().v).toEqual([1, 2]);
  });

  test("a gap longer than splitAfterMs inserts a null break just before the new sample", () => {
    const b = createChartBuffer(8);
    b.push(0, 1, 2000);
    b.push(5000, 2, 2000);
    expect(b.snapshot().t).toEqual([0, 4999, 5000]);
    expect(b.snapshot().v).toEqual([1, null, 2]);
  });

  test("no break when the gap is within splitAfterMs", () => {
    const b = createChartBuffer(8);
    b.push(0, 1, 2000);
    b.push(1500, 2, 2000);
    expect(b.snapshot().t).toEqual([0, 1500]);
    expect(b.snapshot().v).toEqual([1, 2]);
  });

  test("a NaN value is reported as null", () => {
    const b = createChartBuffer(4);
    b.push(0, Number.NaN);
    expect(b.snapshot().v).toEqual([null]);
  });

  test("the gap break never goes backwards in time", () => {
    const b = createChartBuffer(8);
    b.push(100, 1, 0);
    b.push(101, 2, 0);
    expect(b.snapshot().t).toEqual([100, 100, 101]);
    expect(b.snapshot().v).toEqual([1, null, 2]);
  });

  test("a backwards jump over 1 s is a device reboot: the buffer restarts", () => {
    const b = createChartBuffer(8);
    b.push(900_000, 1);
    b.push(901_000, 2);
    expect(b.push(500, 3)).toBe(true);
    expect(b.snapshot().t).toEqual([500]);
    expect(b.push(1500, 4)).toBe(true);
    expect(b.snapshot().t).toEqual([500, 1500]);
  });

  test("a small backwards step (within 1 s) is still dropped", () => {
    const b = createChartBuffer(8);
    b.push(5000, 1);
    expect(b.push(4500, 2)).toBe(false);
    expect(b.snapshot().t).toEqual([5000]);
  });

  test("implausible timestamps are rejected and normal samples continue", () => {
    const b = createChartBuffer(8);
    b.push(100, 1);
    expect(b.push(2 ** 40 + 1, 2)).toBe(false);
    expect(b.push(-5, 3)).toBe(false);
    expect(b.snapshot().t).toEqual([100]);
    expect(b.push(200, 4)).toBe(true);
    expect(b.snapshot().t).toEqual([100, 200]);
  });

  test("a hostile in-range timestamp is recovered by the next real samples", () => {
    const b = createChartBuffer(8);
    b.push(2 ** 39, 1);
    expect(b.push(100, 2)).toBe(true);
    expect(b.snapshot().t).toEqual([100]);
  });

  test("capacity 1 keeps only the newest sample", () => {
    const b = createChartBuffer(1);
    b.push(0, 1);
    b.push(100, 2);
    expect(b.snapshot()).toEqual({ t: [100], v: [2] });
  });

  test("capacity 1 with a gap keeps only the sample after the break", () => {
    const b = createChartBuffer(1);
    b.push(0, 1, 2000);
    b.push(5000, 2, 2000);
    expect(b.snapshot()).toEqual({ t: [5000], v: [2] });
  });

  test("a gap break evicts the oldest sample when the ring is full", () => {
    const b = createChartBuffer(3);
    b.push(0, 1, 2000);
    b.push(100, 2, 2000);
    b.push(5000, 3, 2000);
    expect(b.snapshot()).toEqual({ t: [100, 4999, 5000], v: [2, null, 3] });
  });
});
