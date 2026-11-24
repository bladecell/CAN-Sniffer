import { expect, test } from "bun:test";
import { tripBuffers } from "./tripBuffers";

test("a trip series becomes a chart buffer that keeps gaps and every sample", () => {
  const buffers = tripBuffers([{ pid: 12, t: [1000, 2000, 3000], v: [5, null, 7] }, { pid: 13, t: [], v: [] }]);
  const b = buffers.get(12);
  expect(b?.size).toBe(3);
  expect(b?.snapshot()).toEqual({ t: [1000, 2000, 3000], v: [5, null, 7] });
  expect(b?.snapshot(2000, 1).t).toEqual([1000, 2000, 3000]);
  expect(buffers.has(13)).toBe(false);
});
