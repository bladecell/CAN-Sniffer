import { describe, expect, test } from "bun:test";
import { parseWsFrame } from "./wsFrames";

function pidFrame(len = 20) {
  const b = new Uint8Array(len);
  const v = new DataView(b.buffer);
  b[0] = 0x02;
  b[1] = 18;
  if (len >= 20) {
    v.setUint32(2, 0x0c, true);
    v.setFloat32(6, 1500.5, true);
    v.setUint32(10, 1234, true);
    v.setUint32(14, 256, true);
    b[18] = 1;
    b[19] = 1;
  }
  return b.buffer;
}

function statusFrame(len = 12) {
  const b = new Uint8Array(len);
  const v = new DataView(b.buffer);
  b[0] = 0x03;
  b[1] = 10;
  if (len >= 12) {
    b[2] = 3;
    v.setFloat32(3, 0.5, true);
    v.setFloat32(7, 12.5, true);
    b[11] = 1;
  }
  return b.buffer;
}

describe("parseWsFrame", () => {
  test("empty frame returns null instead of throwing", () => {
    expect(parseWsFrame(new ArrayBuffer(0))).toBeNull();
  });
  test("non-binary data returns null", () => {
    expect(parseWsFrame("hello")).toBeNull();
    expect(parseWsFrame(null)).toBeNull();
  });
  test("parses a 20 byte PID packet", () => {
    expect(parseWsFrame(pidFrame())).toEqual({
      kind: "pid", pid: 0x0c, value: 1500.5, lastUpdated: 1234, rate: 256, isSupported: true, isValid: true,
    });
  });
  test("rejects PID packets that are short or long", () => {
    expect(parseWsFrame(pidFrame(19))).toBeNull();
    expect(parseWsFrame(pidFrame(21))).toBeNull();
    expect(parseWsFrame(pidFrame(2))).toBeNull();
  });
  test("rejects a PID packet whose length byte disagrees", () => {
    const b = new Uint8Array(pidFrame());
    b[1] = 17;
    expect(parseWsFrame(b.buffer)).toBeNull();
  });
  test("parses a 12 byte CAN status packet", () => {
    expect(parseWsFrame(statusFrame())).toEqual({
      kind: "canStatus", state: 3, utilization: 0.5, batteryVoltage: 12.5, busConnected: true,
    });
  });
  test("rejects CAN status packets of the wrong size", () => {
    expect(parseWsFrame(statusFrame(11))).toBeNull();
    expect(parseWsFrame(statusFrame(13))).toBeNull();
    expect(parseWsFrame(statusFrame(1))).toBeNull();
  });
  test("pong is one byte", () => {
    expect(parseWsFrame(new Uint8Array([0xa3]).buffer)).toEqual({ kind: "pong" });
    expect(parseWsFrame(new Uint8Array([0xa3, 0]).buffer)).toBeNull();
  });
  test("unknown type returns null", () => {
    expect(parseWsFrame(new Uint8Array([0x55, 1, 2]).buffer)).toBeNull();
  });
});
