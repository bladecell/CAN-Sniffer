import { describe, expect, test } from "bun:test";
import {
  isIdValidForMode,
  isLengthValidForPid,
  isModeValidForPid,
  isValidDescription,
  isValidFormula,
  isValidHex,
  isValidName,
} from "./pidInputRules";

describe("isValidHex", () => {
  test("requires 0x prefix and at most 4 hex digits", () => {
    expect(isValidHex("0x0C")).toBe(true);
    expect(isValidHex("0XFFFF")).toBe(true);
    expect(isValidHex("0c")).toBe(false);
    expect(isValidHex("0x")).toBe(false);
    expect(isValidHex("0x12345")).toBe(false);
    expect(isValidHex("0xG1")).toBe(false);
  });
});

describe("isValidName and isValidDescription", () => {
  test("names are required and limited to safe characters", () => {
    expect(isValidName("")).toBe(false);
    expect(isValidName("Engine RPM (rev/min)")).toBe(true);
    expect(isValidName("<script>")).toBe(false);
    expect(isValidName("x".repeat(65))).toBe(false);
  });
  test("descriptions may be empty and are capped at 256", () => {
    expect(isValidDescription("")).toBe(true);
    expect(isValidDescription("x".repeat(256))).toBe(true);
    expect(isValidDescription("x".repeat(257))).toBe(false);
  });
});

describe("mode, length and id rules follow the PID range", () => {
  test("mode: 0x00-0xFF is Current Data only, larger PIDs are not", () => {
    expect(isModeValidForPid("0x0C", "0x01")).toBe(true);
    expect(isModeValidForPid("0x0C", "0x22")).toBe(false);
    expect(isModeValidForPid("0x0100", "0x22")).toBe(true);
    expect(isModeValidForPid("0x0100", "")).toBe(false);
  });
  test("length: 2 bytes up to 0xFF, 3 bytes above, derived ignores length", () => {
    expect(isLengthValidForPid("0x0C", 2)).toBe(true);
    expect(isLengthValidForPid("0x0C", 3)).toBe(false);
    expect(isLengthValidForPid("0x0100", 3)).toBe(true);
    expect(isLengthValidForPid("0x0C", undefined, "0x45")).toBe(true);
    expect(isLengthValidForPid("0x0C", undefined)).toBe(false);
  });
  test("CAN id: functional 0x7DF for current/derived, 0x700-0x7FF for read by identifier", () => {
    expect(isIdValidForMode("0x01", "0x7DF")).toBe(true);
    expect(isIdValidForMode("0x01", "0x7E0")).toBe(false);
    expect(isIdValidForMode("0x22", "0x7E0")).toBe(true);
    expect(isIdValidForMode("0x22", "0x800")).toBe(false);
    expect(isIdValidForMode("0x22", "")).toBe(false);
  });
});

describe("isValidFormula", () => {
  test("current data and read by identifier allow A-D and math", () => {
    expect(isValidFormula("0x01", "(A*256+B)/4")).toBe(true);
    expect(isValidFormula("0x01", "A+getPID(0x0C)")).toBe(false);
    expect(isValidFormula("0x01", "A+")).toBe(false);
    expect(isValidFormula("0x01", "")).toBe(false);
  });
  test("derived data allows its functions, hex literals and math only", () => {
    expect(isValidFormula("0x45", "getPID(0x0C)*0.25")).toBe(true);
    expect(isValidFormula("0x45", "bitMask(0x0C,3)")).toBe(true);
    expect(isValidFormula("0x45", "A+1")).toBe(false);
    expect(isValidFormula("0x45", "foo(1)")).toBe(false);
  });
  test("overlong formulas are rejected", () => {
    expect(isValidFormula("0x01", "A+".repeat(600) + "A")).toBe(false);
  });
});
