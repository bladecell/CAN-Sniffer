import { describe, expect, test } from "bun:test";
import { PIN_MAX, pinFromText, pressBackspace, pressDigit } from "./pinPad";

describe("pressDigit", () => {
  test("appends a digit", () => {
    expect(pressDigit("", "4")).toBe("4");
    expect(pressDigit("12", "3")).toBe("123");
  });

  test("stops at the maximum length", () => {
    expect(PIN_MAX).toBe(6);
    expect(pressDigit("123456", "7")).toBe("123456");
    expect(pressDigit("12", "3", 2)).toBe("12");
  });

  test("ignores anything that is not one digit", () => {
    expect(pressDigit("12", "a")).toBe("12");
    expect(pressDigit("12", "")).toBe("12");
    expect(pressDigit("1", "10")).toBe("1");
  });
});

describe("pressBackspace", () => {
  test("removes the last digit", () => {
    expect(pressBackspace("123")).toBe("12");
    expect(pressBackspace("1")).toBe("");
  });

  test("does nothing on an empty value", () => {
    expect(pressBackspace("")).toBe("");
  });
});

describe("pinFromText", () => {
  test("keeps only digits from pasted or typed text", () => {
    expect(pinFromText("12-34 56")).toBe("123456");
    expect(pinFromText("ab")).toBe("");
  });

  test("caps the length", () => {
    expect(pinFromText("12345678")).toBe("123456");
    expect(pinFromText("12345", 4)).toBe("1234");
  });
});
