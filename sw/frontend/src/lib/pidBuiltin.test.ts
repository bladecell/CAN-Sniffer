import { describe, expect, test } from "bun:test";
import { BUILTIN_REFUSAL, builtinRefusal } from "./pidBuiltin";
import { parseJsonResponse } from "./api";

const refusal = (status: number, body: unknown) =>
  new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json" } });

describe("builtinRefusal", () => {
  test("a 403 PID_BUILTIN reply maps to the clear message, not the firmware reason", async () => {
    const error = await parseJsonResponse(refusal(403, { status: "error", reason: "core PID 0x0C is built in", code: "PID_BUILTIN" })).catch((e) => e);
    expect(builtinRefusal(error)).toBe(BUILTIN_REFUSAL);
  });

  test("other firmware errors are not mapped", async () => {
    const error = await parseJsonResponse(refusal(422, { status: "error", reason: "range", code: "INVALID" })).catch((e) => e);
    expect(builtinRefusal(error)).toBeNull();
  });

  test("a plain error or a non-error is not mapped", () => {
    expect(builtinRefusal(new Error("boom"))).toBeNull();
    expect(builtinRefusal("PID_BUILTIN")).toBeNull();
  });

  test("the refusal names the built-in PIDs and asks the user to revert", () => {
    expect(BUILTIN_REFUSAL).toMatch(/built-in/i);
  });
});
