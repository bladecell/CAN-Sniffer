/**
 * Formula workbench maths: parse the test bytes and evaluate a formula against them.
 * Mirrors the firmware rules in docs/pid-config.md: A..D are the first four data bytes,
 * bytes past the end read as 0, ^ is power, getBit(value, bit) returns 0 above bit 31.
 * getPID reads another PID's live value, which the editor does not have, so it is refused here.
 */

export type ParsedBytes = { ok: true; bytes: number[] } | { ok: false; reason: "empty" | "invalid" };

export function parseTestBytes(text: string): ParsedBytes {
  const parts = text.trim().split(/[\s,]+/).filter(Boolean);
  if (parts.length === 0) return { ok: false, reason: "empty" };
  if (parts.length > 8) return { ok: false, reason: "invalid" };
  const bytes: number[] = [];
  for (const p of parts) {
    if (!/^[0-9a-fA-F]{1,2}$/.test(p)) return { ok: false, reason: "invalid" };
    bytes.push(parseInt(p, 16));
  }
  return { ok: true, bytes };
}

export type EvalResult = { ok: true; value: number } | { ok: false; reason: string };

const ALLOWED = new Set<string>(["A", "B", "C", "D", "getBit"]);

function getBit(value: number, bit: number): number {
  if (!(bit >= 0 && bit <= 31)) return 0;
  return ((Math.trunc(value) >>> 0) >>> Math.trunc(bit)) & 1;
}

export function evaluateFormula(formula: string, bytes: number[]): EvalResult {
  const src = formula.trim();
  if (!src) return { ok: false, reason: "Formula is empty" };
  // Hex literals first, so their letters are not read as names.
  const withoutHex = src.replace(/0[xX][0-9a-fA-F]+/g, "0");
  if (!/^[\w\s+\-*/().,%^]*$/.test(withoutHex)) return { ok: false, reason: "Unsupported character" };
  const names: string[] = withoutHex.match(/[A-Za-z_]\w*/g) ?? [];
  if (names.includes("getPID")) return { ok: false, reason: "getPID needs another PID's live value" };
  const unknown = names.find((n) => !ALLOWED.has(n));
  if (unknown) return { ok: false, reason: `Unknown name ${unknown}` };
  const b = (i: number) => bytes[i] ?? 0;
  try {
    const fn = new Function("A", "B", "C", "D", "getBit", `"use strict"; return (${src.replace(/\^/g, "**")});`);
    const value = fn(b(0), b(1), b(2), b(3), getBit);
    if (typeof value !== "number" || !Number.isFinite(value)) return { ok: false, reason: "Result is not a finite number" };
    return { ok: true, value };
  } catch {
    return { ok: false, reason: "Cannot parse the formula" };
  }
}
