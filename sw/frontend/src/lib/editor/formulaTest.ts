/**
 * Formula workbench maths: parse the test bytes and evaluate a formula against them.
 * Mirrors the firmware rules in docs/pid-config.md: A..D are the first four data bytes,
 * bytes past the end read as 0, ^ is power, getBit(value, bit) returns 0 above bit 31.
 * getPID reads another PID's live value, which the editor does not have, so it is refused here.
 *
 * The CAN Bus rules (lib/canBus) use A..H, the eight data bytes of a frame. They compile once with
 * compileFormula and run per frame, so the parse and the function build are not repeated.
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

export const PID_LETTERS = ["A", "B", "C", "D"] as const;
export const FRAME_LETTERS = ["A", "B", "C", "D", "E", "F", "G", "H"] as const;

export type CompiledFormula = { ok: true; run: (bytes: ArrayLike<number>) => EvalResult } | { ok: false; reason: string };

function getBit(value: number, bit: number): number {
  if (!(bit >= 0 && bit <= 31)) return 0;
  return ((Math.trunc(value) >>> 0) >>> Math.trunc(bit)) & 1;
}

/** Checks the text and builds the function once. A syntax error or a refused name gives `ok: false`; nothing runs. */
export function compileFormula(formula: string, letters: readonly string[] = PID_LETTERS): CompiledFormula {
  const src = formula.trim();
  if (!src) return { ok: false, reason: "Formula is empty" };
  // Hex literals first, so their letters are not read as names.
  const withoutHex = src.replace(/0[xX][0-9a-fA-F]+/g, "0");
  if (!/^[\w\s+\-*/().,%^]*$/.test(withoutHex)) return { ok: false, reason: "Unsupported character" };
  const names: string[] = withoutHex.match(/[A-Za-z_]\w*/g) ?? [];
  if (names.includes("getPID")) return { ok: false, reason: "getPID needs another PID's live value" };
  const allowed = new Set<string>([...letters, "getBit"]);
  const unknown = names.find((n) => !allowed.has(n));
  if (unknown) return { ok: false, reason: `Unknown name ${unknown}` };
  let fn: (...args: unknown[]) => unknown;
  try {
    fn = new Function(...letters, "getBit", `"use strict"; return (${src.replace(/\^/g, "**")});`) as (...args: unknown[]) => unknown;
  } catch {
    return { ok: false, reason: "Cannot parse the formula" };
  }
  return {
    ok: true,
    run(bytes) {
      const args = letters.map((_, i) => bytes[i] ?? 0);
      try {
        const value = fn(...args, getBit);
        if (typeof value !== "number" || !Number.isFinite(value)) return { ok: false, reason: "Result is not a finite number" };
        return { ok: true, value };
      } catch {
        return { ok: false, reason: "Cannot parse the formula" };
      }
    },
  };
}

export function evaluateFormula(formula: string, bytes: number[]): EvalResult {
  const compiled = compileFormula(formula);
  return compiled.ok ? compiled.run(bytes) : compiled;
}
