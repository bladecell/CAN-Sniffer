// Writes a formula that reads an unsigned value from frame bytes. The user can edit the result.
import { FRAME_LETTERS } from "../editor/formulaTest";

export type ParseSpec = { start: number; length: number; order: "be" | "le" };

/** Start byte 0 to 7, length 1 to 4, and the value must fit the frame. Errors are for the user. */
export function helperFormula(spec: ParseSpec): { ok: true; formula: string } | { ok: false; reason: string } {
  const { start, length, order } = spec;
  if (!Number.isInteger(start) || start < 0 || start > 7) return { ok: false, reason: "Start byte must be 0 to 7" };
  if (!Number.isInteger(length) || length < 1 || length > 4) return { ok: false, reason: "Length must be 1 to 4 bytes" };
  if (start + length > 8) return { ok: false, reason: "The value runs past byte 7" };
  const terms: string[] = [];
  for (let i = 0; i < length; i++) {
    const letter = FRAME_LETTERS[start + i];
    // Byte i of the value: big endian puts the first byte most significant, little endian the last.
    const shift = order === "be" ? 8 * (length - 1 - i) : 8 * i;
    terms.push(shift === 0 ? letter : `${letter}*${2 ** shift}`);
  }
  return { ok: true, formula: terms.join("+") };
}
