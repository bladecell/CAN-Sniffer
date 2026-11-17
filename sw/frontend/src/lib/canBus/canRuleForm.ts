// The rule editor's text fields <-> a checked rule. Pure, so the form and the import share one validator.
import { checkRule, type Checked, type Rule, type RuleDraft, type RuleMatch } from "./canRules";

export type ByteField = { index: string; value: string; mask: string };
export type RuleForm = {
  name: string;
  id: string;
  /** Empty: exact match (all id bits). */
  idMask: string;
  ext: boolean;
  /** Empty: any DLC. */
  dlc: string;
  bytes: ByteField[];
  formula: string;
};

export const MAX_BYTE_FIELDS = 8;

export function emptyForm(): RuleForm {
  return { name: "", id: "", idMask: "", ext: false, dlc: "", bytes: [], formula: "" };
}

/** Hex text in 0..max, one to eight digits. Null when it is not a valid hex number in range. */
export function parseHex(text: string, max: number): number | null {
  const t = text.trim();
  if (!/^[0-9a-fA-F]{1,8}$/.test(t)) return null;
  const v = parseInt(t, 16);
  return v <= max ? v : null;
}

const hex = (v: number, digits: number) => v.toString(16).toUpperCase().padStart(digits, "0");

export function formFromRule(rule: Rule): RuleForm {
  const m = rule.match;
  const digits = m.ext ? 8 : 3;
  return {
    name: rule.name,
    id: hex(m.id, digits),
    idMask: m.idMask === fullMask(m.ext) ? "" : hex(m.idMask, digits),
    ext: m.ext,
    dlc: m.dlc === undefined ? "" : String(m.dlc),
    bytes: m.bytes.map((b) => ({ index: String(b.index), value: hex(b.value, 2), mask: hex(b.mask, 2) })),
    formula: rule.formula,
  };
}

export function fullMask(ext: boolean): number {
  return ext ? 0x1fffffff : 0x7ff;
}

/** The form as a rule draft. Every failure gives a reason for the user; nothing is saved from a bad form. */
export function ruleFromForm(f: RuleForm): Checked<RuleDraft> {
  const max = fullMask(f.ext);
  const id = parseHex(f.id, max);
  if (id === null) return { ok: false, reason: `ID must be hex, up to ${f.ext ? "8 digits" : "3 digits (7FF)"}` };
  let idMask = max;
  if (f.idMask.trim() !== "") {
    const parsed = parseHex(f.idMask, max);
    if (parsed === null) return { ok: false, reason: "ID mask must be hex in range" };
    idMask = parsed;
  }
  let dlc: number | undefined;
  if (f.dlc !== "") {
    dlc = Number(f.dlc);
    if (!Number.isInteger(dlc) || dlc < 0 || dlc > 8) return { ok: false, reason: "DLC must be 0 to 8" };
  }
  const bytes: RuleMatch["bytes"] = [];
  for (const b of f.bytes) {
    const index = Number(b.index);
    const value = parseHex(b.value, 0xff);
    const mask = parseHex(b.mask, 0xff);
    if (!Number.isInteger(index) || index < 0 || index > 7) return { ok: false, reason: "Byte index must be 0 to 7" };
    if (value === null || mask === null) return { ok: false, reason: `Byte ${index}: value and mask must be hex 00 to FF` };
    bytes.push({ index, value, mask });
  }
  const match: RuleMatch = { id, ext: f.ext, idMask, bytes };
  if (dlc !== undefined) match.dlc = dlc;
  return checkRule({ name: f.name.trim(), formula: f.formula, match });
}

/** Short text for a rule row: `ID 545 · mask 7FF · DLC 8 · B2 = 41/FF`. */
export function matchSummary(m: RuleMatch): string {
  const digits = m.ext ? 8 : 3;
  const parts = [`ID ${hex(m.id, digits)}`];
  if (m.idMask !== fullMask(m.ext)) parts.push(`mask ${hex(m.idMask, digits)}`);
  if (m.dlc !== undefined) parts.push(`DLC ${m.dlc}`);
  for (const b of m.bytes) parts.push(`B${b.index} = ${hex(b.value, 2)}/${hex(b.mask, 2)}`);
  if (m.ext) parts.push("extended");
  return parts.join(" · ");
}
