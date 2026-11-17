// CAN rules: a match on frames plus an optional name and formula. Pure: no Svelte, no storage.
import { compileFormula, FRAME_LETTERS, type CompiledFormula } from "../editor/formulaTest";

export const RULE_CAP = 64;
export const FORMULA_MAX = 200;
export const NAME_MAX = 40;

export type ByteMatch = { index: number; value: number; mask: number };
export type RuleMatch = {
  id: number;
  ext: boolean;
  /** Bits that must equal `id`. Exact match: all id bits (0x7FF standard, 0x1FFFFFFF extended). */
  idMask: number;
  dlc?: number;
  bytes: ByteMatch[];
};
/** `formula` empty: a plain filter that hides the matching frames from the table. */
export type Rule = { id: string; name: string; match: RuleMatch; formula: string };
export type RuleDraft = Omit<Rule, "id">;
export type FrameLike = { id: number; ext: boolean; data: ArrayLike<number> };
export type Checked<T> = { ok: true; value: T } | { ok: false; reason: string };

export function matchesFrame(m: RuleMatch, f: FrameLike): boolean {
  if (m.ext !== f.ext) return false;
  if (((f.id ^ m.id) & m.idMask) !== 0) return false;
  const dlc = f.data.length;
  if (m.dlc !== undefined && m.dlc !== dlc) return false;
  for (const b of m.bytes) {
    if (b.index >= dlc || ((f.data[b.index] ^ b.value) & b.mask) !== 0) return false;
  }
  return true;
}

export type RuleResult = {
  /** A plain filter matched: the frame is hidden from the table. */
  hidden: boolean;
  /** First matching rule with a name. */
  name: string;
  /** "Name value" per matching formula rule, `invalid` for a failed formula. */
  parsed: string;
};

type Compiled = { rule: Rule; formula: CompiledFormula | null };

export function compileRules(rules: readonly Rule[]) {
  const list: Compiled[] = rules.map((rule) => ({
    rule,
    formula: rule.formula.trim() ? compileFormula(rule.formula, FRAME_LETTERS) : null,
  }));
  return {
    apply(f: FrameLike): RuleResult {
      let hidden = false;
      let name = "";
      const parts: string[] = [];
      for (const { rule, formula } of list) {
        if (!matchesFrame(rule.match, f)) continue;
        if (!name && rule.name) name = rule.name;
        if (!formula) {
          hidden = true;
          continue;
        }
        const label = rule.name || "value";
        const r = formula.ok ? formula.run(f.data) : null;
        parts.push(r?.ok ? `${label} ${formatValue(r.value)}` : `${label} invalid`);
      }
      return { hidden, name, parsed: parts.join(", ") };
    },
  };
}

export type CompiledRules = ReturnType<typeof compileRules>;

export function formatValue(value: number): string {
  return String(Number(value.toFixed(2)));
}

/** Newest-first search over per-ID recent frames: the last frame any id produced that the match accepts. */
export function findLastMatch<F extends FrameLike & { t: number }>(frames: Iterable<F>, m: RuleMatch): F | null {
  let best: F | null = null;
  for (const f of frames) if ((!best || f.t > best.t) && matchesFrame(m, f)) best = f;
  return best;
}

const isInt = (v: unknown, min: number, max: number): v is number => typeof v === "number" && Number.isInteger(v) && v >= min && v <= max;
const isObj = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);

function checkMatch(raw: unknown): Checked<RuleMatch> {
  if (!isObj(raw)) return { ok: false, reason: "match is not an object" };
  if (typeof raw.ext !== "boolean") return { ok: false, reason: "match.ext must be true or false" };
  const max = raw.ext ? 0x1fffffff : 0x7ff;
  if (!isInt(raw.id, 0, max)) return { ok: false, reason: `match.id must be 0 to ${max.toString(16).toUpperCase()}` };
  if (!isInt(raw.idMask, 0, max)) return { ok: false, reason: "match.idMask is out of range" };
  if (raw.dlc !== undefined && !isInt(raw.dlc, 0, 8)) return { ok: false, reason: "match.dlc must be 0 to 8" };
  const list = raw.bytes ?? [];
  if (!Array.isArray(list) || list.length > 8) return { ok: false, reason: "match.bytes must be a list of up to 8" };
  const seen = new Set<number>();
  const bytes: ByteMatch[] = [];
  for (const b of list) {
    if (!isObj(b) || !isInt(b.index, 0, 7) || !isInt(b.value, 0, 255) || !isInt(b.mask, 0, 255))
      return { ok: false, reason: "match.bytes entries need index 0 to 7, value and mask 0 to 255" };
    if (seen.has(b.index)) return { ok: false, reason: `byte ${b.index} is listed twice` };
    seen.add(b.index);
    bytes.push({ index: b.index, value: b.value & b.mask, mask: b.mask });
  }
  const match: RuleMatch = { id: raw.id, ext: raw.ext, idMask: raw.idMask, bytes };
  if (raw.dlc !== undefined) match.dlc = raw.dlc;
  return { ok: true, value: match };
}

/** Validates one rule and returns a clean copy: unknown fields are dropped, and the id is not kept. */
export function checkRule(raw: unknown): Checked<RuleDraft> {
  if (!isObj(raw)) return { ok: false, reason: "rule is not an object" };
  const name = raw.name ?? "";
  if (typeof name !== "string" || name.length > NAME_MAX) return { ok: false, reason: `name must be text up to ${NAME_MAX} characters` };
  const formula = raw.formula ?? "";
  if (typeof formula !== "string" || formula.length > FORMULA_MAX) return { ok: false, reason: `formula must be text up to ${FORMULA_MAX} characters` };
  if (formula.trim()) {
    const compiled = compileFormula(formula, FRAME_LETTERS);
    if (!compiled.ok) return { ok: false, reason: `formula: ${compiled.reason}` };
  }
  const match = checkMatch(raw.match);
  if (!match.ok) return match;
  return { ok: true, value: { name, formula: formula.trim(), match: match.value } };
}
