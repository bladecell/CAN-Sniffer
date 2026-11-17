// Versioned JSON for the rules: the export file and the localStorage copy share this format.
import { checkRule, RULE_CAP, type Rule, type RuleDraft } from "./canRules";

export const RULES_FORMAT_VERSION = 1;
/** Hostile input: a file bigger than this is refused before JSON.parse. */
export const RULES_MAX_BYTES = 64 * 1024;

export function exportRules(rules: readonly Rule[]): string {
  return JSON.stringify({ version: RULES_FORMAT_VERSION, rules: rules.map(({ id: _id, ...draft }) => draft) }, null, 2);
}

/** All or nothing: one bad rule returns the reason and no rules, so the caller saves nothing. */
export function parseRules(text: string): { ok: true; rules: RuleDraft[] } | { ok: false; reason: string } {
  if (text.length > RULES_MAX_BYTES) return { ok: false, reason: "File is larger than 64 KB" };
  let doc: unknown;
  try {
    doc = JSON.parse(text);
  } catch {
    return { ok: false, reason: "File is not JSON" };
  }
  if (typeof doc !== "object" || doc === null || Array.isArray(doc)) return { ok: false, reason: "File is not a rules file" };
  const { version, rules } = doc as { version?: unknown; rules?: unknown };
  if (version !== RULES_FORMAT_VERSION) return { ok: false, reason: `Unsupported version ${String(version)}` };
  if (!Array.isArray(rules)) return { ok: false, reason: "rules must be a list" };
  if (rules.length > RULE_CAP) return { ok: false, reason: `At most ${RULE_CAP} rules` };
  const out: RuleDraft[] = [];
  for (let i = 0; i < rules.length; i++) {
    const checked = checkRule(rules[i]);
    if (!checked.ok) return { ok: false, reason: `Rule ${i + 1}: ${checked.reason}. Nothing was saved.` };
    out.push(checked.value);
  }
  return { ok: true, rules: out };
}
