// The rules the user made, kept in this browser's localStorage. Import and export use the same format.
import { compileRules, RULE_CAP, type Rule, type RuleDraft } from "./canRules";
import { exportRules, parseRules } from "./canRulesIo";

const KEY = "can-sniffer.canRules";

export function newRuleId(): string {
  return `r${Date.now().toString(36)}${Math.random().toString(36).slice(2, 7)}`;
}

function loadRules(): Rule[] {
  try {
    const text = localStorage.getItem(KEY);
    if (text === null) return [];
    const parsed = parseRules(text);
    return parsed.ok ? parsed.rules.map((draft) => ({ id: newRuleId(), ...draft })) : [];
  } catch {
    // Storage blocked or unreadable: start with no rules.
    return [];
  }
}

function saveRules(rules: readonly Rule[]) {
  try {
    localStorage.setItem(KEY, exportRules(rules));
  } catch {
    // Not saved (quota, private mode): the rules last for this page only.
  }
}

export type SaveResult = { ok: true } | { ok: false; reason: string };

class RulesStore {
  list = $state.raw<Rule[]>(loadRules());
  /** Recompiled whenever the list changes; the capture model reads it per batch. */
  compiled = $derived(compileRules(this.list));

  private commit(next: Rule[]) {
    this.list = next;
    saveRules(next);
  }

  /** `id` lets the editor keep one stable row key from the first keystroke to the saved rule. */
  add(draft: RuleDraft, id: string = newRuleId()): SaveResult {
    if (this.list.length >= RULE_CAP) return { ok: false, reason: `At most ${RULE_CAP} rules` };
    this.commit([...this.list, { id, ...draft }]);
    return { ok: true };
  }

  update(id: string, draft: RuleDraft): SaveResult {
    if (!this.list.some((r) => r.id === id)) return { ok: false, reason: "That rule no longer exists" };
    this.commit(this.list.map((r) => (r.id === id ? { id, ...draft } : r)));
    return { ok: true };
  }

  /** Pauses or resumes a rule without touching the rest of it. */
  setEnabled(id: string, on: boolean) {
    this.commit(this.list.map((r) => {
      if (r.id !== id) return r;
      const { enabled: _old, ...rest } = r;
      return on ? rest : { ...rest, enabled: false };
    }));
  }

  remove(id: string) {
    this.commit(this.list.filter((r) => r.id !== id));
  }

  /** Replaces every rule (import). The caller has validated the drafts. */
  replaceAll(drafts: readonly RuleDraft[]) {
    this.commit(drafts.map((draft) => ({ id: newRuleId(), ...draft })));
  }

  exportText(): string {
    return exportRules(this.list);
  }
}

export const rulesStore = new RulesStore();
