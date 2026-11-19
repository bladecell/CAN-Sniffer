import { describe, expect, test } from "bun:test";
import { emptyForm, findRuleForId, formForId, formFromRule, matchSummary, parseHex, ruleFromForm } from "./canRuleForm";
import type { Rule } from "./canRules";

describe("parseHex", () => {
  test("accepts hex in range, rejects everything else", () => {
    expect(parseHex("7ff", 0x7ff)).toBe(0x7ff);
    expect(parseHex(" 41 ", 0xff)).toBe(0x41);
    expect(parseHex("800", 0x7ff)).toBeNull();
    expect(parseHex("", 0xff)).toBeNull();
    expect(parseHex("0x41", 0xff)).toBeNull();
    expect(parseHex("123456789", 0xffffffff)).toBeNull();
  });
});

describe("ruleFromForm", () => {
  const base = { ...emptyForm(), name: " Coolant ", id: "545", formula: "A - 40" };

  test("builds a checked rule with exact mask and no DLC", () => {
    const r = ruleFromForm(base);
    expect(r).toEqual({ ok: true, value: { name: "Coolant", formula: "A - 40", match: { id: 0x545, ext: false, idMask: 0x7ff, bytes: [] } } });
  });

  test("optional mask, DLC and byte matches", () => {
    const r = ruleFromForm({ ...base, idMask: "7F0", dlc: "8", bytes: [{ index: "2", value: "41", mask: "FF" }] });
    expect(r.ok && r.value.match).toEqual({ id: 0x545, ext: false, idMask: 0x7f0, dlc: 8, bytes: [{ index: 2, value: 0x41, mask: 0xff }] });
  });

  test("reasons for bad fields, and no rule", () => {
    expect(ruleFromForm({ ...base, id: "800" })).toMatchObject({ ok: false });
    expect(ruleFromForm({ ...base, idMask: "zz" })).toMatchObject({ ok: false });
    expect(ruleFromForm({ ...base, dlc: "9" })).toMatchObject({ ok: false });
    expect(ruleFromForm({ ...base, bytes: [{ index: "1", value: "GG", mask: "FF" }] })).toMatchObject({ ok: false });
    expect(ruleFromForm({ ...base, bytes: [{ index: "8", value: "00", mask: "FF" }] })).toMatchObject({ ok: false });
    expect(ruleFromForm({ ...base, formula: "A +" })).toMatchObject({ ok: false });
  });
});

describe("formFromRule", () => {
  test("round trips a rule", () => {
    const rule: Rule = { id: "x", name: "Coolant", formula: "A", match: { id: 0x545, ext: false, idMask: 0x7f0, dlc: 8, bytes: [{ index: 2, value: 0x41, mask: 0xff }] } };
    const back = ruleFromForm(formFromRule(rule));
    expect(back).toEqual({ ok: true, value: { name: "Coolant", formula: "A", match: rule.match } });
  });

  test("summary", () => {
    expect(matchSummary({ id: 0x545, ext: false, idMask: 0x7ff, bytes: [{ index: 2, value: 0x41, mask: 0xff }] })).toBe("ID 545 · B2 = 41/FF");
    expect(matchSummary({ id: 0x18daf110, ext: true, idMask: 0x1fffffff, dlc: 8, bytes: [] })).toBe("ID 18DAF110 · DLC 8 · extended");
  });
});

describe("create rule", () => {
  test("a new rule is prefilled from the id: ID, extended, DLC and name, no byte match", () => {
    expect(formForId(0x0c1, false, 8)).toEqual({ ...emptyForm(), name: "ID 0C1", id: "0C1", dlc: "8" });
    expect(formForId(0x18daf110, true, 8)).toMatchObject({ name: "ID 18DAF110", id: "18DAF110", ext: true, dlc: "8" });
    expect(ruleFromForm(formForId(0x0c1, false, 8))).toEqual({ ok: true, value: { name: "ID 0C1", formula: "", match: { id: 0xc1, ext: false, idMask: 0x7ff, dlc: 8, bytes: [] } } });
  });

  test("findRuleForId: a rule matching exactly this id and kind", () => {
    const rule: Rule = { id: "a", name: "", formula: "", match: { id: 0xc1, ext: false, idMask: 0x7ff, bytes: [] } };
    expect(findRuleForId([rule], 0xc1, false)).toBe(rule);
    expect(findRuleForId([rule], 0xc1, true)).toBeUndefined();
    expect(findRuleForId([rule], 0xc2, false)).toBeUndefined();
    expect(findRuleForId([{ ...rule, match: { ...rule.match, idMask: 0x7f0 } }], 0xc1, false)).toBeUndefined();
  });
});
