import { describe, expect, test } from "bun:test";
import { emptyForm, formFromRule, matchSummary, parseHex, ruleFromForm } from "./canRuleForm";
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
