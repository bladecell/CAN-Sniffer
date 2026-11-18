import { describe, expect, test } from "bun:test";
import { checkRule, compileRules, findLastMatch, matchesFrame, type Rule, type RuleMatch } from "./canRules";
import { exportRules, parseRules } from "./canRulesIo";

const frame = (id: number, data: number[], ext = false) => ({ id, ext, data: new Uint8Array(data) });
const exact = (id: number, ext = false): RuleMatch => ({ id, ext, idMask: ext ? 0x1fffffff : 0x7ff, bytes: [] });
const rule = (over: Partial<Rule>): Rule => ({ id: "r", name: "", match: exact(0x0c9), formula: "", ...over });

describe("matchesFrame", () => {
  test("exact id, standard versus extended with the same number", () => {
    expect(matchesFrame(exact(0x7e8), frame(0x7e8, [1]))).toBe(true);
    expect(matchesFrame(exact(0x7e8), frame(0x7e8, [1], true))).toBe(false);
    expect(matchesFrame(exact(0x7e8, true), frame(0x7e8, [1], true))).toBe(true);
  });

  test("id mask matches a range", () => {
    const m: RuleMatch = { id: 0x7e0, ext: false, idMask: 0x7f0, bytes: [] };
    expect(matchesFrame(m, frame(0x7e8, [1]))).toBe(true);
    expect(matchesFrame(m, frame(0x7f0, [1]))).toBe(false);
  });

  test("dlc and byte value/mask; a byte past the DLC never matches", () => {
    const m: RuleMatch = { id: 0x545, ext: false, idMask: 0x7ff, dlc: 8, bytes: [{ index: 2, value: 0x41, mask: 0xff }] };
    expect(matchesFrame(m, frame(0x545, [0, 0, 0x41, 0, 0, 0, 0, 0]))).toBe(true);
    expect(matchesFrame(m, frame(0x545, [0, 0, 0x42, 0, 0, 0, 0, 0]))).toBe(false);
    expect(matchesFrame(m, frame(0x545, [0, 0, 0x41]))).toBe(false);
    const high: RuleMatch = { ...m, dlc: undefined, bytes: [{ index: 0, value: 0x30, mask: 0xf0 }] };
    expect(matchesFrame(high, frame(0x545, [0x3f]))).toBe(true);
    expect(matchesFrame(high, frame(0x545, [0x4f]))).toBe(false);
  });
});

describe("compileRules / apply", () => {
  test("a plain filter hides the match; a formula rule adds a parsed value", () => {
    const c = compileRules([
      rule({ id: "f", match: exact(0x5a0) }),
      rule({ id: "e", name: "Engine speed", match: exact(0x0c9), formula: "(A*256+B)/4" }),
    ]);
    expect(c.apply(frame(0x5a0, [1]))).toMatchObject({ hidden: true, name: "", parsed: "" });
    expect(c.apply(frame(0x0c9, [0x0b, 0xb8]))).toMatchObject({ hidden: false, name: "Engine speed", parsed: "Engine speed 750" });
  });

  test("two overlapping rules both apply", () => {
    const c = compileRules([
      rule({ id: "a", name: "Hi", match: exact(0x316), formula: "A" }),
      rule({ id: "b", name: "Lo", match: { id: 0x316, ext: false, idMask: 0x7f0, bytes: [] }, formula: "B" }),
    ]);
    expect(c.apply(frame(0x316, [7, 9])).parsed).toBe("Hi 7, Lo 9");
  });

  test("divide by zero shows invalid for that rule only; the other still works", () => {
    const c = compileRules([
      rule({ id: "bad", name: "Ratio", match: exact(0x100), formula: "A/B" }),
      rule({ id: "ok", name: "Byte", match: exact(0x100), formula: "A" }),
    ]);
    expect(c.apply(frame(0x100, [5, 0])).parsed).toBe("Ratio invalid, Byte 5");
  });

  test("the first named matching rule names the frame", () => {
    const c = compileRules([rule({ id: "a", match: exact(0x1), formula: "A" }), rule({ id: "b", name: "Coolant", match: exact(0x1), formula: "A" })]);
    expect(c.apply(frame(0x1, [1])).name).toBe("Coolant");
  });

  test("formula reads A to H", () => {
    const c = compileRules([rule({ name: "H", match: exact(0x2), formula: "H" })]);
    expect(c.apply(frame(0x2, [0, 0, 0, 0, 0, 0, 0, 0x33])).parsed).toBe("H 51");
  });
});

describe("findLastMatch", () => {
  test("returns the newest accepted frame", () => {
    const frames = [
      { ...frame(0x100, [1]), t: 5 },
      { ...frame(0x100, [2]), t: 9 },
      { ...frame(0x101, [3]), t: 12 },
    ];
    const m: RuleMatch = { id: 0x100, ext: false, idMask: 0x7ff, bytes: [] };
    expect(findLastMatch(frames, m)?.t).toBe(9);
    expect(findLastMatch(frames, { ...m, idMask: 0x7ff, id: 0x7ff })).toBeNull();
  });
});

describe("checkRule", () => {
  const good = { name: "Coolant", formula: "A - 40", match: { id: 0x545, ext: false, idMask: 0x7ff, bytes: [{ index: 2, value: 0x41, mask: 0xff }] } };

  test("accepts a rule and drops unknown fields", () => {
    const r = checkRule({ ...good, extra: 1, match: { ...good.match, junk: true } });
    expect(r).toEqual({ ok: true, value: good });
  });

  test("rejects a bad formula, an out-of-range id, a standard id above 11 bits, and a repeated byte", () => {
    expect(checkRule({ ...good, formula: "A; alert(1)" })).toMatchObject({ ok: false });
    expect(checkRule({ ...good, formula: "foo" })).toMatchObject({ ok: false });
    expect(checkRule({ ...good, formula: "A/0" })).toMatchObject({ ok: true });
    expect(checkRule({ ...good, match: { ...good.match, id: 0x800 } })).toMatchObject({ ok: false });
    expect(checkRule({ ...good, match: { ...good.match, bytes: [{ index: 1, value: 0, mask: 0 }, { index: 1, value: 0, mask: 0 }] } })).toMatchObject({ ok: false });
    expect(checkRule({ ...good, match: { ...good.match, bytes: [{ index: 8, value: 0, mask: 0 }] } })).toMatchObject({ ok: false });
    expect(checkRule({ ...good, formula: "x".repeat(201) })).toMatchObject({ ok: false });
    expect(checkRule({ ...good, name: "n".repeat(41) })).toMatchObject({ ok: false });
    expect(checkRule(null)).toMatchObject({ ok: false });
  });
});

describe("parseRules / exportRules", () => {
  test("round trip keeps every rule and no id", () => {
    const r: Rule = { ...rule({ name: "Coolant", formula: "A" }), id: "x", match: { id: 0x545, ext: false, idMask: 0x7ff, bytes: [] } };
    const text = exportRules([r]);
    expect(JSON.parse(text).version).toBe(1);
    expect(JSON.parse(text).rules[0].id).toBeUndefined();
    expect(parseRules(text)).toEqual({ ok: true, rules: [{ name: "Coolant", formula: "A", match: { id: 0x545, ext: false, idMask: 0x7ff, bytes: [] } }] });
  });

  test("one bad rule saves nothing and says which one", () => {
    const text = JSON.stringify({
      version: 1,
      rules: [
        { name: "ok", formula: "A", match: { id: 1, ext: false, idMask: 0x7ff, bytes: [] } },
        { name: "bad", formula: "A +", match: { id: 1, ext: false, idMask: 0x7ff, bytes: [] } },
      ],
    });
    const r = parseRules(text);
    expect(r.ok).toBe(false);
    expect(r.ok === false && r.reason).toContain("Rule 2");
  });

  test("refuses wrong version, non JSON, too many rules and oversized files", () => {
    expect(parseRules("nope")).toMatchObject({ ok: false });
    expect(parseRules(JSON.stringify({ version: 2, rules: [] }))).toMatchObject({ ok: false });
    expect(parseRules(JSON.stringify({ version: 1, rules: "x" }))).toMatchObject({ ok: false });
    const many = Array.from({ length: 65 }, () => ({ name: "", formula: "", match: { id: 1, ext: false, idMask: 0x7ff, bytes: [] } }));
    expect(parseRules(JSON.stringify({ version: 1, rules: many }))).toMatchObject({ ok: false });
    expect(parseRules(" ".repeat(64 * 1024 + 1))).toMatchObject({ ok: false });
  });
});

describe("rule enabled flag", () => {
  const draft = { name: "x", formula: "", match: { id: 1, ext: false, idMask: 0x7ff, bytes: [] } };
  test("only false is stored and exported; a rule without it stays enabled", () => {
    expect(checkRule(draft)).toEqual({ ok: true, value: draft });
    expect(checkRule({ ...draft, enabled: true })).toEqual({ ok: true, value: draft });
    expect(checkRule({ ...draft, enabled: false })).toEqual({ ok: true, value: { ...draft, enabled: false } });
    expect(checkRule({ ...draft, enabled: "no" }).ok).toBe(false);
  });
  test("a disabled rule survives export and import", () => {
    const r = parseRules(exportRules([{ id: "a", ...draft, enabled: false }]));
    expect(r.ok && r.rules[0].enabled).toBe(false);
  });
  test("a disabled rule neither hides nor parses", () => {
    const f = frame(1, [1]);
    expect(compileRules([rule({ ...draft, id: "a", enabled: false })]).apply(f).hidden).toBe(false);
    expect(compileRules([rule({ ...draft, id: "a" })]).apply(f).hidden).toBe(true);
  });
});
