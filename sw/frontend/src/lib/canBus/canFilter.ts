// Table filter rules for the CAN Bus page: field, operator, value, include or exclude.

export type FilterField = "id" | "dlc" | "dir" | `b${0 | 1 | 2 | 3 | 4 | 5 | 6 | 7}`;
export type FilterOp = "=" | "!=" | ">" | "<" | ">=" | "<=";
export type FilterRule = { key: number; field: FilterField; op: FilterOp; value: string; exclude: boolean };

type FrameView = { id: number; tx: boolean; data: ArrayLike<number> };

export const FILTER_FIELDS: { id: FilterField; label: string }[] = [
  { id: "id", label: "ID" },
  { id: "dlc", label: "DLC" },
  { id: "dir", label: "Direction" },
  ...([0, 1, 2, 3, 4, 5, 6, 7] as const).map((i) => ({ id: `b${i}` as FilterField, label: `Byte ${i}` })),
];
export const FILTER_OPS: FilterOp[] = ["=", "!=", ">", "<", ">=", "<="];

/** Direction has no order, so only equal and not equal apply to it. */
export const opsFor = (field: FilterField): FilterOp[] => (field === "dir" ? ["=", "!="] : FILTER_OPS);

/** The value as a number, or null when the text is not valid for the field. */
export function parseFilterValue(field: FilterField, text: string): number | null {
  const t = text.trim();
  if (field === "dir") return /^rx$/i.test(t) ? 0 : /^tx$/i.test(t) ? 1 : null;
  if (field === "dlc") return /^\d{1,2}$/.test(t) ? Number(t) : null;
  return /^[0-9a-f]{1,8}$/i.test(t) ? parseInt(t, 16) : null;
}

function fieldValue(f: FrameView, field: FilterField): number | null {
  if (field === "id") return f.id;
  if (field === "dlc") return f.data.length;
  if (field === "dir") return f.tx ? 1 : 0;
  const i = Number(field[1]);
  return i < f.data.length ? f.data[i] : null;
}

function compare(a: number, op: FilterOp, b: number): boolean {
  switch (op) {
    case "=": return a === b;
    case "!=": return a !== b;
    case ">": return a > b;
    case "<": return a < b;
    case ">=": return a >= b;
    default: return a <= b;
  }
}

/** Every include rule must match and no exclude rule may. A byte the frame does not carry matches nothing. Rules with a bad value are ignored. */
export function compileFilters(rules: FilterRule[]): (f: FrameView) => boolean {
  const ok = rules.flatMap((r) => {
    const v = parseFilterValue(r.field, r.value);
    return v === null ? [] : [{ r, v }];
  });
  if (ok.length === 0) return () => true;
  return (f) =>
    ok.every(({ r, v }) => {
      const a = fieldValue(f, r.field);
      const hit = a !== null && compare(a, r.op, v);
      return r.exclude ? !hit : hit;
    });
}
