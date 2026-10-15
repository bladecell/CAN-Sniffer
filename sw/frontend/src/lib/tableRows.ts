export type FilterOperator = "contains" | "equals" | "starts_with" | "gt" | "lt";
export type FilterRule = {
  id: string;
  column: string;
  operator: FilterOperator;
  value: string;
};
export type Row = Record<string, any>;

/** Keeps rows matching every rule that has a column and a non-blank value. Returns `rows` itself when no rule applies. */
export function filterRows(rows: Row[], rules: FilterRule[]): Row[] {
  const validRules = rules.filter((rule) => rule.column && rule.value.trim() !== "");
  if (validRules.length === 0) return rows;

  return rows.filter((row) =>
    validRules.every((rule) => {
      const rawVal = row[rule.column];
      const strVal = String(rawVal ?? "").toLowerCase();
      const searchVal = rule.value.toLowerCase().trim();

      switch (rule.operator) {
        case "contains":
          return strVal.includes(searchVal);
        case "equals":
          return strVal === searchVal;
        case "starts_with":
          return strVal.startsWith(searchVal);
        case "gt":
          return Number(rawVal) > Number(rule.value);
        case "lt":
          return Number(rawVal) < Number(rule.value);
        default:
          return true;
      }
    }),
  );
}

/** Sorts a copy by one column. Empty values (null, "", "N/A") always sort last. */
export function sortRows(
  rows: Row[],
  key: string,
  direction: "asc" | "desc",
  isNumeric: boolean,
): Row[] {
  const sign = direction === "asc" ? 1 : -1;

  return [...rows].sort((a, b) => {
    const valA = a[key];
    const valB = b[key];

    const isEmptyA = valA == null || valA === "N/A" || valA === "";
    const isEmptyB = valB == null || valB === "N/A" || valB === "";

    if (isEmptyA && isEmptyB) return 0;
    if (isEmptyA) return 1;
    if (isEmptyB) return -1;

    if (isNumeric) {
      const numA = Number(valA);
      const numB = Number(valB);
      if (!isNaN(numA) && !isNaN(numB)) return (numA - numB) * sign;
    }

    const strA = String(valA).toLowerCase();
    const strB = String(valB).toLowerCase();
    if (strA < strB) return -sign;
    if (strA > strB) return sign;
    return 0;
  });
}

/** Hex and binary views of numeric cells. Empty, "N/A" and non-numeric values are returned unchanged. */
export function formatNumber(value: unknown, format?: string) {
  if (value == null || value === "N/A" || value === "") return value;
  const num = Number(value);
  if (isNaN(num)) return value;

  if (format === "hex") return "0x" + num.toString(16).toUpperCase();
  if (format === "bin") return "0b" + num.toString(2);
  return value;
}
