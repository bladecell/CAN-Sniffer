import { nextOption } from "./appearance";

/** WAI-ARIA tabs: arrows wrap, Home and End jump. Undefined for any other key. */
export function tabTarget(ids: readonly string[], current: string, key: string): string | undefined {
  if (key === "Home") return ids[0];
  if (key === "End") return ids[ids.length - 1];
  return nextOption(ids, current, key);
}
