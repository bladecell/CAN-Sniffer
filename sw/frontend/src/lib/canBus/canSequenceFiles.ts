// Saved sequences on the SD card: /config/send/<name>.json, through the generic SD file routes (docs/api.md).
import type { SdTreeNode } from "../types";

export const SEQ_DIR = "config/send";
export const SEQ_LIST_MAX = 100;
const NAME_RE = /^[A-Za-z0-9_-]{1,32}$/;

export const isSeqName = (name: string): boolean => NAME_RE.test(name);
export const seqPath = (name: string): string => `${SEQ_DIR}/${name}.json`;

/** The saved names in a `/sd_card/tree/config/send` reply: regular `<name>.json` files with an allowed name, sorted, capped. Anything else is ignored. */
export function parseSeqList(tree: unknown): string[] {
  const children = (tree as { children?: unknown } | null)?.children;
  if (!Array.isArray(children)) return [];
  const names: string[] = [];
  for (const c of children as Partial<SdTreeNode>[]) {
    if (c?.type !== "file" || typeof c.name !== "string" || !c.name.endsWith(".json")) continue;
    const base = c.name.slice(0, -5);
    if (isSeqName(base)) names.push(base);
  }
  return names.sort().slice(0, SEQ_LIST_MAX);
}
