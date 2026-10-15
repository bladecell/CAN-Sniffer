import type { SdTreeNode } from "./types";

// Files carry only a name, directories only a path (docs/api.md, /sd_card/tree).
export const nodeKey = (n: SdTreeNode): string => n.path ?? n.name ?? "";
