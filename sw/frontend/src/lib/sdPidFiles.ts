// SD card PID files: folder paths, file names, parsing and writing of PID lists. Pure, so it is unit-tested
// (sdPidFiles.test.ts). Files on the card are hostile input: every parse path returns a reason, never throws
// anything else.
import type { PidDefinition } from "./types";
import { normalizePidDefinition, serializePidDefinition } from "./pidDefinition";
import { MAX_PID_DEFINITIONS, validatePidDefinitionSet } from "./pidValidation";

export const SD_MOUNT = "/sdcard";
export const DEFAULT_SD_FOLDER = `${SD_MOUNT}/configs`;
/** The device accepts at most 64 KiB for a PID list (PUT /pid_def). Files are capped at the same size. */
export const MAX_PID_FILE_BYTES = 64 * 1024;
const MAX_FOLDER_LENGTH = 255;
const MAX_TREE_ENTRIES = 512;

export type FolderResult = { ok: true; folder: string } | { ok: false; reason: string };

/**
 * A folder the user typed: an absolute path under /sdcard, or a path relative to the card root (/pids).
 * Returns the absolute form, with no trailing slash. Parent and current directory parts are refused.
 */
export function normalizeSdFolder(input: string): FolderResult {
  const trimmed = input.trim();
  if (trimmed === "") return { ok: false, reason: "Enter a folder, like /sdcard/pids." };
  const parts = trimmed.split("/").filter((part) => part !== "");
  if (parts.some((part) => part === "." || part === "..")) return { ok: false, reason: "Leave out .. in the path." };
  if (parts[0] === SD_MOUNT.slice(1)) parts.shift();
  if (parts.some((part) => !/^[A-Za-z0-9_-]+$/.test(part))) {
    return { ok: false, reason: "Use letters, digits, - and _ in each part of the path." };
  }
  const folder = SD_MOUNT + parts.map((part) => `/${part}`).join("");
  if (folder.length > MAX_FOLDER_LENGTH) return { ok: false, reason: "At most 255 characters." };
  return { ok: true, folder };
}

/** The card-relative form the file API takes: /sdcard/pids to /pids, the mount itself to "". */
function relativeFolder(folder: string): string {
  return folder.startsWith(SD_MOUNT) ? folder.slice(SD_MOUNT.length) : folder;
}

/** The `GET /sd_card/tree/<path>` path: no leading slash, "" for the card root. */
export function treePath(folder: string): string {
  return relativeFolder(folder).replace(/^\//, "");
}

/** The `/sd_card/file/<path>` path of a file in a folder. */
export function cardFilePath(folder: string, name: string): string {
  return `${relativeFolder(folder)}/${name}`;
}

export type FileNameResult = { ok: true; name: string } | { ok: false; reason: string };

/** A new file's name without the extension. ".json" is added, and the name cannot hold separators or dots. */
export function validFileName(input: string): FileNameResult {
  const base = input.trim().replace(/\.json$/i, "");
  if (base === "") return { ok: false, reason: "Enter a name." };
  if (!/^[A-Za-z0-9_-]{1,32}$/.test(base)) return { ok: false, reason: "Use letters, digits, - and _, up to 32." };
  return { ok: true, name: `${base}.json` };
}

const pidHex = (pid: number) => `0x${pid.toString(16).toUpperCase().padStart(4, "0")}`;

/** A PID list file: a JSON array of PID objects. Throws an Error whose message is the reason it was refused. */
export function parsePidFile(text: string): PidDefinition[] {
  if (new TextEncoder().encode(text).byteLength > MAX_PID_FILE_BYTES) throw new Error("The file is larger than 64 KiB.");
  let raw: unknown;
  try {
    raw = JSON.parse(text);
  } catch {
    throw new Error("Not valid JSON.");
  }
  if (!Array.isArray(raw)) throw new Error("Not a PID list (a JSON array).");
  if (raw.length > MAX_PID_DEFINITIONS) throw new Error(`More than ${MAX_PID_DEFINITIONS} PIDs.`);

  const defs = raw.map((item, index) => {
    try {
      return normalizePidDefinition(item);
    } catch (e) {
      throw new Error(`PID ${index + 1}: ${e instanceof Error ? e.message : "invalid"}`);
    }
  });
  const problem = validatePidDefinitionSet(defs);
  if (problem) throw new Error(problem);

  const seen = new Set<number>();
  for (const def of defs) {
    if (seen.has(def.pid)) throw new Error(`Two PIDs share ${pidHex(def.pid)}.`);
    seen.add(def.pid);
  }
  return defs;
}

/** The file text for a PID list. Compact, so 256 PIDs stay under the 64 KiB cap. */
export function serializePidFile(defs: PidDefinition[]): string {
  return JSON.stringify(defs.map(serializePidDefinition));
}

/**
 * What an import did, from the rows it touched: `imported` is each file PID with whether its row is still pending.
 * New to the list is `added`. Already listed but now different from the device is `changed`.
 */
export function importSummary(before: ReadonlySet<number>, imported: { pid: number; pending: boolean }[]): { added: number; changed: number } {
  const added = imported.filter((item) => !before.has(item.pid)).length;
  const changed = imported.filter((item) => before.has(item.pid) && item.pending).length;
  return { added, changed };
}

export interface SdFile {
  name: string;
  size: number;
}

export type FolderListing =
  | { kind: "ok"; files: SdFile[] }
  /** 503: no card, or the card is busy or not mounted. */
  | { kind: "nocard" }
  /** 404: the folder is not on the card. */
  | { kind: "missing" }
  | { kind: "error"; reason: string };

type Json = Record<string, unknown>;
const isObject = (value: unknown): value is Json => typeof value === "object" && value !== null;
/** A .json file entry. A name with a separator is not a file of this folder, so it is skipped. */
const isJsonFile = (node: unknown): node is Json & { name: string } =>
  isObject(node) &&
  node.type === "file" &&
  typeof node.name === "string" &&
  node.name.toLowerCase().endsWith(".json") &&
  !/[/\\]/.test(node.name);

/** The .json files in a folder, from `GET /sd_card/tree/<path>?depth=0`. */
export async function readFolder(folder: string, tree: (path: string) => Promise<Response>): Promise<FolderListing> {
  let response: Response;
  try {
    response = await tree(treePath(folder));
  } catch {
    return { kind: "error", reason: "The device did not answer." };
  }
  if (response.status === 503) return { kind: "nocard" };
  if (response.status === 404) return { kind: "missing" };
  if (!response.ok) return { kind: "error", reason: `HTTP ${response.status}.` };

  let body: unknown;
  try {
    body = await response.json();
  } catch {
    return { kind: "error", reason: "Invalid response." };
  }
  if (!isObject(body) || !Array.isArray(body.children)) return { kind: "error", reason: "Invalid response." };

  const files = body.children
    .filter(isJsonFile)
    .map((node) => ({ name: node.name, size: typeof node.size === "number" && node.size >= 0 ? node.size : 0 }))
    .sort((a, b) => a.name.localeCompare(b.name));
  return { kind: "ok", files };
}

export interface FolderChoice {
  folder: string;
  jsonCount: number;
}

/** Every folder of a tree (depth-first, the card root first) with the .json files directly in it. Bounded for hostile input. */
export function folderChoices(root: unknown): FolderChoice[] {
  if (!isObject(root)) return [];
  const out: FolderChoice[] = [];
  const stack: { folder: string; children: unknown }[] = [{ folder: SD_MOUNT, children: root.children }];
  let visited = 0;
  while (stack.length > 0 && visited < MAX_TREE_ENTRIES) {
    const { folder, children } = stack.pop()!;
    const entries = Array.isArray(children) ? children : [];
    const dirs: { folder: string; children: unknown }[] = [];
    let jsonCount = 0;
    for (const node of entries) {
      if (++visited > MAX_TREE_ENTRIES) break;
      if (isJsonFile(node)) jsonCount++;
      else if (isObject(node) && typeof node.path === "string") dirs.push({ folder: SD_MOUNT + node.path, children: node.children });
    }
    out.push({ folder, jsonCount });
    stack.push(...dirs.reverse());
  }
  return out;
}
