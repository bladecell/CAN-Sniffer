// Where the dashboards document lives: the SD card (after login) and localStorage (always).
// Nothing here throws; failures come back as values so a bad card or network never blocks the page.
import { defaultDashboards, MAX_DOC_CHARS, parseDashboards, type DashboardsDoc } from "./dashboardModel";

export const DASHBOARDS_PATH = "config/dashboards.json";
/** Not `dashboards.json.tmp`: the firmware's atomic copy uses that name for its own temporary file. */
export const DASHBOARDS_TMP_PATH = "config/dashboards.new.json";
export const LOCAL_KEY = "can-sniffer.dashboards";

export interface SdFiles {
  download(path: string): Promise<Response>;
  upload(path: string, blob: Blob): Promise<Response>;
  copy(source: string, destination: string): Promise<Response>;
  remove(path: string): Promise<Response>;
}
export interface TextStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}
export interface PersistenceDeps { storage: TextStorage; sd: SdFiles; now(): number }

export type DashboardSource = "sd" | "local" | "default";
export type SdSave = "saved" | "skipped" | "failed";
type SdRead = { status: "ok"; doc: DashboardsDoc } | { status: "missing" | "invalid" | "error" };

const msg = (e: unknown) => (e instanceof Error && e.message ? e.message : "Request failed.");

export function createDashboardPersistence({ storage, sd, now }: PersistenceDeps) {
  function loadLocal(): DashboardsDoc | null {
    try {
      return parseDashboards(storage.getItem(LOCAL_KEY));
    } catch {
      return null;
    }
  }

  function saveLocal(doc: DashboardsDoc): boolean {
    try {
      storage.setItem(LOCAL_KEY, JSON.stringify(doc));
      return true;
    } catch {
      return false;
    }
  }

  async function readSd(): Promise<SdRead> {
    try {
      const res = await sd.download(DASHBOARDS_PATH);
      if (res.status === 404) return { status: "missing" };
      if (!res.ok) return { status: "error" };
      const declared = Number(res.headers.get("content-length"));
      if (declared > MAX_DOC_CHARS) return { status: "invalid" };
      const doc = parseDashboards(await res.text());
      return doc ? { status: "ok", doc } : { status: "invalid" };
    } catch {
      return { status: "error" };
    }
  }

  /** Temp file first, then the firmware's atomic copy over the real file. Returns a failure message, or null. */
  async function writeSd(doc: DashboardsDoc): Promise<string | null> {
    try {
      const put = await sd.upload(DASHBOARDS_TMP_PATH, new Blob([JSON.stringify(doc)], { type: "application/json" }));
      if (!put.ok) return `Upload failed (HTTP ${put.status}).`;
      const copied = await sd.copy(DASHBOARDS_TMP_PATH, DASHBOARDS_PATH);
      if (!copied.ok) return `Copy failed (HTTP ${copied.status}).`;
      await sd.remove(DASHBOARDS_TMP_PATH).catch(() => undefined);
      return null;
    } catch (e) {
      return msg(e);
    }
  }

  /** Always saves locally; saves to the SD card only when logged in. */
  async function save(doc: DashboardsDoc, loggedIn: boolean): Promise<{ sd: SdSave; message?: string }> {
    saveLocal(doc);
    if (!loggedIn) return { sd: "skipped" };
    const message = await writeSd(doc);
    return message === null ? { sd: "saved" } : { sd: "failed", message };
  }

  /** Before login: the local copy, else the default template. */
  function initial(): { doc: DashboardsDoc; source: "local" | "default" } {
    const local = loadLocal();
    return local ? { doc: local, source: "local" } : { doc: defaultDashboards(now()), source: "default" };
  }

  /** After login: the newest copy wins; a tie keeps the SD card's. An unreadable SD file is never overwritten here. */
  async function sync(): Promise<{ doc: DashboardsDoc; source: DashboardSource; unreadable: boolean }> {
    const local = loadLocal();
    const remote = await readSd();
    // The SD copy exists but cannot be read or parsed. Only a missing file is written over.
    const unreadable = remote.status === "invalid" || remote.status === "error";
    if (remote.status === "ok") {
      if (local && local.updatedAt > remote.doc.updatedAt) {
        await writeSd(local);
        return { doc: local, source: "local", unreadable };
      }
      saveLocal(remote.doc);
      return { doc: remote.doc, source: "sd", unreadable };
    }
    if (local) {
      if (remote.status === "missing") await writeSd(local);
      return { doc: local, source: "local", unreadable };
    }
    // Nothing stored anywhere: not saved either, so the page keeps offering the starting layouts until one is chosen.
    return { doc: defaultDashboards(now()), source: "default", unreadable };
  }

  return { loadLocal, saveLocal, readSd, writeSd, save, initial, sync };
}
