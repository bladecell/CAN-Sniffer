// Settings screen model: groups, rows, search, field checks and PATCH builders. Pure, so it runs under bun test.
// Limits mirror docs/api.md (Settings rules). The firmware still validates everything; these checks only stop bad
// input before it leaves the browser.

export type GroupId = "look" | "wifi" | "modem" | "vehicle" | "recorder" | "storage" | "access" | "device";

export interface RowMeta {
  readonly title: string;
  /** Extra words the search matches besides the title. */
  readonly kw: string;
}

export interface GroupMeta {
  readonly id: GroupId;
  readonly title: string;
  readonly icon: string;
  readonly kw: string;
  readonly rows: readonly RowMeta[];
}

export const ROW = {
  theme: { title: "Theme", kw: "system light dark appearance" },
  accent: { title: "Accent", kw: "colour color orange yellow green teal blue violet rose neutral" },
  ctl: { title: "Control fill", kw: "strong clear contrast button field" },
  hotspotName: { title: "Network name", kw: "ssid hotspot access point" },
  hotspotPassword: { title: "Password", kw: "hotspot wpa key" },
  channel: { title: "Channel", kw: "wifi radio 2.4 ghz" },
  join: { title: "Join your Wi-Fi", kw: "station sta network internet time updates" },
  networks: { title: "Networks found", kw: "scan nearby ssid" },
  staSsid: { title: "Other network", kw: "hidden station sta ssid join" },
  staPassword: { title: "Network password", kw: "join station key" },
  modem: { title: "Modem", kw: "lte cellular hw v2" },
  apn: { title: "APN", kw: "lte cellular sim data" },
  mqtt: { title: "MQTT broker", kw: "mqtt cloud server" },
  gnss: { title: "GNSS position", kw: "gps location" },
  bitrate: { title: "Bitrate", kw: "baud speed kbps can bus obd" },
  rxQueue: { title: "Receive queue", kw: "frames buffer rx" },
  txQueue: { title: "Transmit queue", kw: "frames buffer tx send" },
  autoPoll: { title: "Auto poll", kw: "keep polling resume ignition trip" },
  sdUsage: { title: "Card usage", kw: "storage capacity sd used full" },
  pidFolder: { title: "PID library folder", kw: "pid json library path" },
  sdFormat: { title: "Format card", kw: "erase wipe destroy sd card" },
  session: { title: "Session", kw: "log out login" },
  savedPin: { title: "Saved PIN", kw: "remember forget pin browser" },
  changePin: { title: "Change PIN", kw: "pin password 6 digits" },
  firmware: { title: "Installed", kw: "firmware version app" },
  uptime: { title: "Uptime", kw: "up running time" },
  lastReset: { title: "Last reset", kw: "restart reason boot" },
  freeMemory: { title: "Free memory", kw: "heap ram memory" },
  restart: { title: "Restart", kw: "reboot device reset" },
} satisfies Record<string, RowMeta>;

export const GROUP: Record<GroupId, GroupMeta> = {
  look: { id: "look", title: "Appearance", icon: "palette", kw: "theme accent colour control fill dark light", rows: [ROW.theme, ROW.accent, ROW.ctl] },
  wifi: {
    id: "wifi",
    title: "Wi-Fi",
    icon: "wifi",
    kw: "wifi hotspot ssid password channel scan network join",
    rows: [ROW.hotspotName, ROW.hotspotPassword, ROW.channel, ROW.join, ROW.networks, ROW.staSsid, ROW.staPassword],
  },
  modem: { id: "modem", title: "Cellular & GNSS", icon: "waves-sine", kw: "lte gnss mqtt apn cellular modem", rows: [ROW.modem, ROW.apn, ROW.mqtt, ROW.gnss] },
  vehicle: { id: "vehicle", title: "Vehicle & CAN", icon: "engine", kw: "bitrate baud queue obd vin poll can", rows: [ROW.bitrate, ROW.rxQueue, ROW.txQueue] },
  recorder: { id: "recorder", title: "Recorder", icon: "route", kw: "recorder trip start polling", rows: [ROW.autoPoll] },
  storage: { id: "storage", title: "Storage", icon: "sd-card", kw: "sd card storage format folder library", rows: [ROW.sdUsage, ROW.pidFolder, ROW.sdFormat] },
  access: { id: "access", title: "Access", icon: "lock", kw: "pin login log out password", rows: [ROW.session, ROW.savedPin, ROW.changePin] },
  device: { id: "device", title: "Device", icon: "bolt", kw: "firmware update restart reset uptime heap factory", rows: [ROW.firmware, ROW.uptime, ROW.lastReset, ROW.freeMemory, ROW.restart] },
};

export const GROUP_IDS: readonly GroupId[] = ["look", "wifi", "modem", "vehicle", "recorder", "storage", "access", "device"];

/** Side rail and phone list sections, in order. */
export const RAIL: readonly { label: string; ids: readonly GroupId[] }[] = [
  { label: "Look", ids: ["look"] },
  { label: "Connections", ids: ["wifi", "modem"] },
  { label: "Vehicle", ids: ["vehicle", "recorder"] },
  { label: "Device", ids: ["storage", "access", "device"] },
];

/** True when every word of the query appears in the text (case-insensitive). An empty query matches everything. */
export function matches(query: string, text: string): boolean {
  const terms = query.trim().toLowerCase().split(/\s+/).filter(Boolean);
  const haystack = text.toLowerCase();
  return terms.every((term) => haystack.includes(term));
}

/** A row shows when the query matches its own title or keywords, or the group's title or keywords (then all rows show). */
export function rowShown(query: string, group: GroupMeta, row: RowMeta): boolean {
  return matches(query, `${group.title} ${group.kw}`) || matches(query, `${row.title} ${row.kw}`);
}

/** A section shows when any of its rows does, so a filtered-out section leaves no heading behind. */
export function anyShown(query: string, group: GroupMeta, rows: readonly RowMeta[]): boolean {
  return rows.some((row) => rowShown(query, group, row));
}

export function groupShown(query: string, group: GroupMeta): boolean {
  return matches(query, `${group.title} ${group.kw}`) || group.rows.some((row) => matches(query, `${row.title} ${row.kw}`));
}

const encoder = new TextEncoder();
export const byteLength = (text: string): number => encoder.encode(text).length;

/** Whole number from text, or null when it is not a plain decimal in range. */
export function parseIntField(text: string, min: number, max: number): number | null {
  const t = text.trim();
  if (!/^\d{1,9}$/.test(t)) return null;
  const n = Number(t);
  return n >= min && n <= max ? n : null;
}

export function checkSsid(ssid: string): string | null {
  const n = byteLength(ssid);
  return n >= 1 && n <= 32 ? null : "Use 1 to 32 bytes.";
}

export function checkStaSsid(ssid: string): string | null {
  return byteLength(ssid) <= 32 ? null : "Use up to 32 bytes.";
}

/** Blank keeps the saved password (the API never returns one). */
export function checkPassword(password: string): string | null {
  if (password === "") return null;
  const n = byteLength(password);
  return n >= 8 && n <= 63 ? null : "Use 8 to 63 bytes, or leave blank to keep the saved one.";
}

export function checkPidPath(path: string): string | null {
  if (!path.startsWith("/sdcard/")) return "Start with /sdcard/.";
  if (path.split("/").includes("..")) return "No .. parts.";
  return byteLength(path) <= 255 ? null : "Use at most 255 bytes.";
}

export const BITRATES = [125000, 250000, 500000, 1000000] as const;
export const bitrateLabel = (bps: number): string => (bps >= 1000000 ? `${bps / 1000000} Mbps` : `${bps / 1000} kbps`);

export const CHANNELS = Array.from({ length: 13 }, (_, i) => i + 1);

export interface SettingsDoc {
  wifi: { ssid: string; channel: number; mode: number; sta_ssid: string };
  can: { bitrate: number; rx_queue_size: number; tx_queue_depth: number };
  system: { pid_def_path: string };
}

const isObject = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null && !Array.isArray(v);
const str = (v: unknown): string | null => (typeof v === "string" ? v : null);
const num = (v: unknown): number | null => (typeof v === "number" && Number.isFinite(v) ? v : null);

/** Reads the GET /settings reply. Returns null when a field the screen uses is missing or the wrong type. */
export function parseSettingsDoc(raw: unknown): SettingsDoc | null {
  if (!isObject(raw) || !isObject(raw.wifi) || !isObject(raw.can) || !isObject(raw.system)) return null;
  const w = raw.wifi;
  const c = raw.can;
  const s = raw.system;
  const doc = {
    wifi: { ssid: str(w.ssid), channel: num(w.channel), mode: num(w.mode), sta_ssid: str(w.sta_ssid) },
    can: { bitrate: num(c.bitrate), rx_queue_size: num(c.rx_queue_size), tx_queue_depth: num(c.tx_queue_depth) },
    system: { pid_def_path: str(s.pid_def_path) },
  };
  const { ssid, channel, mode, sta_ssid } = doc.wifi;
  const { bitrate, rx_queue_size, tx_queue_depth } = doc.can;
  const { pid_def_path } = doc.system;
  if (ssid === null || channel === null || mode === null || sta_ssid === null) return null;
  if (bitrate === null || rx_queue_size === null || tx_queue_depth === null) return null;
  if (pid_def_path === null) return null;
  return {
    wifi: { ssid, channel, mode, sta_ssid },
    can: { bitrate, rx_queue_size, tx_queue_depth },
    system: { pid_def_path },
  };
}

export interface WifiDraft {
  ssid: string;
  password: string;
  channel: number;
  join: boolean;
  staSsid: string;
  staPassword: string;
}

export function wifiDraft(s: SettingsDoc["wifi"]): WifiDraft {
  return { ssid: s.ssid, password: "", channel: s.channel, join: s.mode === 1 || s.mode === 3, staSsid: s.sta_ssid, staPassword: "" };
}

export type Patch = Record<string, string | number>;
export interface Built {
  patch: Patch;
  errors: Record<string, string>;
}

/** Changed wifi fields only, plus per-field errors. Mode is 1 STA, 2 AP, 3 AP+STA; "join" keeps STA on. */
export function buildWifiPatch(saved: SettingsDoc["wifi"], d: WifiDraft): Built {
  const patch: Patch = {};
  const errors: Record<string, string> = {};
  const ssidErr = d.ssid === saved.ssid ? null : checkSsid(d.ssid);
  if (ssidErr) errors.ssid = ssidErr;
  else if (d.ssid !== saved.ssid) patch.ssid = d.ssid;
  const pwErr = checkPassword(d.password);
  if (pwErr) errors.password = pwErr;
  else if (d.password !== "") patch.password = d.password;
  if (d.channel !== saved.channel) {
    if (CHANNELS.includes(d.channel)) patch.channel = d.channel;
    else errors.channel = "Pick 1 to 13.";
  }
  const joined = saved.mode === 1 || saved.mode === 3;
  if (d.join !== joined) patch.mode = d.join ? 3 : 2;
  const staErr = checkStaSsid(d.staSsid);
  if (staErr) errors.staSsid = staErr;
  else if (d.staSsid !== saved.sta_ssid) patch.sta_ssid = d.staSsid;
  const staPwErr = checkPassword(d.staPassword);
  if (staPwErr) errors.staPassword = staPwErr;
  else if (d.staPassword !== "") patch.sta_password = d.staPassword;
  return { patch, errors };
}

/** Queue fields are text inputs in the draft; saved values are numbers. */
export function buildCanPatch(saved: SettingsDoc["can"], d: { bitrate: number; rx: string; tx: string }): Built {
  const patch: Patch = {};
  const errors: Record<string, string> = {};
  if (d.bitrate !== saved.bitrate) {
    if ((BITRATES as readonly number[]).includes(d.bitrate)) patch.bitrate = d.bitrate;
    else errors.bitrate = "Pick a listed rate.";
  }
  const rx = parseIntField(d.rx, 1, 256);
  if (rx === null) errors.rx_queue_size = "Whole number from 1 to 256.";
  else if (rx !== saved.rx_queue_size) patch.rx_queue_size = rx;
  const tx = parseIntField(d.tx, 1, 64);
  if (tx === null) errors.tx_queue_depth = "Whole number from 1 to 64.";
  else if (tx !== saved.tx_queue_depth) patch.tx_queue_depth = tx;
  return { patch, errors };
}

export function buildPidPathPatch(saved: SettingsDoc["system"], path: string): Built {
  const error = checkPidPath(path);
  if (error) return { patch: {}, errors: { pid_def_path: error } };
  return { patch: path === saved.pid_def_path ? {} : { pid_def_path: path }, errors: {} };
}

/** "3 h 12 min", "42 min", "17 s". */
export function formatUptime(seconds: number): string {
  const s = Math.max(0, Math.floor(seconds));
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  if (h > 0) return `${h} h ${m} min`;
  if (m > 0) return `${m} min`;
  return `${s} s`;
}

export interface ScanNetwork {
  ssid: string;
  rssi: number;
  open: boolean;
}

export interface ScanReply {
  state: "idle" | "scanning" | "complete";
  networks: ScanNetwork[];
}

/** Reads GET /wifi/scan. Unknown states and malformed entries are dropped; at most 20 networks. */
export function parseScanReply(raw: unknown): ScanReply | null {
  if (!isObject(raw)) return null;
  const state = raw.state;
  if (state !== "idle" && state !== "scanning" && state !== "complete") return null;
  const list = Array.isArray(raw.networks) ? raw.networks : [];
  const networks: ScanNetwork[] = [];
  for (const entry of list.slice(0, 20)) {
    if (!isObject(entry)) continue;
    const ssid = str(entry.ssid);
    const rssi = num(entry.rssi);
    if (ssid === null || ssid === "" || rssi === null) continue;
    networks.push({ ssid, rssi, open: entry.auth_mode === 0 });
  }
  return { state, networks };
}

/** Joining a locked network needs a WPA password of 8 to 63 bytes (a blank one is not a join). */
export function checkJoinPassword(password: string): string | null {
  const n = byteLength(password);
  return n >= 8 && n <= 63 ? null : "Use 8 to 63 bytes.";
}

/** GET /sd_card/info (docs/api.md): capacity in MB, used_space_mb in MB, is_present is card detect, is_mounted the FAT mount. */
export interface SdInfoWire {
  capacity: number;
  used_space_mb: number;
  is_present: boolean;
  is_mounted: boolean;
}

export function parseSdInfo(raw: unknown): SdInfoWire | null {
  if (!isObject(raw)) return null;
  const capacity = num(raw.capacity);
  const used = num(raw.used_space_mb);
  if (capacity === null || used === null || capacity < 0 || used < 0) return null;
  if (typeof raw.is_present !== "boolean" || typeof raw.is_mounted !== "boolean") return null;
  return { capacity, used_space_mb: used, is_present: raw.is_present, is_mounted: raw.is_mounted };
}

export type SdView =
  | { kind: "no-card" }
  | { kind: "not-mounted" }
  | { kind: "ok"; usedGb: string; totalGb: string; percent: number };

const MB_PER_GB = 1024;

/** Card detect first (no card), then the mount (card present but unusable), then the meter. */
export function sdView(info: SdInfoWire): SdView {
  if (!info.is_present) return { kind: "no-card" };
  if (!info.is_mounted || info.capacity <= 0) return { kind: "not-mounted" };
  const percent = Math.min(100, Math.round((info.used_space_mb / info.capacity) * 100));
  return {
    kind: "ok",
    usedGb: (info.used_space_mb / MB_PER_GB).toFixed(1),
    totalGb: (info.capacity / MB_PER_GB).toFixed(1),
    percent,
  };
}

/** The API drops scan results after 30 s; the client keeps its copy and marks it old after the same time. */
export const SCAN_STALE_MS = 30_000;

export function scanAge(scannedAt: number, now: number): { seconds: number; stale: boolean } {
  const ms = Math.max(0, now - scannedAt);
  return { seconds: Math.floor(ms / 1000), stale: ms > SCAN_STALE_MS };
}
