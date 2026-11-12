// One WebSocket session per device (docs/api.md, WebSocket). The page names itself with a session id so the
// firmware can tell its own reconnect from another tab or device.

/** Close code: another session holds the device. Reconnect with `takeover=1` to replace it. */
export const WS_CLOSE_IN_USE = 4000;
/** Close code: this session was replaced by a newer one. */
export const WS_CLOSE_REPLACED = 4001;

/** `active`: connected or reconnecting. `inUse` / `replaced`: asking the user. `parked`: the user declined; no reconnect. */
export type SessionState = "active" | "inUse" | "replaced" | "parked";

export function closeOutcome(code: number): "inUse" | "replaced" | "drop" {
  if (code === WS_CLOSE_IN_USE) return "inUse";
  if (code === WS_CLOSE_REPLACED) return "replaced";
  return "drop";
}

const SID_KEY = "ws_sid";
const validSid = (s: unknown): s is string => typeof s === "string" && /^[0-9a-f]{1,8}$/.test(s) && Number.parseInt(s, 16) !== 0;

/** 8 hex digits, never zero (the firmware reads zero as "no session id"). */
export function newSessionId(random: () => number = () => crypto.getRandomValues(new Uint32Array(1))[0]): string {
  return ((random() >>> 0) || 1).toString(16).padStart(8, "0");
}

/** The tab's session id: kept in sessionStorage so a reload is the same session, made fresh when storage is unusable. */
export function sessionId(storage: Pick<Storage, "getItem" | "setItem"> | null, make: () => string = newSessionId): string {
  try {
    const saved = storage?.getItem(SID_KEY);
    if (validSid(saved)) return saved;
  } catch {
    // Storage blocked: fall through to a fresh id.
  }
  const sid = make();
  try {
    storage?.setItem(SID_KEY, sid);
  } catch {
    // Not saved: a reload gets a new id.
  }
  return sid;
}

export function wsUrl(location: { protocol: string; host: string }, sid: string, takeover: boolean): string {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  return `${scheme}//${location.host}/ws?sid=${sid}${takeover ? "&takeover=1" : ""}`;
}
