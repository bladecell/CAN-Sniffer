import type { CanBusStatus, DtcData, DtcGroupWire, DtcModeData, Obd2StatusCore } from "./types";

type Rec = Record<string, unknown>;

const isRec = (v: unknown): v is Rec => typeof v === "object" && v !== null && !Array.isArray(v);
const isNum = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);

function need(ok: boolean, what: string): void {
  if (!ok) throw new Error(`Invalid ${what} response`);
}

export function parseObd2Status(raw: unknown): Obd2StatusCore {
  need(isRec(raw), "OBD2 status");
  const r = raw as Rec;
  need(typeof r.auto_poll === "boolean" && typeof r.polling === "boolean" && typeof r.pid_initialized === "boolean", "OBD2 status");
  need(isNum(r.pid_def_count) && isNum(r.pid_data_count) && isNum(r.poll_task_utilization), "OBD2 status");
  need(isRec(r.supported_pids), "OBD2 status");
  const sp = r.supported_pids as Rec;
  need(isNum(sp.count) && Array.isArray(sp.groups) && sp.groups.every(isNum), "OBD2 status");
  return {
    auto_poll: r.auto_poll as boolean,
    polling: r.polling as boolean,
    pid_initialized: r.pid_initialized as boolean,
    pid_def_count: r.pid_def_count as number,
    pid_data_count: r.pid_data_count as number,
    poll_task_utilization: r.poll_task_utilization as number,
    supported_pids: { count: sp.count as number, groups: sp.groups as number[] },
  };
}

export function parseCanBusStatus(raw: unknown): CanBusStatus {
  need(isRec(raw) && typeof raw.state === "string" && isNum(raw.bitrate), "CAN status");
  // Only state (any string, so a newer firmware state cannot break the load) and bitrate are checked; the other fields are display-only.
  return raw as unknown as CanBusStatus;
}

type DtcGroupRaw = Pick<DtcGroupWire, "mode" | "dtc_count" | "dtc">;

/** `GET /dtc`. A missing `dtcs` is an empty list; malformed groups throw. */
export function parseDtcGroups(raw: unknown): DtcGroupRaw[] {
  need(isRec(raw), "DTC");
  const dtcs = (raw as Rec).dtcs;
  if (dtcs === undefined) return [];
  need(Array.isArray(dtcs), "DTC");
  return (dtcs as unknown[]).map((g) => {
    need(isRec(g) && isNum(g.mode) && Array.isArray(g.dtc), "DTC");
    const rec = g as Rec;
    const codes = (rec.dtc as unknown[]).filter((c): c is string => typeof c === "string");
    return { mode: rec.mode as number, dtc_count: isNum(rec.dtc_count) ? rec.dtc_count : codes.length, dtc: codes };
  });
}

/** `GET /dtc?codes=...` as a code to description map. */
export function parseDtcDescriptions(raw: unknown): Record<string, string> {
  const out: Record<string, string> = {};
  if (!isRec(raw) || raw.status !== "success" || !Array.isArray(raw.dtcs)) return out;
  for (const item of raw.dtcs) {
    if (isRec(item) && typeof item.dtc === "string" && typeof item.description === "string") {
      out[item.dtc] = item.description;
    }
  }
  return out;
}

export const emptyDtcData = (): DtcData => ({
  confirmed: { mode: 3, dtc_count: 0, dtc: [] },
  pending: { mode: 7, dtc_count: 0, dtc: [] },
  permanent: { mode: 10, dtc_count: 0, dtc: [] },
});

export function buildDtcData(groups: DtcGroupRaw[], descriptions: Record<string, string>): DtcData {
  const data = emptyDtcData();
  const slot: Record<number, keyof DtcData> = { 3: "confirmed", 7: "pending", 10: "permanent" };
  for (const g of groups) {
    const key = slot[g.mode];
    if (!key) continue;
    const group: DtcModeData = {
      mode: g.mode,
      dtc_count: g.dtc_count,
      dtc: g.dtc.map((code) => ({ dtc: code, description: descriptions[code] || "Description not available", mode: g.mode })),
    };
    data[key] = group;
  }
  return data;
}
