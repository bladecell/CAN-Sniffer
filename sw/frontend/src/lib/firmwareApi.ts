import { sdCardFileEndpoint } from "./api";
import type {
  CanBusStatus, DtcDescriptionResponse, DtcResponse, Obd2StatusWire, PidDataWire, PidDefinitionWire,
  SDCardInfo, SdTreeNode, SystemStatus,
} from "./types";

type Json = Record<string, unknown>;
export interface FirmwareApiDeps {
  fetch(url: string, init?: RequestInit): Promise<Response>;
  gatedFetch(url: string, init?: RequestInit): Promise<Response>;
}
export type FirmwareResponse<T> = Response & { readonly responseType?: T };
const jsonHeaders = { "Content-Type": "application/json" };
const body = (method: string, value?: unknown): RequestInit => ({
  method,
  ...(value === undefined ? {} : { headers: jsonHeaders, body: JSON.stringify(value) }),
});

/** Typed HTTP boundary for every documented non-auth firmware operation. It never executes calls by itself. */
export function createFirmwareApi(deps: FirmwareApiDeps) {
  const open = <T>(url: string, init?: RequestInit): Promise<FirmwareResponse<T>> => deps.fetch(url, init);
  const gated = <T>(url: string, init?: RequestInit): Promise<FirmwareResponse<T>> => deps.gatedFetch(url, init);
  return {
    index: () => open<string>("/"),
    system: () => open<SystemStatus>("/api/v1/system"),
    reboot: () => gated<Json>("/api/v1/system/reboot", body("POST")),
    canBus: () => open<CanBusStatus>("/api/v1/can_bus"),
    obd2: () => open<Obd2StatusWire>("/api/v1/obd2"),
    setAutoPoll: (on: boolean) => open<{ auto_poll: boolean; polling: boolean }>("/api/v1/obd2", body("PATCH", { auto_poll: on })),
    pidDefinitions: () => open<{ data: PidDefinitionWire[]; count: number }>("/api/v1/pid_def"),
    getPidDefinition: (pid: number) => open<PidDefinitionWire>(`/api/v1/pid_def/${pid}`),
    addPidDefinitions: (defs: unknown[]) => gated<Json>("/api/v1/pid_def", body("POST", defs)),
    replacePidDefinitions: (defs: unknown[]) => gated<void>("/api/v1/pid_def", body("PUT", defs)),
    deletePidDefinition: (pid: number) => gated<void>(`/api/v1/pid_def/${pid}`, body("DELETE")),
    savePidDefinitions: (pid_def_path?: string) => gated<Json>("/api/v1/pid_def/save", body("POST", pid_def_path === undefined ? undefined : { pid_def_path })),
    loadPidDefinitions: (pid_def_path?: string) => gated<Json>("/api/v1/pid_def/load", body("POST", pid_def_path === undefined ? undefined : { pid_def_path })),
    pidData: () => open<{ data: PidDataWire[]; count: number }>("/api/v1/pid_data"),
    getPidData: (pid: number) => open<PidDataWire>(`/api/v1/pid_data/${pid}`),
    requestStaticPid: () => open<Json>("/api/v1/req/static_pid", body("POST")),
    vin: () => open<{ vin: string }>("/api/v1/vin"),
    requestVin: () => open<Json>("/api/v1/req/vin", body("POST")),
    dtc: (mode?: 3 | 7 | 10) => open<DtcResponse>(`/api/v1/dtc${mode === undefined ? "" : `?mode=${mode}`}`),
    dtcDescriptions: (codes: readonly string[]) => open<DtcDescriptionResponse>(`/api/v1/dtc/descriptions?codes=${codes.map(encodeURIComponent).join(",")}`),
    requestDtc: (mode?: 3 | 7 | 10) => open<Json>(`/api/v1/req/dtc${mode === undefined ? "" : `?mode=${mode}`}`, body("POST")),
    clearDtc: () => gated<Json>("/api/v1/req/clear_dtc", body("POST")),
    sdInfo: () => open<SDCardInfo>("/api/v1/sd_card/info"),
    sdTree: (path = "", query: { depth?: number; limit?: number; offset?: number } = {}) => {
      const qs = new URLSearchParams(Object.entries(query).filter(([, v]) => v !== undefined).map(([k, v]) => [k, String(v)]));
      return gated<SdTreeNode>(`/api/v1/sd_card/tree${path ? `/${path.split("/").map(encodeURIComponent).join("/")}` : ""}${qs.size ? `?${qs}` : ""}`);
    },
    downloadSdFile: (path: string) => gated(sdCardFileEndpoint(path) + "?download=true"),
    uploadSdFile: (path: string, file: Blob) => gated(sdCardFileEndpoint(path), { method: "PUT", headers: { "Content-Type": "application/octet-stream" }, body: file }),
    createSdDirectory: (path: string) => gated(sdCardFileEndpoint(path, true), { method: "PUT", body: "" }),
    deleteSdFile: (path: string, directory = false) => gated(sdCardFileEndpoint(path, directory), body("DELETE")),
    copySdFile: (source: string, destination: string) => gated<Json>("/api/v1/sd_card/copy", body("POST", { source, destination })),
    formatSdCard: () => gated<Json>("/api/v1/sd_card/format", body("POST")),
    getSettings: () => gated<Json>("/api/v1/settings"),
    patchSettings: (settings: Json) => gated<Json>("/api/v1/settings", body("PATCH", settings)),
    startWifiScan: () => gated<Json>("/api/v1/wifi/scan", body("POST")),
    wifiScan: () => gated<Json>("/api/v1/wifi/scan"),
  };
}
