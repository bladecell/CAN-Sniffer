// REST-backed dashboard, diagnostics and PID configuration state.
import { SvelteMap } from "svelte/reactivity";
import type { PidDefinition, Obd2Status, DtcData, SystemStatus, CanBusStatus } from "./types";
import { parseSdInfo, type SdInfoWire } from "./settingsModel";
import { recordingStore } from "./recordingStore.svelte";
import { alertStore } from "./alertStore.svelte";
import { validatePidDefinitionSet } from "./pidValidation";
import { parseJsonResponse } from "./api";
import { normalizePidDefinition, parsePidList, serializePidDefinition } from "./pidDefinition";
import { createLatest } from "./latest";
import { singleFlight } from "./singleFlight";
import { createLimiter, rebooted, type UptimeSample } from "./openReload";
import { retryBusy } from "./retryBusy";
import { firmwareApi } from "./firmwareApi.svelte";
import { livePids } from "./livePidStore.svelte";
import { onConnectionDrop, onConnectionOpen } from "./connectionEvents";
import { idle as clearIdle, press as clearPress, settle as clearSettle, tick as clearTick, cancel as clearCancel, type ClearState } from "./clearFlow";
import { parseObd2Status, parseCanBusStatus, parseDtcGroups, parseDtcDescriptions, buildDtcData, emptyDtcData } from "./guards";

export class VehicleStore {
    system = $state<SystemStatus>({
        api_version: 0,
        app_version: "",
        uptime_s: 0,
        restart_reason: "",
        mac: "",
        state: 0,
        battery_voltage: 0,
        sd_card_detected: false,
        sd_card_unavailable: false,
        component_status: [],
        heap_free: 0,
        heap_min_free: 0,
        tasks: [],
    });

    // Every read the page makes shares two slots, so mount-time loads and the open reload never burst the device's HTTP server.
    private limit = createLimiter(2);
    /** Bumped on every connection drop. A resource read in the current epoch and within FRESH_MS is not read again. */
    private epoch = 0;
    private loaded = new Map<string, { epoch: number; at: number }>();
    private static FRESH_MS = 10_000;

    /** Skips a read that already succeeded this session, else runs it through the limiter. `skipped` is the result when skipped. */
    private once<T>(key: string, read: () => Promise<T>, skipped: T, ok: (result: T) => boolean = () => true): Promise<T> {
        const seen = this.loaded.get(key);
        if (seen && seen.epoch === this.epoch && Date.now() - seen.at < VehicleStore.FRESH_MS) return Promise.resolve(skipped);
        const epoch = this.epoch;
        return this.limit(async () => {
            const result = await read();
            if (ok(result)) this.loaded.set(key, { epoch, at: Date.now() });
            return result;
        });
    }

    constructor() {
        onConnectionDrop(() => {
            this.epoch++;
            this.definitionsRetries = 0;
        });
    }

    private uptimeSample: UptimeSample | null = null;
    /** A reboot was seen since the definitions were last read. */
    private definitionsStale = false;

    requestSystem = singleFlight(() => this.once("system", () => this.readSystem(), true, Boolean));

    private async readSystem(): Promise<boolean> {
        try {
            const response = await retryBusy(() => firmwareApi.system());
            const result = await parseJsonResponse<SystemStatus>(response);

            const now = Date.now();
            if (this.uptimeSample && rebooted(this.uptimeSample, result.uptime_s, now)) this.definitionsStale = true;
            this.uptimeSample = { uptimeS: result.uptime_s, at: now };
            this.system = result;
            return true;
        } catch (e) {
            console.error("Failed to load system status", e);
            return false;
        }
    }

    /** Null until read, and after a failed read (a busy card answers 503). */
    sdInfo = $state<SdInfoWire | null>(null);

    requestSDInfo = singleFlight(() => this.once("sdInfo", () => this.readSDInfo(), true, Boolean));

    private async readSDInfo(): Promise<boolean> {
        try {
            const response = await retryBusy(() => firmwareApi.sdInfo());
            this.sdInfo = response.ok ? parseSdInfo(await parseJsonResponse<unknown>(response)) : null;
            return response.ok;
        } catch (e) {
            this.sdInfo = null;
            console.error("Failed to load SD card info", e);
            return false;
        }
    }

    pidDefinitions = $state<PidDefinition[]>([]);

    /** Reads that run on every connection share one request while it is in flight. */
    loadDefinitions = singleFlight(() => this.once("definitions", () => this.readDefinitions(), true, Boolean));

    /** On a reconnect the PID set is read again only when the device restarted (its PIDs are loaded from the card at boot). */
    reloadDefinitionsIfChanged() {
        if (!this.definitionsStale && this.pidDefinitions.length > 0) return Promise.resolve(true);
        this.loaded.delete("definitions");
        return this.loadDefinitions();
    }

    private definitionsRetries = 0;
    private definitionsTimer: ReturnType<typeof setTimeout> | undefined;
    /** A read that failed even after its retries is tried again in 15 s, up to 4 times per connection. */
    private retryDefinitionsLater() {
        if (this.definitionsRetries >= 4 || this.definitionsTimer) return;
        this.definitionsRetries++;
        const epoch = this.epoch;
        this.definitionsTimer = setTimeout(() => {
            this.definitionsTimer = undefined;
            if (epoch !== this.epoch) return;
            this.loaded.delete("definitions");
            void this.loadDefinitions();
        }, 15_000);
    }

    private async readDefinitions(): Promise<boolean> {
        try {
            // The PID map is busy while OBD2 starts, so this read waits longer than the others (it was a 5 s loop).
            const response = await retryBusy(() => firmwareApi.pidDefinitions(), { attempts: 6 });
            const definitions = parsePidList(await parseJsonResponse<unknown>(response));
            const currentPids = new Set(definitions.map((definition) => definition.pid));
            for (const pid of livePids.keys()) {
                if (!currentPids.has(pid)) livePids.delete(pid);
            }
            this.pidDefinitions = definitions;
            this.definitionsStale = false;
            this.definitionsRetries = 0;
            return true;
        } catch (e) {
            console.error("Failed to load PIDs", e);
            this.retryDefinitionsLater();
            return false;
        }
    }

    canStatus = $state<CanBusStatus | null>(null);
    canCurrentRate = $state(5000);

    getCanStatus = singleFlight(() => this.once("canStatus", () => this.readCanStatus(), true, Boolean));

    private async readCanStatus(): Promise<boolean> {
        try {
            const response = await retryBusy(() => firmwareApi.canBus());
            this.canStatus = parseCanBusStatus(await parseJsonResponse<unknown>(response));
            return true;
        } catch (e) {
            console.error("Failed to load CAN status", e);
            return false;
        }
    }

    obd2Status = $state<Obd2Status | null>(null);

    getObd2Status = singleFlight(() => this.once("obd2", () => this.readObd2Status(), true, Boolean));

    private async readObd2Status(): Promise<boolean> {
        try {
            const response = await retryBusy(() => firmwareApi.obd2());
            const result = parseObd2Status(await parseJsonResponse<unknown>(response));

            const supportedGroupsMap = new SvelteMap<number, boolean>();
            result.supported_pids.groups.forEach((mask, groupIndex) => {
                for (let i = 0; i < 32; i++) {
                    supportedGroupsMap.set(groupIndex * 32 + i + 1, ((mask >>> (31 - i)) & 1) === 1);
                }
            });

            this.obd2Status = {
                ...result,
                supported_pids: { count: result.supported_pids.count, groups: supportedGroupsMap },
            };
            return true;
        } catch (e) {
            console.error("Failed to load OBD2 status", e);
            return false;
        }
    }

    async setAutoPoll(on: boolean) {
        try {
            const response = await firmwareApi.setAutoPoll(on);
            const result = await parseJsonResponse<{ auto_poll: boolean; polling: boolean }>(response);
            // The PATCH reply carries both fields that change; a full read is needed only when nothing is loaded yet.
            if (this.obd2Status) {
                this.obd2Status = { ...this.obd2Status, auto_poll: result.auto_poll, polling: result.polling };
            } else {
                await this.readObd2Status();
            }
        } catch (e) {
            console.error("Failed to set OBD2 auto poll", e);
        }
    }

    vin = $state("");

    getVin = singleFlight(() => this.once("vin", () => this.readVin(), true, Boolean));

    private async readVin(): Promise<boolean> {
        try {
            const response = await retryBusy(() => firmwareApi.vin());
            const result = await parseJsonResponse<{ vin: string }>(response);

            this.vin = result.vin;
            return true;
        } catch (e) {
            console.error("Failed to load VIN", e);
            return false;
        }
    }

    async requestVin() {
        try {
            const response = await firmwareApi.requestVin();
            await parseJsonResponse(response);
            await this.readVin();
        } catch (e) {
            console.error("Failed to request VIN", e);
        }
    }

    dtc = $state<DtcData>(emptyDtcData());
    totalDTCs = $state(0);
    private dtcLatest = createLatest();

    getDTC = singleFlight(() => this.once("dtc", () => this.readDtc(), true, Boolean));

    private async readDtc(): Promise<boolean> {
        // Overlapping calls: only the newest one may write the store.
        const isCurrent = this.dtcLatest.begin();
        try {
            const response = await retryBusy(() => firmwareApi.dtc());
            const groups = parseDtcGroups(await parseJsonResponse<unknown>(response));
            if (!isCurrent()) return false;

            const codesArray = Array.from(new Set(groups.flatMap((g) => g.dtc)));
            const descriptions: Record<string, string> = {};
            // GET /dtc/descriptions takes at most 30 codes.
            const CHUNK_SIZE = 30;

            for (let i = 0; i < codesArray.length; i += CHUNK_SIZE) {
                const codeChunk = codesArray.slice(i, i + CHUNK_SIZE);

                try {
                    const descResponse = await firmwareApi.dtcDescriptions(codeChunk);
                    Object.assign(descriptions, parseDtcDescriptions(await parseJsonResponse<unknown>(descResponse)));
                } catch (chunkErr) {
                    // If one chunk fails, log it but don't break the whole app
                    console.error(`Failed to fetch descriptions for chunk ${codeChunk.join(",")}:`, chunkErr);
                }
                if (!isCurrent()) return false;
            }

            // The response may omit an empty mode; building from scratch cannot leave stale codes.
            this.dtc = buildDtcData(groups, descriptions);
            this.totalDTCs =
                this.dtc.confirmed.dtc_count +
                this.dtc.pending.dtc_count +
                this.dtc.permanent.dtc_count;
            return true;
        } catch (e) {
            console.error("Failed to load DTC", e);
            return false;
        }
    }

    async requestDTC(mode = -1) {
        try {
            const response = await firmwareApi.requestDtc(mode === -1 ? undefined : mode as 3 | 7 | 10);
            const result = await parseJsonResponse<{ status?: string; reason?: string }>(response);
            await this.readDtc();

            if (mode === -1 && result.status === "partial_success") {
                alertStore.add(result.reason || "DTC refresh incomplete.", "warning");
            }
        } catch (e) {
            console.error("Failed to request DTC", e);
            const reason = e instanceof Error ? `: ${e.message}` : "";
            alertStore.add(`Failed to request DTCs${reason}`, "error");
        }
    }

    /** Two-press clear (clearFlow.ts): press 1 arms, press 2 sends one POST. */
    clearState = $state<ClearState>(clearIdle);
    private clearTimer: ReturnType<typeof setTimeout> | null = null;

    get isClearing() {
        return this.clearState.kind === "sending";
    }

    /** POST clear_dtc with the session token. Returns true when the ECU acknowledged. */
    private async postClear(): Promise<boolean> {
        try {
            const response = await firmwareApi.clearDtc();
            await parseJsonResponse(response);
            return true;
        } catch (e) {
            console.error("Failed to clear DTCs:", e);
            const reason = e instanceof Error ? `: ${e.message}` : "";
            alertStore.add(`Failed to clear DTCs.${reason}`, "error");
            return false;
        }
    }

    private setClearState(next: ClearState) {
        this.clearState = next;
        if (this.clearTimer) clearTimeout(this.clearTimer);
        this.clearTimer = null;
        if (next.kind === "armed") {
            // Revert on expiry. Never sends anything.
            this.clearTimer = setTimeout(() => this.setClearState(clearTick(this.clearState, performance.now())), Math.max(0, next.expiresAt - performance.now()));
        }
    }

    /** The Clear Codes button. Press 1 arms it, press 2 (while armed) clears. */
    pressClearDTCs = async () => {
        const p = clearPress(this.clearState, performance.now());
        this.setClearState(p.state);
        if (!p.send) return;
        const ok = await this.postClear();
        this.setClearState(clearSettle());
        if (ok) await this.requestDTC();
    };

    /** Blur or Escape: disarm without sending anything. */
    cancelClearDTCs = () => {
        this.setClearState(clearCancel(this.clearState));
    };

    /** Apply, step 1: PUT the whole set to the device, then reload it. Returns the failure reason, or null. */
    async writePidDefinitions(definitions: PidDefinition[]): Promise<string | null> {
        try {
            const normalized = definitions.map((definition) => normalizePidDefinition(definition));
            const validationError = validatePidDefinitionSet(normalized);
            if (validationError) throw new Error(validationError);
            const payload = normalized.map((definition) => serializePidDefinition(definition));

            const payloadBytes = new TextEncoder().encode(JSON.stringify(payload)).byteLength;
            if (payloadBytes > 64 * 1024) throw new Error("PID definitions payload exceeds 64 KiB (HTTP 413).");

            const response = await firmwareApi.replacePidDefinitions(payload);
            if (response.status !== 204) {
                // An error reply carries the device's reason; parseJsonResponse throws it.
                await parseJsonResponse(response);
                throw new Error(`Expected HTTP 204 from PID replacement, received ${response.status}.`);
            }

            if (!(await this.readDefinitions())) alertStore.add("PIDs were replaced, but refreshing definitions failed.", "warning");
            return null;
        } catch (e) {
            console.error("Failed to replace PIDs:", e);
            return e instanceof Error ? e.message : "Unknown error";
        }
    }

    /** Apply, step 2: POST /pid_def/save writes the device's set to its configured file. Returns the failure reason, or null. */
    async saveConfiguredPids(): Promise<string | null> {
        try {
            const result = await parseJsonResponse<{ status?: string; reason?: string }>(await firmwareApi.savePidDefinitions());
            return result.status === "success" ? null : result.reason || "Unknown error";
        } catch (e) {
            console.error("Failed to save PIDs:", e);
            return e instanceof Error ? e.message : "Unknown error";
        }
    }
}
export const vehicleStore = new VehicleStore();
// The reads run through the store's limiter. System goes first: its uptime says whether the device rebooted and the PID set must be re-read.
onConnectionOpen(() => {
    void (async () => {
        await vehicleStore.requestSystem();
        await Promise.allSettled([
            vehicleStore.reloadDefinitionsIfChanged(),
            vehicleStore.getObd2Status(),
            vehicleStore.getVin(),
            vehicleStore.getDTC(),
            vehicleStore.getCanStatus(),
            vehicleStore.requestSDInfo(),
            recordingStore.refreshIfStale(),
        ]);
    })();
});
