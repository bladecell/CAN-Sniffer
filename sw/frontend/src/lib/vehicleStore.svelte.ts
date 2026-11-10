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
import { runLimited } from "./openReload";
import { firmwareApi } from "./firmwareApi.svelte";
import { livePids } from "./livePidStore.svelte";
import { onConnectionOpen } from "./connectionEvents";
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
        component_status: [],
        heap_free: 0,
        heap_min_free: 0,
        tasks: [],
    });

    requestSystem = singleFlight(() => this.readSystem());

    private async readSystem() {
        try {
            const response = await firmwareApi.system();
            const result = await parseJsonResponse<SystemStatus>(response);

            this.system = result;
        } catch (e) {
            console.error("Failed to load system status", e);
        }
    }

    /** Null until read, and after a failed read (a busy card answers 503). */
    sdInfo = $state<SdInfoWire | null>(null);

    requestSDInfo = singleFlight(() => this.readSDInfo());

    private async readSDInfo() {
        try {
            const response = await firmwareApi.sdInfo();
            this.sdInfo = response.ok ? parseSdInfo(await parseJsonResponse<unknown>(response)) : null;
        } catch (e) {
            this.sdInfo = null;
            console.error("Failed to load SD card info", e);
        }
    }

    pidDefinitions = $state<PidDefinition[]>([]);

    /** Reads that run on every connection share one request while it is in flight. */
    loadDefinitions = singleFlight(() => this.readDefinitions());

    private async readDefinitions(): Promise<boolean> {
        try {
            const response = await firmwareApi.pidDefinitions();
            const definitions = parsePidList(await parseJsonResponse<unknown>(response));
            const currentPids = new Set(definitions.map((definition) => definition.pid));
            for (const pid of livePids.keys()) {
                if (!currentPids.has(pid)) livePids.delete(pid);
            }
            this.pidDefinitions = definitions;
            return true;
        } catch (e) {
            console.error("Failed to load PIDs", e);
            return false;
        }
    }

    canStatus = $state<CanBusStatus | null>(null);
    canCurrentRate = $state(5000);

    getCanStatus = singleFlight(() => this.readCanStatus());

    private async readCanStatus() {
        try {
            const response = await firmwareApi.canBus();
            this.canStatus = parseCanBusStatus(await parseJsonResponse<unknown>(response));
        } catch (e) {
            console.error("Failed to load CAN status", e);
        }
    }

    obd2Status = $state<Obd2Status | null>(null);

    getObd2Status = singleFlight(() => this.readObd2Status());

    private async readObd2Status() {
        try {
            const response = await firmwareApi.obd2();
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

        } catch (e) {
            console.error("Failed to load OBD2 status", e);
        }
    }

    async setAutoPoll(on: boolean) {
        try {
            const response = await firmwareApi.setAutoPoll(on);
            const result = await parseJsonResponse<{ auto_poll: boolean; polling: boolean }>(response);
            if (this.obd2Status) {
                this.obd2Status = { ...this.obd2Status, auto_poll: result.auto_poll, polling: result.polling };
            }

            await this.readObd2Status();
        } catch (e) {
            console.error("Failed to set OBD2 auto poll", e);
        }
    }

    vin = $state("");

    getVin = singleFlight(() => this.readVin());

    private async readVin() {
        try {
            const response = await firmwareApi.vin();
            const result = await parseJsonResponse<{ vin: string }>(response);

            this.vin = result.vin;
        } catch (e) {
            console.error("Failed to load VIN", e);
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

    getDTC = singleFlight(() => this.readDtc());

    private async readDtc() {
        // Overlapping calls: only the newest one may write the store.
        const isCurrent = this.dtcLatest.begin();
        try {
            const response = await firmwareApi.dtc();
            const groups = parseDtcGroups(await parseJsonResponse<unknown>(response));
            if (!isCurrent()) return;

            const codesArray = Array.from(new Set(groups.flatMap((g) => g.dtc)));
            const descriptions: Record<string, string> = {};
            const CHUNK_SIZE = 15;

            for (let i = 0; i < codesArray.length; i += CHUNK_SIZE) {
                const codeChunk = codesArray.slice(i, i + CHUNK_SIZE);

                try {
                    const descResponse = await firmwareApi.dtcDescriptions(codeChunk);
                    Object.assign(descriptions, parseDtcDescriptions(await parseJsonResponse<unknown>(descResponse)));
                } catch (chunkErr) {
                    // If one chunk fails, log it but don't break the whole app
                    console.error(`Failed to fetch descriptions for chunk ${codeChunk.join(",")}:`, chunkErr);
                }
                if (!isCurrent()) return;
            }

            // The response may omit an empty mode; building from scratch cannot leave stale codes.
            this.dtc = buildDtcData(groups, descriptions);
            this.totalDTCs =
                this.dtc.confirmed.dtc_count +
                this.dtc.pending.dtc_count +
                this.dtc.permanent.dtc_count;
        } catch (e) {
            console.error("Failed to load DTC", e);
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
// Staggered so the device's HTTP server is not hit with every read at once.
const OPEN_RELOAD_CONCURRENCY = 2;
onConnectionOpen(() => {
    void runLimited(
        [
            () => vehicleStore.loadDefinitions(),
            () => vehicleStore.getObd2Status(),
            () => vehicleStore.getVin(),
            () => vehicleStore.getDTC(),
            () => vehicleStore.requestSystem(),
            () => vehicleStore.getCanStatus(),
            () => vehicleStore.requestSDInfo(),
            () => recordingStore.refresh(),
        ],
        OPEN_RELOAD_CONCURRENCY,
    );
});
