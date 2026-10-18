// src/lib/canStore.svelte.ts
import { SvelteMap } from "svelte/reactivity";
import { alertStore } from "./alertStore.svelte";
import type {
    PidValue,
    WsCanStatus,
    PidDefinition,
    Obd2Status,
    DtcData,
    DtcModeData,
    singleDtc,
    SystemStatus,
    SDCardInfo,
    CanBusStatus
} from "./types";
import { validatePidDefinitionSet } from "./pidValidation";
import { parseJsonResponse, parseJsonResponseAllowEmpty } from "./api";
import { auth } from "./auth.svelte";
import { normalizePidDefinition, parsePidList, serializePidDefinition } from "./pidDefinition";
import type { PidRow } from "./telemetryStore.svelte";
import { parseWsFrame, type WsFrame } from "./wsFrames";
import { dispatchToListeners } from "./dispatch";
import { reconnectDelay } from "./backoff";
import { createHeartbeat, CONNECT_TIMEOUT_MS } from "./heartbeat";
import { createStreamRefCount } from "./streamRefCount";
import { createThrottle } from "./throttle";
import { createLatest } from "./latest";
import {
    idle as clearIdle,
    press as clearPress,
    settle as clearSettle,
    tick as clearTick,
    cancel as clearCancel,
    type ClearState,
} from "./clearFlow";
import {
    parseObd2Status,
    parseCanBusStatus,
    parseDtcGroups,
    parseDtcDescriptions,
    buildDtcData,
    emptyDtcData,
} from "./guards";

export const COMMANDS = {
    START_LOG: 0xa0,
    STOP_LOG: 0xa1,
    PING_REQUEST: 0xa2,
    PING_RESPONSE: 0xa3,
};

export type MessageListener = (update: {
    pid: number;
    value: number;
    timestamp: number;
}) => void;

export class CanStore {
    connected = $state(false);
    pids = new SvelteMap<number, PidValue>();
    listeners = new Set<MessageListener>();
    isStreaming = $state(false);
    socket: WebSocket | null = null;
    pollInterval: ReturnType<typeof setInterval> | null = null;

    intentionalDisconnect = false;
    reconnectTimeout: ReturnType<typeof setTimeout> | null = null;
    reconnectAttempts = 0;
    outageAlerted = false;
    pageshowBound = false;

    // The server's PID stream flag is global: send 0xA0 on the first user and 0xA1 when the last one leaves.
    private streamRefs = createStreamRefCount(
        () => { this.sendByte(COMMANDS.START_LOG); this.isStreaming = true; },
        () => { this.sendByte(COMMANDS.STOP_LOG); this.isStreaming = false; },
    );

    connectTimer: ReturnType<typeof setTimeout> | null = null;

    private heartbeat = createHeartbeat({
        ping: () => {
            if (this.socket?.readyState !== WebSocket.OPEN) return false;
            this.socket.send(new Uint8Array([COMMANDS.PING_REQUEST]));
            return true;
        },
        onTimeout: () => {
            console.warn("Heartbeat missed! ESP32 appears offline.");
            if (this.socket) this.socket.onclose = null;
            this.socket?.close();
            this.handleConnectionDrop();
        },
        // Reset the backoff only once the link has answered, so a socket that opens and dies keeps backing off.
        onPong: () => {
            this.reconnectAttempts = 0;
        },
    });

    connect() {
        if (this.socket) return;

        this.intentionalDisconnect = false;

        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }

        if (!this.pageshowBound && typeof window !== "undefined") {
            this.pageshowBound = true;
            window.addEventListener("pageshow", (e) => this.handlePageShow(e));
        }

        const host = window.location.host;
        const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
        const wsUrl = `${protocol}//${host}/ws`;

        console.log(`Connecting to ${wsUrl}...`);

        try {
            this.socket = new WebSocket(wsUrl);
        } catch (e) {
            console.error("WebSocket construction failed", e);
            this.handleConnectionDrop();
            return;
        }
        this.socket.binaryType = "arraybuffer";

        const socket = this.socket;
        this.clearConnectTimer();
        this.connectTimer = setTimeout(() => {
            this.connectTimer = null;
            if (socket.readyState === WebSocket.OPEN) return;
            console.warn("WebSocket connect timed out");
            socket.onclose = null;
            socket.close();
            this.handleConnectionDrop();
        }, CONNECT_TIMEOUT_MS);

        this.socket.onopen = () => {
            console.log("WebSocket Connected");
            alertStore.add("WebSocket Connected", "success");
            this.clearConnectTimer();
            this.connected = true;
            this.outageAlerted = false;
            // A new connection may find the server flag off (it resets when clients leave).
            if (this.streamRefs.count() > 0) this.sendByte(COMMANDS.START_LOG);
            this.isStreaming = this.streamRefs.count() > 0;

            this.heartbeat.start();

            this.loadDefinitions();
            this.getObd2Status();

            this.getVin();
            this.getDTC();
            // Session and claim state come from GET /auth; a reconnect may follow a reboot that cleared the token.
            auth.refresh();
        };

        this.socket.onclose = () => {
            console.log("WebSocket Disconnected cleanly.");
            this.handleConnectionDrop();
        };

        this.socket.onmessage = (event) => {
            try {
                this.handleMessage(event.data);
            } catch (e) {
                console.error("Failed to handle WebSocket message", e);
            }
        };
    }

    // A page restored from the back/forward cache has a dead socket and no timers.
    handlePageShow(e: PageTransitionEvent) {
        if (this.intentionalDisconnect) return;
        if (e.persisted && this.socket && this.socket.readyState !== WebSocket.OPEN) {
            this.socket.onclose = null;
            this.socket.close();
            this.handleConnectionDrop(true);
        }
        if (!this.socket) {
            if (this.reconnectTimeout) {
                clearTimeout(this.reconnectTimeout);
                this.reconnectTimeout = null;
            }
            this.connect();
        }
    }

    clearConnectTimer() {
        if (this.connectTimer) clearTimeout(this.connectTimer);
        this.connectTimer = null;
    }

    handleConnectionDrop(silent = false) {
        this.clearConnectTimer();
        this.connected = false;
        this.isStreaming = false;
        this.socket = null;
        this.wsCanStatus.state = "Not Connected";

        this.heartbeat.stop();

        if (!this.intentionalDisconnect) {
            if (!this.outageAlerted && !silent) {
                this.outageAlerted = true;
                alertStore.add("WebSocket Disconnected, reconnecting...", "error");
            }
            if (this.reconnectTimeout) clearTimeout(this.reconnectTimeout);
            const delay = reconnectDelay(this.reconnectAttempts++);
            this.reconnectTimeout = setTimeout(() => {
                this.reconnectTimeout = null;
                this.connect();
            }, delay);
        }
    }

    disconnect() {
        console.log("WebSocket disconnecting manually.");

        this.intentionalDisconnect = true;

        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }

        this.clearConnectTimer();
        this.heartbeat.stop();

        if (this.socket) {
            this.socket.close(1000, "User disconnected");
            this.socket = null;
        }

        if (this.pollInterval) {
            clearInterval(this.pollInterval);
            this.pollInterval = null;
        }

        if (this.canStatus) {
            this.canStatus = { ...this.canStatus, state: "not_connected" };
        }
    }

    /** Ask for the PID stream. Returns a release function; call it when the view goes away. */
    acquireStream(): () => void {
        return this.streamRefs.acquire();
    }

    sendByte(byte: number) {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            const buffer = new Uint8Array([byte]);
            this.socket.send(buffer);
        } else {
            console.warn("Cannot send command: WebSocket not open");
        }
    }

    handleMessage(data: unknown) {
        const frame = parseWsFrame(data);
        if (!frame) return;
        switch (frame.kind) {
            case "pid":
                this.applyPidFrame(frame);
                break;
            case "canStatus":
                this.applyCanStatusFrame(frame);
                break;
            case "pong":
                this.heartbeat.pong();
                break;
        }
    }

    subscribe(callback: MessageListener) {
        this.listeners.add(callback);
        // Return an unsubscribe function to prevent memory leaks
        return () => this.listeners.delete(callback);
    }

    wsCanStatus = $state<WsCanStatus>({
        state: "Not Connected",
        utilization: 0,
        battery_voltage: 0,
    });


    battery_below_12v_alerted = $state(false);

    private utilizationAlertAllowed = createThrottle(60_000);

    applyCanStatusFrame(frame: Extract<WsFrame, { kind: "canStatus" }>) {
        const previousState = this.wsCanStatus?.state;
        const utilization = frame.utilization;
        const battery_voltage = frame.batteryVoltage;
        const can_bus_state_name = [
            "CAN Not Initialized",
            "CAN Off",
            "CAN Not Connected",
            "CAN Connected",
        ];

        this.wsCanStatus = {
            state: can_bus_state_name[frame.state] || "Unknown",
            utilization: Math.round(utilization * 100),
            battery_voltage: battery_voltage,
        };

        const currentState = this.wsCanStatus?.state;

        if (previousState !== currentState) {
            if (currentState === "CAN Connected") {
                alertStore.add("CAN Bus " + currentState, "success");
            } else {
                alertStore.add("CAN Bus " + currentState, "error");
            }
        }

        if (utilization > 0.9 && this.utilizationAlertAllowed()) {
            alertStore.add("CAN Bus Utilization is above 90%", "warning");
        }

        if (
            battery_voltage < 11.8 &&
            battery_voltage > 1 &&
            !this.battery_below_12v_alerted
        ) {
            alertStore.add("Car battery voltage is below 12V", "warning");
            this.battery_below_12v_alerted = true;
        } else if (battery_voltage >= 12 && this.battery_below_12v_alerted) {
            this.battery_below_12v_alerted = false;
        }
    }

    applyPidFrame(frame: Extract<WsFrame, { kind: "pid" }>) {
        const value = Number(frame.value.toFixed(2));

        this.pids.set(frame.pid, {
            value,
            lastUpdated: frame.lastUpdated,
            rate: frame.rate,
            isValid: frame.isValid,
            isSupported: frame.isSupported,
        });

        dispatchToListeners(this.listeners, {
            pid: frame.pid,
            value,
            timestamp: frame.lastUpdated,
        });
    }

    system = $state<SystemStatus>({
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

    async requestSystem() {
        try {
            const response = await fetch("/api/v1/system");
            const result = await parseJsonResponse<SystemStatus>(response);

            this.system = result;
        } catch (e) {
            console.error("Failed to load PIDs", e);
        }
    }

    sdInfo = $state<SDCardInfo | null>(null);

    async requestSDInfo() {
        try {
            const response = await fetch("/api/v1/sd_card/info");
            const result = await parseJsonResponse<SDCardInfo>(response);

            this.sdInfo = result;
        } catch (e) {
            console.error("Failed to load SD card info", e);
        }
    }

    pidDefinitions = $state<PidDefinition[]>([]);

    async loadDefinitions(): Promise<boolean> {
        try {
            const response = await fetch("/api/v1/pid_def");
            this.pidDefinitions = parsePidList(await parseJsonResponse<unknown>(response));
            return true;
        } catch (e) {
            console.error("Failed to load PIDs", e);
            return false;
        }
    }

    canStatus = $state<CanBusStatus | null>(null);
    canCurrentRate = $state(5000);

    async getCanStatus() {
        try {
            const response = await fetch("/api/v1/can_bus");
            this.canStatus = parseCanBusStatus(await parseJsonResponse<unknown>(response));
        } catch (e) {
            console.error("Failed to load CAN status", e);
        }
    }

    obd2Status = $state<Obd2Status | null>(null);

    async getObd2Status() {
        try {
            const response = await fetch("/api/v1/obd2");
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
            const response = await fetch("/api/v1/obd2", {
                method: "PATCH",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ auto_poll: on }),
            });
            const result = await parseJsonResponse<{ auto_poll: boolean; polling: boolean }>(response);
            if (this.obd2Status) {
                this.obd2Status = { ...this.obd2Status, auto_poll: result.auto_poll, polling: result.polling };
            }

            this.getObd2Status();
        } catch (e) {
            console.error("Failed to set OBD2 auto poll", e);
        }
    }

    vin = $state("");

    async getVin() {
        try {
            const response = await fetch("/api/v1/vin");
            const result = await parseJsonResponse<{ vin: string }>(response);

            this.vin = result.vin;
        } catch (e) {
            console.error("Failed to load VIN", e);
        }
    }

    async requestVin() {
        try {
            const response = await fetch("/api/v1/req/vin", {
                method: "POST",
            });
            await parseJsonResponse(response);
            this.getVin();
        } catch (e) {
            console.error("Failed to request VIN", e);
        }
    }

    dtc = $state<DtcData>(emptyDtcData());
    totalDTCs = $state(0);
    private dtcLatest = createLatest();

    async getDTC() {
        // Overlapping calls: only the newest one may write the store.
        const isCurrent = this.dtcLatest.begin();
        try {
            const response = await fetch("/api/v1/dtc");
            const groups = parseDtcGroups(await parseJsonResponse<unknown>(response));
            if (!isCurrent()) return;

            const codesArray = Array.from(new Set(groups.flatMap((g) => g.dtc)));
            const descriptions: Record<string, string> = {};
            const CHUNK_SIZE = 15;

            for (let i = 0; i < codesArray.length; i += CHUNK_SIZE) {
                const codesParam = codesArray.slice(i, i + CHUNK_SIZE).map(encodeURIComponent).join(",");

                try {
                    const descResponse = await fetch(`/api/v1/dtc?codes=${codesParam}`);
                    Object.assign(descriptions, parseDtcDescriptions(await parseJsonResponse<unknown>(descResponse)));
                } catch (chunkErr) {
                    // If one chunk fails, log it but don't break the whole app
                    console.error(`Failed to fetch descriptions for chunk ${codesParam}:`, chunkErr);
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
            const request_url =
                mode === -1 ? "/api/v1/req/dtc" : `/api/v1/req/dtc?mode=${mode}`;
            const response = await fetch(request_url, {
                method: "POST",
            });
            const result = await parseJsonResponse<{ status?: string; reason?: string }>(response);
            await this.getDTC();

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
            const response = await auth.gatedFetch("/api/v1/req/clear_dtc", { method: "POST" });
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

    serializePidDefinition(def: unknown) {
        return serializePidDefinition(normalizePidDefinition(def));
    }

    async replacePids(rows: PidRow[]): Promise<boolean> {
        try {
            const desiredDefinitions = rows
                .filter((row) => row.loaded)
                .map((row) => normalizePidDefinition(row.def));

            const validationError = validatePidDefinitionSet(desiredDefinitions);
            if (validationError) {
                throw new Error(validationError);
            }

            const payload = desiredDefinitions.map((definition) =>
                this.serializePidDefinition(definition),
            );
            const serializedPayload = JSON.stringify(payload);
            const payloadBytes = new TextEncoder().encode(serializedPayload).byteLength;
            if (payloadBytes > 64 * 1024) {
                throw new Error("PID definitions payload exceeds 64 KiB (HTTP 413).");
            }

            const response = await auth.gatedFetch("/api/v1/pid_def", {
                method: "PUT",
                headers: { "Content-Type": "application/json" },
                body: serializedPayload,
            });

            if (response.status !== 204) {
                throw new Error(`Expected HTTP 204 from PID replacement, received ${response.status}.`);
            }

            if (!(await this.loadDefinitions())) {
                alertStore.add("PIDs were replaced, but refreshing definitions failed.", "warning");
            }
            return true;
        } catch (e) {
            const reason = e instanceof Error ? e.message : "Unknown error";
            console.error("Failed to replace PIDs:", e);
            alertStore.add(`Failed to replace PIDs: ${reason}`, "error");
            return false;
        }
    }

    async savePids(rows: PidRow[]) {
        if (!(await this.replacePids(rows))) {
            return;
        }

        try {
            const response = await auth.gatedFetch('/api/v1/pid_def/save', {
                method: 'POST'
            });

            const result = await parseJsonResponse<{ status?: string; reason?: string }>(response);
            if (result.status === "success") {
                alertStore.add("PIDs saved successfully to FS.", "success");
            } else {
                throw new Error(result.reason || "Unknown error");
            }
        } catch (e) {
            console.error("Failed to save PIDs:", e);
            alertStore.add("Failed to save PIDs.", "error");
        }
    }


    async updatePids(rows: PidRow[]) {
        if (await this.replacePids(rows)) {
            alertStore.add("Successfully synced PIDs with device.", "success");
        }
    }

    isRecording = $state(false);


}

export const canStore = new CanStore();
