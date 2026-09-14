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
    SDCardInfo
} from "./types";
import { validatePidDefinitionSet } from "./pidValidation";

function normalizePidDefinition(def: any): PidDefinition {
    if (!def || typeof def !== "object") {
        throw new Error("Invalid PID definition");
    }

    const normalized = {
        id: def.id,
        mode: def.mode,
        pid: def.pid,
        length: def.length ?? def.len,
        name: def.name,
        unit: def.unit,
        description: def.description ?? def.desc,
        formula: def.formula,
        minValue: def.minValue ?? def.minV,
        maxValue: def.maxValue ?? def.maxV,
        update_interval_ms: def.update_interval_ms ?? def.interval,
        color: def.color,
        priority: def.priority,
        icon: def.icon,
    };

    if (Object.values(normalized).some((value) => value === undefined || value === null)) {
        throw new Error("Incomplete PID definition");
    }

    return normalized;
}

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
    pollInterval: any = null;

    intentionalDisconnect = false;
    reconnectTimeout: ReturnType<typeof setTimeout> | null = null;

    // Heartbeat timers
    pingInterval: ReturnType<typeof setInterval> | null = null;
    pongTimeout: ReturnType<typeof setTimeout> | null = null;

    connect() {
        if (this.socket) return;

        this.intentionalDisconnect = false;

        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }

        const host = window.location.host;
        const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
        const wsUrl = `${protocol}//${host}/ws`;

        console.log(`Connecting to ${wsUrl}...`);

        this.socket = new WebSocket(wsUrl);
        this.socket.binaryType = "arraybuffer";

        this.socket.onopen = () => {
            console.log("WebSocket Connected");
            alertStore.add("WebSocket Connected", "success");
            this.connected = true;

            this.startHeartbeat();

            this.loadDefinitions();
            this.getObd2Status();

            this.getVin();
            this.getDTC();
        };

        this.socket.onclose = () => {
            console.log("WebSocket Disconnected cleanly.");
            this.handleConnectionDrop();
        };

        this.socket.onmessage = (event) => {
            // Check if the message is an ArrayBuffer (binary data)
            if (event.data instanceof ArrayBuffer) {
                const buffer = new Uint8Array(event.data);
            }

            this.handleMessage(event.data);
        };
    }

    // --- HEARTBEAT LOGIC ---

    startHeartbeat() {
        this.pingInterval = setInterval(() => {
            if (this.socket && this.socket.readyState === WebSocket.OPEN) {
                const pingCommand = new Uint8Array([COMMANDS.PING_REQUEST]);
                this.socket.send(pingCommand);

                this.pongTimeout = setTimeout(() => {
                    console.warn("Heartbeat missed! ESP32 appears offline.");
                    // alertStore.add("Connection lost to device", "warning");

                    if (this.socket) {
                        this.socket.onclose = null;

                        this.socket.close();
                    }

                    this.handleConnectionDrop();
                }, 1000);
            }
        }, 2000);
    }

    handleConnectionDrop() {
        this.connected = false;
        this.isStreaming = false;
        this.socket = null;
        this.wsCanStatus.state = "Not Connected";

        this.stopHeartbeat();

        if (!this.intentionalDisconnect) {
            alertStore.add("WebSocket Disconnected, reconnecting...", "error");
            this.reconnectTimeout = setTimeout(() => this.connect(), 2000);
        }
    }

    handlePong() {
        if (this.pongTimeout) {
            clearTimeout(this.pongTimeout);
            this.pongTimeout = null;
        }
    }

    stopHeartbeat() {
        if (this.pingInterval) clearInterval(this.pingInterval);
        if (this.pongTimeout) clearTimeout(this.pongTimeout);
        this.pingInterval = null;
        this.pongTimeout = null;
    }

    disconnect() {
        console.log("WebSocket disconnecting manually.");

        this.intentionalDisconnect = true;

        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
            this.reconnectTimeout = null;
        }

        this.stopHeartbeat();

        if (this.socket) {
            this.socket.close(1000, "User disconnected");
            this.socket = null;
        }

        if (this.pollInterval) {
            clearInterval(this.pollInterval);
            this.pollInterval = null;
        }

        if (this.canStatus) {
            this.canStatus = { ...this.canStatus, state: "disconnected" };
        }
    }

    startLogging() {
        console.log("Sending START_LOG");
        this.sendByte(COMMANDS.START_LOG);
        this.isStreaming = true;
    }

    stopLogging() {
        console.log("Sending STOP_LOG");
        this.sendByte(COMMANDS.STOP_LOG);
        this.isStreaming = false;
    }

    toggleLogging() {
        if (this.isStreaming) {
            this.stopLogging();
        } else {
            this.startLogging();
        }
    }

    sendByte(byte: number) {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            const buffer = new Uint8Array([byte]);
            this.socket.send(buffer);
        } else {
            console.warn("Cannot send command: WebSocket not open");
        }
    }

    handleMessage(data: any) {
        if (!(data instanceof ArrayBuffer)) return;
        const view = new DataView(data);
        const type = view.getUint8(0);

        switch (type) {
            case 0x02:
                this.parsePidPacket(view);
                break;
            case 0x03:
                this.parseCanStatusPacket(view);
                break;
            case COMMANDS.PING_RESPONSE:
                this.handlePong();
                break;
            default:
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

    parseCanStatusPacket(view: DataView) {
        const previousState = this.wsCanStatus?.state;
        const length = view.getUint8(1);
        if (length !== view.byteLength - 2) {
            console.warn("Invalid Can Status packet length");
            return;
        }
        const stateIdx = view.getUint8(2);
        const utilization = view.getFloat32(3, true);
        const battery_voltage = view.getFloat32(7, true);
        const can_bus_state_name = [
            "CAN Not Initialized",
            "CAN Off",
            "CAN Not Connected",
            "CAN Connected",
        ];

        this.wsCanStatus = {
            state: can_bus_state_name[stateIdx] || "Unknown",
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

        if (utilization > 0.9) {
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

    parsePidPacket(view: DataView) {
        const length = view.getUint8(1);
        if (length !== view.byteLength - 2) {
            console.warn("Invalid PID %d packet length", view.getUint32(2, true));
            return;
        }

        const fullPid = view.getUint32(2, true);

        // Calculate this once for better performance
        const rawValue = view.getFloat32(6, true);
        const value = Number(rawValue.toFixed(2));

        const time = view.getUint32(10, true);
        const rate = view.getUint32(14, true);
        const isValid = view.getUint8(18) !== 0;
        const isSupported = view.getUint8(19) !== 0;

        const existing = this.pids.get(fullPid);

        // 2. Create the completely new object using consistent property names!
        const newData = {
            value: value,
            lastUpdated: time,
            rate: rate,
            isValid: isValid,         // Matches usePidData perfectly
            isSupported: isSupported, // Matches usePidData perfectly
        };

        // 3. Trigger the SvelteMap update
        this.pids.set(fullPid, newData);

        // 4. (Optional) If you used the "Tick Sledgehammer" fix for the universal table, 
        // uncomment the line below to force the parent array to rebuild!
        // this.updateTick++;

        // 5. Fire your traditional listeners
        const update = {
            pid: fullPid,
            value: value,
            timestamp: time,
        };

        for (const listener of this.listeners) {
            listener(update);
        }
    }

    system = $state<SystemStatus[]>([]);

    async requestSystem() {
        try {
            const response = await fetch("/api/v1/system");
            const result = await response.json();

            this.system = result;
        } catch (e) {
            console.error("Failed to load PIDs", e);
        }
    }

    sdInfo = $state<SDCardInfo | null>(null);

    async requestSDInfo() {
        try {
            const response = await fetch("/api/v1/sd_card/info");
            const result = await response.json();

            this.sdInfo = result;
        } catch (e) {
            console.error("Failed to load SD card info", e);
        }
    }

    pidDefinitions = $state<PidDefinition[]>([]);

    async loadDefinitions(): Promise<boolean> {
        try {
            const response = await fetch("/api/v1/pid_def");
            if (!response.ok) {
                throw new Error(`HTTP Error: ${response.status}`);
            }

            const result = await response.json();

            if (!Array.isArray(result.data)) {
                throw new Error("Invalid PID definition response");
            }

            this.pidDefinitions = result.data.map(normalizePidDefinition);
            return true;
        } catch (e) {
            console.error("Failed to load PIDs", e);
            return false;
        }
    }

    canStatus = $state<any>(null);
    canCurrentRate = $state(5000);

    async getCanStatus() {
        try {
            const response = await fetch("/api/v1/can_bus");
            if (!response.ok) throw new Error(response.statusText);

            const result = await response.json();
            this.canStatus = result;
        } catch (e) {
            console.error("Failed to load CAN status", e);
        }
    }

    obd2Status = $state<Obd2Status | null>(null);

    async getObd2Status() {
        try {
            const response = await fetch("/api/v1/obd2");
            const result = await response.json();

            const supportedGroupsMap = new SvelteMap<number, boolean>();

            if (result.supported_pids && Array.isArray(result.supported_pids.groups)) {
                result.supported_pids.groups.forEach((mask: number, groupIndex: number) => {
                    const pidBase = groupIndex * 32;

                    for (let i = 0; i < 32; i++) {
                        const bitPosition = 31 - i;

                        const isSupported = ((mask >>> bitPosition) & 1) === 1;

                        const pidNumber = pidBase + i + 1;

                        supportedGroupsMap.set(pidNumber, isSupported);
                    }
                });
            }

            // 3. Assemble the strongly-typed object and assign it to state
            this.obd2Status = {
                continuous_running: result.continuous_running,
                pid_initialized: result.pid_initialized,
                pid_def_count: result.pid_def_count,
                pid_data_count: result.pid_data_count,
                poll_task_utilization: result.poll_task_utilization,
                supported_pids: {
                    count: result.supported_pids.count,
                    groups: supportedGroupsMap
                }
            };

        } catch (e) {
            console.error("Failed to load OBD2 status", e);
        }
    }

    async setContinuousPolling(running: boolean) {
        try {
            await fetch(`/api/v1/req/pid_poll?running=${running}`, {
                method: "POST",
            });

            this.getObd2Status();
        } catch (e) {
            console.error("Failed to set OBD2 continuous polling", e);
        }
    }

    vin = $state("");

    async getVin() {
        try {
            const response = await fetch("/api/v1/vin");
            const result = await response.json();

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

            const result = await response.json();

            if (result.status === "success") {
                this.getVin();
            }
        } catch (e) {
            console.error("Failed to request VIN", e);
        }
    }

    dtc = $state<DtcData>({
        confirmed: { mode: 3, dtc_count: 0, dtc: [] },
        pending: { mode: 7, dtc_count: 0, dtc: [] },
    });
    totalDTCs = $state(0);

    async getDTC() {
        try {
            const response = await fetch("/api/v1/dtc");
            const result = await response.json();
            const rawDtcs = result.dtcs || [];

            const uniqueCodes = new Set<string>();
            for (const group of rawDtcs) {
                if (Array.isArray(group.dtc)) {
                    group.dtc.forEach((code: string) => uniqueCodes.add(code));
                }
            }

            let descriptionsMap: Record<string, string> = {};
            const codesArray = Array.from(uniqueCodes);
            const CHUNK_SIZE = 15;

            for (let i = 0; i < codesArray.length; i += CHUNK_SIZE) {
                const chunk = codesArray.slice(i, i + CHUNK_SIZE);
                const codesParam = chunk.join(",");

                try {
                    const descResponse = await fetch(`/api/v1/dtc?codes=${codesParam}`);
                    if (!descResponse.ok) throw new Error(`HTTP Error: ${descResponse.status}`);

                    const descResult = await descResponse.json();

                    if (descResult.status === 'success' && Array.isArray(descResult.dtcs)) {
                        for (const item of descResult.dtcs) {
                            descriptionsMap[item.dtc] = item.description;
                        }
                    }
                } catch (chunkErr) {
                    // If one chunk fails, log it but don't break the whole app
                    console.error(`Failed to fetch descriptions for chunk ${codesParam}:`, chunkErr);
                }
            }

            for (const group of rawDtcs) {
                if (Array.isArray(group.dtc)) {
                    const mappedDtcs = group.dtc.map((code: string) => ({
                        dtc: code,
                        description: descriptionsMap[code] || "Description not available",
                        mode: group.mode,
                    }));

                    if (group.mode === 3) {
                        this.dtc.confirmed = {
                            mode: group.mode,
                            dtc_count: group.dtc_count,
                            dtc: mappedDtcs,
                        };
                    } else if (group.mode === 7) {
                        this.dtc.pending = {
                            mode: group.mode,
                            dtc_count: group.dtc_count,
                            dtc: mappedDtcs,
                        };
                    }
                }
            }

            this.totalDTCs = this.dtc.confirmed.dtc_count;
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

            const result = await response.json();

            if (result.status === "success") {
                this.getDTC();
            }
        } catch (e) {
            console.error("Failed to request DTC", e);
        }
    }

    isClearing = $state(false);

    clearDTCs = async () => {
        this.isClearing = true;

        try {
            const response = await fetch("/api/v1/req/clear_dtc", {
                method: "POST",
            });

            if (!response.ok) {
                throw new Error(`HTTP Error: ${response.status}`);
            }

            await this.requestDTC();

        } catch (e) {
            console.error("Failed to clear DTCs:", e);
            alertStore.add("Failed to clear DTCs.", "error");
        } finally {
            this.isClearing = false;
        }
    }
    serializePidDefinition(def: any) {
        const normalized = normalizePidDefinition(def);

        return {
            id: normalized.id,
            mode: normalized.mode,
            pid: normalized.pid,
            len: normalized.length,
            name: normalized.name,
            unit: normalized.unit,
            desc: normalized.description,
            formula: normalized.formula,
            minV: normalized.minValue,
            maxV: normalized.maxValue,
            priority: normalized.priority,
            interval: normalized.update_interval_ms,
            color: normalized.color,
            icon: normalized.icon,
        };
    }

    async replacePids(rows: any[]): Promise<boolean> {
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

            const response = await fetch("/api/v1/pid_def", {
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

    async savePids(rows: any[]) {
        if (!(await this.replacePids(rows))) {
            return;
        }

        try {
            const response = await fetch('/api/v1/pid_def/save', {
                method: 'POST'
            });

            if (!response.ok) {
                throw new Error(`HTTP Error: ${response.status}`);
            }

            const result = await response.json();
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


    async updatePids(rows: any[]) {
        if (await this.replacePids(rows)) {
            alertStore.add("Successfully synced PIDs with device.", "success");
        }
    }

    isRecording = $state(false);


}

export const canStore = new CanStore();
