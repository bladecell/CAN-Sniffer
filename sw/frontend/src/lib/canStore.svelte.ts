// src/lib/canStore.svelte.ts
import { SvelteMap } from 'svelte/reactivity';
import { alertStore } from './alertStore.svelte';
import type { PidValue, WsCanStatus, PidDefinition, Obd2Status, DtcModeData } from './types';

export const COMMANDS = {
    START_LOG: 0xA0,
    STOP_LOG: 0xA1
};

export type MessageListener = (update: { pid: number; value: number; timestamp: number }) => void;

export class CanStore {
    connected = $state(false);
    pids = new SvelteMap<number, PidValue>();
    listeners = new Set<MessageListener>();
    isStreaming = $state(false);
    socket: WebSocket | null = null;
    pollInterval: any = null;

    connect() {
        if (this.socket) return;

        const host = window.location.host;
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${host}/ws`;

        console.log(`Connecting to ${wsUrl}...`);

        this.socket = new WebSocket(wsUrl);
        this.socket.binaryType = "arraybuffer";

        this.socket.onopen = () => {
            console.log("WebSocket Connected");
            alertStore.add("WebSocket Connected", "success");
            this.connected = true;

            // 1. Load static/status definitions
            this.loadDefinitions();
            this.getObd2Status();

            // 2. Fetch last known data immediately from ESP32 storage
            this.getVin();
            this.getDTC();

            // 3. Trigger a fresh request to the vehicle ECU
            this.requestVin();
            this.requestDTC();
        };

        this.socket.onclose = () => {
            console.log("WebSocket Disconnected");
            alertStore.add("WebSocket Disconnected", "error");
            this.connected = false;
            this.isStreaming = false;
            this.socket = null;
            setTimeout(() => this.connect(), 2000);
        };

        this.socket.onmessage = (event) => {
            this.handleMessage(event.data);
        };
    }

    disconnect() {
        if (this.socket) {
            this.socket.close(1000, "User disconnected");
            this.socket = null;
        }

        if (this.pollInterval) {
            clearInterval(this.pollInterval);
            this.pollInterval = null;
        }

        if (this.canStatus) {
            this.canStatus = { ...this.canStatus, state: 'disconnected' };
        }

        console.log("WebSocket disconnected manually.");
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
            default:
                break
        }
    }

    subscribe(callback: MessageListener) {
        this.listeners.add(callback);
        // Return an unsubscribe function to prevent memory leaks
        return () => this.listeners.delete(callback);
    }

    wsCanStatus = $state<WsCanStatus>({ state: "Not Connected", utilization: 0, battery_voltage: 0 });

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
        const can_bus_state_name = ["Not Initialized", "Off", "Not Connected", "Connected"];

        this.wsCanStatus = {
            state: can_bus_state_name[stateIdx] || "Unknown",
            utilization: Math.round(utilization * 100),
            battery_voltage: battery_voltage,
        };

        const currentState = this.wsCanStatus?.state;

        if (previousState !== currentState) {
            if (currentState === "Connected") {
                alertStore.add("CAN Bus Connected", "success");
            } else {
                alertStore.add("CAN Bus " + currentState, "error");
            }
        }

        if (utilization > 0.9) {
            alertStore.add("CAN Bus Utilization is above 90%", "warning");
        }

        if (battery_voltage < 11.8 && battery_voltage > 1) {
            alertStore.add("Car battery voltage is below 12V", "warning");
        }
    }

    parsePidPacket(view: DataView) {
        const length = view.getUint8(1);
        if (length !== view.byteLength - 2) {
            console.warn("Invalid PID %d packet length", view.getUint32(2, true));
            return;
        }
        const fullPid = view.getUint32(2, true);
        const value = view.getFloat32(6, true);
        const time = view.getUint32(10, true);
        const rate = view.getUint32(14, true);
        const isValid = view.getUint8(18) !== 0;
        const isSupported = view.getUint8(19) !== 0;

        let existing = this.pids.get(fullPid);

        if (!existing) {
            existing = {
                value: value,
                timestamp: time,
                rate: rate,
                valid: isValid,
                supported: isSupported,
                history: []
            };
        } else {
            existing.history.push({
                value: value,
                timestamp: time,
            });

            if (existing.history.length > 1000) {
                existing.history.shift();
            }
            existing.value = Number(value.toFixed(2));
            existing.timestamp = time;
            existing.rate = rate;
            existing.valid = isValid;
            existing.supported = isSupported;
        }

        this.pids.set(fullPid, {
            ...existing,
            value: Number(value.toFixed(2)),
            timestamp: time,
            rate: rate,
            valid: isValid,
            supported: isSupported
        });

        const update = {
            pid: fullPid,
            value: Number(value.toFixed(2)),
            timestamp: time
        };

        for (const listener of this.listeners) {
            listener(update);
        }
    }

    system = $state<any[]>([]);

    async requestSystem() {
        try {
            const response = await fetch("/api/v1/system");
            const result = await response.json();

            this.system = result.data;
        } catch (e) {
            console.error("Failed to load PIDs", e);
        }
    }

    pidDefinitions = $state<PidDefinition[]>([]);

    async loadDefinitions() {
        try {
            const response = await fetch("/api/v1/pid_def");
            const result = await response.json();

            this.pidDefinitions = result.data;
        } catch (e) {
            console.error("Failed to load PIDs", e);
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

            this.obd2Status = result;
        } catch (e) {
            console.error("Failed to load OBD2 status", e);
        }
    }

    async setContinuousPolling(running: boolean) {
        try {
            await fetch(`/api/v1/req/pid_poll?running=${running}`, {
                method: "POST"
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
                method: "POST"
            });

            const result = await response.json();

            if (result.status === "success") {
                this.getVin();
            }
        } catch (e) {
            console.error("Failed to request VIN", e);
        }
    }

    dtc = $state<DtcModeData[]>([]);
    totalDTCs = $state(0);

    async getDTC() {
        try {
            const response = await fetch("/api/v1/dtc");
            const result = await response.json();
            this.dtc = result.dtcs;
            this.totalDTCs = result.dtcs.reduce((sum: number, mode: DtcModeData) => sum + mode.dtc_count, 0);
        } catch (e) {
            console.error("Failed to load DTC", e);
        }
    }

    async requestDTC(mode = -1) {
        try {
            const request_url = mode === -1 ? "/api/v1/req/dtc" : `/api/v1/req/dtc?mode=${mode}`;
            const response = await fetch(request_url, {
                method: "POST"
            });

            const result = await response.json();

            if (result.status === "success") {
                this.getDTC();
            }
        } catch (e) {
            console.error("Failed to request DTC", e);
        }
    }
}

export const canStore = new CanStore();