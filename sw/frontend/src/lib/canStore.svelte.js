// src/lib/canStore.svelte.js
import { SvelteMap } from 'svelte/reactivity';
import { alertStore } from './alertStore.svelte';

export const COMMANDS = {
    START_LOG: 0xA0,
    STOP_LOG: 0xA1
};

export class CanStore {
    connected = $state(false);
    pids = new SvelteMap();

    isStreaming = $state(false);
    socket = null;

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

        this.canStatus = { ...this.canStatus, state: 'disconnected' };

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

    sendByte(byte) {
        if (this.socket && this.socket.readyState === WebSocket.OPEN) {
            const buffer = new Uint8Array([byte]);
            this.socket.send(buffer);
        } else {
            console.warn("Cannot send command: WebSocket not open");
        }
    }

    handleMessage(data) {
        if (!(data instanceof ArrayBuffer)) return;
        const view = new DataView(data);
        if (view.byteLength < 19) return;

        const type = view.getUint8(0);

        if (type === 0x02) {
            this.parsePidPacket(view);
        }
    }

    parsePidPacket(view) {
        const fullPid = view.getUint32(1, true);
        const value = view.getFloat32(5, true);
        const time = view.getUint32(9, true);
        const rate = view.getUint32(13, true);
        const isValid = view.getUint8(17) !== 0;
        const isSupported = view.getUint8(18) !== 0;

        this.pids.set(fullPid, {
            value: Number(value.toFixed(2)),
            timestamp: time,
            rate: rate,
            valid: isValid,
            supported: isSupported
        });
    }

    pidDefinitions = $state([]);

    async loadDefinitions() {
        try {
            const response = await fetch("/api/v1/pid_def");
            const result = await response.json();

            this.pidDefinitions = result.data;
        } catch (e) {
            console.error("Failed to load PIDs", e);
        }
    }

    canStatus = $state(null);
    canPollInterval = null;
    canCurrentRate = $state(5000);

    async getCanStatus() {
        try {
            const previousState = this.canStatus?.state;

            const response = await fetch("/api/v1/can_bus");
            if (!response.ok) throw new Error(response.statusText);

            const result = await response.json();
            this.canStatus = result;

            const currentState = this.canStatus?.state;

            if (previousState === "connected" && currentState !== "connected") {
                alertStore.add("CAN Bus Disconnected", "error");
            }

            if (previousState !== "connected" && currentState === "connected") {
                if (previousState !== undefined) {
                    alertStore.add("CAN Bus Connected", "success");
                }
            }

        } catch (e) {
            console.error("Failed to load CAN status", e);
        }
    }

    startCanPolling(intervalMs = this.canCurrentRate) {
        this.stopCanPolling();

        this.canCurrentRate = intervalMs;

        this.getCanStatus();

        this.canPollInterval = setInterval(() => {
            this.getCanStatus();
        }, intervalMs);
    }

    stopCanPolling() {
        if (this.canPollInterval) {
            clearInterval(this.canPollInterval);
            this.canPollInterval = null;
        }
    }


    obd2Status = $state(null);

    async getObd2Status() {
        try {
            const response = await fetch("/api/v1/obd2");
            const result = await response.json();

            this.obd2Status = result;
        } catch (e) {
            console.error("Failed to load OBD2 status", e);
        }
    }

    async setContinuousPolling(running) {
        try {
            const response = await fetch(`/api/v1/req/pid_poll?running=${running}`, {
                method: "POST"
            });

            this.getObd2Status();
        } catch (e) {
            console.error("Failed to set OBD2 continuous polling", e);
        }
    }

}

export const canStore = new CanStore();