// WebSocket lifecycle, heartbeat, PID stream ownership and live frame state.
import { alertStore } from "./alertStore.svelte";
import type { WsCanStatus } from "./types";
import { auth } from "./auth.svelte";
import { parseWsFrame, type WsFrame } from "./wsFrames";
import { dispatchToListeners } from "./dispatch";
import { reconnectDelay } from "./backoff";
import { createHeartbeat, CONNECT_TIMEOUT_MS } from "./heartbeat";
import { createStreamRefCount } from "./streamRefCount";
import { createThrottle } from "./throttle";
import { livePids } from "./livePidStore.svelte";
import { notifyConnectionOpen } from "./connectionEvents";

export const COMMANDS = { START_LOG: 0xa0, STOP_LOG: 0xa1, PING_REQUEST: 0xa2, PING_RESPONSE: 0xa3 };
export type MessageListener = (update: { pid: number; value: number; timestamp: number }) => void;
export class ConnectionStore {
    connected = $state(false);
    pids = livePids;
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

            notifyConnectionOpen();
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

}
export const connectionStore = new ConnectionStore();
