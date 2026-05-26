import type { Alert } from "./types";

let alerts = $state<Alert[]>([]);

export const alertStore = {
    get alerts() { return alerts },

    add(message: string, type: Alert["type"] = "info", duration = 3000) {
        // FIX: Use Date.now() instead of crypto.randomUUID() for local HTTP support
        const id = Date.now() + Math.random();
        const newAlert: Alert = { id, message, type };

        alerts.push(newAlert);
        console.log("Alert added:", newAlert); // Debug log

        setTimeout(() => {
            this.remove(id);
        }, duration);
    },

    remove(id: number) {
        alerts = alerts.filter(a => a.id !== id);
    }
};