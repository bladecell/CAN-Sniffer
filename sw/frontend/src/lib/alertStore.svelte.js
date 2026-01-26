let alerts = $state([]);

export const alertStore = {
    get alerts() { return alerts },

    add(message, type = "info", duration = 3000) {
        // FIX: Use Date.now() instead of crypto.randomUUID() for local HTTP support
        const id = Date.now() + Math.random();
        const newAlert = { id, message, type };

        alerts.push(newAlert);
        console.log("Alert added:", newAlert); // Debug log

        setTimeout(() => {
            this.remove(id);
        }, duration);
    },

    remove(id) {
        alerts = alerts.filter(a => a.id !== id);
    }
};