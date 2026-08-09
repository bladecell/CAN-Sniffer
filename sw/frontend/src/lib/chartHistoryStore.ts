import { canStore } from "./canStore.svelte";

export const MAX_HISTORY_SEC = 60;

class ChartHistoryStore {
    private history = new Map<number, { timeData: number[], valueData: (number | null)[] }>();
    private listeners = new Map<number, Set<(timeData: number[], valueData: (number | null)[]) => void>>();

    constructor() {
        canStore.subscribe((update: any) => {
            let pidHistory = this.history.get(update.pid);
            if (!pidHistory) {
                pidHistory = { timeData: [], valueData: [] };
                this.history.set(update.pid, pidHistory);
            }

            const lastTime = pidHistory.timeData.length > 0 ? pidHistory.timeData[pidHistory.timeData.length - 1] : 0;
            if (lastTime > 0 && update.timestamp - lastTime > 2000) {
                pidHistory.timeData.push(update.timestamp - 1);
                pidHistory.valueData.push(null);
            }

            pidHistory.timeData.push(update.timestamp);
            pidHistory.valueData.push(update.value);

            const cutoff = update.timestamp - MAX_HISTORY_SEC * 1000;
            let firstValidIdx = 0;
            while (firstValidIdx < pidHistory.timeData.length && pidHistory.timeData[firstValidIdx] < cutoff) {
                firstValidIdx++;
            }
            if (firstValidIdx > 0) {
                // Modifying arrays in place to maintain references if possible, 
                // but since we are replacing the array, we just slice it.
                pidHistory.timeData = pidHistory.timeData.slice(firstValidIdx);
                pidHistory.valueData = pidHistory.valueData.slice(firstValidIdx);
            }

            const pidListeners = this.listeners.get(update.pid);
            if (pidListeners) {
                pidListeners.forEach(listener => listener(pidHistory.timeData, pidHistory.valueData));
            }
        });
    }

    subscribe(pid: number, callback: (timeData: number[], valueData: (number | null)[]) => void) {
        let pidListeners = this.listeners.get(pid);
        if (!pidListeners) {
            pidListeners = new Set();
            this.listeners.set(pid, pidListeners);
        }
        pidListeners.add(callback);
        
        // Immediately trigger with current history
        const currentHistory = this.history.get(pid);
        if (currentHistory) {
            callback(currentHistory.timeData, currentHistory.valueData);
        } else {
            callback([], []);
        }

        return () => {
            pidListeners?.delete(callback);
        };
    }
}

export const chartHistoryStore = new ChartHistoryStore();
