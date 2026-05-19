// src/lib/dashboardStore.svelte.ts
import { untrack } from "svelte";
import type { DashboardItem, CardType, PidDisplayMode } from "./types";
import { getModuleBounds } from "./types";

const STORAGE_KEY = "dashboard-unified-flow";

export class DashboardStore {
  items = $state<DashboardItem[]>([]);
  isInitialized = $state(false);

  constructor() {
    this.load();
  }

  load() {
    const stored = localStorage.getItem(STORAGE_KEY);
    if (stored) {
      try {
        this.items = JSON.parse(stored);
      } catch (e) {
        console.error("Failed to parse stored dashboard layout", e);
        this.items = [];
      }
    }
    this.isInitialized = true;
  }

  save() {
    if (!this.isInitialized) return;
    localStorage.setItem(STORAGE_KEY, JSON.stringify(this.items));
  }

  addItem(type: CardType, pid?: number) {
    const newItem = this.createDefaultItem(type, pid);
    this.items = [...this.items, newItem];
    this.save();
    return newItem;
  }

  private createDefaultItem(type: CardType, pid?: number): DashboardItem {
    const id = crypto.randomUUID();
    if (type === "pid") {
      return {
        id,
        cardType: "pid",
        pid: pid || 0,
        displayMode: "card",
        w: 10,
        h: 7,
      };
    } else {
      const bounds = getModuleBounds({ cardType: type, id, w: 0, h: 0, displayMode: "default" } as any);
      return {
        id,
        cardType: type,
        displayMode: "default",
        w: bounds.min.w,
        h: bounds.min.h,
      } as DashboardItem;
    }
  }

  updateItem(updatedItem: DashboardItem) {
    const index = this.items.findIndex(i => i.id === updatedItem.id);
    if (index !== -1) {
      this.items[index] = updatedItem;
      this.items = [...this.items];
      this.save();
    }
  }

  deleteItem(id: string) {
    this.items = this.items.filter(i => i.id !== id);
    this.save();
  }

  setItems(newItems: DashboardItem[]) {
    this.items = newItems;
    this.save();
  }
}

export const dashboardStore = new DashboardStore();
