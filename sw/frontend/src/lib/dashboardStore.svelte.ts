// src/lib/dashboardStore.svelte.ts
import { untrack } from "svelte";
import type { DashboardItem, CardType, PidDisplayMode, OverviewGridItem } from "./types";
import { getModuleBounds } from "./types";
import { parseStoredLayout } from "./layoutStorage";

const STORAGE_KEY = "dashboard-unified-flow";

export class DashboardStore {
  items = $state<DashboardItem[]>([]);
  isInitialized = $state(false);
  activeMenuId = $state<string | null>(null);
  isEditMode = $state(false); // <-- FIXED: Strictly declared as reactive state

  constructor() {
    this.load();
  }

  load() {
    try {
      this.items = parseStoredLayout(localStorage.getItem(STORAGE_KEY));
    } catch (e) {
      console.error("Failed to read stored dashboard layout", e);
      this.items = [];
    }
    this.isInitialized = true;
  }

  save() {
    if (!this.isInitialized) return;
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(this.items));
    } catch (e) {
      console.error("Failed to save dashboard layout", e);
    }
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
    } else if (type === "overview") {
      const bounds = getModuleBounds({ cardType: type });
      return {
        id,
        cardType: "overview",
        displayMode: "default",
        w: bounds.min.w,
        h: bounds.min.h,
        pids: [0x0c, 0x2206, 0x04], // RPM, Gear, Load (defaults)
        color: "#3b82f6"
      } as OverviewGridItem;
    } else {
      const bounds = getModuleBounds({ cardType: type });
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