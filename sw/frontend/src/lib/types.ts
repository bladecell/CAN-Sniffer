// src/lib/types.ts

/**
 * CORE DASHBOARD TYPES
 */

export type CardType = "pid" | "battery" | "dtcs" | "overview";
export type PidDisplayMode = "card" | "chart" | "gauge" | "bar";
export type SpecialDisplayMode = "default" | "compact" | "detailed";

export interface BaseGridItem {
  id: string;
  w: number;
  h: number;
  cardType: CardType;
}

export interface PidGridItem extends BaseGridItem {
  cardType: "pid";
  pid: number;
  displayMode: PidDisplayMode;
}

export interface SpecialGridItem extends BaseGridItem {
  cardType: "battery" | "dtcs";
  displayMode: SpecialDisplayMode;
}

export interface OverviewGridItem extends BaseGridItem {
  cardType: "overview";
  displayMode: SpecialDisplayMode;
  pids: number[]; // Array of exactly 3 PIDs
  color: string;  // Hex color string
}

export type DashboardItem = PidGridItem | SpecialGridItem | OverviewGridItem;

/**
 * DATA STRUCTURES
 */

export interface DtcModeData {
  mode: number;
  dtc_count: number;
  dtc: string[];
}

/**
 * MODULE CONSTRAINTS AND CONFIGURATIONS
 */

export interface Dimension {
  w: number;
  h: number;
}

export interface Bounds {
  min: Dimension;
  max: Dimension;
}

export type ModuleConfigs = {
  pid: Record<PidDisplayMode, Bounds>;
  battery: Bounds;
  dtcs: Bounds;
  overview: Bounds;
};

export const MODULE_CONFIGS: ModuleConfigs = {
  pid: {
    card: { min: { w: 10, h: 7 }, max: { w: 20, h: 12 } },
    chart: { min: { w: 15, h: 12 }, max: { w: 60, h: 30 } },
    gauge: { min: { w: 12, h: 12 }, max: { w: 24, h: 24 } },
    bar: { min: { w: 10, h: 8 }, max: { w: 20, h: 15 } }
  },
  battery: {
    min: { w: 10, h: 7 },
    max: { w: 18, h: 12 }
  },
  dtcs: {
    min: { w: 14, h: 8 },
    max: { w: 30, h: 20 }
  },
  overview: {
    min: { w: 17, h: 11 },
    max: { w: 60, h: 20 }
  }
};

/**
 * HELPER FUNCTIONS
 */

export function getModuleBounds(item: DashboardItem): Bounds {
  const DEFAULT_BOUNDS = MODULE_CONFIGS.pid.card;
  
  if (!item) return DEFAULT_BOUNDS;

  if (item.cardType === "pid") {
    return MODULE_CONFIGS.pid[item.displayMode] || DEFAULT_BOUNDS;
  }
  
  return (MODULE_CONFIGS as any)[item.cardType] || DEFAULT_BOUNDS;
}
