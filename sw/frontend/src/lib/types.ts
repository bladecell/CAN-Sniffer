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

export interface BatteryGridItem extends BaseGridItem {
  cardType: "battery";
  displayMode: SpecialDisplayMode;
}

export interface DtcGridItem extends BaseGridItem {
  cardType: "dtcs";
  displayMode: SpecialDisplayMode;
}

export interface OverviewGridItem extends BaseGridItem {
  cardType: "overview";
  displayMode: SpecialDisplayMode;
  pids: [number, number, number]; // Strictly typed 3-element tuple
  color: string;                  // Hex color string
}

export type DashboardItem = PidGridItem | BatteryGridItem | DtcGridItem | OverviewGridItem;

/**
 * DATA STRUCTURES
 */

export interface DtcData {
  confirmed: DtcModeData;
  pending: DtcModeData;
}

export interface DtcModeData {
  mode: number;
  dtc_count: number;
  dtc: singleDtc[];
}

export interface singleDtc {
  dtc: string;
  description: string;
  mode: number;
}

export interface PidValue {
  value: number;
  timestamp: number;
  rate: number;
  valid: boolean;
  supported: boolean;
  history: { value: number; timestamp: number }[];
}

export interface WsCanStatus {
  state: string;
  utilization: number;
  battery_voltage: number;
}

export interface Alert {
  id: number;
  message: string;
  type: "info" | "success" | "warning" | "error";
}

export interface PidDefinition {
  pid: number;
  name: string;
  description: string;
  unit: string;
  icon: string;
  color: number; // Decimal color representation
  minValue?: number;
  maxValue?: number;
  updateIntervalMs?: number;
  supported?: boolean;
}

export interface Obd2Status {
  continuous_running: boolean;
  last_request_time: number;
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

// Using 'satisfies' preserves explicit literal structures without losing type verification
export const MODULE_CONFIGS = {
  pid: {
    card: { min: { w: 10, h: 7 }, max: { w: 20, h: 12 } },
    chart: { min: { w: 15, h: 12 }, max: { w: 60, h: 30 } },
    gauge: { min: { w: 12, h: 12 }, max: { w: 24, h: 24 } },
    bar: { min: { w: 10, h: 8 }, max: { w: 20, h: 15 } }
  },
  battery: { min: { w: 10, h: 7 }, max: { w: 18, h: 12 } },
  dtcs: { min: { w: 14, h: 8 }, max: { w: 30, h: 20 } },
  overview: { min: { w: 17, h: 11 }, max: { w: 60, h: 20 } }
} satisfies Record<string, Bounds | Record<string, Bounds>>;

export type ModuleConfigs = typeof MODULE_CONFIGS;

/**
 * HELPER FUNCTIONS
 */
export function getModuleBounds(item: DashboardItem): Bounds {
  const DEFAULT_BOUNDS: Bounds = MODULE_CONFIGS.pid.card;
  
  if (!item) return DEFAULT_BOUNDS;

  if (item.cardType === "pid") {
    return MODULE_CONFIGS.pid[item.displayMode] ?? DEFAULT_BOUNDS;
  }
  
  // Safe runtime lookup without 'any' typecasting
  if (item.cardType in MODULE_CONFIGS) {
    const staticConfig = MODULE_CONFIGS[item.cardType as keyof Omit<ModuleConfigs, "pid">];
    return staticConfig ?? DEFAULT_BOUNDS;
  }

  return DEFAULT_BOUNDS;
}

/**
 * TABLE CONSTRAINTS
 */
export type ColumnType = "text" | "number" | "code" | "badge";

export interface Column {
  label: string;
  key: string;
  type: ColumnType;
  unit?: string;
  width?: string;
}

export interface DTCFaultProps {
  code: string;
  description: string;
  mode: 3 | 7 | 10 | number | string; // Keeps specific diagnostic modes explicitly listed
  priority?: string;
  status?: string;
  [key: string]: unknown; // Prefer 'unknown' over 'any' for safer dynamic index signature checking
}