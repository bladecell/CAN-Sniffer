// src/lib/types.ts
import { SvelteMap } from "svelte/reactivity";
/**
 * CORE DASHBOARD TYPES
 */
export type CardType = "pid" | "battery" | "dtcs" | "overview" | "control";
export type ControlType = "record" | "log" | "request_static_data";
export type PidDisplayMode = "card" | "chart" | "gauge" | "bar";
export type SpecialDisplayMode = "default" | "compact" | "detailed";

export interface BaseGridItem {
  id: string;
  w: number;
  h: number;
  cardType: CardType;
}

export type GaugeStyle = "arc" | "donut" | "half" | "speedometer" | "gradient";
export const GAUGE_STYLES: readonly GaugeStyle[] = ["arc", "donut", "half", "speedometer", "gradient"];

/** Car battery card: glyph, bar, gauge or chart. */
export type BatteryDisplayMode = "glyph" | "bar" | "gauge" | "chart";
export const BATTERY_DISPLAY_MODES: readonly BatteryDisplayMode[] = ["glyph", "bar", "gauge", "chart"];
/** Bar styles: one fill (solid) or zones (threshold bands for batteries, equal thirds for PIDs). */
export type BarStyle = "solid" | "zones";
export const BAR_STYLES: readonly BarStyle[] = ["solid", "zones"];

export interface PidGridItem extends BaseGridItem {
  cardType: "pid";
  pid: number;
  displayMode: PidDisplayMode;
  gaugeStyle?: GaugeStyle;
  /** Bar mode only. Missing means solid. */
  barStyle?: BarStyle;
}

/** Car battery (12 V) card. Older saved layouts carry a placeholder displayMode; layoutStorage maps it to glyph. */
export interface BatteryGridItem extends BaseGridItem {
  cardType: "battery";
  displayMode: BatteryDisplayMode;
  gaugeStyle?: GaugeStyle;
  barStyle?: BarStyle;
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

export interface ControlsGridItem extends BaseGridItem {
  cardType: "control";
  displayMode: SpecialDisplayMode;
  control_elements: ControlType[];
  color: string;
}

export type SpecialGridItem = BatteryGridItem | DtcGridItem;

export type DashboardItem = PidGridItem | BatteryGridItem | DtcGridItem | OverviewGridItem | ControlsGridItem;

/**
 * DATA STRUCTURES
 */

export interface DtcData {
  confirmed: DtcModeData;
  pending: DtcModeData;
  permanent: DtcModeData;
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
  lastUpdated: number;
  rate: number;
  isValid: boolean;
  isSupported: boolean;
}

export interface WsCanStatus {
  state: string;
  utilization: number;
  battery_voltage: number;
}

/** `GET /system`. `state` is the supervisor enum (docs/api.md). */
export interface SystemStatus {
  app_version: string;
  uptime_s: number;
  restart_reason: string;
  mac: string;
  state: number;
  battery_voltage: number;
  sd_card_detected: boolean;
  component_status: ComponentStatus[];
  heap_free: number;
  heap_min_free: number;
  tasks: TaskStack[];
}

export interface TaskStack {
  name: string;
  stack_free: number;
}

export type CanBusState = "not_initialized" | "bus_off" | "not_connected" | "connected";
export type TwaiErrorState = "error_active" | "error_warning" | "error_passive" | "bus_off";

/** `GET /can_bus` */
export interface CanBusStatus {
  state: CanBusState | (string & {}); // unknown future states are kept as strings
  bitrate: number;
  rs_mode: "high_speed" | "slope_control";
  debug_mode: boolean;
  initialized: boolean;
  bus_connected: boolean;
  node_status: {
    twai_error_state: TwaiErrorState;
    tx_error_count: number;
    rx_error_count: number;
  };
}

/** One item of `GET /pid_data`. `value` is null when the formula result is NaN. */
export interface PidDataWire {
  id: number;
  pid: number;
  value: number | null;
  lastUpdated: number;
  isSupported: boolean;
  isValid: boolean;
  update_interval_ms: number;
}

/** `GET /dtc` without a query: one group per mode (3, 7, 10). */
export interface DtcGroupWire {
  mode: number;
  type: string;
  dtc: string[];
  dtc_count: number;
}

export interface DtcResponse {
  dtcs: DtcGroupWire[];
  count: number;
  status: string;
}

/** `GET /dtc?codes=...` */
export interface DtcDescriptionResponse {
  status: string;
  dtc_count: number;
  dtcs: { dtc: string; description: string }[];
}

export interface ComponentStatus {
  name: string;
  status: string;
}

export interface SDCardInfo {
  name: string;
  mount_path: string;
  capacity: number;
  used_space_mb: number;
  max_freq_mhz: number;
  is_sdio: boolean;
  is_mmc: boolean;
  is_mounted: boolean;
  is_present: boolean;
}

/** One node of `GET /sd_card/tree`: a directory has `children`, a file has `size`. */
export interface SdTreeNode {
  name?: string;
  path: string;
  type?: string;
  size?: number;
  children?: SdTreeNode[];
}

export interface Alert {
  id: number;
  message: string;
  type: "info" | "success" | "warning" | "error";
}

export interface PidDefinition {
  id: number;
  mode: number;
  pid: number;
  length: number;
  name: string;
  unit: string;
  description: string;
  formula: string;
  minValue: number;
  maxValue: number;
  update_interval_ms: number;
  color: number; // Decimal color representation
  priority: number;
  icon: string;
}

export interface PidDataMetrics {
  min: number;
  max: number;
  label: string;
  id: string;
  description: string;
  unit: string;
  icon: string;
  color: string;
  updateInterval: number;
  formula: string;

  currentValue: number;
  isValid: boolean;
  isSupported: boolean;
  displayValue: string;
}

/** `GET /obd2` as sent: `groups` are 7 uint32 bitmaps. */
export interface Obd2StatusWire {
  auto_poll: boolean;
  polling: boolean;
  pid_initialized: boolean;
  pid_def_count: number;
  pid_data_count: number;
  poll_task_utilization: number;
  pid_def_path: string;
  dtc_desc_path: string;
  supported_pids: { count: number; groups: number[] };
}

/** The fields of `Obd2StatusWire` the frontend validates and keeps. */
export type Obd2StatusCore = Omit<Obd2StatusWire, "pid_def_path" | "dtc_desc_path">;

/** Store model: the bitmaps expanded to a PID -> supported map. */
export interface Obd2Status extends Omit<Obd2StatusCore, "supported_pids"> {
  supported_pids: SupportedPids;
}

export interface SupportedPids {
  count: number;
  groups: SvelteMap<number, boolean>;
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
  battery: {
    glyph: { min: { w: 10, h: 7 }, max: { w: 18, h: 12 } },
    bar: { min: { w: 10, h: 8 }, max: { w: 20, h: 15 } },
    gauge: { min: { w: 12, h: 12 }, max: { w: 24, h: 24 } },
    chart: { min: { w: 15, h: 12 }, max: { w: 60, h: 30 } },
  },
  dtcs: { min: { w: 14, h: 8 }, max: { w: 30, h: 20 } },
  overview: { min: { w: 17, h: 11 }, max: { w: 60, h: 20 } }
} satisfies Record<string, Bounds | Record<string, Bounds>>;

export type ModuleConfigs = typeof MODULE_CONFIGS;

/**
 * HELPER FUNCTIONS
 */
export function getModuleBounds(item: { cardType: CardType; displayMode?: string }): Bounds {
  const DEFAULT_BOUNDS: Bounds = MODULE_CONFIGS.pid.card;

  if (!item) return DEFAULT_BOUNDS;

  if (item.cardType === "pid") {
    return MODULE_CONFIGS.pid[item.displayMode as PidDisplayMode] ?? DEFAULT_BOUNDS;
  }
  if (item.cardType === "battery") {
    return MODULE_CONFIGS.battery[item.displayMode as BatteryDisplayMode] ?? MODULE_CONFIGS.battery.glyph;
  }

  // Safe runtime lookup without 'any' typecasting
  if (item.cardType in MODULE_CONFIGS) {
    const staticConfig = MODULE_CONFIGS[item.cardType as keyof Omit<ModuleConfigs, "pid" | "battery">];
    return staticConfig ?? DEFAULT_BOUNDS;
  }

  return DEFAULT_BOUNDS;
}


export interface DTCFaultProps {
  code: string;
  description: string;
  mode: 3 | 7 | 10 | number | string; // Keeps specific diagnostic modes explicitly listed
  priority?: string;
  status?: string;
  [key: string]: unknown; // Prefer 'unknown' over 'any' for safer dynamic index signature checking
}

/**
 * TABLE CONSTRAINTS
 */
export type ColumnType = "text" | "string" | "number" | "code" | "badge" | "checkbox" | "toggle";

export interface Column {
  label: string;
  key: string;
  type: ColumnType;
  unit?: string;
  width_px?: number;
  unitKey?: string;
  showTooltip?: boolean;
  tooltipKey?: string;
  colorKey?: string;
  hidden?: boolean;
  formatKey?: string;
  autoShowKey?: string;
  autoShowValue?: unknown;
  showHeaderToggle?: boolean;
  showHeaderLabel?: boolean;
}

export interface DataTableProps {
  columns: Column[];
  data: Record<string, any>[];
  selectedRowIndex?: number;
  onExport?: () => void;
  onImport?: () => void;
}
