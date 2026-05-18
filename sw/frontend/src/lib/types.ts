// All widgets on the grid must have these basic layout fields
export interface BaseGridItem {
  id: string;          // Unique instance string (crypto.randomUUID())
  x: number;
  y: number;
  w: number;
  h: number;
  displayMode: string; // "card" | "chart" | "gauge" | "bar" etc.
}

// Layout shape specifically for a standard vehicle PID card
export interface PidGridItem extends BaseGridItem {
  cardType: "pid";
  pid: number;         // Hex code pointer to the firmware definition
  displayMode: "card" | "chart" | "gauge" | "bar";
}

// Layout shape for your upcoming special modules
export interface SpecialGridItem extends BaseGridItem {
  cardType: "battery" | "dtcs" | "overview";
  displayMode: "default" | "compact" | "detailed";
}

// The master union type for your layout state array
export type DashboardItem = PidGridItem | SpecialGridItem;

export const MODULE_CONFIGS = {
  // PID Cards break down into explicit displayMode bounds
  pid: {
    card: {
      min: { w: 12, h: 7 },
      max: { w: 18, h: 10 }
    },
    chart: {
      min: { w: 18, h: 14 }, // Needs more room for axes and gridlines
      max: { w: 53, h: 28 }
    },
    gauge: {
      min: { w: 14, h: 14 }, // Square layout proportions look best for dials
      max: { w: 20, h: 20 }
    },
    bar: {
      min: { w: 12, h: 10 },
      max: { w: 18, h: 14 }
    }
  },
  
  // Special cards remain bound to their core module shapes
  battery: {
    min: { w: 12, h: 12 },
    max: { w: 18, h: 18 }
  },
  dtcs: {
    min: { w: 24, h: 14 },
    max: { w: 53, h: 28 }
  },
  overview: {
    min: { w: 36, h: 16 },
    max: { w: 78, h: 32 }
  }
} as const; // Frozen configuration block