// Starter dashboards as data. The PIDs are the ones the stock PID set on the SD card defines (engine load,
// coolant, RPM, odometer, fuel amount, interior temperature), so the cards have data out of the box.
// A card whose PID is missing shows the normal no-data state. Sizes are columns x rows.
// Thresholds are left out: their defaults need the PID's range, which a template cannot know.
import type { Card } from "./dashboardModel";

export type TemplateId = "driving" | "diagnostics" | "chart" | "minimal";
export type TemplateCard = Omit<Card, "id">;
export interface TemplateSection { title: string; cards: TemplateCard[] }
export interface DashboardTemplate { id: TemplateId; name: string; summary: string; sections: TemplateSection[] }

const LOAD = 4;
const COOLANT = 5;
const RPM = 12;
const ODOMETER = 8707;
const FUEL = 8710;
const INTERIOR = 9747;

const value = (pid: number, w: number, h: number, style = "big"): TemplateCard => ({ type: "value", style, pids: [pid], w, h });

export const TEMPLATES: readonly DashboardTemplate[] = [
  {
    id: "driving",
    name: "Driving",
    summary: "RPM, coolant and load, with the polling and recorder controls.",
    sections: [
      {
        title: "Engine",
        cards: [
          { type: "gauge", style: "arc", pids: [RPM], w: 3, h: 3 },
          value(COOLANT, 3, 2),
          { type: "bar", style: "plain", pids: [LOAD], w: 3, h: 1 },
        ],
      },
      {
        title: "Vehicle",
        cards: [value(FUEL, 3, 2), value(ODOMETER, 3, 2), { type: "battery", style: "glyph", pids: [], w: 3, h: 2 }, value(INTERIOR, 3, 2)],
      },
      {
        title: "Trip",
        cards: [
          { type: "status", style: "chips", pids: [], w: 6, h: 3 },
          { type: "polling", style: "switch", pids: [], w: 3, h: 2 },
          { type: "recorder", style: "switch", pids: [], w: 3, h: 2 },
          { type: "dtc", style: "rows", pids: [], w: 6, h: 3 },
        ],
      },
    ],
  },
  {
    id: "diagnostics",
    name: "Diagnostics",
    summary: "Trouble codes, the VIN and the status of the device, with recording.",
    sections: [
      {
        title: "Vehicle",
        cards: [
          { type: "overview", style: "list", pids: [], items: [...[COOLANT, RPM, LOAD, FUEL].map((pid) => ({ t: "pid" as const, pid })), { t: "vin" as const }, { t: "dtc" as const }], w: 6, h: 4 },
          { type: "status", style: "list", pids: [], w: 6, h: 5 },
        ],
      },
      {
        title: "Faults",
        cards: [
          { type: "dtc", style: "rows", pids: [], w: 6, h: 4 },
          { type: "recorder", style: "switch", pids: [], w: 3, h: 2 },
          { type: "polling", style: "switch", pids: [], w: 3, h: 2 },
        ],
      },
    ],
  },
  {
    id: "chart",
    name: "Chart view",
    summary: "Trends for RPM and load, temperatures and fuel amount.",
    sections: [
      { title: "RPM and load", cards: [{ type: "chart", style: "line", pids: [RPM, LOAD], w: 6, h: 3 }] },
      { title: "Temperatures", cards: [{ type: "chart", style: "lanes", pids: [COOLANT, INTERIOR], w: 6, h: 3 }] },
      { title: "Fuel", cards: [{ type: "chart", style: "area", pids: [FUEL], w: 6, h: 3 }] },
    ],
  },
  {
    id: "minimal",
    name: "Minimal",
    summary: "RPM, coolant and load, with the polling control.",
    sections: [
      {
        title: "",
        cards: [value(RPM, 3, 2), value(COOLANT, 3, 2), value(LOAD, 3, 2), { type: "polling", style: "switch", pids: [], w: 3, h: 2 }],
      },
    ],
  },
];

export const templateById = (id: TemplateId): DashboardTemplate => TEMPLATES.find((t) => t.id === id) ?? TEMPLATES[0];

/** A dashboard keeps no record of its template, so Reset uses the template with the same name, or Driving. */
export const templateForName = (name: string): DashboardTemplate => TEMPLATES.find((t) => t.name === name) ?? TEMPLATES[0];
