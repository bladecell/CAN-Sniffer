// Display names of the PID icons, shared by the icon picker and the import sheet. Other icons use their file name.
const friendly: Record<string, string> = {
  thermometer: "Thermometer",
  gauge: "Gauge",
  droplet: "Fluid",
  "battery-full": "Battery",
  "gas-pump": "Fuel",
  fan: "Fan",
  stopwatch: "Timing",
  bolt: "Voltage",
  engine: "Engine",
};

export function iconLabel(name: string): string {
  return friendly[name] ?? name.split("-").map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join(" ");
}
