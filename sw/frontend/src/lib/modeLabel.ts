const modeLabels: Record<number, string> = {
  0x01: "Current Data",
  0x02: "Freeze Frame",
  0x03: "DTCs",
  0x04: "Clear DTCs",
  0x05: "Test Results O2",
  0x06: "Test Results Other",
  0x07: "Pending DTCs",
  0x08: "Control",
  0x09: "Vehicle Info",
  0x0a: "Permanent DTCs",
  0x10: "Diagnostic Session Control",
  0x22: "Read Data By Identifier",
  0x45: "Derived Data",
};

export function getModeLabel(mode: number): string {
  if (typeof mode !== "number" || !Number.isFinite(mode)) return "Unknown";
  return modeLabels[mode] ?? `0x${mode.toString(16).toUpperCase().padStart(2, "0")}`;
}
