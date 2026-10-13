const HEX6 = /^#[0-9a-fA-F]{6}$/;

/** "#rrggbb" or a 24-bit integer to "#rrggbb"; anything else (also CSS injection attempts) gives the fallback. */
export function safeHexColor(value: unknown, fallback = "#10b981"): string {
  if (typeof value === "string") return HEX6.test(value) ? value : fallback;
  if (typeof value === "number" && Number.isInteger(value) && value >= 0 && value <= 0xffffff) {
    return `#${value.toString(16).padStart(6, "0")}`;
  }
  return fallback;
}
