/** The units offered by the Unit grid. Values are stored in the PID file as-is, so they must not change. "" is no unit. */
export const UNITS: string[] = [
  "",
  "%",
  "kPa",
  "Pa",
  "rpm",
  "km/h",
  "mph",
  "g/s",
  "s",
  "min",
  "ratio",
  "count",
  "km",
  "V",
  "A",
  "°C",
  "°F",
  "°",
  "L/h",
  "L",
  "bar",
];

/** The list, plus a saved unit that is not in it, so a PID loaded from a file still shows its unit. */
export function unitsWith(saved: string): string[] {
  if (!saved || UNITS.includes(saved)) return UNITS;
  return [...UNITS, saved];
}
