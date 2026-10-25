export interface SelectMenuOption<T> {
  value: T;
  label: string;
  icon?: string;
  disabled?: boolean;
}

/** Index of the selected option if it is enabled, else the first enabled option, else -1. */
export function defaultIndex<T>(options: readonly SelectMenuOption<T>[], selected: number): number {
  if (selected >= 0 && selected < options.length && !options[selected].disabled) return selected;
  return scan(options, 0, 1);
}

/**
 * Index to move to for a navigation key, or undefined for any other key.
 * Skips disabled options and stays on the current index at either end.
 */
export function navigate<T>(
  options: readonly SelectMenuOption<T>[],
  current: number,
  key: string,
): number | undefined {
  switch (key) {
    case "ArrowDown":
      return orCurrent(scan(options, current + 1, 1), current);
    case "ArrowUp":
      return orCurrent(scan(options, current - 1, -1), current);
    case "Home":
      return orCurrent(scan(options, 0, 1), current);
    case "End":
      return orCurrent(scan(options, options.length - 1, -1), current);
    default:
      return undefined;
  }
}

function scan<T>(options: readonly SelectMenuOption<T>[], start: number, step: 1 | -1): number {
  for (let i = start; i >= 0 && i < options.length; i += step) {
    if (!options[i].disabled) return i;
  }
  return -1;
}

function orCurrent(found: number, current: number): number {
  return found < 0 ? current : found;
}
