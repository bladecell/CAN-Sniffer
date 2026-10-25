import {
  applyAppearance,
  parseAppearance,
  readAppearance,
  writeAppearance,
  type Appearance,
} from "./appearance";

function localStore(): Storage | null {
  try {
    return window.localStorage;
  } catch {
    return null;
  }
}

export const appearance: Appearance = $state(readAppearance(localStore()));
applyAppearance(document.documentElement, appearance);

export function setAppearance<K extends keyof Appearance>(key: K, value: Appearance[K]): void {
  Object.assign(appearance, parseAppearance({ ...appearance, [key]: value }));
  applyAppearance(document.documentElement, appearance);
  writeAppearance(localStore(), appearance);
}
