// When the raw stream keeps feeding the capture model. Pure: the store decides nothing else.

/** localStorage key of the "Log in background" toggle. Default off. */
export const KEEP_KEY = "can-sniffer.canBus.logInBackground";

/** Only an explicit "1" turns the toggle on; a missing or unknown value is off. */
export function readKeep(stored: string | null): boolean {
  return stored === "1";
}

export function storeKeep(on: boolean): string {
  return on ? "1" : "0";
}

/** The stream runs while the page is open, or while the user keeps logging in the background. */
export function captureActive(keepInBackground: boolean, pageOpen: boolean): boolean {
  return pageOpen || keepInBackground;
}
