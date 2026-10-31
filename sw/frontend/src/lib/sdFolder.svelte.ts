import { DEFAULT_SD_FOLDER, normalizeSdFolder, type FolderResult } from "./sdPidFiles";

// The SD folder the PID files tab lists. Frontend only: kept in this browser, never sent to the device.
// Stored as the normalised absolute path; a stored value that no longer validates falls back to the default.
const KEY = "can-sniffer.sd.folder";

function load(): string {
  try {
    const stored = localStorage.getItem(KEY);
    const result = stored === null ? null : normalizeSdFolder(stored);
    return result?.ok ? result.folder : DEFAULT_SD_FOLDER;
  } catch {
    return DEFAULT_SD_FOLDER;
  }
}

let folder = $state(load());

export const sdFolder = {
  get path() {
    return folder;
  },
  /** Validates, normalises and stores the folder. An invalid one changes nothing and says why. */
  set(input: string): FolderResult {
    const result = normalizeSdFolder(input);
    if (!result.ok) return result;
    folder = result.folder;
    try {
      localStorage.setItem(KEY, result.folder);
    } catch {
      // Storage unavailable (private mode, quota): the folder lasts until reload.
    }
    return result;
  },
};
