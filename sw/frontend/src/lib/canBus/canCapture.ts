// Capture file names and labels for the Captures tab. Pure.

/** The only names the firmware accepts for list, download and delete. Checked here before a URL is built. */
export const CAPTURE_NAME = /^cap_\d{6}\.csv$/;

export function validCaptureName(name: string): boolean {
  return CAPTURE_NAME.test(name);
}

export function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

const ENDED: Record<string, string> = {
  stop: "Stopped",
  stream: "Stream ended",
  limit: "Reached the 8 MB limit",
  error: "Stopped by a write error",
};

const ERROR: Record<string, string> = {
  NO_SD: "No SD card",
  SD_ERROR: "The SD card could not be written",
};

/** One line for the last capture: why it ended, or the card error. Empty when nothing has run. */
export function captureEndedText(ended: string | undefined, error: string | undefined): string {
  if (error) return ERROR[error] ?? error;
  if (ended) return ENDED[ended] ?? ended;
  return "";
}
