import { alertStore } from "./alertStore.svelte";
import type { SdCardReason, WsFrame } from "./wsFrames";

export const SD_UNAVAILABLE_TEXT = "SD card unavailable. The device stopped using it after repeated errors. Reinsert the card, then try again.";

/** SD card state from the 0x06 WebSocket frame. `frame` is null until the first one arrives. */
class SdCardStore {
  frame = $state<Omit<Extract<WsFrame, { kind: "sdCard" }>, "kind"> | null>(null);
  /** Given up after I/O timeouts: every SD route answers 503 until the card is removed, mounted or formatted. */
  unavailable = $derived(this.frame?.unavailable === true);
  reason = $derived<SdCardReason | null>(this.frame?.reason ?? null);

  /** Short state for the Storage settings: what the card is doing right now. Null until the first frame. */
  statusText = $derived.by(() => {
    if (!this.frame) return null;
    if (this.frame.mounted) return "Mounted";
    switch (this.frame.reason) {
      case "removed":
        return "No card";
      case "io-timeouts":
        return "Unavailable";
      default:
        return "Not mounted";
    }
  });

  /** The frame repeats at 1 Hz, so only a change of state raises an alert; the first frame never does. */
  applyFrame(frame: Extract<WsFrame, { kind: "sdCard" }>) {
    const prev = this.frame;
    this.frame = { mounted: frame.mounted, unavailable: frame.unavailable, reason: frame.reason };
    if (!prev || (prev.mounted === frame.mounted && prev.unavailable === frame.unavailable)) return;
    if (frame.mounted) alertStore.add("SD card inserted", "success");
    else if (frame.unavailable) alertStore.add(SD_UNAVAILABLE_TEXT, "error");
    else if (frame.reason === "removed") alertStore.add("SD card removed", "warning");
    else alertStore.add("SD card unmounted", "info");
  }

  /** The text for a failed SD action: the card-level message while the card is unavailable, else the error itself. */
  explain(error: unknown): string {
    if (this.unavailable) return SD_UNAVAILABLE_TEXT;
    return error instanceof Error ? error.message : String(error);
  }
}

export const sdCardStore = new SdCardStore();
