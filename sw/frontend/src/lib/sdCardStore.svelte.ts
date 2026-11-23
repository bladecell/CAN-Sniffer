import type { SdCardReason, WsFrame } from "./wsFrames";

export const SD_UNAVAILABLE_TEXT = "SD card unavailable. The device stopped using it after repeated errors. Reinsert the card, then try again.";

/** SD card state from the 0x06 WebSocket frame. `frame` is null until the first one arrives. */
class SdCardStore {
  frame = $state<Omit<Extract<WsFrame, { kind: "sdCard" }>, "kind"> | null>(null);
  /** Given up after I/O timeouts: every SD route answers 503 until the card is removed, mounted or formatted. */
  unavailable = $derived(this.frame?.unavailable === true);
  reason = $derived<SdCardReason | null>(this.frame?.reason ?? null);

  applyFrame(frame: Extract<WsFrame, { kind: "sdCard" }>) {
    this.frame = { mounted: frame.mounted, unavailable: frame.unavailable, reason: frame.reason };
  }

  /** The text for a failed SD action: the card-level message while the card is unavailable, else the error itself. */
  explain(error: unknown): string {
    if (this.unavailable) return SD_UNAVAILABLE_TEXT;
    return error instanceof Error ? error.message : String(error);
  }
}

export const sdCardStore = new SdCardStore();
