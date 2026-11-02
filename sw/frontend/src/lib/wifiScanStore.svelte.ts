// Last Wi-Fi scan, kept across visits to Settings. The firmware drops results after 30 s (docs/api.md, Wi-Fi scan
// lifecycle), so this copy is kept on the client and marked old after the same time. Nothing scans on its own.
import { parseJsonResponse } from "./api";
import { firmwareApi } from "./firmwareApi.svelte";
import { parseScanReply, type ScanNetwork } from "./settingsModel";

const POLL_MS = 1000;
const MAX_POLLS = 20;
const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

class WifiScanStore {
  networks = $state.raw<ScanNetwork[]>([]);
  /** Client clock (ms) when the last complete scan arrived. null until one has. */
  scannedAt = $state<number | null>(null);
  scanning = $state(false);
  error = $state<string | null>(null);

  /** POST starts one scan; GET is polled until it reports complete. A second call while one runs is ignored. */
  async scan(): Promise<void> {
    if (this.scanning) return;
    this.scanning = true;
    this.error = null;
    try {
      await parseJsonResponse<unknown>(await firmwareApi.startWifiScan());
      for (let i = 0; i < MAX_POLLS; i++) {
        await new Promise((resolve) => setTimeout(resolve, POLL_MS));
        const reply = parseScanReply(await parseJsonResponse<unknown>(await firmwareApi.wifiScan()));
        if (reply === null) throw new Error("Unexpected scan reply");
        if (reply.state === "complete") {
          this.networks = reply.networks;
          this.scannedAt = Date.now();
          return;
        }
      }
      this.error = "The scan took too long. Try again.";
    } catch (e) {
      this.error = errorText(e);
    } finally {
      this.scanning = false;
    }
  }
}

export const wifiScanStore = new WifiScanStore();
