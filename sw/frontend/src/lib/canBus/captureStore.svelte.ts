// SD captures of the raw stream: the status poll, start and stop, and the file list. Every write is gated.
import { parseJsonResponse } from "$lib/api";
import { firmwareApi } from "$lib/firmwareApi.svelte";
import { validCaptureName } from "./canCapture";
import type { CaptureList, CaptureStatus } from "$lib/types";

const POLL_MS = 1000;

/** The reason from a failed response (the firmware sends `{status:"error", reason}`), else the HTTP status. */
async function failure(res: Response): Promise<Error> {
  const body = (await res.json().catch(() => null)) as { reason?: unknown } | null;
  return new Error(typeof body?.reason === "string" ? body.reason : `HTTP ${res.status}`);
}

const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

class CaptureStore {
  status = $state.raw<CaptureStatus | null>(null);
  files = $state.raw<CaptureList["captures"]>([]);
  truncated = $state(false);
  listError = $state<string | null>(null);
  busy = $state(false);

  get capturing() {
    return this.status?.state === "capturing";
  }

  /** Polls the status while a view that shows it is open. Returns the stop function. */
  watch(): () => void {
    void this.readStatus();
    const timer = setInterval(() => void this.readStatus(), POLL_MS);
    return () => clearInterval(timer);
  }

  async readStatus() {
    try {
      this.status = await parseJsonResponse<CaptureStatus>(await firmwareApi.canCapture());
    } catch {
      // Keep the last status; the next poll tries again.
    }
  }

  async refreshList() {
    try {
      const list = await parseJsonResponse<CaptureList>(await firmwareApi.captureList());
      this.files = list.captures;
      this.truncated = list.truncated;
      this.listError = null;
    } catch (e) {
      this.listError = errorText(e);
    }
  }

  /** Save to SD: starts a capture, or stops the running one. Returns the reason on failure, else null. */
  async toggle(): Promise<string | null> {
    this.busy = true;
    try {
      const res = this.capturing ? await firmwareApi.stopCapture() : await firmwareApi.startCapture();
      this.status = await parseJsonResponse<CaptureStatus>(res);
      if (!this.capturing) await this.refreshList();
      return null;
    } catch (e) {
      return errorText(e);
    } finally {
      this.busy = false;
    }
  }

  async download(name: string): Promise<string | null> {
    if (!validCaptureName(name)) return "Not a capture file";
    try {
      const res = await firmwareApi.downloadCapture(name);
      if (!res.ok) throw await failure(res);
      const url = URL.createObjectURL(await res.blob());
      const link = document.createElement("a");
      link.href = url;
      link.download = name;
      link.click();
      URL.revokeObjectURL(url);
      return null;
    } catch (e) {
      return errorText(e);
    }
  }

  async remove(name: string): Promise<string | null> {
    if (!validCaptureName(name)) return "Not a capture file";
    try {
      const res = await firmwareApi.deleteCapture(name);
      if (!res.ok) throw await failure(res);
      await this.refreshList();
      return null;
    } catch (e) {
      return errorText(e);
    }
  }
}

export const captures = new CaptureStore();
