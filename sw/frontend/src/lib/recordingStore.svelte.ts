import { alertStore } from "./alertStore.svelte";
import { auth } from "./auth.svelte";
import { parseJsonResponse, parseJsonResponseAllowEmpty } from "./api";
import { firmwareApi } from "./firmwareApi.svelte";
import { singleFlight } from "./singleFlight";
import { createStreamRefCount } from "./streamRefCount";
import {
  cooldownSeconds,
  isTripCsvName,
  retryAfterSeconds,
  shouldLoadTrips,
  sidecarName,
  START_COOLDOWN_MS,
  validTrips,
  type TripEntry,
} from "./recorderView";
import type { RecorderSettingsPatch, RecorderStatus, RecorderTrips } from "./types";
import type { WsFrame } from "./wsFrames";

const TRIPS_DIR = "logs/trips";
const START_COOLDOWN_S = START_COOLDOWN_MS / 1000;
const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

/** Recorder state from GET /recorder and the 1 Hz WebSocket frame, plus the trip list and cooldown clock. */
class RecordingStore {
  status = $state<RecorderStatus | null>(null);
  trips = $state<TripEntry[]>([]);
  tripsTruncated = $state(false);
  /** Set when the trip list could not be read; `no-sd` for 503 (no card or the card is busy). */
  tripsError = $state<"no-sd" | "failed" | null>(null);
  tripsLoading = $state(false);
  /** A start or stop request is in flight. */
  busy = $state(false);
  /** `Date.now()` when this session was first seen recording, 0 when not recording. Client clock: the API has no start time. */
  startedAt = $state(0);

  /** `Date.now()` value the start cooldown ends at, and the clock the countdown reads. */
  cooldownUntil = $state(0);
  clock = $state(Date.now());
  coolingDown = $derived(this.cooldownUntil > this.clock);
  cooldownLeft = $derived(cooldownSeconds(this.cooldownUntil, this.clock));

  private setStatus(next: RecorderStatus) {
    this.status = next;
    if (next.state !== "recording") this.startedAt = 0;
    else if (this.startedAt === 0) this.startedAt = Date.now();
  }

  private startCooldown(seconds: number) {
    this.clock = Date.now();
    this.cooldownUntil = this.clock + seconds * 1000;
  }

  /** WebSocket 0x04 frame: live counters. The file size and last error come from GET /recorder. */
  applyFrame(frame: Extract<WsFrame, { kind: "recorder" }>) {
    const previous = this.status?.state;
    this.setStatus({
      state: frame.state,
      last_error: frame.hasError ? (this.status?.last_error ?? null) : null,
      trip: frame.trip,
      rows: frame.rows,
      dropped: frame.dropped,
      file_bytes: this.status?.file_bytes ?? 0,
      trigger: this.status?.trigger ?? "manual",
      continuous_polling: this.status?.continuous_polling ?? false,
    });
    if (previous !== frame.state && shouldLoadTrips(auth.canUseGated, this.tripsWatch.count() > 0)) void this.loadTrips();
  }

  /** The Recorder view lists trips, so it wants them even without a session (the read then asks for a login). Returns the release. */
  private tripsWatch = createStreamRefCount(() => {}, () => {});
  watchTrips = () => this.tripsWatch.acquire();

  refresh = singleFlight(() => this.readStatus());

  private lastRead = 0;
  /** Mount-time and open-time loads ask for the same status within a moment of each other: the second one reuses the first. */
  refreshIfStale = () => (Date.now() - this.lastRead < 2_000 ? Promise.resolve() : this.refresh());

  /**
   * The WebSocket frame has no file size, so GET /recorder is re-read every 5 s while anything shows it.
   * One timer for the page, however many views ask. Returns the release.
   */
  private sizeTimer: ReturnType<typeof setInterval> | undefined;
  private sizeWatch = createStreamRefCount(
    () => (this.sizeTimer = setInterval(() => void this.refresh(), 5_000)),
    () => clearInterval(this.sizeTimer),
  );
  watchFileSize = () => this.sizeWatch.acquire();

  private async readStatus() {
    this.lastRead = Date.now();
    try {
      const response = await firmwareApi.recorder();
      this.setStatus(await parseJsonResponse<RecorderStatus>(response));
    } catch (e) {
      console.error("Failed to read recorder", e);
    }
  }

  async loadTrips() {
    this.tripsLoading = true;
    try {
      const response = await firmwareApi.recorderTrips();
      if (response.status === 503) {
        this.trips = [];
        this.tripsError = "no-sd";
        return;
      }
      const data = await parseJsonResponse<RecorderTrips>(response);
      // Names come from the device: only recorder trip names may reach a file request.
      this.trips = validTrips(data.trips);
      this.tripsTruncated = data.truncated;
      this.tripsError = null;
    } catch (e) {
      console.error("Failed to load trips", e);
      this.tripsError = "failed";
    } finally {
      this.tripsLoading = false;
    }
  }

  /**
   * Start with the PIDs OBD2 already polls. Cooldown (firmware `kStartCooldownMs`, docs/api.md Recorder):
   * an accepted start opens it, a 429 sets it from Retry-After, and a 503 (card failure) counts as a
   * failed start that opens it. A 409 or a network error does not.
   */
  async start() {
    if (this.busy || this.coolingDown) return;
    this.busy = true;
    try {
      const response = await firmwareApi.startRecorder();
      if (response.status === 429) {
        const seconds = retryAfterSeconds(response.headers.get("Retry-After"));
        this.startCooldown(seconds);
        alertStore.add(`Recorder is cooling down. Try again in ${seconds} s.`, "warning");
        return;
      }
      if (response.status === 503) this.startCooldown(START_COOLDOWN_S);
      this.setStatus(await parseJsonResponse<RecorderStatus & { status?: string }>(response));
      this.startCooldown(START_COOLDOWN_S);
      void this.loadTrips();
    } catch (e) {
      alertStore.add(`Failed to start recording: ${errorText(e)}`, "error");
      await this.readStatus();
    } finally {
      this.busy = false;
    }
  }

  /** Saves the trigger mode and continuous polling (PATCH /recorder). The reply carries the new settings, so the controls follow the device. */
  async saveSettings(patch: RecorderSettingsPatch) {
    try {
      this.setStatus(await parseJsonResponse<RecorderStatus & { status?: string }>(await firmwareApi.patchRecorder(patch)));
    } catch (e) {
      alertStore.add(`Failed to save the recorder settings: ${errorText(e)}`, "error");
      await this.readStatus();
    }
  }

  /** Stop drains the queue and can take up to 5 s. The next start is refused for the cooldown after it. */
  async stop() {
    if (this.busy) return;
    this.busy = true;
    try {
      this.setStatus(await parseJsonResponse<RecorderStatus & { status?: string }>(await firmwareApi.stopRecorder()));
      this.startCooldown(START_COOLDOWN_S);
      void this.loadTrips();
    } catch (e) {
      alertStore.add(`Failed to stop recording: ${errorText(e)}`, "error");
      await this.readStatus();
    } finally {
      this.busy = false;
    }
  }

  /** Saves the CSV to a file on this device. The SD file endpoint needs the session token, so it is fetched, not linked. */
  async download(name: string) {
    if (!isTripCsvName(name)) throw new Error("Not a trip file");
    const response = await firmwareApi.downloadSdFile(`${TRIPS_DIR}/${name}`);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const url = URL.createObjectURL(await response.blob());
    const link = document.createElement("a");
    link.href = url;
    link.download = name;
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1_000);
  }

  /**
   * Sidecar first, then the CSV, so a failure never leaves an invisible sidecar behind a visible CSV.
   * A sidecar already gone (404) is fine. If the CSV delete fails the error propagates and the list is
   * reloaded, so the row stays (it may now show without a summary).
   */
  async deleteTrip(name: string, sidecar: boolean) {
    if (!isTripCsvName(name)) throw new Error("Not a trip file");
    try {
      if (sidecar) {
        const response = await firmwareApi.deleteSdFile(`${TRIPS_DIR}/${sidecarName(name)}`);
        if (response.status !== 404) await parseJsonResponseAllowEmpty(response);
      }
      await parseJsonResponseAllowEmpty(await firmwareApi.deleteSdFile(`${TRIPS_DIR}/${name}`));
    } finally {
      await this.loadTrips();
    }
  }
}

export const recordingStore = new RecordingStore();
