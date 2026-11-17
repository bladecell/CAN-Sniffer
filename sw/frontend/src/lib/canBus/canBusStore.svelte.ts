// CAN Bus page state: the raw stream into the capture model, a 10 Hz view snapshot, and the 1 Hz counters.
// Pause freezes the snapshot only. The model keeps counting, so the rates and drop counters stay live.
import { parseJsonResponse } from "$lib/api";
import { untrack } from "svelte";
import { connectionStore, type RawListener } from "$lib/connectionStore.svelte";
import { firmwareApi } from "$lib/firmwareApi.svelte";
import type { CanBusStatus, CanModeStatus } from "$lib/types";
import type { RawStats } from "$lib/wsFrames";
import { compileFilters, type FilterRule } from "./canFilter";
import { byIdRows, CanModel, ecuRows, logRows, recentFrames, type CanFrame, type EcuRow, type IdView, type LogRow, type TableFilter } from "./canModel";
import { findLastMatch, type RuleMatch } from "./canRules";
import { rulesStore } from "./canRulesStore.svelte";
import { idKey } from "./canIds";
import { captureRecords } from "./captureRetention";


const VIEW_MS = 100;
const SECOND_MS = 1000;
/** Stats frames come once a second; no frame for this long means the stream is not running. */
const STATS_STALE_MS = 3000;

export type IdRow = { key: number; id: number; ext: boolean; count: number; hz: number; name: string; seenMs: number; gapMs: number; hidden: boolean };

export type IdDetail = {
  key: number;
  id: number;
  ext: boolean;
  name: string;
  hz: number;
  count: number;
  dlc: number;
  changed: number;
  last: CanFrame;
  /** Newest first. */
  recent: CanFrame[];
};

export type TableMode = "log" | "byId";

export type Snapshot = {
  /** Device time of the first frame, the zero of the time column. */
  startUs: number | null;
  /** Latest value text per formula rule id. */
  ruleValues: Map<string, string>;
  rows: LogRow[];
  byId: IdView[];
  ids: IdRow[];
  ecus: EcuRow[];
  selected: IdDetail | null;
};

export type Counters = {
  now: number;
  fps: number;
  fpsHistory: number[];
  frames: number;
  streaming: boolean;
  filterOn: boolean;
  load: number | null;
  dropped: { driver: number; ring: number; socket: number } | null;
};

const emptySnapshot = (): Snapshot => ({ startUs: null, ruleValues: new Map(), rows: [], byId: [], ids: [], ecus: [], selected: null });
const emptyCounters = (): Counters => ({ now: Date.now(), fps: 0, fpsHistory: [], frames: 0, streaming: false, filterOn: false, load: null, dropped: null });

class CanBusStore {
  filter = $state<TableFilter>("all");
  /** Table filter rules (canFilter.ts). */
  rules = $state.raw<FilterRule[]>([]);
  tableMode = $state<TableMode>("log");
  /** Logging: the stream feeds the table. Off by default, not Raw mode (Monitor, polling unchanged). */
  logging = $state(false);
  selectedKey = $state<number | null>(null);
  snapshot = $state.raw<Snapshot>(emptySnapshot());
  counters = $state.raw<Counters>(emptyCounters());
  /** Device stats frame (0x11), null until the first one after 0xB0. */
  stats = $state.raw<RawStats | null>(null);
  bus = $state.raw<CanBusStatus | null>(null);
  mode = $state.raw<CanModeStatus | null>(null);

  private model = new CanModel();
  private statsMs = 0;
  private capture: { release: () => void; unsubscribe: () => void; second: ReturnType<typeof setInterval> } | null = null;
  private view: ReturnType<typeof setInterval> | null = null;

  private onRaw: RawListener = (frame) => {
    if (!captureRecords(this.logging, connectionStore.session === "active")) return;
    if (frame.kind === "rawBatch") {
      this.model.setRules(rulesStore.compiled);
      this.model.ingest(frame, Date.now());
      return;
    }
    this.stats = frame;
    this.statsMs = Date.now();
    this.refreshCounters();
  };

  constructor() {
    // A session that ends (replaced or parked) stops the device stream. Logging ends and the table is dropped with it.
    $effect.root(() => {
      $effect(() => {
        if (connectionStore.session !== "active") untrack(() => this.dropCapture());
      });
    });
  }

  /** The page is open: the table refreshes. Returns the teardown for the page; logging keeps running after it. */
  attach(): () => void {
    this.startView();
    return () => {
      if (this.view) clearInterval(this.view);
      this.view = null;
    };
  }

  /** Start and Pause. Start keeps the table that is there; Pause stops the stream and keeps the rows. */
  setLogging(on: boolean) {
    this.logging = on;
    if (on) this.startCapture();
    else this.stopCapture();
  }

  /** Clear: the rows and the id list start over. Logging stays as it is. */
  clearTable() {
    this.model.reset();
    this.refreshCounters();
    this.refreshSnapshot();
  }

  private startCapture() {
    if (this.capture) return;
    this.statsMs = 0;
    const release = connectionStore.acquireRawStream();
    const unsubscribe = connectionStore.subscribeRaw((f) => this.onRaw(f));
    let last = Date.now();
    // Rates keep counting while nobody watches, so the table is current when the page opens again.
    const second = setInterval(() => {
      const now = Date.now();
      this.model.tick((now - last) / SECOND_MS);
      last = now;
      this.refreshCounters();
    }, SECOND_MS);
    this.capture = { release, unsubscribe, second };
    this.refreshCounters();
    void this.refreshBus();
  }

  private stopCapture() {
    if (!this.capture) return;
    clearInterval(this.capture.second);
    this.capture.unsubscribe();
    this.capture.release();
    this.capture = null;
    this.refreshCounters();
  }

  private dropCapture() {
    this.logging = false;
    this.stopCapture();
    this.model.reset();
    this.refreshSnapshot();
  }

  private startView() {
    if (this.view) return;
    this.view = setInterval(() => {
      this.refreshSnapshot();
    }, VIEW_MS);
    this.refreshSnapshot();
  }

  setFilter(filter: TableFilter) {
    this.filter = filter;
    this.refreshSnapshot();
  }

  setFilterRules(rules: FilterRule[]) {
    this.rules = rules;
    this.refreshSnapshot();
  }

  setTableMode(mode: TableMode) {
    this.tableMode = mode;
    this.refreshSnapshot();
  }

  select(key: number | null) {
    this.selectedKey = key;
    this.refreshSnapshot();
  }

  /** GET /can_bus and GET /can/mode. Open routes; a failed read keeps the last value. */
  async refreshBus() {
    try {
      this.bus = await parseJsonResponse<CanBusStatus>(await firmwareApi.canBus());
    } catch {
      // Keep the last known bus status.
    }
    try {
      this.mode = await parseJsonResponse<CanModeStatus>(await firmwareApi.canMode());
    } catch {
      // Keep the last known mode.
    }
  }

  private refreshCounters() {
    const now = Date.now();
    // Not logging: the rates and the drop counters are not running, so they read empty rather than stale.
    if (!this.logging) {
      this.counters = { ...emptyCounters(), now, frames: this.model.frames };
      return;
    }
    const s = this.stats;
    this.counters = {
      now,
      fps: this.model.fps.at(-1) ?? 0,
      fpsHistory: this.model.fps.slice(),
      frames: this.model.frames,
      streaming: s !== null && s.active && now - this.statsMs < STATS_STALE_MS,
      filterOn: s?.filterOn ?? false,
      load: s?.busLoadPermille ?? null,
      dropped: s ? { driver: s.droppedDriver, ring: s.droppedRing, socket: s.droppedSocket } : null,
    };
  }

  private refreshSnapshot() {
    const m = this.model;
    const ids: IdRow[] = [];
    for (const e of m.ids.values()) {
      ids.push({ key: idKey(e.id, e.ext), id: e.id, ext: e.ext, count: e.count, hz: e.hz, name: e.name, seenMs: e.seenMs, gapMs: e.gapMs, hidden: e.hidden });
    }
    ids.sort((a, b) => a.id - b.id || Number(a.ext) - Number(b.ext));
    const keep = compileFilters(this.rules);
    const selected = this.selectedKey === null ? null : this.detailFor(this.selectedKey);
    this.snapshot = { startUs: m.startUs, ruleValues: new Map(m.ruleValues), rows: logRows(m, this.filter, keep), byId: byIdRows(m, this.filter, keep), ids, ecus: ecuRows(m, Date.now()), selected };
  }

  /** The newest frame any id produced that the match accepts (the draft rule preview). */
  lastMatchFor(match: RuleMatch): CanFrame | null {
    return findLastMatch(recentFrames(this.model), match);
  }

  private detailFor(key: number): IdDetail | null {
    const e = this.model.ids.get(key);
    if (!e) return null;
    return {
      key,
      id: e.id,
      ext: e.ext,
      name: e.name,
      hz: e.hz,
      count: e.count,
      dlc: e.last.data.length,
      changed: e.changed,
      last: e.last,
      recent: e.recent.slice().reverse(),
    };
  }
}

export const canBus = new CanBusStore();
