import type { RecorderState } from "./types";

// Binary WebSocket frames, little-endian, exact sizes per docs/api.md.
export type WsFrame =
  | { kind: "pid"; pid: number; value: number; lastUpdated: number; rate: number; isSupported: boolean; isValid: boolean }
  | { kind: "canStatus"; state: number; utilization: number; batteryVoltage: number; busConnected: boolean }
  | { kind: "recorder"; state: RecorderState; trip: number; rows: number; dropped: number; hasError: boolean }
  | { kind: "pong" }
  | ({ kind: "rawBatch" } & RawBatch)
  | ({ kind: "rawStats" } & RawStats);

/** One CAN frame from a 0x10 batch. `t` is not here: the batch's baseTs plus the running sum of `dt` gives it. */
export type RawRecord = { dt: number; id: number; ext: boolean; rtr: boolean; tx: boolean; data: Uint8Array };
export type RawBatch = { dropped: number; baseTs: number; records: RawRecord[] };
export type RawStats = {
  active: boolean;
  filterOn: boolean;
  streamed: number;
  droppedDriver: number;
  droppedRing: number;
  droppedSocket: number;
  ringCapacity: number;
  ringPeak: number;
  /** Permille of the bit rate, or null while the device does not know it (0xFFFF). */
  busLoadPermille: number | null;
};

export const RAW_BATCH_MIN_BYTES = 15;
export const RAW_BATCH_MAX_BYTES = 257;
export const RAW_BATCH_MAX_RECORDS = 16;
export const RAW_STATS_BYTES = 25;
const RAW_RECORD_HEADER_BYTES = 7;

export const PID_FRAME_BYTES = 20;
export const CAN_STATUS_FRAME_BYTES = 12;
export const RECORDER_FRAME_BYTES = 16;
const RECORDER_STATES: RecorderState[] = ["idle", "recording", "stopping", "error"];

/** Returns null for anything that is not an exactly sized, known frame. */
export function parseWsFrame(data: unknown): WsFrame | null {
  if (!(data instanceof ArrayBuffer) || data.byteLength === 0) return null;
  const view = new DataView(data);
  const type = view.getUint8(0);

  if (type === 0x02) {
    if (view.byteLength !== PID_FRAME_BYTES || view.getUint8(1) !== PID_FRAME_BYTES - 2) return null;
    return {
      kind: "pid",
      pid: view.getUint32(2, true),
      value: view.getFloat32(6, true),
      lastUpdated: view.getUint32(10, true),
      rate: view.getUint32(14, true),
      isSupported: view.getUint8(18) !== 0,
      isValid: view.getUint8(19) !== 0,
    };
  }
  if (type === 0x03) {
    if (view.byteLength !== CAN_STATUS_FRAME_BYTES || view.getUint8(1) !== CAN_STATUS_FRAME_BYTES - 2) return null;
    return {
      kind: "canStatus",
      state: view.getUint8(2),
      utilization: view.getFloat32(3, true),
      batteryVoltage: view.getFloat32(7, true),
      busConnected: view.getUint8(11) !== 0,
    };
  }
  if (type === 0x04) {
    const state = RECORDER_STATES[view.getUint8(2)];
    if (view.byteLength !== RECORDER_FRAME_BYTES || view.getUint8(1) !== RECORDER_FRAME_BYTES - 2 || state === undefined) return null;
    return { kind: "recorder", state, trip: view.getUint32(3, true), rows: view.getUint32(7, true), dropped: view.getUint32(11, true), hasError: view.getUint8(15) !== 0 };
  }
  if (type === 0xa3) return view.byteLength === 1 ? { kind: "pong" } : null;
  if (type === 0x10) {
    const batch = parseRawBatch(view);
    return batch ? { kind: "rawBatch", ...batch } : null;
  }
  if (type === 0x11) {
    const stats = parseRawStats(view);
    return stats ? { kind: "rawStats", ...stats } : null;
  }
  return null;
}

/** 0x10: a malformed record fails the whole batch, so nothing from a hostile frame reaches the table. */
function parseRawBatch(view: DataView): RawBatch | null {
  const total = view.byteLength;
  if (total < RAW_BATCH_MIN_BYTES || total > RAW_BATCH_MAX_BYTES || view.getUint8(1) !== total - 2) return null;
  const records: RawRecord[] = [];
  let off = 8;
  while (off < total) {
    if (records.length >= RAW_BATCH_MAX_RECORDS || off + RAW_RECORD_HEADER_BYTES > total) return null;
    const dt = view.getInt16(off, true);
    const word = view.getUint32(off + 2, true);
    const ctl = view.getUint8(off + 6);
    const len = ctl & 0x0f;
    if (len > 8 || (ctl & 0x70) !== 0 || off + RAW_RECORD_HEADER_BYTES + len > total) return null;
    const ext = (word & 0x80000000) !== 0;
    const rtr = (word & 0x40000000) !== 0;
    const id = word & 0x1fffffff;
    if ((word & 0x20000000) !== 0 || (!ext && id > 0x7ff)) return null;
    records.push({
      dt,
      id,
      ext,
      rtr,
      tx: (ctl & 0x80) !== 0,
      data: new Uint8Array(view.buffer, view.byteOffset + off + RAW_RECORD_HEADER_BYTES, len).slice(),
    });
    off += RAW_RECORD_HEADER_BYTES + len;
  }
  if (records.length === 0) return null;
  return { dropped: view.getUint16(2, true), baseTs: view.getUint32(4, true), records };
}

/** 0x11: flags, cumulative counters since 0xB0, and the bus load. */
function parseRawStats(view: DataView): RawStats | null {
  if (view.byteLength !== RAW_STATS_BYTES || view.getUint8(1) !== RAW_STATS_BYTES - 2) return null;
  const flags = view.getUint8(2);
  const load = view.getUint16(23, true);
  return {
    active: (flags & 0x01) !== 0,
    filterOn: (flags & 0x02) !== 0,
    streamed: view.getUint32(3, true),
    droppedDriver: view.getUint32(7, true),
    droppedRing: view.getUint32(11, true),
    droppedSocket: view.getUint32(15, true),
    ringCapacity: view.getUint16(19, true),
    ringPeak: view.getUint16(21, true),
    busLoadPermille: load === 0xffff ? null : Math.min(load, 1000),
  };
}
