// Binary WebSocket frames, little-endian, exact sizes per docs/api.md.
export type WsFrame =
  | { kind: "pid"; pid: number; value: number; lastUpdated: number; rate: number; isSupported: boolean; isValid: boolean }
  | { kind: "canStatus"; state: number; utilization: number; batteryVoltage: number; busConnected: boolean }
  | { kind: "pong" };

export const PID_FRAME_BYTES = 20;
export const CAN_STATUS_FRAME_BYTES = 12;

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
  if (type === 0xa3) return view.byteLength === 1 ? { kind: "pong" } : null;
  return null;
}
