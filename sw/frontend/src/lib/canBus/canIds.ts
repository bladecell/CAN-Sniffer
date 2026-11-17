// CAN id helpers for the CAN Bus page. An id is `(id, ext)`: standard ids are 11 bits, extended 29.

/** Map key that keeps a standard and an extended id with the same number apart. */
export function idKey(id: number, ext: boolean): number {
  return ext ? (id | 0x80000000) >>> 0 : id;
}

/** Hex as shown in the table: 3 digits standard, 8 extended. */
export function formatId(id: number, ext: boolean): string {
  return id.toString(16).toUpperCase().padStart(ext ? 8 : 3, "0");
}

/** OBD requests (0x7DF functional, 0x7E0-0x7E7 physical, 18DB33F1 extended functional) and responses. */
export function isObdId(id: number, ext: boolean): boolean {
  if (ext) return id === 0x18db33f1 || (id & 0xffffff00) === 0x18daf100 || (id & 0xffff00ff) === 0x18da00f1;
  return id === 0x7df || (id >= 0x7e0 && id <= 0x7ef);
}

/**
 * Request id an ECU answers, when the id follows the OBD response pattern: 0x7E8-0x7EF (request 0x7E0-0x7E7)
 * or 18DAF1xx (request 18DAxxF1). Null for every other id. Passive: nothing is sent.
 */
export function ecuRequestId(id: number, ext: boolean): number | null {
  if (!ext) return id >= 0x7e8 && id <= 0x7ef ? id - 8 : null;
  if ((id & 0xffffff00) !== 0x18daf100) return null;
  return 0x18da00f1 | ((id & 0xff) << 8);
}

export function hexBytes(data: ArrayLike<number>): string {
  let out = "";
  for (let i = 0; i < data.length; i++) out += (i ? " " : "") + data[i].toString(16).toUpperCase().padStart(2, "0");
  return out;
}
