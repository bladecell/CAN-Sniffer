// When the raw stream records into the table. Pure: the store decides nothing else.

/** Records only while logging is on and the device session is this page's. */
export function captureRecords(logging: boolean, sessionActive: boolean): boolean {
  return logging && sessionActive;
}
