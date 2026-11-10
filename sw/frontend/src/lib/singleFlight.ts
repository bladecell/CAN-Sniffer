/** Callers that arrive while a request is in flight share it. The slot clears when it settles. */
export function singleFlight<T>(fn: () => Promise<T>): () => Promise<T> {
  let flight: Promise<T> | null = null;
  return () => (flight ??= fn().finally(() => (flight = null)));
}
