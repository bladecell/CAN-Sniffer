/** Call every listener; one throwing listener must not starve the rest. */
export function dispatchToListeners<T>(listeners: Iterable<(update: T) => void>, update: T): void {
  for (const listener of listeners) {
    try {
      listener(update);
    } catch (e) {
      console.error("WebSocket listener failed", e);
    }
  }
}
