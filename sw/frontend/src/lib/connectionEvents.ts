const listeners = new Set<() => void>();

export function onConnectionOpen(listener: () => void): () => void {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

export function notifyConnectionOpen(): void {
  for (const listener of listeners) listener();
}
