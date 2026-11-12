const listeners = new Set<() => void>();

export function onConnectionOpen(listener: () => void): () => void {
  listeners.add(listener);
  return () => listeners.delete(listener);
}

export function notifyConnectionOpen(): void {
  for (const listener of listeners) listener();
}

const dropListeners = new Set<() => void>();

export function onConnectionDrop(listener: () => void): () => void {
  dropListeners.add(listener);
  return () => dropListeners.delete(listener);
}

export function notifyConnectionDrop(): void {
  for (const listener of dropListeners) listener();
}
