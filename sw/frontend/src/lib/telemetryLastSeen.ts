import { connectionStore } from "./connectionStore.svelte";

/** Frontend receipt time per PID (Date.now()), not the device clock. Bounded by the 256-definition limit. */
const received = new Map<number, number>();
connectionStore.subscribe((update) => received.set(update.pid, Date.now()));

export function lastReceivedMs(pid: number): number | null {
  return received.get(pid) ?? null;
}
