import { SvelteMap } from "svelte/reactivity";
import type { PidValue } from "./types";

/** Current in-memory values only; cardinality is bounded by firmware's 256-definition limit. */
export const livePids = new SvelteMap<number, PidValue>();
