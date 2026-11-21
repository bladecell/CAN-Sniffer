import { cubicOut } from "svelte/easing";
import { reduced } from "./dashboard/flip";

/** One open and close animation for every accordion: `transition:slide={accordion()}`. */
export const accordion = () => ({ duration: reduced() ? 0 : 130, easing: cubicOut });
