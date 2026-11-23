// The one frame loop for every chart: capped at 60 fps, or 30 on a coarse pointer, and idle while the device sleeps.
import { createFrameLoop } from "./frameLoop";
import { connectionStore } from "./connectionStore.svelte";

const coarse = typeof matchMedia === "function" ? matchMedia("(pointer: coarse)") : null;

export const chartFrames = createFrameLoop(
  { request: (cb) => requestAnimationFrame(cb) },
  { maxFps: () => (coarse?.matches ? 30 : 60), active: () => !connectionStore.sleeping },
);

// Wake from sleep restarts the loop: it stops on its own while inactive, and nothing else schedules it.
$effect.root(() => {
  $effect(() => {
    if (!connectionStore.sleeping) chartFrames.kick();
  });
});
