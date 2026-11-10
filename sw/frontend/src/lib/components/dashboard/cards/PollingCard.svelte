<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { updatesPerSecond } from "$lib/dashboardCardView";
  import { pidUpdateCount } from "$lib/livePidStore.svelte";
  import { pollingControl } from "./controls.svelte";
  import ToggleControl from "./ToggleControl.svelte";

  let { card }: { card: Card } = $props();

  const control = pollingControl();
  const on = $derived(control.on);

  // PIDs/s: PID updates counted over each second while polling runs and the card is 3 rows or taller.
  let perSecond = $state<number | null>(null);
  $effect(() => {
    if (!on || card.h < 3) {
      perSecond = null;
      return;
    }
    let last = pidUpdateCount();
    let at = Date.now();
    const timer = setInterval(() => {
      const count = pidUpdateCount();
      const now = Date.now();
      perSecond = updatesPerSecond(count - last, now - at);
      last = count;
      at = now;
    }, 1000);
    return () => clearInterval(timer);
  });
  const readout = $derived(on && perSecond !== null ? { value: String(perSecond), unit: "PIDs/s" } : null);
</script>

<ToggleControl
  {card}
  name="Polling"
  word={control.word}
  {on}
  reason={control.reason}
  {readout}
  onToggle={control.toggle}
/>
