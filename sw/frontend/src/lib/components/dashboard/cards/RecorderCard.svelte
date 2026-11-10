<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { recordingStore } from "$lib/recordingStore.svelte";
  import { elapsedClock, formatBytes } from "$lib/recorderView";
  import { recorderControl } from "./controls.svelte";
  import ToggleControl from "./ToggleControl.svelte";

  let { card }: { card: Card } = $props();

  const control = recorderControl();
  const on = $derived(control.on);
  const rec = $derived(recordingStore.status);
  // Elapsed time ticks once a second while a session records and the card is 3 rows or taller. The file size
  // is re-read every 5 s (GET /recorder), since the 1 Hz frame does not carry it.
  let now = $state(Date.now());
  $effect(() => {
    if (!on || card.h < 3) return;
    let ticks = 0;
    now = Date.now();
    const timer = setInterval(() => {
      now = Date.now();
      if (++ticks % 5 === 0) void recordingStore.refresh();
    }, 1000);
    return () => clearInterval(timer);
  });
  const readout = $derived.by(() => {
    if (!on || recordingStore.startedAt === 0) return null;
    // The file size comes from GET /recorder, which the card re-reads with the status, not from the 1 Hz frame.
    const unit = card.w >= 4 && rec ? `${formatBytes(rec.file_bytes)} written` : null;
    return { value: elapsedClock(now - recordingStore.startedAt), unit };
  });
  const note = $derived(rec?.state === "error" && rec.last_error ? rec.last_error : on && rec ? `Trip ${rec.trip}, ${rec.rows} rows` : null);
</script>

<ToggleControl {card} name="Recorder" word={control.word} {on} reason={control.reason} {note} {readout} onToggle={control.toggle} />
