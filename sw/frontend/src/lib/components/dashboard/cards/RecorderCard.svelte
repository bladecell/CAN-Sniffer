<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { auth } from "$lib/auth.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { recordingStore } from "$lib/recordingStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { controlReason } from "$lib/dashboardCardView";
  import { loadWhileMounted } from "./cardSources.svelte";
  import ToggleControl from "./ToggleControl.svelte";

  let { card }: { card: Card } = $props();

  let sdRead = $state(false);
  loadWhileMounted(async () => {
    void recordingStore.refresh();
    await vehicleStore.requestSystem();
    sdRead = true;
  });

  const rec = $derived(recordingStore.status);
  const on = $derived(rec?.state === "recording");
  const stopping = $derived(rec?.state === "stopping" || recordingStore.busy);
  const word = $derived(
    rec === null ? "No data"
    : stopping ? "Stopping"
    : rec.state === "recording" ? "Recording"
    : rec.state === "error" ? "Error"
    : "Off",
  );
  const reason = $derived(
    controlReason({
      linked: connectionStore.connected,
      busUp: connectionStore.wsCanStatus.state === "CAN Connected",
      sdPresent: sdRead ? vehicleStore.system.sd_card_detected : null,
      needsLogin: true,
      loggedIn: auth.view.loggedIn,
    }) ??
      (rec === null ? "No data" : recordingStore.coolingDown ? `Wait ${recordingStore.cooldownLeft} s` : stopping ? "Working" : null),
  );
  const note = $derived(rec?.state === "error" && rec.last_error ? rec.last_error : on && rec ? `Trip ${rec.trip}, ${rec.rows} rows` : null);

  function toggle() {
    if (reason !== null) return;
    if (on) void recordingStore.stop();
    else void recordingStore.start();
  }
</script>

<ToggleControl {card} name="Recorder" {word} {on} {reason} {note} onToggle={toggle} />
