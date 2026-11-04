<script lang="ts">
  import type { Card } from "$lib/dashboardModel";
  import { auth } from "$lib/auth.svelte";
  import { connectionStore } from "$lib/connectionStore.svelte";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { controlReason } from "$lib/dashboardCardView";
  import { loadWhileMounted } from "./cardSources.svelte";
  import ToggleControl from "./ToggleControl.svelte";

  let { card }: { card: Card } = $props();

  // Polling is switched only by this control. The dashboard never starts it.
  loadWhileMounted(() => vehicleStore.getObd2Status());

  let busy = $state(false);
  const status = $derived(vehicleStore.obd2Status);
  const on = $derived(status?.auto_poll ?? false);
  const reason = $derived(
    controlReason({
      linked: connectionStore.connected,
      busUp: connectionStore.wsCanStatus.state === "CAN Connected",
      sdPresent: null,
      needsLogin: false,
      loggedIn: auth.view.loggedIn,
    }) ?? (status === null ? "No data" : null),
  );

  async function toggle() {
    if (busy || reason !== null) return;
    busy = true;
    try {
      await vehicleStore.setAutoPoll(!on);
    } finally {
      busy = false;
    }
  }
</script>

<ToggleControl
  {card}
  name="Polling"
  word={status === null ? "No data" : on ? "On" : "Off"}
  {on}
  reason={reason ?? (busy ? "Working" : null)}
  onToggle={toggle}
/>
