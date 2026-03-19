<script>
  import DataCard from "./lib/components/DataCard.svelte";
  import { canStore } from "./lib/canStore.svelte.js";
  import { onMount } from "svelte";

  let batteryVoltage = $state(10);

  onMount(() => {
    // Simulate real-time voltage updates
    const interval = setInterval(() => {
      batteryVoltage = 10 + Math.random() * 4; // Random voltage between 10 and 14
    }, 2000);

    canStore.requestVin(); // Fetch VIN on mount

    return () => clearInterval(interval);
  });
</script>

<DataCard
  label="Battery Voltage"
  value={batteryVoltage}
  unit=" V"
  statusColor={batteryVoltage >= 12.0
    ? "var(--normal-color)"
    : "var(--error-color)"}
/>

<DataCard label="VIN" value={canStore.vin} />
