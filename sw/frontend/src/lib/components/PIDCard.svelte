<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { usePidData } from "$lib/pidHelpers.svelte.ts";
  import type { PidGridItem } from "$lib/types"; // Import your contract type

  interface Props {
    item: PidGridItem; // Expect our structural routing definition explicitly
    [key: string]: any;
  }

  let { item, ...rest }: Props = $props();

  // Consume our shared reactive helper library using the pid inside the layout item
  const metric = usePidData(() => item.pid);

  // Compute status state purely dependent on our helper values
  const status = $derived.by((): string => {
    if (!metric.supported) return "#6b7280";
    if (!metric.isValid) return "#ef4444";

    const pct =
      ((metric.currentValue - metric.min) / (metric.max - metric.min)) * 100;

    if (pct >= 20 && pct <= 80) return "var(--normal-color)";
    return "var(--warning-color)";
  });
</script>

<article
  class="dashboard-card pid-card"
  class:disabled={!metric.supported}
  style="--module-accent: {metric.color};"
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-icon">
      <Icon name={metric.icon} size={32} />
    </div>
    <div class="dashboard-card-status-dot" style:--status-color={status}></div>
  </header>

  <div class="dashboard-card-body">
    <span class="dashboard-card-label">{metric.label}</span>
    <div class="dashboard-card-value-group">
      <span class="dashboard-card-number">{metric.displayValue}</span>
      <span class="dashboard-card-unit">{metric.unit}</span>
    </div>
  </div>
</article>

<style>
  /* Local overrides if needed, but most are now global */
  .pid-card {
    justify-content: space-between;
  }
</style>
