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
  class="pid-card"
  class:disabled={!metric.supported}
  style="background: color-mix(in srgb, {metric.color} 5%, transparent); --module-accent: {metric.color};"
  {...rest}
>
  <header class="card-header">
    <div
      class="icon"
      style="background: color-mix(in srgb, {metric.color} 20%, transparent);"
    >
      <Icon name={metric.icon} size={32} />
    </div>
    <div class="status" style:--status-color={status}></div>
  </header>

  <div class="card-body">
    <span class="label">{metric.label}</span>
    <div class="value">
      <span class="number">{metric.displayValue}</span>
      <span class="unit">{metric.unit}</span>
    </div>
  </div>
</article>

<style>
  /* Your layout styles are beautiful and remain exactly the same */
  .pid-card {
    width: 100%;
    height: 100%;
    box-sizing: border-box;
    margin: 0;
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    padding: 16px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 12px;
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .pid-card:hover {
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    border-color: color-mix(
      in srgb,
      var(--module-accent) 40%,
      var(--pico-muted-border-color)
    ) !important;
  }

  .pid-card.disabled {
    opacity: 0.4;
    filter: grayscale(100%);
  }

  .pid-card.disabled:hover {
    transform: none;
    box-shadow: none;
  }

  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 8px;
    background: none;
    border: none;
    flex-shrink: 0;
  }

  .icon {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 48px;
    width: 48px;
    border-radius: 10px;
    transition: transform 0.2s ease;
  }

  .pid-card:hover .icon {
    transform: scale(1.05);
  }

  .status {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--status-color);
    box-shadow: 0 0 10px var(--status-color);
    animation: pulse-glow 2s infinite alternate;
  }

  .disabled .status {
    animation: none;
  }

  .card-body {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .label {
    font-size: 0.75rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--pico-muted-color);
    font-weight: 500;
  }

  .value {
    display: flex;
    align-items: baseline;
    gap: 6px;
  }

  .number {
    font-size: 2rem;
    font-weight: 700;
    line-height: 1;
    font-family: var(--pico-font-family-monospace);
  }

  .unit {
    font-size: 1rem;
    font-weight: 400;
    color: var(--pico-muted-color);
    font-family: var(--pico-font-family-monospace);
  }

  @keyframes pulse-glow {
    0% {
      opacity: 1;
      box-shadow: 0 0 1px var(--status-color);
    }
    50% {
      opacity: 0.85;
      box-shadow: 0 0 6px var(--status-color);
    }
    100% {
      opacity: 1;
      box-shadow: 0 0 8px var(--status-color);
    }
  }
</style>
