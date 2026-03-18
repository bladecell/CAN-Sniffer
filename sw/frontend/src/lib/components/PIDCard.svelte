<script>
  import Icon from "$lib/Icon.svelte";
  import { canStore } from "$lib/canStore.svelte.js";

  let {
    pid,
    label = "Metric",
    unit = "%",
    icon = "gear",
    color = "#10b981",
    min = 0,
    max = 100,
  } = $props();

  // FIX 1: Use 'pids' (from store) and .get() (for Map)
  // Svelte 5 reactivity handles Map.get() inside $derived automatically
  const pidData = $derived(canStore.pids.get(pid));

  // FIX 2: Handle data not arriving yet (undefined check)
  const currentValue = $derived(pidData?.value ?? 0);

  // FIX 3: Store property is named 'valid', not 'isValid'
  const isValid = $derived(pidData?.valid ?? false);

  // 'supported' isn't in the binary stream yet, default to true
  const supported = $derived(pidData?.supported ?? false);

  const status = $derived.by(() => {
    if (!supported) return "#6b7280";
    if (!isValid) return "#ef4444";

    const pct = ((currentValue - min) / (max - min)) * 100;

    if (pct >= 20 && pct <= 80) return "var(--normal-color)"; // Green (Normal)
    return "var(--warning-color)"; // Orange (Warning)
  });

  // Handle initial state where currentValue might be 0/undefined
  const displayValue = $derived(supported ? currentValue.toFixed(1) : "···");
</script>

<article
  class="pid-card"
  class:disabled={!supported}
  style="background: color-mix(in srgb, {color} 5%, transparent);"
>
  <div class="card-header">
    <div
      data-swapy-handle
      class="icon"
      style="background: color-mix(in srgb, {color} 20%, transparent);"
    >
      <Icon name={icon} size={32} />
    </div>
    <div class="status" style:--status-color={status}></div>
  </div>

  <div class="card-body">
    <span class="label">{label}</span>
    <div class="value">
      <span class="number">{displayValue}</span>
      <span class="unit">{unit}</span>
    </div>
  </div>
</article>

<style>
  :global(:root) {
    --card-pref-width: 280px; /* Your "Preferred" Size */
    --card-gap: 1rem;
  }
  .pid-card {
    display: flex;
    flex-direction: column;
    justify-content: space-between;
    height: var(--card-height);
    width: 100%;
    justify-self: start;
    padding: 16px;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 12px;
    transition:
      transform 0.2s ease,
      box-shadow 0.2s ease;
  }

  .pid-card:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
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
    margin-bottom: 12px;
  }

  .icon {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 48px;
    width: 48px;
    border-radius: 10px;
    transition: transform 0.2s ease;
    cursor: grab;
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
