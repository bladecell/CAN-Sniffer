<script>
  let {
    label = "Data Card",
    value = null,
    unit = "",
    statusColor = "var(--pico-primary)",
    dot = false,
    ...rest
  } = $props();

  const displayValue = $derived(
    value !== null && value !== undefined ? value : "N/A",
  );

  const isNumeric = $derived(!isNaN(Number(displayValue)));
  const formattedValue = $derived(
    isNumeric ? parseFloat(displayValue).toFixed(1) : displayValue,
  );
  const hasUnit = $derived(unit && unit.trim() !== "");
  const unitWithSpace = $derived(hasUnit ? ` ${unit}` : "");
</script>

<article class="data-card" style:--status-color={statusColor} {...rest}>
  <div class="card-content">
    <div class="info">
      <small>{label}</small>
      <h3>{formattedValue}{unitWithSpace}</h3>
    </div>
    <div class="icon-wrapper">
      <div class="dot" style:display={dot ? "block" : "none"}></div>
    </div>
  </div>
</article>

<style>
  .data-card {
    padding: var(--pico-block-spacing-vertical)
      var(--pico-block-spacing-horizontal);

    border-left: 4px solid var(--status-color);
    margin-bottom: 0;
    background: var(--pico-card-background-color);

    display: flex;
    flex-direction: column;
    justify-content: center;
    /* max-width: var(--card-max-width); */
  }

  .card-content {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 1.5rem; /* Increased to match Switch breathing room */
    width: 100%;
  }

  .info small {
    color: var(--pico-muted-color);
    text-transform: uppercase;
    font-size: 0.75rem;
    display: block;
  }

  .info h3 {
    margin: 0.25rem 0 0 0;
    text-transform: capitalize;
    font-size: 1.1rem;
    line-height: 1.2;
    white-space: nowrap; /* Prevent wrapping like in Switch */
  }

  .icon-wrapper {
    width: 40px;
    height: 40px;
    display: flex;
    align-items: center;
    justify-content: flex-end; /* Align dot to the right */
  }

  .dot {
    width: 16px;
    height: 16px;
    border-radius: 50%;
    background-color: var(--status-color);
    animation: pulse-glow 2s infinite alternate;
  }

  @keyframes pulse-glow {
    0% {
      opacity: 1;
      box-shadow: 0 0 1px var(--status-color);
    }
    50% {
      opacity: 0.85;
      box-shadow: 0 0 8px var(--status-color);
    }
    100% {
      opacity: 1;
      box-shadow: 0 0 10px var(--status-color);
    }
  }
</style>
