<script>
    import Icon from "./Icon.svelte";

    let {
        label = "Metric",
        value = 0,
        unit = "%",
        icon = "gear",
        color = "#10b981",
        min = 0,
        max = 100,
        supported = true,
        valid = true,
    } = $props();

    // Reactive status calculation
    const status = $derived(
        !supported
            ? "#6b7280"
            : !valid
              ? "#ef4444"
              : ((value - min) / (max - min)) * 100 >= 20 &&
                  ((value - min) / (max - min)) * 100 <= 80
                ? "#10b981"
                : "#f59e0b",
    );

    const displayValue = $derived(supported ? value : "···");
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
        <div class="status" style="background: {status};"></div>
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
    .pid-card {
        border: 1px solid var(--pico-muted-border-color);
        border-radius: 12px;
        padding: 16px;
        min-width: 100px;
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
        animation: pulse 2s infinite;
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

    @keyframes pulse {
        0%,
        100% {
            opacity: 1;
        }
        50% {
            opacity: 0.5;
        }
    }
</style>
