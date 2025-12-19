<script>
    import Icon from "./Icon.svelte";

    let {
        label = "Metric",
        value = 0,
        unit = "%",
        icon = "gear",
        status = "normal", // normal, warning, critical, inactive
        color = "#10b981",
    } = $props();

    const statusColors = {
        normal: "#10b981", // green
        warning: "#f59e0b", // amber
        critical: "#ef4444", // red
        inactive: "#6b7280", // gray
    };
</script>

<article
    class="pid-card"
    style="background: color-mix(in srgb, {color} 5%, transparent);"
>
    <div class="card-header">
        <div
            class="icon"
            style="background: color-mix(in srgb, {color} 20%, transparent);"
        >
            <Icon name={icon} size={32} />
        </div>
        <div class="status" style="background: {statusColors[status]};"></div>
    </div>

    <div class="card-body">
        <span class="label">{label}</span>
        <div class="value">
            <span class="number">{value}</span>
            <span class="unit">{unit}</span>
        </div>
    </div>
</article>

<style>
    .pid-card {
        /* background: var(--pico-card-background-color); */
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
