<script>
    // Define the props the parent can pass in
    let {
        checked = $bindable(false), // $bindable allows the parent to bind:checked
        label = "SETTING", // The small top text
        statusText = "", // The large H3 text (e.g. "Running")
        onchange = () => {}, // Function to run when clicked
        class: className = "", // Allow custom classes from parent
        ...rest
    } = $props();

    // If statusText is not provided, generate a default based on state
    const derivedStatusText = $derived(
        statusText || (checked ? "Enabled" : "Disabled"),
    );

    // Dynamic Color: Green if checked, Red if unchecked
    const statusColor = $derived(
        checked ? "var(--pico-ins-color)" : "var(--pico-del-color)",
    );

    function handleChange(e) {
        // Update the bound value
        checked = e.target.checked;
        // Trigger the custom action passed from parent
        onchange(checked);
    }
</script>

<article
    class="switch-card {className}"
    style:--status-color={statusColor}
    {...rest}
>
    <div class="card-content">
        <div class="info">
            <small>{label}</small>
            <h3>{derivedStatusText}</h3>
        </div>

        <input
            type="checkbox"
            role="switch"
            {checked}
            onchange={handleChange}
        />
    </div>
</article>

<style>
    .switch-card {
        height: 100%;
        width: 100%;
        padding: 0.5rem 1rem;
        /* Dynamic left border color */
        border-left: 4px solid var(--status-color);
        margin-bottom: 0;
        background: var(--pico-card-background-color);
        display: flex;
        flex-direction: column;
        justify-content: center;
    }

    .card-content {
        display: flex;
        justify-content: space-between;
        align-items: center;
        gap: 1rem;
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
        white-space: nowrap;
        overflow: hidden;
        text-overflow: ellipsis;
    }
</style>
