<script>
    import { alertStore } from "$lib/alertStore.svelte.js";
    import { fly } from "svelte/transition";
    import { flip } from "svelte/animate";
</script>

<div class="toast-container">
    {#each alertStore.alerts as alert (alert.id)}
        <div
            class="toast"
            class:success={alert.type === "success"}
            class:error={alert.type === "error"}
            class:info={alert.type === "info"}
            transition:fly={{ y: 20, duration: 300 }}
            animate:flip
            onclick={() => alertStore.remove(alert.id)}
            role="alert"
        >
            {alert.message}
        </div>
    {/each}
</div>

<style>
    .toast-container {
        position: fixed;
        bottom: 1.5rem;
        right: 1.5rem;
        z-index: 9999; /* Always on top */
        display: flex;
        flex-direction: column;
        gap: 0.75rem;
        pointer-events: none; /* Let clicks pass through the empty container area */
    }

    .toast {
        pointer-events: auto; /* Re-enable clicks for the toasts themselves */
        min-width: 250px;
        max-width: 350px;
        padding: 0.75rem 1.25rem;
        border-radius: 6px;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        background: var(--pico-card-background-color);
        color: var(--pico-color);
        border-left: 5px solid var(--pico-primary);
        cursor: pointer;
        font-size: 0.9rem;
    }

    /* TYPE VARIANTS using Pico colors */
    .success {
        border-color: var(--pico-ins-color);
    }
    .error {
        border-color: var(--pico-del-color);
    }
    .info {
        border-color: var(--pico-primary);
    }
</style>
