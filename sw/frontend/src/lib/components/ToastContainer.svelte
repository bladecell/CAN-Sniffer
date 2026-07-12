<script>
  import { alertStore } from "$lib/alertStore.svelte";
  import { fly } from "svelte/transition";
  import { flip } from "svelte/animate";
</script>

<div class="toast-container">
  {#each alertStore.alerts as alert (alert.id)}
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
    <div
      class="toast blur-background"
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
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
    --color: #ffffff;
    background-color: color-mix(
      in srgb,
      var(--color) 1%,
      transparent
    ) !important;
    border: none;
    color: var(--color);
    cursor: pointer;
    font-size: 0.9rem;
  }

  /* TYPE VARIANTS using Pico colors */
  .success {
    --color: var(--pico-ins-color);
    background-color: color-mix(
      in srgb,
      var(--color) 10%,
      transparent
    ) !important;
  }
  .error {
    --color: var(--pico-del-color);
    background-color: color-mix(
      in srgb,
      var(--color) 10%,
      transparent
    ) !important;
  }
  .info {
    --color: var(--pico-primary);
    background-color: color-mix(
      in srgb,
      var(--color) 10%,
      transparent
    ) !important;
  }

  @media (max-width: 768px) {
    .toast-container {
      position: fixed;
      top: 1.5rem;
      right: 1.5rem;
    }
  }
</style>
