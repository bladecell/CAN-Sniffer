<script>
  import { alertStore } from "$lib/alertStore.svelte";
  import { fly } from "svelte/transition";
  import { flip } from "svelte/animate";
  import Icon from "$lib/Icon.svelte"; // Adjust this path to where your Icon component is saved

  // Maps the alert type to the keys in your iconData
  const getIconName = (type) => {
    switch (type) {
      case "success":
        return "circle-check";
      case "error":
        return "circle-xmark";
      case "warning":
        return "circle-exclamantion"; // Using your exact spelling from iconData
      case "info":
        return "circle-info";
      default:
        return "circle-info";
    }
  };
</script>

<div class="toast-container">
  {#each alertStore.alerts as alert (alert.id)}
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
    <div
      class="toast blur-background"
      class:success={alert.type === "success"}
      class:error={alert.type === "error"}
      class:warning={alert.type === "warning"}
      class:info={alert.type === "info"}
      transition:fly={{ y: 20, duration: 300 }}
      animate:flip
      onclick={() => alertStore.remove(alert.id)}
      role="alert"
    >
      <div class="toast-content">
        <Icon name={getIconName(alert.type)} size={20} class="toast-icon" />
        <span>{alert.message}</span>
      </div>
    </div>
  {/each}
</div>

<style>
  .toast-container {
    position: fixed;
    bottom: 1.5rem;
    right: 1.5rem;
    z-index: 9999;
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    gap: 0.75rem;
    pointer-events: none;
  }

  .toast {
    pointer-events: auto;
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

  /* Flexbox to align icon and text */
  .toast-content {
    display: flex;
    align-items: center;
    gap: 0.75rem;
  }

  /* TYPE VARIANTS */
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
  .warning {
    --color: #f59e0b;
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
      bottom: auto; /* Required to let top take over on mobile */
      top: 1.5rem;
      right: 1.5rem;
    }
  }
</style>
