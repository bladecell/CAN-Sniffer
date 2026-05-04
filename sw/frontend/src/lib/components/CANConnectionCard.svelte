<script>
  import { canStore } from "$lib/canStore.svelte.js";
  import { onMount } from "svelte";
  import { on } from "svelte/events";

  let { class: className = "", ...rest } = $props();

  const isConnected = $derived(
    canStore.wsCanStatus?.state === "Connected" ? true : false,
  );
  const statusText = $derived(canStore.wsCanStatus?.state);

  // Pico CSS Semantic Colors
  const statusColor = $derived(
    isConnected ? "var(--pico-ins-color)" : "var(--pico-del-color)",
  );
</script>

<article
  class="can-connection-card {className}"
  style:--status-color={statusColor}
  {...rest}
>
  <div class="card-content">
    <div class="info">
      <small>CAN BUS</small>
      <h3>{statusText}</h3>
    </div>

    <div class="icon-wrapper">
      <div class="dot"></div>
    </div>
  </div>
</article>

<style>
  .can-connection-card {
    padding: var(--pico-block-spacing-vertical)
      var(--pico-block-spacing-horizontal);

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
