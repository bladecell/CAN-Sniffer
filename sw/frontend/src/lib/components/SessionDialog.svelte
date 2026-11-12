<script lang="ts">
  import { connectionStore } from "$lib/connectionStore.svelte";
  import Dialog from "./Dialog.svelte";

  // The device serves one session. `inUse`: this page was refused. `replaced`: another page took over.
  const session = $derived(connectionStore.session);
  const asking = $derived(session === "inUse" || session === "replaced");
  const replaced = $derived(session === "replaced");
</script>

<Dialog
  open={asking}
  title={replaced ? "Session moved" : "Device in use"}
  message={replaced
    ? "The device is now connected from another tab or device. Live data has stopped here."
    : "Another tab or device is connected to this device. Taking over disconnects it."}
  confirmLabel={replaced ? "Use here" : "Take over"}
  cancelLabel={replaced ? "Close" : "Not now"}
  onConfirm={() => connectionStore.takeOver()}
  onCancel={() => connectionStore.parkSession()}
/>

{#if session === "parked"}
  <aside class="session-parked" role="status">
    <p><strong>Not connected.</strong> The device is in use from another tab or device.</p>
    <button type="button" class="secondary" onclick={() => connectionStore.takeOver()}>Take over</button>
  </aside>
{/if}

<style>
  .session-parked {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 0.75rem;
    margin: 0 0 1rem;
    padding: 0.75rem 1rem;
    border-radius: var(--pico-border-radius);
    border: 1px solid var(--warning-color, #f59e0b);
    background: var(--pico-background-color);
  }
  .session-parked p {
    margin: 0;
  }
  .session-parked button {
    min-height: 44px;
    width: auto;
    margin: 0;
  }
</style>
