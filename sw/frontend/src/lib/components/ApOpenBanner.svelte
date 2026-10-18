<script lang="ts">
  import { auth } from "$lib/auth.svelte";

  // Dismissed for this page load only; it returns on the next visit while the AP stays open.
  let dismissed = $state(false);
</script>

{#if auth.view.apOpen && !dismissed}
  <aside class="ap-open" aria-labelledby="ap-open-title">
    <p id="ap-open-title">
      <strong>The Wi-Fi access point is open.</strong> Anyone in range can see the traffic and the PIN.
    </p>
    <button type="button" class="secondary" onclick={() => (dismissed = true)}>Dismiss</button>
  </aside>
{/if}

<style>
  .ap-open {
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
  .ap-open p {
    margin: 0;
  }
  .ap-open button {
    min-height: 44px;
    width: auto;
    margin: 0;
  }
</style>
