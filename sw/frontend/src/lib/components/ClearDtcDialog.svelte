<script lang="ts">
  import { onMount } from "svelte";

  let { onconfirm, oncancel }: { onconfirm: () => void; oncancel: () => void } = $props();

  let cancelButton: HTMLButtonElement;
  let dialog: HTMLDivElement;

  onMount(() => {
    const opener = document.activeElement as HTMLElement | null;
    // Focus the safe choice, never the destructive one.
    cancelButton.focus();
    return () => opener?.focus?.();
  });

  function onkeydown(e: KeyboardEvent) {
    if (e.key === "Escape") {
      e.preventDefault();
      oncancel();
    } else if (e.key === "Tab") {
      const items = dialog.querySelectorAll<HTMLElement>("button");
      const first = items[0];
      const last = items[items.length - 1];
      if (e.shiftKey && document.activeElement === first) {
        e.preventDefault();
        last.focus();
      } else if (!e.shiftKey && document.activeElement === last) {
        e.preventDefault();
        first.focus();
      }
    }
  }
</script>

<div class="backdrop" role="presentation" onclick={oncancel}>
  <div
    bind:this={dialog}
    class="dialog"
    role="dialog"
    aria-modal="true"
    aria-labelledby="clear-dtc-title"
    aria-describedby="clear-dtc-desc"
    tabindex="-1"
    {onkeydown}
    onclick={(e) => e.stopPropagation()}
  >
    <h3 id="clear-dtc-title">Clear trouble codes?</h3>
    <p id="clear-dtc-desc">
      This clears the stored codes on the vehicle itself, not only in this
      app. It also resets readiness monitors. Confirm within 10 seconds.
    </p>
    <div class="actions">
      <button type="button" class="btn" bind:this={cancelButton} onclick={oncancel}>Cancel</button>
      <button type="button" class="btn btn-danger" onclick={onconfirm}>Clear codes</button>
    </div>
  </div>
</div>

<style>
  .backdrop {
    position: fixed;
    inset: 0;
    z-index: 1000;
    display: grid;
    place-items: center;
    background: rgba(0, 0, 0, 0.6);
  }
  .dialog {
    width: min(92vw, 420px);
    padding: 1.5rem;
    border-radius: 0.5rem;
    background: var(--pico-card-background-color, #1b1b1f);
    border: 1px solid var(--pico-muted-border-color);
  }
  .dialog h3 {
    margin: 0 0 0.75rem;
  }
  .actions {
    display: flex;
    gap: 1rem;
    justify-content: flex-end;
  }
  .btn-danger {
    --color: 231, 75, 26;
  }
</style>
