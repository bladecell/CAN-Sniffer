<script lang="ts">
  import Sheet from "$lib/components/Sheet.svelte";

  interface Props {
    title: string;
    initial: string;
    /** Returns an error message, or null when the name was accepted. */
    onSave: (name: string) => string | null;
    onClose: () => void;
  }

  let { title, initial, onSave, onClose }: Props = $props();
  // svelte-ignore state_referenced_locally
  let name = $state(initial);
  let message = $state("");

  function save(e: SubmitEvent) {
    e.preventDefault();
    const err = onSave(name);
    if (err) message = err;
    else onClose();
  }
</script>

<Sheet open {title} subtitle="Up to 40 characters." {onClose}>
  <form class="db-form" onsubmit={save}>
    <label class="db-field">
      <span class="db-label">Name</span>
      <input class="db-input" type="text" maxlength="40" autocomplete="off" bind:value={name} />
    </label>
    {#if message}
      <p class="db-error" role="alert">{message}</p>
    {/if}
    <div class="db-actions">
      <button type="button" class="rv-btn" onclick={onClose}>Cancel</button>
      <button type="submit" class="rv-btn pri">Save</button>
    </div>
  </form>
</Sheet>

<style>
  .db-form {
    display: grid;
    gap: 16px;
    padding: 4px 0 8px;
  }
  .db-actions {
    display: flex;
    justify-content: flex-end;
    gap: 10px;
    flex-wrap: wrap;
  }
</style>
