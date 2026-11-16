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
  <form id="name-form" class="db-form" onsubmit={save}>
    <label class="db-field">
      <span class="db-label">Name</span>
      <input class="in" type="text" maxlength="40" autocomplete="off" bind:value={name} />
    </label>
    {#if message}
      <p class="db-error" role="alert">{message}</p>
    {/if}
  </form>
  {#snippet footer()}
    <button type="button" class="btn" onclick={onClose}>Cancel</button>
    <button type="submit" form="name-form" class="btn pri">Save</button>
  {/snippet}
</Sheet>

<style>
  .db-form {
    display: grid;
    gap: 16px;
    padding: 4px 0 8px;
  }
</style>
