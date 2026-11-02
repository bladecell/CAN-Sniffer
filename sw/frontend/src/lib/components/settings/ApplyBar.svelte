<script lang="ts">
  import Icon from "$lib/Icon.svelte";

  interface Props {
    count: number;
    busy: boolean;
    /** Per-field errors block Apply. */
    blocked: boolean;
    onRevert: () => void;
    onApply: () => void;
  }

  let { count, busy, blocked, onRevert, onApply }: Props = $props();
</script>

{#if count > 0}
  <div class="st-apply st-float" role="status">
    <span><b>{count}</b> unsaved {count === 1 ? "change" : "changes"}</span>
    <div class="actions">
      <button type="button" class="st-btn" onclick={onRevert} disabled={busy}>Revert</button>
      <button type="button" class="st-btn pri" onclick={onApply} disabled={busy || blocked}>
        {#if busy}<Icon name="reload" size={16} />{/if}Apply
      </button>
    </div>
  </div>
{/if}

<style>
  .actions {
    display: flex;
    gap: 10px;
  }
</style>
