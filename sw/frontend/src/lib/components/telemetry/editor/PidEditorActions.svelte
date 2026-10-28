<script lang="ts">
  import { telemetryStore } from "$lib/telemetryStore.svelte";

  /** Pinned sheet footer (mockup: status at left, Clear and Save PID at right). Save submits the form by id. */
  interface Props {
    formId: string;
  }

  let { formId }: Props = $props();
</script>

<span class="st" aria-live="polite">{telemetryStore.activeIndex === -1 ? "New PID, not saved" : "Changes not saved yet"}</span>
<button type="button" class="btn" onclick={() => telemetryStore.clearForm()}>Clear</button>
<button type="submit" form={formId} class="btn pri">Save PID</button>

<style>
  /* Status: a small blurred layer that fits its text, so content under it stays readable (mockup). */
  .st {
    flex: 0 1 auto;
    min-width: 0;
    display: inline-flex;
    align-items: center;
    box-sizing: border-box;
    min-height: 48px;
    padding: 0 16px;
    line-height: 1;
    border-radius: 14px;
    font-size: 0.875rem;
    color: var(--muted);
    background: var(--float);
    -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
    box-shadow: var(--shadow-float);
  }
  .btn {
    flex: none;
    min-height: 48px;
    padding: 0 24px;
    border: 0;
    border-radius: 14px;
    background: var(--ctl);
    color: var(--ctl-text);
    font: 750 1rem/1 var(--font);
    cursor: pointer;
  }
  .btn.pri {
    background: var(--accent);
    color: var(--on-accent);
    font-weight: 800;
  }
  .btn:disabled {
    opacity: 0.55;
    cursor: not-allowed;
  }
  /* A disabled primary stays solid: a translucent button would show the fields under it. */
  .btn.pri:disabled {
    opacity: 1;
    background: var(--ctl);
    color: var(--ctl-sub);
  }
  /* Phone: the status line goes and the two buttons share the width. */
  @media (max-width: 640px) {
    .st {
      display: none;
    }
    .btn {
      flex: 1 1 0;
      padding: 0 12px;
    }
  }
</style>
