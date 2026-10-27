<script lang="ts">
  import { telemetryStore } from "$lib/telemetryStore.svelte";

  /** Pinned sheet footer. Save submits the editor form by id, so it works from outside the form. */
  interface Props {
    formId: string;
  }

  let { formId }: Props = $props();
</script>

<div class="acts">
  <button type="button" class="btn" onclick={() => telemetryStore.clearForm()}>Clear form</button>
  <button type="submit" form={formId} class="btn pri" disabled={!telemetryStore.isFormValid} aria-disabled={!telemetryStore.isFormValid}>
    {telemetryStore.activeIndex === -1 ? "Add PID definition" : "Save changes"}
  </button>
</div>

<style>
  .acts {
    display: flex;
    flex: 1 1 auto;
    gap: 10px;
    justify-content: flex-end;
    min-width: 0;
  }
  .acts .btn {
    flex: 1 1 0;
    min-width: 0;
    min-height: 44px;
    padding: 0 12px;
  }
  @media (hover: hover) and (min-width: 600px) {
    .acts .btn {
      flex: 0 1 auto;
      min-height: 40px;
    }
  }
  .btn {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    box-sizing: border-box;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: 700 1rem/1 var(--font);
    cursor: pointer;
  }
  .btn:hover {
    background: var(--ctl-h2);
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
</style>
