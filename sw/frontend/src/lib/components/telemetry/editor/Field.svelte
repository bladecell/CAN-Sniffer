<script lang="ts">
  import type { Snippet } from "svelte";

  /** One form field: label row, control and message row with fixed heights, so every row lines up. */
  interface Props {
    id: string;
    label: string;
    hint: string;
    error?: string;
    children: Snippet<[labelId: string, msgId: string]>;
  }

  let { id, label, hint, error, children }: Props = $props();
  const labelId = $derived(`${id}-l`);
  const msgId = $derived(`${id}-m`);
</script>

<div class="fld" class:bad={!!error} data-field={id}>
  <span class="lab" id={labelId}>{label}</span>
  <div class="ctl">
    {@render children(labelId, msgId)}
  </div>
  <p class="msg" id={msgId}>{error ?? hint}</p>
</div>

<style>
  .fld {
    display: grid;
    grid-template-rows: auto auto auto;
    align-content: start;
    min-width: 0;
    scroll-margin-bottom: 140px;
  }
  .lab {
    display: block;
    height: 24px;
    padding-bottom: 3px;
    font-size: 0.82rem;
    font-weight: 700;
    line-height: 1.3;
  }
  /* Pico pads inputs and overlay.css adds a 14px bottom margin to them. Cancel both so every control is --ctl-h tall. */
  .ctl {
    min-width: 0;
  }
  .ctl > :global(*),
  .ctl :global(input),
  .ctl :global(button) {
    margin: 0 !important;
  }
  .ctl :global(input) {
    height: var(--ctl-h) !important;
    padding-top: 0 !important;
    padding-bottom: 0 !important;
  }
  .msg {
    min-height: calc(0.78rem * 1.3 + 4px);
    margin: 0;
    padding-top: 4px;
    color: var(--muted);
    font-size: 0.78rem;
    line-height: 1.3;
    overflow-wrap: anywhere;
  }
  .fld.bad .msg {
    color: var(--crit);
    font-weight: 700;
  }
</style>
