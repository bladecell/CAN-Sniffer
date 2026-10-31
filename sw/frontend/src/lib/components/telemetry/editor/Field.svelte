<script lang="ts">
  import type { Snippet } from "svelte";
  import SidebarHint from "../SidebarHint.svelte";

  /** One form field: label row, control and message row that shows only an error. */
  interface Props {
    id: string;
    label: string;
    /** Help note, shown in the info button next to the name. */
    hint?: string;
    error?: string;
    children: Snippet<[labelId: string, msgId: string]>;
  }

  let { id, label, hint, error, children }: Props = $props();
  const labelId = $derived(`${id}-l`);
  const msgId = $derived(`${id}-m`);
</script>

<div class="fld" class:bad={!!error} data-field={id}>
  <div class="labrow">
    <span class="lab" id={labelId}>{label}</span>
    {#if hint}<SidebarHint inline text={hint} label="About {label}" />{/if}
  </div>
  <div class="ctl">
    {@render children(labelId, msgId)}
  </div>
  <p class="msg" id={msgId}>{error ?? ""}</p>
</div>

<style>
  .fld {
    display: grid;
    grid-template-rows: auto auto auto;
    align-content: start;
    min-width: 0;
    scroll-margin-bottom: 140px;
  }
  .labrow {
    display: flex;
    align-items: center;
    gap: 2px;
    height: 24px;
    margin-bottom: 4px;
  }
  .lab {
    display: block;
    font-size: 0.875rem;
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
    min-height: 22px;
    margin: 0;
    padding-top: 6px;
    color: var(--muted);
    font-size: 0.78rem;
    line-height: 1.3;
    overflow-wrap: anywhere;
  }
  .msg:empty {
    min-height: 0;
    padding-top: 0;
  }
  .fld.bad .msg {
    color: var(--crit);
    font-weight: 700;
  }
</style>
