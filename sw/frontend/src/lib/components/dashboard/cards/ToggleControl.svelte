<script lang="ts">
  import type { Card } from "$lib/dashboardModel";

  interface Props {
    card: Card;
    name: string;
    /** Word under the name: On, Off, Recording, Stopping, Error, No data. */
    word: string;
    on: boolean;
    /** Disabled when set: the one-line reason. */
    reason: string | null;
    /** A second line for a warning or an error from the device. */
    note?: string | null;
    onToggle: () => void;
  }

  let { card, name, word, on, reason, note = null, onToggle }: Props = $props();
  const disabled = $derived(reason !== null);
  const label = $derived(card.style === "button" ? (on ? `Stop ${name.toLowerCase()}` : `Start ${name.toLowerCase()}`) : name);
</script>

<div class="t" class:disabled>
  <div class="top">
    <span class="nm">{name}</span>
    {#if card.style === "switch"}
      <button type="button" role="switch" class="sw" aria-checked={on} aria-label={name} aria-describedby="{card.id}-reason" {disabled} onclick={onToggle}>
        <span class="knob"></span>
      </button>
    {:else}
      <button type="button" class="btn" aria-pressed={on} aria-label={label} aria-describedby="{card.id}-reason" {disabled} onclick={onToggle}>
        <svg viewBox="0 0 10 10" width="10" height="10" aria-hidden="true"><rect width="10" height="10" rx="1.5" /></svg>
        {on ? "Stop" : "Start"}
      </button>
    {/if}
  </div>
  <span class="wd">{word}{#if note && !reason} <span class="nt">{note}</span>{/if}</span>
  <span class="why" id="{card.id}-reason">{reason ?? ""}</span>
</div>

<style>
  .t {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 2px;
    height: 100%;
    min-width: 0;
    padding: 8px 12px 10px;
    color: var(--text);
  }
  .top {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    min-width: 0;
  }
  .nm {
    flex: 1;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-size: 0.95rem;
    font-weight: 800;
  }
  .wd {
    font-size: 0.95rem;
    font-weight: 800;
  }
  .nt {
    color: var(--muted);
    font-size: 0.78rem;
    font-weight: 600;
  }
  .t.disabled .nm,
  .t.disabled .wd {
    color: var(--muted);
  }
  .why {
    min-height: 1.2em;
    overflow: hidden;
    color: var(--muted);
    font-size: 0.78rem;
    font-weight: 600;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .sw {
    position: relative;
    flex: none;
    width: 64px;
    min-width: 44px;
    height: 44px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: 99px;
    background: var(--ctl);
    cursor: pointer;
  }
  .sw[aria-checked="true"] {
    background: var(--ok);
  }
  .knob {
    position: absolute;
    top: 6px;
    left: 6px;
    width: 32px;
    height: 32px;
    border-radius: 50%;
    background: var(--float-solid);
    transition: transform 0.2s;
  }
  .sw[aria-checked="true"] .knob {
    transform: translateX(20px);
  }
  .btn {
    display: inline-flex;
    flex: none;
    align-items: center;
    gap: 6px;
    min-width: 44px;
    min-height: 44px;
    padding: 0 14px;
    border: 0;
    border-radius: 14px;
    background: var(--ctl);
    color: var(--ctl-text);
    font: 800 0.95rem/1 var(--font);
    cursor: pointer;
  }
  .btn svg {
    fill: currentColor;
  }
  .btn[aria-pressed="true"] {
    background: var(--ok);
    color: var(--float-solid);
  }
  .sw:disabled,
  .btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
  .sw:focus-visible,
  .btn:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
</style>
