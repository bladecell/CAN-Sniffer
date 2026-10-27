<script lang="ts">
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import PidList from "./PidList.svelte";
  import PidChanges from "./PidChanges.svelte";

  interface Props {
    /** rail: vertical activity bar on the left (desktop). bar: segmented switch on top (phone sheet). */
    variant: "rail" | "bar";
    onImport: () => void;
    onNew: () => void;
    onEdit: (index: number) => void;
  }

  let { variant, onImport, onNew, onEdit }: Props = $props();

  type Panel = "pids" | "changes";
  let panel = $state<Panel>("pids");

  let pendingCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);

  const panels: { id: Panel; label: string; icon: string }[] = $derived([
    { id: "pids", label: "PIDs", icon: "list" },
    {
      id: "changes",
      label: pendingCount > 0 ? `Changes, ${pendingCount} pending` : "Changes",
      icon: "code-compare",
    },
  ]);
</script>

<div class="sb" class:rail={variant === "rail"} class:bar={variant === "bar"}>
  <div class="switch" role="group" aria-label="PID panels">
    {#each panels as p (p.id)}
      <button
        type="button"
        aria-pressed={panel === p.id}
        aria-label={p.label}
        title={p.label}
        onclick={() => (panel = p.id)}
      >
        <Icon name={p.icon} size={20} />
        {#if variant === "bar"}<span>{p.id === "pids" ? "PIDs" : "Changes"}</span>{/if}
        {#if p.id === "changes" && pendingCount > 0}<span class="badge num" aria-hidden="true">{pendingCount}</span>{/if}
      </button>
    {/each}
  </div>

  <div class="panel">
    {#if panel === "pids"}
      <PidList {onNew} {onEdit} />
    {:else}
      <PidChanges {onImport} {onEdit} />
    {/if}
  </div>
</div>

<style>
  .sb {
    display: flex;
    min-height: 0;
    flex: 1 1 auto;
  }
  .sb.rail {
    gap: 10px;
  }
  .sb.bar {
    flex-direction: column;
    gap: 12px;
  }
  .panel {
    display: flex;
    flex-direction: column;
    min-width: 0;
    min-height: 0;
    flex: 1 1 auto;
  }
  /* Rail: the panel scrolls, so the sticky header and Changes bar sit over the list. Bar (phone): the sheet scrolls. */
  .sb.rail .panel {
    overflow-y: auto;
    overscroll-behavior: contain;
  }

  .switch {
    display: flex;
    flex: none;
    gap: 2px;
  }
  .switch button {
    position: relative;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    margin: 0;
    padding: 0;
    border: 0;
    background: transparent;
    color: var(--ctl-sub);
    cursor: pointer;
    font: 700 0.95rem/1.2 var(--font);
  }
  .switch button[aria-pressed='true'] {
    color: var(--text);
  }
  .switch button:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 1px;
    border-radius: var(--r-ctl);
  }
  .switch .badge {
    display: inline-grid;
    place-items: center;
    min-width: 18px;
    height: 18px;
    padding: 0 5px;
    border-radius: 9px;
    background: var(--accent);
    color: var(--on-accent);
    font-size: 0.7rem;
    font-weight: 800;
  }

  /* Rail: 44px column of icon buttons. The selected one gets a 3px accent bar on its left edge. */
  .sb.rail .switch {
    flex-direction: column;
    width: 44px;
    padding-top: 6px;
  }
  .sb.rail .switch button {
    width: 44px;
    height: 44px;
    border-radius: var(--r-ctl);
  }
  .sb.rail .switch button[aria-pressed='true'] {
    background: var(--ctl);
  }
  .sb.rail .switch button[aria-pressed='true']::before {
    content: '';
    position: absolute;
    left: -10px;
    top: 10px;
    bottom: 10px;
    width: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .sb.rail .switch .badge {
    position: absolute;
    top: 3px;
    right: 1px;
  }
  @media (hover: hover) {
    .sb.rail .switch button:hover {
      background: var(--hov);
    }
  }

  /* Bar: a segmented control, the same vocabulary as the Live/Recorded switch. */
  .sb.bar .switch {
    display: flex;
    padding: 3px;
    border-radius: calc(var(--r-ctl) + 2px);
    background: var(--ctl);
  }
  .sb.bar .switch button {
    flex: 1;
    min-height: calc(var(--ctl-h) - 8px);
    border-radius: var(--r-ctl);
    background: transparent;
  }
  .sb.bar .switch button[aria-pressed='true'] {
    background: var(--float-solid);
    font-weight: 800;
  }
  @media (pointer: coarse) {
    .sb.bar .switch button {
      min-height: 44px;
    }
  }
</style>
