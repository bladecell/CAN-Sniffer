<script module lang="ts">
  export type PidPanel = "pids" | "changes" | "sd" | "settings";
</script>

<script lang="ts">
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import PidList from "./PidList.svelte";
  import PidChanges from "./PidChanges.svelte";
  import PidSdTab from "./PidSdTab.svelte";
  import PidSettingsTab from "./PidSettingsTab.svelte";

  interface Props {
    /** rail: vertical icon rail beside the panel (desktop). bar: segmented switch on top (phone sheet). */
    variant: "rail" | "bar";
    onNew: () => void;
    onEdit: (index: number) => void;
    /** The active tab. The phone sheet titles itself with it. */
    panel?: PidPanel;
  }

  let { variant, onNew, onEdit, panel = $bindable("pids") }: Props = $props();

  let pendingCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);

  const panels: { id: PidPanel; label: string; icon: string }[] = $derived([
    { id: "pids", label: "PIDs", icon: "list" },
    { id: "changes", label: pendingCount > 0 ? `Changes, ${pendingCount} pending` : "Changes", icon: "code-compare" },
    { id: "sd", label: "SD card", icon: "sd-card" },
    { id: "settings", label: "Settings", icon: "sliders" },
  ]);
</script>

<div class="sb" class:rail={variant === "rail"} class:bar={variant === "bar"}>
  <div class="switch" aria-label="PID panels">
    {#each panels as p (p.id)}
      <button
        type="button"
        aria-pressed={panel === p.id}
        aria-label={p.label}
        title={p.label}
        onclick={() => (panel = p.id)}
      >
        <Icon name={p.icon} size={variant === "rail" ? 24 : 22} />
        {#if variant === "bar"}<span class="lbl">{p.id === "sd" ? "SD" : p.label.split(",")[0]}</span>{/if}
        {#if p.id === "changes" && pendingCount > 0}<span class="badge num" aria-hidden="true">{pendingCount}</span>{/if}
      </button>
    {/each}
  </div>

  {#if panel === "pids"}
    <PidList phone={variant === "bar"} {onNew} {onEdit} />
  {:else if panel === "changes"}
    <PidChanges {onEdit} />
  {:else if panel === "sd"}
    <PidSdTab />
  {:else}
    <PidSettingsTab />
  {/if}
</div>

<style>
  .sb {
    display: flex;
    align-items: stretch;
    min-height: 0;
    flex: 1 1 auto;
  }

  /* Rail (desktop): a 56px card of icon buttons, the selected one with a 3px accent bar on its left. */
  .sb.rail {
    gap: 10px;
  }
  .sb.rail .switch {
    display: flex;
    flex: none;
    flex-direction: column;
    gap: 4px;
    width: 56px;
    padding: 6px;
    border-radius: 20px;
    background: var(--card);
    align-self: flex-start;
  }
  .sb.rail .switch button {
    position: relative;
    display: grid;
    place-items: center;
    width: 44px;
    height: 48px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: 14px;
    background: none;
    color: var(--ctl-sub);
    cursor: pointer;
  }
  .sb.rail .switch button[aria-pressed='true'] {
    color: var(--accent-text);
  }
  .sb.rail .switch button[aria-pressed='true']::before {
    content: '';
    position: absolute;
    left: -6px;
    top: 22%;
    bottom: 22%;
    width: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  @media (hover: hover) {
    .sb.rail .switch button:hover {
      background: var(--hov);
    }
  }
  /* Phone: a segmented switch with icon over label, inside the sheet. The tab is not a card, and its header and footer
     reach 12px past the tab so they span the sheet body as before. The list keeps a 2px inset. */
  .sb.bar {
    flex-direction: column;
    gap: 8px;
  }
  .sb.bar :global(.sb-panel) {
    background: none;
    border-radius: 0;
  }
  .sb.bar :global(.sb-body) {
    padding-left: 2px;
    padding-right: 2px;
    padding-bottom: var(--sb-foot, 0px);
    border-radius: 0;
  }
  .sb.bar :global(.sb-top),
  .sb.bar :global(.sb-foot) {
    left: -12px;
    right: -12px;
  }
  .sb.bar :global(.sb-foot) {
    bottom: 0;
  }
  .sb.bar :global(.sb-top) {
    border-radius: 0;
  }
  .sb.bar .switch {
    display: flex;
    flex: none;
    gap: 4px;
    padding: 3px;
    border-radius: 18px;
    background: var(--ctl);
  }
  .sb.bar .switch button {
    position: relative;
    flex: 1;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 2px;
    min-height: 56px;
    padding: 0 2px;
    border: 0;
    border-radius: 15px;
    background: none;
    color: var(--ctl-sub);
    font: 700 0.78rem/1.2 var(--font);
    cursor: pointer;
  }
  .sb.bar .switch button[aria-pressed='true'] {
    background: var(--float-solid);
    color: var(--text);
    font-weight: 800;
  }
  .sb.bar .switch button[aria-pressed='true']::after {
    content: '';
    position: absolute;
    left: 24%;
    right: 24%;
    bottom: 3px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  @media (pointer: coarse) {
    .sb.bar .switch button {
      min-height: 56px;
    }
  }

  /* The phone sheet already shows the tab name as its title. */
  .sb.bar :global(.sb-top h2) {
    display: none;
  }
  .switch .badge {
    position: absolute;
    top: 2px;
    right: 0;
    display: grid;
    place-items: center;
    min-width: 18px;
    height: 18px;
    padding: 0 5px;
    border-radius: 9px;
    background: var(--accent);
    color: var(--on-accent);
    font-size: 0.7rem;
    font-weight: 800;
    line-height: 1;
  }
  .sb.bar .switch .badge {
    position: static;
    margin-left: 2px;
  }

</style>
