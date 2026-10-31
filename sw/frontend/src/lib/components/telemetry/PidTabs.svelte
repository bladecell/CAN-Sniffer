<script module lang="ts">
  export type PidPanel = "pids" | "changes" | "sd" | "settings";
</script>

<script lang="ts">
  import { telemetryStore } from "$lib/telemetryStore.svelte";
  import Icon from "$lib/Icon.svelte";

  interface Props {
    /** rail: vertical icon rail beside the panel (desktop). bar: segmented switch in the phone sheet's header. */
    variant: "rail" | "bar";
    panel?: PidPanel;
  }

  let { variant, panel = $bindable("pids") }: Props = $props();

  let pendingCount = $derived(telemetryStore.local_piddef.filter((r) => r.pendingChanges === "Yes").length);

  const panels: { id: PidPanel; label: string; icon: string }[] = $derived([
    { id: "pids", label: "PIDs", icon: "list" },
    { id: "changes", label: pendingCount > 0 ? `Changes, ${pendingCount} pending` : "Changes", icon: "code-compare" },
    { id: "sd", label: "SD card", icon: "sd-card" },
    { id: "settings", label: "Settings", icon: "sliders" },
  ]);
</script>

<div class="switch {variant}" aria-label="PID panels">
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

<style>
  /* Rail (desktop): a 56px card of icon buttons, the selected one with a 3px accent bar on its left. */
  .switch.rail {
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
  .switch.rail button {
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
  .switch.rail button[aria-pressed='true'] {
    color: var(--accent-text);
  }
  .switch.rail button[aria-pressed='true']::before {
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
    .switch.rail button:hover {
      background: var(--hov);
    }
  }
  /* Phone: a segmented switch with icon over label, in the sheet's header layer (the mockup's .pswitch). */
  .switch.bar {
    display: flex;
    flex: none;
    gap: 4px;
    padding: 3px;
    border-radius: 18px;
    background: var(--ctl);
  }
  .switch.bar button {
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
  .switch.bar button[aria-pressed='true'] {
    background: var(--float-solid);
    color: var(--text);
    font-weight: 800;
  }
  .switch.bar button[aria-pressed='true']::after {
    content: '';
    position: absolute;
    left: 24%;
    right: 24%;
    bottom: 3px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
  }
  .badge {
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
  /* The count sits on the icon's top right corner (the mockup), not in the label line. */
  .switch.bar .badge {
    top: 4px;
    left: calc(50% + 4px);
    right: auto;
    margin: 0;
  }
</style>
