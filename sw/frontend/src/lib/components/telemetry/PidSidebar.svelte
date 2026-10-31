<script lang="ts">
  import PidList from "./PidList.svelte";
  import PidChanges from "./PidChanges.svelte";
  import PidSdTab from "./PidSdTab.svelte";
  import PidSettingsTab from "./PidSettingsTab.svelte";
  import PidTabs, { type PidPanel } from "./PidTabs.svelte";

  interface Props {
    /** rail: the icon rail sits beside the panel (desktop). bar: the phone sheet, whose header holds PidTabs. */
    variant: "rail" | "bar";
    onNew: () => void;
    onEdit: (index: number) => void;
    /** The active tab. */
    panel?: PidPanel;
  }

  let { variant, onNew, onEdit, panel = $bindable("pids") }: Props = $props();
</script>

<div class="sb" class:rail={variant === "rail"} class:bar={variant === "bar"}>
  {#if variant === "rail"}
    <PidTabs variant="rail" bind:panel />
  {/if}

  {#if panel === "pids"}
    <PidList phone={variant === "bar"} {onNew} {onEdit} />
  {:else if panel === "changes"}
    <PidChanges phone={variant === "bar"} />
  {:else if panel === "sd"}
    <PidSdTab phone={variant === "bar"} onSettings={() => (panel = "settings")} />
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
  .sb.rail {
    gap: 10px;
  }
  .sb.bar {
    flex-direction: column;
  }
  /* The mockup's phone frame: the header, tabs and cards sit on the sheet's 12px inset (the cards 4px in, as PidList). */
  .sb.bar :global(.sb-panel) {
    background: none;
    border-radius: 0;
  }
  .sb.bar :global(.sb-body) {
    padding-left: 0;
    padding-right: 0;
    border-radius: 0;
  }
  .sb.bar :global(.sb-top),
  .sb.bar :global(.sb-foot) {
    left: 0;
    right: 0;
  }
  /* 8px under the tabs, as the mockup's counts row, then the counts, the selection row and 6px. */
  .sb.bar :global(.sb-top) {
    padding: 8px 0 6px;
    border-radius: 0;
  }
  /* The phone sheet already shows the tab name as its title. */
  .sb.bar :global(.sb-top h2) {
    display: none;
  }
</style>
