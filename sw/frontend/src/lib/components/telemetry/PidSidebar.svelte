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
  /* Phone sheet: the status strip is normal flow above the scroller. No background, border, radius, blur or shadow:
     it is not a layer. Counts and the info button share the first row; the selection row and its actions the second. */
  .sb.bar :global(.sb-top) {
    position: relative;
    z-index: auto;
    inset: auto;
    display: grid;
    /* Two fixed 44 px rows, whatever the tab shows, so the block has the same height on every tab. */
    grid-template-columns: minmax(0, 1fr) auto;
    grid-template-rows: 44px 44px;
    align-items: center;
    column-gap: 8px;
    margin: 0;
    padding: 0 0 8px;
    border-radius: 0;
    background: none;
    box-shadow: none;
    -webkit-backdrop-filter: none;
    backdrop-filter: none;
  }
  .sb.bar :global(.sb-counts) {
    grid-column: 1;
    grid-row: 1;
    display: flex;
    align-items: center;
    min-height: 44px;
  }
  .sb.bar :global(.sb-top .hint) {
    position: static;
    grid-column: 2;
    grid-row: 1;
  }
  .sb.bar :global(.sb-selrow) {
    grid-column: 1 / -1;
    grid-row: 2;
    min-height: 44px;
  }
  /* Tabs without a selection row (Changes with nothing pending, Settings) keep the second row's height. */
  /* The scroller starts below the strip, so its first row is 8px under it. */
  .sb.bar :global(.sb-body) {
    padding-top: 8px;
  }
  /* Phone sheet action bar (New PID / Remove): one layer that runs to the sheet's edges. Its fill is nearly opaque and
     its blur is full strength, so rows scrolling beneath it cannot be read through it. The offsets take the panel's
     22 px inset to the sheet edge; the buttons keep their old place (36 px sides, 22 px from the bottom). */
  .sb.bar :global(.sb-foot) {
    left: -22px;
    right: -22px;
    bottom: 0;
    padding: 10px 36px 22px;
  }
  /* The bar's blur is a ::before behind its buttons: a blurred bar would drop the buttons' own blur (nested glass). */
  .sb.bar :global(.sb-foot)::before {
    content: "";
    position: absolute;
    inset: 0;
    z-index: -1;
    background: color-mix(in srgb, var(--float-solid) 92%, transparent);
    -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
    backdrop-filter: blur(var(--blur)) saturate(1.35);
  }
  @supports not ((backdrop-filter: blur(1px)) or (-webkit-backdrop-filter: blur(1px))) {
    .sb.bar :global(.sb-foot)::before {
      background: var(--float-solid);
    }
  }
  /* The phone sheet already shows the tab name as its title. */
  .sb.bar :global(.sb-top h2) {
    display: none;
  }
</style>
