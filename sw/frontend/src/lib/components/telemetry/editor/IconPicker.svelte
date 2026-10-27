<script lang="ts">
  import Icon from "$lib/Icon.svelte";

  /** Icon field: a trigger that opens a labelled icon grid (4 columns) in a popover. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: string;
  }

  let { id, labelId, describedBy, value = $bindable() }: Props = $props();

  const iconFiles = import.meta.glob("$lib/icons_svg/*.svg");
  // Friendly names for the icons a PID usually shows. Other icons use their file name.
  const friendly: Record<string, string> = {
    thermometer: "Thermometer",
    gauge: "Gauge",
    droplet: "Fluid",
    "battery-full": "Battery",
    "gas-pump": "Fuel",
    fan: "Fan",
    stopwatch: "Timing",
    bolt: "Voltage",
    engine: "Engine",
  };
  // PID icons only: the UI glyphs (check, chevron, plus...) are not offered. A saved icon outside this list still shows.
  const pidIcons = ["thermometer", "gauge", "droplet", "battery-full", "gas-pump", "fan", "stopwatch", "bolt", "engine", "dial", "clock", "route", "timeline-arrow", "chart"];
  const available = new Set(Object.keys(iconFiles).map((path) => path.split("/").pop()!.replace(".svg", "")));
  const names = $derived(value && !pidIcons.includes(value) && available.has(value) ? [value, ...pidIcons] : pidIcons);
  const options = $derived(
    names
      .filter((name) => available.has(name))
      .map((name) => ({
        name,
        label: friendly[name] ?? name.split("-").map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join(" "),
      })),
  );

  const selected = $derived(options.find((o) => o.name === value));
  let trigger: HTMLButtonElement | undefined = $state();
  let pop: HTMLDivElement | undefined = $state();
  const popId = $derived(`${id}-pop`);

  function place() {
    if (!trigger || !pop) return;
    const r = trigger.getBoundingClientRect();
    const width = Math.min(340, window.innerWidth - 16);
    const height = pop.offsetHeight;
    const below = r.bottom + 8;
    pop.style.width = `${width}px`;
    pop.style.left = `${Math.min(Math.max(8, r.left), window.innerWidth - width - 8)}px`;
    pop.style.top = `${below + height > window.innerHeight - 8 ? Math.max(8, r.top - height - 8) : below}px`;
  }

  function onToggle(e: Event) {
    if ((e as ToggleEvent).newState === "open") place();
    else trigger?.focus();
  }

  function choose(name: string) {
    value = name;
    pop?.hidePopover();
  }
</script>

<button
  type="button"
  bind:this={trigger}
  {id}
  class="trig"
  aria-labelledby={`${labelId} ${id}-v`}
  aria-describedby={describedBy}
  aria-haspopup="dialog"
  aria-controls={popId}
  onclick={() => (pop?.matches(":popover-open") ? pop.hidePopover() : pop?.showPopover())}
>
  {#if selected}
    <span class="ico"><Icon name={selected.name} size={22} /></span>
    <span id={`${id}-v`}>{selected.label}</span>
  {:else}
    <span class="ico empty" aria-hidden="true"></span>
    <span id={`${id}-v`} class="none">No icon</span>
  {/if}
  <Icon name="chevron-down" size={16} class="chev" />
</button>

<div bind:this={pop} id={popId} class="pop" popover="auto" role="dialog" aria-label="Icon" ontoggle={onToggle}>
  <div class="grid" role="group" aria-label="Icons">
    {#each options as opt (opt.name)}
      <button
        type="button"
        class="opt"
        aria-pressed={value === opt.name}
        class:on={value === opt.name}
        onclick={() => choose(opt.name)}
      >
        <Icon name={opt.name} size={26} />
        <span>{opt.label}</span>
      </button>
    {/each}
  </div>
</div>

<style>
  .trig {
    display: flex;
    align-items: center;
    gap: 10px;
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 12px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
    font-weight: 600;
    text-align: left;
    cursor: pointer;
  }
  .trig:hover {
    background: var(--ctl-h2);
  }
  .trig > span:nth-child(2) {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .trig :global(.chev) {
    margin-left: auto;
    flex: none;
  }
  .ico {
    display: grid;
    place-items: center;
    flex: none;
    width: 26px;
    height: 26px;
  }
  .ico.empty {
    border-radius: 50%;
    box-shadow: inset 0 0 0 1.5px color-mix(in srgb, var(--text) 30%, transparent);
  }
  .none {
    color: var(--ctl-sub);
  }
  .pop {
    box-sizing: border-box;
    margin: 0;
    padding: 8px;
    border: 0;
    border-radius: var(--r-float);
    background: var(--float-solid);
    color: var(--text);
    box-shadow: var(--shadow-float);
    max-width: calc(100vw - 16px);
    max-height: min(420px, calc(100dvh - 16px));
    overflow: auto;
    position: fixed;
    inset: auto;
  }
  @supports (background: color-mix(in srgb, red 50%, blue)) {
    .pop {
      background: var(--float);
      -webkit-backdrop-filter: blur(var(--blur)) saturate(1.35);
      backdrop-filter: blur(var(--blur)) saturate(1.35);
    }
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(4, minmax(0, 1fr));
    gap: 4px;
  }
  .opt {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 4px;
    min-height: 68px;
    padding: 6px 4px;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    color: var(--text);
    font: inherit;
    font-size: 0.76rem;
    font-weight: 700;
    line-height: 1.2;
    text-align: center;
    cursor: pointer;
  }
  .opt:hover {
    background: var(--hov);
  }
  .opt.on {
    color: var(--accent-text);
    box-shadow: inset 0 -3px 0 var(--accent);
  }
  .opt span {
    overflow-wrap: anywhere;
  }
  .pop:popover-open {
    animation: pop-in 0.22s var(--ease-out);
  }
  @keyframes pop-in {
    from {
      opacity: 0;
      transform: translateY(-6px) scale(0.97);
    }
  }
  @media (prefers-reduced-motion: reduce) {
    .pop:popover-open {
      animation: none;
    }
  }
</style>
