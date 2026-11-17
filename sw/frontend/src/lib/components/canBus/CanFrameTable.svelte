<script lang="ts">
  import type { LogRow, IdView, TableFilter } from "$lib/canBus/canModel";
  import type { TableMode } from "$lib/canBus/canBusStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import { describeRule, FILTER_FIELDS, opsFor, parseFilterValue, type FilterField, type FilterOp, type FilterRule } from "$lib/canBus/canFilter";
  import { relTime } from "$lib/canBus/canFormat";
  import { measureHeight, trackUnder } from "$lib/layers";
  import { formatId, idColorIndex, idKey } from "$lib/canBus/canIds";

  type Row = LogRow | IdView;

  interface Props {
    mode: TableMode;
    rows: LogRow[];
    byId: IdView[];
    startUs: number | null;
    selected: number | null;
    onselect: (key: number) => void;
    ondetails: (key: number) => void;
    onmode: (mode: TableMode) => void;
    onclear: () => void;
    filter: TableFilter;
    rules: FilterRule[];
    onfilter: (filter: TableFilter) => void;
    onaddrule: (rule: Omit<FilterRule, "key">) => void;
    onremoverule: (key: number) => void;
  }

  let { mode, rows, byId, startUs, selected, onselect, ondetails, onmode, onclear, filter, rules, onfilter, onaddrule, onremoverule }: Props = $props();

  // The bar sits beside the scroller (Gecko drops the blur inside one), so mirror its horizontal scroll and scrollbar width.
  function syncBar(el: HTMLElement) {
    const host = el.parentElement;
    const sync = () => {
      host?.style.setProperty("--sl", `${-el.scrollLeft}px`);
      host?.style.setProperty("--sbw", `${el.offsetWidth - el.clientWidth}px`);
    };
    el.addEventListener("scroll", sync, { passive: true });
    const ro = new ResizeObserver(sync);
    ro.observe(el);
    sync();
    return () => {
      el.removeEventListener("scroll", sync);
      ro.disconnect();
    };
  }

  const FILTERS: { id: TableFilter; label: string }[] = [
    { id: "all", label: "All" },
    { id: "changed", label: "Changed" },
    { id: "obd", label: "OBD" },
  ];

  let adding = $state(false);
  let exclude = $state(false);
  let field = $state<FilterField>("id");
  let op = $state<FilterOp>("=");
  let value = $state("");
  let wrapEl = $state<HTMLDivElement>();
  const valid = $derived(parseFilterValue(field, value) !== null);
  const hint = $derived(field === "dir" ? "RX or TX" : field === "dlc" ? "0-8" : "hex, e.g. 7E8");

  const fieldOptions = FILTER_FIELDS.map((f) => ({ value: f.id, label: f.label }));
  const opOptions = $derived(opsFor(field).map((o) => ({ value: o, label: o })));

  function onField() {
    if (!opsFor(field).includes(op)) op = "=";
  }

  function add() {
    if (!valid) return;
    onaddrule({ field, op, value: value.trim(), exclude });
    value = "";
    adding = false;
  }

  const count = $derived(mode === "log" ? rows.length : byId.length);
</script>

<svelte:window onpointerdown={(e) => adding && !wrapEl?.contains(e.target as Node) && !(e.target as Element).closest(".select-menu-list") && (adding = false)} />

<section class="card ft" aria-label="Frames">
  <div class="ft-head">
    <div class="seg" role="radiogroup" aria-label="Table">
      <button role="radio" aria-checked={mode === "log"} onclick={() => onmode("log")}>Log</button>
      <button role="radio" aria-checked={mode === "byId"} onclick={() => onmode("byId")}>By ID</button>
    </div>
    <div class="seg" role="radiogroup" aria-label="Show">
      {#each FILTERS as f (f.id)}
        <button role="radio" aria-checked={filter === f.id} onclick={() => onfilter(f.id)}>{f.label}</button>
      {/each}
    </div>
    <div class="ft-tools">
      <div class="ft-add" bind:this={wrapEl}>
        <button class="btn sm" aria-expanded={adding} aria-haspopup="dialog" onclick={() => (adding = !adding)}>
          <Icon name="plus" size={16} />
          <span>Filter</span>
        </button>
        {#if adding}
          <!-- svelte-ignore a11y_no_noninteractive_element_interactions -->
          <form
            class="popup ft-pop"
            aria-label="Add a filter"
            onsubmit={(e) => {
              e.preventDefault();
              add();
            }}
            onkeydown={(e) => {
              if (e.key === "Escape") adding = false;
            }}
          >
            <div class="seg" role="radiogroup" aria-label="Rule type">
              <button type="button" role="radio" aria-checked={!exclude} onclick={() => (exclude = false)}>Include</button>
              <button type="button" role="radio" aria-checked={exclude} onclick={() => (exclude = true)}>Exclude</button>
            </div>
            <div class="ft-f">
              <span class="muted">Row</span>
              <SelectMenu ariaLabel="Row" options={fieldOptions} bind:value={field} onChange={onField} />
            </div>
            <div class="ft-f">
              <span class="muted">Is</span>
              <SelectMenu ariaLabel="Operator" options={opOptions} bind:value={op} />
            </div>
            <label class="ft-f">
              <span class="muted">Value</span>
              <!-- svelte-ignore a11y_autofocus -->
              <input class="in mono" bind:value placeholder={hint} autofocus aria-invalid={value !== "" && !valid ? "true" : undefined} />
            </label>
            <button class="btn pri" disabled={!valid}>Add filter</button>
          </form>
        {/if}
      </div>
      <span class="muted">{count} shown</span>
      <button class="btn sm" disabled={count === 0} onclick={onclear}>
        <Icon name="trash" size={16} />
        <span>Clear</span>
      </button>
    </div>
  </div>

  {#if rules.length}
    <ul class="ft-chips" aria-label="Active filters">
      {#each rules as r (r.key)}
        <li class:ex={r.exclude}>
          <span>{describeRule(r)}</span>
          <button aria-label="Remove filter" onclick={() => onremoverule(r.key)}><Icon name="x" size={12} /></button>
        </li>
      {/each}
    </ul>
  {/if}

  {#if count === 0}
    <p class="muted ft-empty">{rules.length || filter !== "all" ? "No frames match the filter." : "No frames. Press Start to log the bus."}</p>
  {:else}
    <div class="ft-body">
      <div class="ft-bar float-layer rest-top" aria-hidden="true" {@attach measureHeight("--hdr-h")}>
        <div class="ft-clip">
        <div class="ft-cols">
          <span>Time (s)</span>
          <span>Direction</span>
          <span>ID</span>
          <span class="hide-s">DLC</span>
          <span>Data</span>
          <span>Parsed</span>
        </div>
        </div>
      </div>
      <div class="ft-scroll" {@attach trackUnder} {@attach syncBar}>
      <table class="ft-tbl">
        <colgroup>
          <col class="w-t" />
          <col class="w-d" />
          <col class="w-i" />
          <col class="w-l hide-s" />
          <col class="w-b" />
          <col />
        </colgroup>
        <thead class="sr-only">
          <tr>
            <th scope="col">Time (s)</th>
            <th scope="col">Direction</th>
            <th scope="col">ID</th>
            <th scope="col" class="hide-s">DLC</th>
            <th scope="col">Data</th>
            <th scope="col">Parsed</th>
          </tr>
        </thead>
        <tbody>
          {#if mode === "log"}
            {#each rows as row (row)}
              {@render line(row)}
            {/each}
          {:else}
            {#each byId as row (row.key)}
              {@render line(row)}
            {/each}
          {/if}
        </tbody>
      </table>
      </div>
    </div>
  {/if}
</section>

{#snippet line(row: Row)}
  {@const key = idKey(row.id, row.ext)}
  <tr
    tabindex="0"
    class:sel={selected === key}
    onclick={() => onselect(key)}
    ondblclick={() => ondetails(key)}
    onkeydown={(e) => {
      if (e.key === "Enter") ondetails(key);
    }}
  >
    <td class="mono muted">{relTime(row.t, startUs)}</td>
    <td class="dir" class:tx={row.tx}>{row.tx ? "TX" : "RX"}</td>
    <td class="mono id" style:color="var(--c{idColorIndex(key)})">{formatId(row.id, row.ext)}</td>
    <td class="hide-s">{row.data.length}</td>
    <td class="mono data">
      {#if row.rtr}
        <span class="muted">remote request</span>
      {:else}
        {#each Array.from(row.data) as b, i (i)}
          <span class="byte" class:chg={(row.changed & (1 << i)) !== 0} class:z={b === 0}>{b.toString(16).toUpperCase().padStart(2, "0")}</span>
        {/each}
      {/if}
    </td>
    <td class="mono parsed" class:muted={!row.parsed}>{row.parsed || "-"}</td>
  </tr>
{/snippet}

<style>
  .ft {
    padding: 8px 12px;
    min-width: 0;
  }
  .ft-head {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    padding: 4px 4px 8px;
  }
  .ft-tools {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  .ft-tools .btn {
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  .ft-add {
    position: relative;
  }
  .ft-pop {
    position: absolute;
    right: 0;
    top: calc(100% + 6px);
    z-index: 20;
    width: 260px;
    display: grid;
    gap: 10px;
    padding: 12px;
  }
  .ft-f {
    display: grid;
    gap: 4px;
  }
  .ft-chips {
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
    margin: 0;
    padding: 0 4px 8px;
    list-style: none;
  }
  .ft-chips li {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 4px 6px 4px 10px;
    border-radius: 999px;
    background: var(--ctl);
    font-size: 0.85rem;
  }
  .ft-chips li.ex {
    color: var(--crit);
  }
  .ft-chips button {
    display: grid;
    place-items: center;
    width: 20px;
    height: 20px;
    padding: 0;
    border: 0;
    border-radius: 50%;
    background: transparent;
    color: inherit;
    cursor: pointer;
  }
  .ft-empty {
    margin: 12px;
  }
  .ft-body {
    --w-t: 84px;
    --w-d: 92px;
    --w-i: 92px;
    --w-l: 56px;
    --w-b: 300px;
    --min: calc(var(--w-t) + var(--w-d) + var(--w-i) + var(--w-l) + var(--w-b) + 140px);
    position: relative;
  }
  .ft-scroll {
    overflow: auto;
    max-height: 60vh;
    border-radius: var(--r-ctl);
    padding-top: var(--hdr-h, 30px);
  }
  .ft-bar {
    position: absolute;
    top: 0;
    left: 0;
    right: var(--sbw, 0px);
    z-index: 2;
    border-radius: var(--r-ctl);
    box-shadow: none;
    pointer-events: none;
  }
  .ft-clip {
    overflow: hidden;
    border-radius: inherit;
  }
  .ft-cols {
    display: grid;
    grid-template-columns: var(--w-t) var(--w-d) var(--w-i) var(--w-l) var(--w-b) 1fr;
    min-width: var(--min);
    margin-left: var(--sl, 0px);
  }
  .ft-cols span {
    padding: 6px 12px;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    font-size: 0.75rem;
    font-weight: 800;
    color: var(--muted);
    white-space: nowrap;
  }
  .ft-tbl {
    width: 100%;
    min-width: var(--min);
    table-layout: fixed;
    border-collapse: separate;
    border-spacing: 0 4px;
    font-size: 0.92rem;
  }
  .w-t {
    width: var(--w-t);
  }
  .w-d {
    width: var(--w-d);
  }
  .w-i {
    width: var(--w-i);
  }
  .w-l {
    width: var(--w-l);
  }
  .w-b {
    width: var(--w-b);
  }
  .sr-only {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip-path: inset(50%);
  }
  .ft-tbl td {
    padding: 9px 12px;
    background: var(--row);
    white-space: nowrap;
    font-variant-numeric: tabular-nums;
  }
  .ft-tbl td:first-child {
    border-radius: var(--r-ctl) 0 0 var(--r-ctl);
    position: relative;
  }
  .ft-tbl td:last-child {
    border-radius: 0 var(--r-ctl) var(--r-ctl) 0;
  }
  .ft-tbl tbody tr {
    cursor: pointer;
  }
  .ft-tbl tbody tr:hover td {
    background: var(--hov);
  }
  .ft-tbl tr.sel td,
  .ft-tbl tr.sel:hover td {
    background: color-mix(in srgb, var(--accent) 24%, var(--hov));
  }
  .id {
    font-weight: 800;
    letter-spacing: 0.02em;
  }
  .byte {
    display: inline-block;
    min-width: 2.1ch;
    padding: 1px 3px;
    border-radius: 6px;
    text-align: center;
  }
  .byte + .byte {
    margin-left: 0.25em;
  }
  .byte.z {
    color: var(--muted);
  }
  .byte.chg {
    background: color-mix(in srgb, var(--accent) 30%, transparent);
    color: var(--accent-text);
    font-weight: 800;
  }
  .parsed:not(.muted) {
    color: var(--accent-text);
  }
  .dir {
    font-weight: 600;
    color: var(--ok);
  }
  .dir.tx {
    color: var(--accent-text);
  }
  @media (max-width: 859px) {
    .hide-s {
      display: none;
    }
    .ft-body {
      --min: calc(var(--w-t) + var(--w-d) + var(--w-i) + var(--w-b) + 140px);
    }
    .ft-cols {
      grid-template-columns: var(--w-t) var(--w-d) var(--w-i) var(--w-b) 1fr;
    }
  }
</style>
