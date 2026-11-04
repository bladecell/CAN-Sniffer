<script lang="ts">
  import { onMount, untrack } from "svelte";
  import Sheet from "$lib/components/Sheet.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import Field from "$lib/components/telemetry/editor/Field.svelte";
  import NumberStepper from "$lib/components/telemetry/editor/NumberStepper.svelte";
  import SizePicker from "./SizePicker.svelte";
  import { cardBodies } from "./cardRegistry";
  import { dashboardDocStore } from "$lib/dashboardDocStore.svelte";
  import { CARD_STYLES, CARD_TYPES, type Card, type CardType, clampSize, defaultSize, sizeLimits, type OverviewItem, type Thresholds } from "$lib/dashboardModel";
  import type { CardInput } from "$lib/dashboardOps";
  import {
    STYLE_LABELS,
    TYPE_LABELS,
    defaultThresholds,
    needsRange,
    pidSlots,
    rangeOf,
    thresholdsApply,
    thresholdsError,
  } from "$lib/dashboardCardForm";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import type { SelectMenuOption } from "$lib/selectMenu";

  interface Props {
    /** Edit: the card being changed. Add: the section picker starts on `sectionId`. */
    card?: Card;
    sectionId?: string;
    sections: { id: string; label: string }[];
    onClose: () => void;
  }

  let { card, sectionId: initialSection = "", sections, onClose }: Props = $props();
  const editing = untrack(() => card !== undefined);

  // The form starts from the card (edit) or the first type (add). The sheet is mounted once per open, so this runs once.
  const seed = untrack(() => {
    const t: CardType = card?.type ?? "value";
    const sizeSeed = defaultSize({ type: t, style: CARD_STYLES[t][0], pids: [] });
    return {
      type: t,
      style: card?.style ?? CARD_STYLES[t][0],
      pids: card ? [...card.pids] : [],
      items: card?.items ? card.items.map((i) => ({ ...i })) : [],
      sectionId: initialSection || sections[0]?.id || "",
      w: card?.w ?? sizeSeed.w,
      h: card?.h ?? sizeSeed.h,
      warn: card?.thresholds?.warn,
      critical: card?.thresholds?.critical,
    };
  });

  let type = $state<CardType>(seed.type);
  let style = $state<string>(seed.style);
  let pids = $state<number[]>(seed.pids);
  let items = $state<OverviewItem[]>(seed.items);
  let sectionId = $state(seed.sectionId);
  let w = $state(seed.w);
  let h = $state(seed.h);
  let lowWarns = $state(false);
  let warn = $state<number | undefined>(seed.warn);
  let rMin = $state<number | undefined>(untrack(() => card?.range?.min));
  let rMax = $state<number | undefined>(untrack(() => card?.range?.max));
  let critical = $state<number | undefined>(seed.critical);
  let message = $state("");

  const [lo, hi] = $derived(pidSlots(type, style, 0));
  const shape = $derived({ type, style, pids, items });
  const limits = $derived(sizeLimits(shape));
  const size = $derived(clampSize(shape, w, h));
  const catalog = $derived(vehicleStore.pidDefinitions);
  const picked = $derived(pids.length === 1 ? catalog.find((d) => d.pid === pids[0]) : undefined);
  const range = $derived(picked ? rangeOf(picked) : null);
  const withThresholds = $derived(thresholdsApply(type, pids.length));
  const styles = $derived(CARD_STYLES[type] as readonly string[]);
  const styleOptions = $derived(styles.map((s) => ({ value: s, label: STYLE_LABELS[s] ?? s })));
  const typeOptions = CARD_TYPES.map((t) => ({ value: t, label: TYPE_LABELS[t] }));

  // The preview takes the thresholds as typed, so warning and critical states show before saving.
  const previewThresholds = $derived<Thresholds | undefined>(
    withThresholds && warn !== undefined && critical !== undefined ? { warn, critical } : undefined,
  );
  const previewCard = $derived<Card>({
    id: "preview",
    type,
    style,
    pids,
    w: size.w,
    h: size.h,
    items: type === "overview" ? items : undefined,
    thresholds: previewThresholds,
  });
  const Body = $derived(cardBodies[type]);

  let catalogState = $state<"loading" | "ready" | "failed">("loading");
  onMount(() => {
    if (vehicleStore.pidDefinitions.length > 0) {
      catalogState = "ready";
      return;
    }
    void vehicleStore.loadDefinitions().then((ok) => {
      catalogState = ok ? "ready" : "failed";
    });
  });

  // Thresholds are per card: the defaults follow the PID's range and the low-value switch.
  // A gauge opened without its own range takes the PID range once the PID list has loaded.
  let rangeSeeded = false;
  $effect(() => {
    if (!rangeSeeded && range && rMin === undefined && rMax === undefined) {
      rangeSeeded = true;
      setRangeDefaults();
    }
  });
  // The gauge scale is prefilled from the PID range and can be changed for this card.
  function setRangeDefaults() {
    rMin = range?.min;
    rMax = range?.max;
  }

  function setDefaults() {
    const d = defaultThresholds(range, lowWarns);
    warn = d?.warn;
    critical = d?.critical;
  }

  // Each change resets what depends on it: a new type starts at its first style and default size.
  function pickType(next: CardType) {
    type = next;
    style = CARD_STYLES[next][0];
    const [, max] = pidSlots(next, style, 0);
    pids = pids.slice(0, max);
    const d = defaultSize({ type: next, style, pids, items });
    w = d.w;
    h = d.h;
    message = "";
    setDefaults();
  }

  function pickStyle(next: string) {
    style = next;
    const d = defaultSize({ type, style, pids, items });
    w = d.w;
    h = d.h;
    setDefaults();
  }

  // Overview items: each PID once, the VIN once, the DTC count once, at most six in all.
  const ITEM_MAX = 6;
  const itemKey = (it: OverviewItem) => (it.t === "pid" ? `pid:${it.pid}` : it.t);
  const hasItem = (key: string) => items.some((it) => itemKey(it) === key);
  const kindName = (it: OverviewItem) => (it.t === "vin" ? "VIN" : it.t === "dtc" ? "DTC count" : (catalog.find((d) => d.pid === it.pid)?.name ?? `PID ${it.pid}`));
  const itemUnit = (it: OverviewItem) => (it.t === "vin" ? "VIN" : it.t === "dtc" ? "DTC" : (catalog.find((d) => d.pid === it.pid)?.unit || "PID"));
  const addItem = (key: string) => {
    message = "";
    if (items.length >= ITEM_MAX || hasItem(key)) return;
    const next: OverviewItem = key.startsWith("pid:") ? { t: "pid", pid: Number(key.slice(4)) } : { t: key === "vin" ? "vin" : "dtc" };
    items = [...items, next];
  };
  const removeItem = (i: number) => {
    message = "";
    items = items.filter((_, j) => j !== i);
  };
  const moveItem = (i: number, d: number) => {
    const j = i + d;
    if (j < 0 || j >= items.length) return;
    const next = [...items];
    [next[i], next[j]] = [next[j], next[i]];
    items = next;
  };
  const setLabel = (i: number, label: string) => {
    items = items.map((it, j) => (j === i ? { ...it, label: label || undefined } : it));
  };
  const itemOptions = $derived<SelectMenuOption<string>[]>([
    ...catalog.map((d) => ({ value: `pid:${d.pid}`, label: `${d.name}${d.unit ? ` (${d.unit})` : ""}`, disabled: hasItem(`pid:${d.pid}`) })),
    { value: "vin", label: "VIN", disabled: hasItem("vin") },
    { value: "dtc", label: "DTC count", disabled: hasItem("dtc") },
  ]);

  // One PID select per slot. Choosing in the last, empty slot adds a PID; a PID already in another slot is disabled.
  function setSlot(index: number, pid: number) {
    message = "";
    const next = [...pids];
    if (index < next.length) next[index] = pid;
    else next.push(pid);
    pids = next.slice(0, hi);
    if (pids.length === 1) {
      setDefaults();
      setRangeDefaults();
    }
  }
  function clearSlot(index: number) {
    message = "";
    pids = pids.filter((_, i) => i !== index);
  }
  const slotCount = $derived(Math.max(1, Math.min(hi, pids.length + 1)));
  const pidOptions = (index: number): SelectMenuOption<number>[] =>
    catalog.map((d) => ({
      value: d.pid,
      label: `${d.name}${d.unit ? ` (${d.unit})` : ""}`,
      disabled: pids.some((p, i) => i !== index && p === d.pid),
    }));

  function toggleLow(on: boolean) {
    lowWarns = on;
    setDefaults();
  }

  function buildInput(): { input?: CardInput; error?: string } {
    if (type === "overview" ? items.length === 0 : pids.length < lo) {
      return { error: type === "overview" ? "Add at least one item." : "Choose a PID." };
    }
    if (needsRange(type, style) && !range) {
      return { error: "This PID has no range, so zones need one. Set a range for it first." };
    }
    let cardRange: { min: number; max: number } | undefined;
    if (type === "gauge" && pids.length === 1 && rMin !== undefined && rMax !== undefined) {
      if (!(rMin < rMax)) return { error: "The range minimum must be below the maximum." };
      cardRange = { min: rMin, max: rMax };
    }
    let thresholds: Thresholds | undefined;
    if (withThresholds) {
      const t = { warn: warn ?? NaN, critical: critical ?? NaN };
      const err = thresholdsError(range, t);
      if (err) return { error: err };
      thresholds = t;
    }
    return {
      input: {
        type,
        style,
        pids: [...pids],
        w: size.w,
        h: size.h,
        ...(thresholds ? { thresholds } : {}),
        ...(cardRange ? { range: cardRange } : {}),
        ...(type === "overview" ? { items: items.map((it) => ({ ...it })) } : {}),
      },
    };
  }

  function save() {
    const { input, error } = buildInput();
    if (!input) {
      message = error ?? "That card is not valid.";
      return;
    }
    const err = editing && card ? dashboardDocStore.updateCard(card.id, input) : dashboardDocStore.addCard(sectionId, input);
    if (err) {
      message = err;
      return;
    }
    onClose();
  }

  const title = editing ? "Edit card" : "Add card";
</script>

<Sheet open title={title} subtitle="Nothing changes until you save." {onClose}>
  {#snippet head()}
    <section class="db-preview" aria-label="Preview at the card's size">
      <div class="db-grid db-preview-grid" style:--w={size.w}>
        <div class="db-tile" style:grid-column="span {size.w}" style:grid-row="span {size.h}" data-type={type}>
          <Body card={previewCard} />
        </div>
      </div>
    </section>
  {/snippet}

  <div class="db-form">
    {#if !editing}
      <div class="db-span">
        <Field id="db-type" label="Card type">
                    <div class="db-seg db-wrap" role="radiogroup" aria-label="Card type">
              {#each typeOptions as o (o.value)}
                <button type="button" role="radio" data-value={o.value} aria-checked={type === o.value} tabindex={type === o.value ? 0 : -1} onclick={() => pickType(o.value)}>{o.label}</button>
              {/each}
            </div>
        </Field>
      </div>
    {/if}

    <div class="db-span db-row">
      {#if styles.length > 1}
        <Field id="db-style" label="Style">
          <div class="db-seg eq" role="radiogroup" aria-label="Style">
            {#each styleOptions as o (o.value)}
              <button type="button" role="radio" data-value={o.value} aria-checked={style === o.value} tabindex={style === o.value ? 0 : -1} onclick={() => pickStyle(o.value)}>{o.label}</button>
            {/each}
          </div>
        </Field>
      {/if}
      <SizePicker min={limits.min} max={limits.max} w={size.w} h={size.h} onChange={(nw, nh) => { w = nw; h = nh; }} />
    </div>

    {#if hi > 0 && type !== "overview"}
      {#if catalogState === "loading"}
        <p class="db-hint" role="status">Loading PIDs...</p>
      {:else if catalogState === "failed"}
        <p class="db-error" role="alert">PID list unavailable. Close and try again.</p>
      {:else if catalog.length === 0}
        <p class="db-hint">No PIDs yet. Add PIDs in Settings first.</p>
      {:else}
        {#each Array.from({ length: slotCount }, (_, i) => i) as i (i)}
          <Field id="db-pid-{i}" label={hi === 1 ? "PID" : `PID ${i + 1}`}>
                          <div class="db-slot">
                <SelectMenu
                  id="db-pid-{i}"
                  ariaLabel={hi === 1 ? "PID" : `PID ${i + 1}`}
                  placeholder="Choose a PID"
                  options={pidOptions(i)}
                  value={pids[i]}
                  onChange={(pid) => setSlot(i, pid)}
                />
                {#if hi > 1 && i < pids.length && pids.length > lo}
                  <button type="button" class="rv-btn sm" onclick={() => clearSlot(i)}>Remove</button>
                {/if}
              </div>
          </Field>
        {/each}
      {/if}
    {/if}

    {#if type === "overview"}
      <div class="db-field db-span">
        <span class="db-label">Items <span class="db-count">{items.length} of {ITEM_MAX}</span></span>
        {#if items.length === 0}
          <p class="db-hint">No items yet. Add a PID value, the VIN or the DTC count.</p>
        {:else}
          <ul class="db-items" aria-label="Overview items, in display order">
            {#each items as it, i (i)}
              <li class="db-item">
                <div class="db-item-main">
                  <div class="db-item-head">
                    <strong title={kindName(it)}>{kindName(it)}</strong>
                    <span class="db-unit">{itemUnit(it)}</span>
                  </div>
                  <input class="db-label-in" type="text" maxlength="24" placeholder="Label (optional)" value={it.label ?? ""} aria-label="Label for {kindName(it)}" oninput={(e) => setLabel(i, e.currentTarget.value)} />
                </div>
                <div class="db-item-acts">
                  <button type="button" class="db-icon" aria-label="Move {kindName(it)} up" aria-disabled={i === 0} onclick={() => i > 0 && moveItem(i, -1)}>
                    <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true"><path d="M6 15l6-6 6 6" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
                  </button>
                  <button type="button" class="db-icon" aria-label="Move {kindName(it)} down" aria-disabled={i === items.length - 1} onclick={() => i < items.length - 1 && moveItem(i, 1)}>
                    <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true"><path d="M6 9l6 6 6-6" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
                  </button>
                  <button type="button" class="db-icon db-icon-danger" aria-label="Remove {kindName(it)}" onclick={() => removeItem(i)}>
                    <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true"><path d="M5 7h14M10 7V5h4v2M8 7l1 12h6l1-12" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" /></svg>
                  </button>
                </div>
              </li>
            {/each}
          </ul>
        {/if}
        <div class="db-add-row">
          <div class="db-add-pick">
            {#key items.length}
            <SelectMenu id="db-add" ariaLabel="Add item" placeholder={items.length >= ITEM_MAX ? "Up to 6 items" : "+ Add item"} options={itemOptions} onChange={addItem} disabled={items.length >= ITEM_MAX} />
            {/key}
          </div>
          <span class="db-hint">{items.length} of {ITEM_MAX} items</span>
        </div>
      </div>
    {/if}

    {#if type === "gauge" && pids.length === 1}
      <div class="db-field">
        <span class="db-label">Range</span>
        <div class="db-two">
          <Field id="db-rmin" label="Min">
            {#snippet children(labelId, msgId)}
              <NumberStepper id="db-rmin" {labelId} describedBy={msgId} bind:value={rMin} step={1} />
            {/snippet}
          </Field>
          <Field id="db-rmax" label="Max">
            {#snippet children(labelId, msgId)}
              <NumberStepper id="db-rmax" {labelId} describedBy={msgId} bind:value={rMax} step={1} />
            {/snippet}
          </Field>
        </div>
        <p class="db-hint">Prefilled from the PID. You can change it for this card.</p>
      </div>
    {/if}

    {#if withThresholds}
      <div class="db-field">
        <span class="db-label">Thresholds</span>
        <Switch checked={lowWarns} label="Warn on low values" onchange={toggleLow} />
        <div class="db-two">
          <Field id="db-warn" label="Warning at">
            {#snippet children(labelId, msgId)}
              <NumberStepper id="db-warn" {labelId} describedBy={msgId} bind:value={warn} step={range ? (range.max - range.min) / 100 : 1} />
            {/snippet}
          </Field>
          <Field id="db-critical" label="Critical at">
            {#snippet children(labelId, msgId)}
              <NumberStepper id="db-critical" {labelId} describedBy={msgId} bind:value={critical} step={range ? (range.max - range.min) / 100 : 1} />
            {/snippet}
          </Field>
        </div>
        <p class="db-hint">
          {range ? `PID range ${range.min} to ${range.max}.` : "This PID has no range. Enter the values yourself."}
        </p>
      </div>
    {/if}

    {#if !editing && sections.length > 1}
      <Field id="db-section" label="Section">
        <SelectMenu
          id="db-section"
          ariaLabel="Section"
          options={sections.map((s) => ({ value: s.id, label: s.label }))}
          bind:value={sectionId}
        />
      </Field>
    {/if}

    {#if message}
      <p class="db-error db-span" role="alert">{message}</p>
    {/if}
  </div>

  {#snippet footer()}
    <button type="button" class="rv-btn" onclick={onClose}>Cancel</button>
    <button type="button" class="rv-btn pri" onclick={save}>{editing ? "Save card" : "Add card"}</button>
  {/snippet}
</Sheet>

<style>
  .db-form {
    display: grid;
    gap: 14px;
    padding: 4px 0 8px;
    min-width: 0;
  }
  .db-wrap {
    display: flex;
    flex-wrap: wrap;
  }
  .db-span {
    grid-column: 1 / -1;
    min-width: 0;
  }
  .db-row {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(min(100%, 220px), 1fr));
    gap: 14px 18px;
    align-items: start;
  }
  .db-count {
    margin-left: 6px;
    padding: 1px 8px;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--ctl-text);
    font-size: 0.75rem;
    font-weight: 800;
  }
  .db-items {
    display: grid;
    gap: 6px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .db-item {
    display: flex;
    align-items: center;
    gap: 10px;
    min-width: 0;
    padding: 8px 10px 8px 12px;
    border-radius: 18px;
    background: var(--card);
  }
  .db-item-main {
    display: grid;
    flex: 1;
    gap: 4px;
    min-width: 0;
  }
  .db-item-head {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .db-item-head strong {
    overflow: hidden;
    font-size: 0.9rem;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .db-unit {
    flex: none;
    padding: 1px 8px;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--ctl-text);
    font-size: 0.72rem;
    font-weight: 800;
  }
  .db-label-in {
    box-sizing: border-box;
    width: 100%;
    min-height: 44px;
    padding: 0 12px;
    border: 0;
    border-radius: 12px;
    background: var(--ctl);
    color: var(--ctl-text);
    font: 600 0.9rem/1 var(--font);
  }
  .db-item-acts {
    display: flex;
    flex: none;
    gap: 6px;
  }
  .db-icon-danger {
    color: var(--crit);
  }
  .db-add-row {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 12px;
  }
  .db-add-pick {
    flex: 0 1 220px;
  }
  .db-two {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }
  /* Desktop: two columns, so the sheet fits a 900 px screen without scrolling. */
  @media (min-width: 860px) {
    .db-form {
      grid-template-columns: repeat(2, minmax(0, 1fr));
      column-gap: 18px;
    }
  }
  .db-slot {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .db-slot :global(.select-menu) {
    flex: 1;
  }
  .db-preview {
    display: grid;
    justify-items: center;
    padding: 4px 0 10px;
    pointer-events: none;
  }
  /* The shared six-column section grid, fitted to the sheet: a 6-wide preview never runs past the edge. */
  .db-preview-grid {
    grid-template-columns: repeat(var(--w), minmax(0, 1fr));
    width: min(100%, calc(var(--w) * 90px));
  }
  .db-preview-grid .db-tile {
    min-width: 0;
    min-height: 0;
    border-radius: 22px;
    background: var(--card);
    overflow: hidden;
  }
</style>
