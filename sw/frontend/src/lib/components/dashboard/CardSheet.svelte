<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { onMount, untrack } from "svelte";
  import Sheet from "$lib/components/Sheet.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import Field from "$lib/components/telemetry/editor/Field.svelte";
  import NumberStepper from "$lib/components/telemetry/editor/NumberStepper.svelte";
  import ItemList from "./ItemList.svelte";
  import SizePicker from "./SizePicker.svelte";
  import { cardBodies } from "./cardRegistry";
  import { dashboardDocStore } from "$lib/dashboardDocStore.svelte";
  import { CARD_STYLES, CARD_TYPES, OVERVIEW_MAX_ITEMS, STATUS_ITEMS, type Card, type CardType, type StatusItemKey, clampSize, defaultSize, sizeLimits, type OverviewItem, type Thresholds } from "$lib/dashboardModel";
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
      hidden: card?.hidden ? [...card.hidden] : [],
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
  let hidden = $state<StatusItemKey[]>(seed.hidden);
  let sectionId = $state(seed.sectionId);
  let w = $state(seed.w);
  let h = $state(seed.h);
  let lowWarns = $state(false);
  // Off stores no thresholds: the card never turns warning or critical.
  let alertsOn = $state(untrack(() => (card ? card.thresholds !== undefined : true)));
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
    withThresholds && alertsOn && warn !== undefined && critical !== undefined ? { warn, critical } : undefined,
  );
  const previewCard = $derived<Card>({
    id: "preview",
    type,
    style,
    pids,
    w: size.w,
    h: size.h,
    items: type === "overview" ? items : undefined,
    hidden: type === "status" ? hidden : undefined,
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

  // Overview items: each PID once, the VIN once, the DTC count once, at most OVERVIEW_MAX_ITEMS in all.
  const ITEM_MAX = OVERVIEW_MAX_ITEMS;
  const itemKey = (it: OverviewItem) => (it.t === "pid" ? `pid:${it.pid}` : it.t);
  const hasItem = (key: string) => items.some((it) => itemKey(it) === key);
  const kindName = (it: OverviewItem) => (it.t === "vin" ? "VIN" : it.t === "dtc" ? "DTC count" : (catalog.find((d) => d.pid === it.pid)?.name ?? `PID ${it.pid}`));
  const itemUnit = (it: OverviewItem) => (it.t === "pid" ? (catalog.find((d) => d.pid === it.pid)?.unit ?? "") : "");
  const pidColor = (pid: number) => {
    const c = catalog.find((d) => d.pid === pid)?.color;
    return c === undefined ? undefined : `#${(c & 0xffffff).toString(16).padStart(6, "0")}`;
  };
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
  const pidName = (p: number) => catalog.find((d) => d.pid === p)?.name ?? `PID ${p}`;
  const pidUnit = (p: number) => catalog.find((d) => d.pid === p)?.unit || "PID";
  const pidItemOptions = $derived<SelectMenuOption<string>[]>(catalog.map((d) => ({ value: String(d.pid), label: `${d.name}${d.unit ? ` (${d.unit})` : ""}`, disabled: pids.includes(d.pid) })));
  const reorderPids = (ids: string[]) => {
    const next = ids.map(Number).filter((p) => pids.includes(p));
    if (next.length === pids.length) pids = next;
  };
  const reorderItems = (ids: string[]) => {
    const byKey = new Map(items.map((it) => [itemKey(it), it]));
    const next = ids.flatMap((id) => byKey.get(id) ?? []);
    if (next.length === items.length) items = next;
  };
  const movePid = (i: number, d: number) => {
    const j = i + d;
    if (j < 0 || j >= pids.length) return;
    const next = [...pids];
    [next[i], next[j]] = [next[j], next[i]];
    pids = next;
  };
  const pidOptions = (index: number): SelectMenuOption<number>[] =>
    catalog.map((d) => ({
      value: d.pid,
      label: `${d.name}${d.unit ? ` (${d.unit})` : ""}`,
      disabled: pids.some((p, i) => i !== index && p === d.pid),
    }));

  function toggleAlerts(on: boolean) {
    alertsOn = on;
    if (on && (warn === undefined || critical === undefined)) setDefaults();
  }

  // A status row is shown unless it is listed in `hidden`. `show` is true when the row should appear.
  function toggleRow(key: StatusItemKey, show: boolean) {
    hidden = show ? hidden.filter((k) => k !== key) : [...hidden.filter((k) => k !== key), key];
  }

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
    if (withThresholds && alertsOn) {
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
        ...(type === "status" ? { hidden: [...hidden] } : {}),
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
                    <div class="seg" role="radiogroup" aria-label="Card type">
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
          <div class="seg eq" role="radiogroup" aria-label="Style">
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
        {#if hi === 1}
          <Field id="db-pid-0" label="PID">
            <SelectMenu id="db-pid-0" ariaLabel="PID" placeholder="Choose a PID" options={pidOptions(0)} value={pids[0]} onChange={(pid) => setSlot(0, pid)} />
          </Field>
        {:else}
          <ItemList
            title="PIDs"
            rows={pids.map((p) => ({ id: String(p), name: pidName(p), unit: pidUnit(p), color: pidColor(p) }))}
            options={pidItemOptions}
            max={hi}
            min={lo}
            onAdd={(key) => setSlot(pids.length, Number(key))}
            onRemove={clearSlot}
            onMove={movePid}
            onReorder={reorderPids}
          />
        {/if}
      {/if}
    {/if}

    {#if type === "overview"}
      <ItemList
        title="Items"
        rows={items.map((it) => ({ id: itemKey(it), name: kindName(it), unit: itemUnit(it), color: it.t === "pid" ? pidColor(it.pid) : undefined }))}
        options={itemOptions}
        max={ITEM_MAX}
        emptyHint="No items yet. Add a PID value, the VIN or the DTC count."
        onAdd={addItem}
        onRemove={removeItem}
        onMove={moveItem}
        onReorder={reorderItems}
      />
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

    {#if type === "status"}
      <fieldset class="db-field db-span rows-pick">
        <legend class="db-label">Rows shown</legend>
        {#each STATUS_ITEMS as item (item.key)}
          {@const shown = !hidden.includes(item.key)}
          <button type="button" role="switch" aria-checked={shown} class:off={!shown} onclick={() => toggleRow(item.key, !shown)}>
            <Icon name={shown ? "eye" : "eye-slash"} size={18} />
            {item.label}
          </button>
        {/each}
      </fieldset>
    {/if}

    {#if withThresholds}
      <div class="db-field">
        <span class="db-label">Thresholds</span>
        <Switch checked={alertsOn} label="Show warnings" onchange={toggleAlerts} />
        {#if alertsOn}
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
        {/if}
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
    <button type="button" class="btn" onclick={onClose}>Cancel</button>
    <button type="button" class="btn pri" onclick={save}>{editing ? "Save card" : "Add card"}</button>
  {/snippet}
</Sheet>

<style>
  .db-form {
    display: grid;
    gap: 14px;
    padding: 4px 0 8px;
    min-width: 0;
  }
  .rows-pick {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
    gap: 2px 16px;
    margin: 0;
    padding: 0;
    border: 0;
  }
  .rows-pick button {
    display: flex;
    align-items: center;
    gap: 10px;
    width: auto;
    min-height: 40px;
    margin: 0;
    padding: 0 8px;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    color: var(--text);
    font-weight: 700;
    text-align: left;
    cursor: pointer;
  }
  .rows-pick button:hover {
    background: var(--hov);
  }
  .rows-pick button :global(svg) {
    flex: none;
    color: var(--accent-text);
  }
  .rows-pick button.off {
    color: var(--muted);
    font-weight: 600;
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
    border-radius: var(--r-card);
    background: var(--card);
    -webkit-backdrop-filter: var(--card-blur);
    backdrop-filter: var(--card-blur);
    overflow: hidden;
  }
</style>
