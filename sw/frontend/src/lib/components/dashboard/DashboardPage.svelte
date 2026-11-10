<script lang="ts">
  import { onMount, untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import Dialog from "$lib/components/Dialog.svelte";
  import Sheet from "$lib/components/Sheet.svelte";
  import SidebarHint from "$lib/components/telemetry/SidebarHint.svelte";
  import ViewHeader from "$lib/components/listDetail/ViewHeader.svelte";
  import { auth } from "$lib/auth.svelte";
  import { dashboardDocStore as store } from "$lib/dashboardDocStore.svelte";
  import { type Card, sizeLimits } from "$lib/dashboardModel";
  import { TEMPLATES, templateById, templateForName, type TemplateId } from "$lib/dashboardTemplates";
  import type { SectionRef } from "$lib/dashboardGrid";
  import DashboardTabs from "./DashboardTabs.svelte";
  import SectionView from "./SectionView.svelte";
  import CardSheet from "./CardSheet.svelte";
  import NameSheet from "./NameSheet.svelte";
  import NewDashboardSheet from "./NewDashboardSheet.svelte";
  import SizePicker from "./SizePicker.svelte";
  import TemplatePicker from "./TemplatePicker.svelte";
  import "./dashboard.css";

  type SheetState =
    | { kind: "card-add"; sectionId: string }
    | { kind: "card-edit"; cardId: string }
    | { kind: "size"; cardId: string }
    | { kind: "new-dashboard" }
    | { kind: "template" }
    | { kind: "edit" }
    | null;

  const view = $derived(store.view);
  const active = $derived(view.active);
  const sections = $derived(active.sections);
  const editing = $derived(view.editing);
  const fresh = $derived(view.fresh);
  const refs = $derived<SectionRef[]>(
    sections.map((s, i) => ({ id: s.id, label: s.title || `Section ${i + 1}`, count: s.cards.length })),
  );

  let sheet = $state<SheetState>(null);
  let message = $state("");
  let saving = $state(false);
  let confirmDelete = $state(false);
  let confirmReset = $state(false);
  let editName = $state("");
  let editError = $state("");
  let phone = $state(false);
  // The layout picked on the first-run page or in the template sheet.
  let pickId = $state<TemplateId>(TEMPLATES[0].id);
  const picked = $derived(templateById(pickId));

  // Phones show one section per row and use the size picker instead of the corner handle.
  onMount(() => {
    const mq = window.matchMedia("(max-width: 859px)");
    const sync = () => (phone = mq.matches);
    sync();
    mq.addEventListener("change", sync);
    return () => mq.removeEventListener("change", sync);
  });

  // The layout on the SD card is read once the user is logged in. Before that the local copy is shown.
  $effect(() => {
    if (auth.canUseGated) untrack(() => void store.sync());
  });


  const statusLine = $derived.by(() => {
    if (fresh) return "Pick a layout to start with. You can change it later.";
    return "";
  });
  const editHelp = $derived(`Editing ${active.name}. Cards snap to 6 columns. Nothing is saved until you press Done.`);

  function startEdit() {
    message = "";
    store.startEdit();
  }

  async function done() {
    saving = true;
    message = "";
    try {
      await store.done();
    } finally {
      saving = false;
    }
  }

  function cancel() {
    message = "";
    confirmDelete = false;
    confirmReset = false;
    sheet = null;
    store.cancel();
  }

  function fail(err: string | null) {
    if (err) message = err;
    return !err;
  }

  // Reset works on the draft: the sections are replaced, and Done saves them or Cancel drops them.
  function resetDashboard() {
    confirmReset = false;
    const template = templateForName(active.name);
    if (!fail(store.resetToTemplate(template))) return;
    message = `Reset to the ${template.name} template. Press Done to save.`;
  }

  // Loads a template onto the active dashboard as a draft. Done saves it.
  function loadTemplate(template: (typeof TEMPLATES)[number], rename: boolean) {
    message = "";
    if (!editing) store.startEdit();
    if (!fail(store.resetToTemplate(template))) return false;
    if (rename) fail(store.renameDashboard(active.id, template.name));
    return true;
  }

  // The first-run page saves at once: there is nothing to lose yet.
  async function startWith() {
    if (!loadTemplate(picked, true)) return;
    await done();
  }

  function useTemplate() {
    if (!loadTemplate(picked, false)) return;
    sheet = null;
    message = `Loaded the ${picked.name} template. Press Done to save.`;
  }

  function addCardTo(sectionId: string) {
    message = "";
    sheet = { kind: "card-add", sectionId };
  }

  // An empty dashboard has no section yet: the first card gets a new one.
  function addCardToEmpty() {
    message = "";
    if (!editing) store.startEdit();
    if (sections.length === 0 && !fail(store.addSection(""))) return;
    sheet = { kind: "card-add", sectionId: sections[0]?.id ?? "" };
  }

  // Tabs: a new dashboard or a rename needs edit mode, so the plus starts it.
  function newDashboard() {
    if (!editing) store.startEdit();
    message = "";
    sheet = { kind: "new-dashboard" };
  }

  function removeDashboard() {
    confirmDelete = false;
    fail(store.removeDashboard(active.id));
  }

  // The pencil on the selected tab opens the edit dialog. Name changes apply on Save; reset and delete ask first.
  function openEdit() {
    message = "";
    editName = active.name;
    editError = "";
    sheet = { kind: "edit" };
  }

  function saveName(e: SubmitEvent) {
    e.preventDefault();
    const err = store.renameDashboard(active.id, editName);
    if (err) editError = err;
    else sheet = null;
  }

  function askReset() {
    sheet = null;
    confirmReset = true;
  }

  function askDelete() {
    sheet = null;
    confirmDelete = true;
  }

  // A drag ends in one zone; the zone's final list is applied card by card. Cards that left another zone are moved here.
  function locate(cardId: string): { sid: string; index: number } | null {
    for (const s of sections) {
      const index = s.cards.findIndex((c) => c.id === cardId);
      if (index >= 0) return { sid: s.id, index };
    }
    return null;
  }
  function commitZone(sid: string, items: Card[]) {
    for (const [i, card] of items.entries()) {
      const at = locate(card.id);
      if (!at || (at.sid === sid && at.index === i)) continue;
      if (!fail(store.moveCard(card.id, sid, i))) return;
    }
  }

  function findCard(cardId: string): Card | undefined {
    return sections.flatMap((s) => s.cards).find((c) => c.id === cardId);
  }

  const sheetCard = $derived(sheet && (sheet.kind === "card-edit" || sheet.kind === "size") ? findCard(sheet.cardId) : undefined);
  const sheetSections = $derived(refs.map((r) => ({ id: r.id, label: r.label })));

  function resizeCard(card: Card, w: number, h: number) {
    fail(store.updateCard(card.id, { type: card.type, style: card.style, pids: card.pids, w, h, thresholds: card.thresholds, items: card.items, range: card.range }));
  }
</script>

<div class="rv-wrap db-page">
  {#if !fresh}
    <ViewHeader title="Dashboard">
      {#snippet info()}
        {#if editing}
          <SidebarHint inline text={editHelp} label="About edit mode" />
        {/if}
      {/snippet}
      {#snippet status()}
        {#if statusLine}<p class="db-status">{statusLine}</p>{/if}
        {#if view.save.sd === "failed"}
          <p class="db-status db-status-save" role="status">The SD card write failed: {view.save.message ?? "unknown error"}</p>
        {/if}
      {/snippet}

      {#snippet meta()}
        <div class="db-meta-row">
          <DashboardTabs
            dashboards={view.dashboards}
            activeId={active.id}
            onSelect={(id) => {
              message = "";
              confirmDelete = false;
              store.setActive(id);
            }}
            onAdd={newDashboard}
            {editing}
            onEdit={openEdit}
          />
          {#if editing && !phone}
            <button type="button" class="rv-btn" onclick={cancel}>Cancel</button>
            <button type="button" class="rv-btn pri" onclick={done} disabled={saving}>Done</button>
          {/if}
          {#if !editing}
            <button type="button" class="db-edit" aria-label="Edit dashboard" title="Edit dashboard" onclick={startEdit}><Icon name="edit" size={22} /></button>
          {/if}
        </div>
      {/snippet}

    </ViewHeader>
  {:else}
    <ViewHeader title="Dashboard">
      {#snippet status()}
        <p class="db-status">{statusLine}</p>
      {/snippet}
    </ViewHeader>
  {/if}

  {#if editing && phone}
    <div class="db-bar" role="toolbar" aria-label="Edit mode">
      <button type="button" class="rv-btn" onclick={cancel}>Cancel</button>
      <button type="button" class="rv-btn pri" onclick={done} disabled={saving}>Done</button>
    </div>
  {/if}

  {#if message}
    <p class="db-error db-msg" role="alert">{message}</p>
  {/if}

  {#if view.loading}
    <section class="db-panel" role="status">
      <h2><span class="db-spin" aria-hidden="true"></span> Loading your dashboard</h2>
      <p>Reading the layout from the SD card.</p>
    </section>
  {:else if fresh}
    <section class="db-panel" aria-labelledby="db-start-h">
      <h2 id="db-start-h">Start with a layout</h2>
      <TemplatePicker value={pickId} onChange={(id) => (pickId = id)} />
      <div class="db-panel-foot">
        <p class="db-hint">Reset to template in Edit brings a layout back.</p>
        <button type="button" class="rv-btn pri" onclick={startWith} disabled={saving}>Start with {picked.name}</button>
      </div>
    </section>
  {:else if sections.length === 0}
    {#if view.unreadable}
      {@render unreadable()}
    {/if}
    <section class="db-panel" aria-labelledby="db-empty-h">
      <h2 id="db-empty-h">This dashboard is empty</h2>
      <p>Add a card to start, or begin from a template.</p>
      <div class="db-acts">
        <button type="button" class="rv-btn pri" onclick={addCardToEmpty}>
          <Icon name="plus" size={18} />Add card
        </button>
        <button type="button" class="rv-btn" onclick={() => (sheet = { kind: "template" })}>Use a template</button>
      </div>
    </section>
  {:else}
    {#if view.unreadable}
      {@render unreadable()}
    {/if}
    <div class="db-sections">
      {#each sections as s, i (s.id)}
        <SectionView
          section={s}
          index={i}
          count={sections.length}
          sections={refs}
          {editing}
          {phone}
          onDrop={commitZone}
          onAdd={addCardTo}
          onEdit={(cardId) => {
            message = "";
            sheet = { kind: "card-edit", cardId };
          }}
          onSize={(cardId) => (sheet = { kind: "size", cardId })}
          onError={(m) => (message = m)}
        />
      {/each}
    </div>
    {#if editing}
      <button type="button" class="db-add-section" onclick={() => fail(store.addSection(""))}>
        <Icon name="plus" size={18} />Add section
      </button>
    {/if}
  {/if}

</div>

{#snippet unreadable()}
  <section class="db-panel db-warn" role="alert">
    <span class="db-chip">Layout file unreadable</span>
    <p>The SD card copy of the dashboards could not be read, so this is the copy saved in this browser. Your changes are saved here until the SD card works again.</p>
    <div><button type="button" class="rv-btn" onclick={() => void store.sync()}>Try again</button></div>
  </section>
{/snippet}

{#if sheet?.kind === "card-add"}
  <CardSheet sections={sheetSections} sectionId={sheet.sectionId} onClose={() => (sheet = null)} />
{:else if sheet?.kind === "card-edit" && sheetCard}
  <CardSheet card={sheetCard} sections={sheetSections} onClose={() => (sheet = null)} />
{:else if sheet?.kind === "size" && sheetCard}
  {@const target = sheetCard}
  <Sheet open title="Card size" subtitle="Whole columns and rows, inside the card's range." onClose={() => (sheet = null)}>
    <div class="db-form">
      <SizePicker min={sizeLimits(target).min} max={sizeLimits(target).max} w={target.w} h={target.h} onChange={(w, h) => resizeCard(target, w, h)} />
    </div>
  </Sheet>
{:else if sheet?.kind === "new-dashboard"}
  <NewDashboardSheet onSave={(name, template) => store.addDashboard(name, template)} onClose={() => (sheet = null)} />
{:else if sheet?.kind === "edit"}
  <Sheet open title="Edit dashboard" subtitle="Nothing is saved until you press Done." onClose={() => (sheet = null)}>
    <form id="db-edit-form" class="db-form" onsubmit={saveName}>
      <label class="db-field">
        <span class="db-label">Name</span>
        <input class="db-input" type="text" maxlength="40" autocomplete="off" bind:value={editName} />
      </label>
      {#if editError}
        <p class="db-error" role="alert">{editError}</p>
      {/if}
    </form>
    <div class="db-edit-acts">
      <button type="button" class="rv-btn" onclick={askReset}>Reset to template</button>
      <button type="button" class="rv-btn db-danger" onclick={askDelete} disabled={view.dashboards.length <= 1}>Delete dashboard</button>
    </div>
    {#if view.dashboards.length <= 1}
      <p class="db-hint">Add another dashboard before you can delete this one.</p>
    {/if}
    {#snippet footer()}
      <button type="button" class="rv-btn" onclick={() => (sheet = null)}>Cancel</button>
      <button type="submit" form="db-edit-form" class="rv-btn pri">Save</button>
    {/snippet}
  </Sheet>
{:else if sheet?.kind === "template"}
  <Sheet open title="Use a template" subtitle="Replaces the sections and cards on this dashboard. Press Done to save." onClose={() => (sheet = null)}>
    <div class="db-form">
      <TemplatePicker value={pickId} onChange={(id) => (pickId = id)} />
    </div>
    {#snippet footer()}
      <button type="button" class="rv-btn" onclick={() => (sheet = null)}>Cancel</button>
      <button type="button" class="rv-btn pri" onclick={useTemplate}>Use {picked.name}</button>
    {/snippet}
  </Sheet>
{/if}

<Dialog
  open={confirmReset}
  title={`Reset "${active.name}"?`}
  message={`Replaces every section and card on this dashboard with the ${templateForName(active.name).name} template. Other dashboards are kept. Nothing is saved until you press Done.`}
  confirmLabel="Reset"
  danger
  onConfirm={resetDashboard}
  onCancel={() => (confirmReset = false)}
/>

<Dialog
  open={confirmDelete}
  title={`Delete "${active.name}"?`}
  message="Deletes this dashboard with its sections and cards. Other dashboards are kept. Nothing is saved until you press Done."
  confirmLabel="Delete"
  danger
  onConfirm={removeDashboard}
  onCancel={() => (confirmDelete = false)}
/>

<style>
  /* The rename field sits under the tabs in edit mode, so the row grows; the buttons keep the tab strip's 52 px. */
  .db-meta-row {
    display: flex;
    align-items: flex-start;
    min-height: 52px;
    gap: 8px;
    min-width: 0;
  }
  .db-meta-row > :global(:first-child) {
    flex: 1 1 auto;
    min-width: 0;
  }
  /* The select-menu root is block level; a fixed box keeps it from taking the row's width. */
  .db-edit {
    flex: none;
    display: grid;
    place-items: center;
    width: 52px;
    height: 52px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: 16px;
    background: var(--ctl);
    color: var(--ctl-text);
    cursor: pointer;
  }
  .db-edit:hover {
    background: var(--ctl-h2);
  }
  .db-edit-acts {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 10px;
    margin: 16px 0 20px;
  }
  .db-edit-acts > button {
    width: 100%;
    min-width: 0;
    margin: 0;
    padding: 0 12px;
  }
  .db-edit-acts .db-danger:not(:disabled) {
    color: var(--crit);
  }
  .db-meta-row :global(.rv-btn) {
    margin: 0;
    height: 52px;
    min-height: 0;
  }
  .db-page {
    gap: 16px;
  }
  .db-status {
    margin: 0;
    color: var(--muted);
    font-size: 0.95rem;
  }
  .db-status-save {
    font-weight: 600;
    color: var(--crit);
  }
  .db-msg {
    margin: 0;
  }
  .db-sections {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(min(100%, 340px), 1fr));
    gap: 20px;
    align-items: start;
    min-width: 0;
  }
  .db-add-section {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    min-height: 64px;
    width: 100%;
    max-width: 370px;
    border: 0;
    border-radius: 24px;
    background: var(--ctl);
    color: var(--ctl-text);
    font: 800 1rem/1 var(--font);
    cursor: pointer;
  }
  .db-add-section:hover {
    background: var(--ctl-h2);
  }
  .db-panel {
    display: grid;
    gap: 14px;
    padding: var(--pad-card);
    background: var(--card);
    border-radius: var(--r-card);
  }
  .db-panel h2 {
    margin: 0;
    color: var(--text);
    font-size: 1.15rem;
    font-weight: 700;
  }
  .db-panel p {
    margin: 0;
    color: var(--muted);
  }
  .db-warn p {
    max-width: 60ch;
  }
  .db-chip {
    display: inline-flex;
    align-self: start;
    align-items: center;
    gap: 6px;
    padding: 2px 11px 2px 9px;
    border-radius: 99px;
    background: var(--warn-f);
    color: var(--warn);
    font-size: 0.8rem;
    font-weight: 800;
  }
  .db-spin {
    display: inline-block;
    width: 14px;
    height: 14px;
    border: 2px solid var(--ctl);
    border-top-color: var(--accent);
    border-radius: 50%;
    animation: db-spin 0.9s linear infinite;
  }
  @keyframes db-spin {
    to {
      transform: rotate(360deg);
    }
  }
  .db-panel-foot {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
  }
  .db-acts {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
  }
  .db-bar {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
  }
  .db-bar .rv-btn {
    min-height: 48px;
  }
  /* Phone: the menu sits on the title row, top right, instead of a row of its own under the tabs. */
  @media (max-width: 699px) {
    .db-page :global(.rv-head) {
      position: relative;
    }
    .db-page :global(.rv-actions) {
      position: absolute;
      top: var(--pad-card);
      right: var(--pad-card);
    }
    .db-page :global(.rv-head-main h1) {
      padding-right: 52px;
    }
  }
  .db-form {
    display: grid;
    gap: 16px;
    padding: 4px 0 8px;
  }
  @media (prefers-reduced-motion: reduce) {
    .db-spin {
      animation: none;
    }
  }
</style>
