<script lang="ts">
  import { tick } from "svelte";
  import { dragHandle } from "svelte-dnd-action";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import type { SelectMenuOption } from "$lib/selectMenu";
  import { dashboardDocStore } from "$lib/dashboardDocStore.svelte";
  import { clampSize, type Card } from "$lib/dashboardModel";
  import type { CardInput } from "$lib/dashboardOps";
  import { columnPitch, keyResize, resizeTo, stepIndex, type SectionRef } from "$lib/dashboardGrid";
  import { editFaceName, TYPE_LABELS } from "$lib/dashboardCardForm";
  import { vehicleStore } from "$lib/vehicleStore.svelte";
  import { cardBodies } from "./cardRegistry";

  interface Props {
    card: Card;
    editing: boolean;
    phone: boolean;
    /** Where the card sits now: its section, its index in that section, and every section (for Move to and Shift + arrows). */
    sectionId: string;
    index: number;
    sectionIndex: number;
    sections: SectionRef[];
    onEdit: (cardId: string) => void;
    onSize: (cardId: string) => void;
    onError: (message: string) => void;
  }

  let { card, editing, phone, sectionId, index, sectionIndex, sections, onEdit, onSize, onError }: Props = $props();

  const Body = $derived(cardBodies[card.type]);
  const label = $derived(TYPE_LABELS[card.type]);
  const plural = (n: number, word: string) => `${n} ${word}${n === 1 ? "" : "s"}`;
  // Spoken after a keyboard move or resize. Screen readers get no other sign that the key did anything.
  let announcement = $state("");
  const clamp = (w: number, h: number) => clampSize(card, w, h);

  // The card's own fields, with a new size. Everything else is kept as it is.
  function withSize(w: number, h: number): CardInput {
    return { type: card.type, style: card.style, pids: card.pids, w, h, thresholds: card.thresholds, items: card.items, range: card.range };
  }

  // Edit mode draws a face (name, type and size) instead of the live card, as in the mockup. The names need the PID list.
  $effect(() => {
    if (editing && vehicleStore.pidDefinitions.length === 0) void vehicleStore.loadDefinitions();
  });
  const faceName = $derived(editFaceName(card, (pid) => vehicleStore.pidDefinitions.find((d) => d.pid === pid)?.name));

  function resize(w: number, h: number) {
    if (w === card.w && h === card.h) return;
    const err = dashboardDocStore.updateCard(card.id, withSize(w, h));
    if (err) onError(err);
  }

  // Pointer resize: snaps to whole columns and rows from the grid's own width.
  let drag: { x: number; y: number; w: number; h: number; pitch: number } | null = null;
  function resizeDown(e: PointerEvent) {
    const handle = e.currentTarget as HTMLElement;
    handle.setPointerCapture(e.pointerId);
    const grid = handle.closest<HTMLElement>(".db-grid");
    drag = { x: e.clientX, y: e.clientY, w: card.w, h: card.h, pitch: columnPitch(grid?.clientWidth ?? 0) };
  }
  function resizeMove(e: PointerEvent) {
    if (!drag) return;
    const next = resizeTo(clamp, { w: drag.w, h: drag.h }, e.clientX - drag.x, e.clientY - drag.y, drag.pitch);
    resize(next.w, next.h);
  }
  function resizeUp() {
    drag = null;
  }
  function resizeKey(e: KeyboardEvent) {
    const next = keyResize(clamp, { w: card.w, h: card.h }, e.key);
    if (!next) return;
    e.preventDefault();
    resize(next.w, next.h);
    announcement = `${faceName} is now ${plural(next.w, "column")} by ${plural(next.h, "row")}.`;
  }

  // Keyboard move: arrows reorder inside the section; Shift with up or down moves the card to the end of the next section.
  async function moveTo(sid: string, at: number) {
    const err = dashboardDocStore.moveCard(card.id, sid, at);
    if (err) {
      onError(err);
      return;
    }
    announcement = `${faceName} moved to position ${at + 1} in ${sections.find((s) => s.id === sid)?.label ?? "the section"}.`;
    await tick();
    document.querySelector<HTMLElement>(`[data-card="${card.id}"] .db-grip`)?.focus();
  }
  function gripKey(e: KeyboardEvent) {
    // Space and Enter belong to the library's own pick-up mode, which this grip does not use.
    if (e.key === " " || e.key === "Enter") {
      e.preventDefault();
      e.stopPropagation();
      return;
    }
    const vertical = e.key === "ArrowUp" || e.key === "ArrowDown";
    const dir = e.key === "ArrowUp" || e.key === "ArrowLeft" ? -1 : e.key === "ArrowDown" || e.key === "ArrowRight" ? 1 : 0;
    if (!dir) return;
    e.preventDefault();
    e.stopPropagation();
    if (e.shiftKey && vertical) {
      const next = sections[sectionIndex + dir];
      if (next) void moveTo(next.id, next.count);
      return;
    }
    const at = stepIndex(index, sections[sectionIndex]?.count ?? 0, dir);
    if (at !== null) void moveTo(sectionId, at);
  }

  function duplicate() {
    const err = dashboardDocStore.duplicateCard(card.id);
    if (err) onError(err);
  }
  function remove() {
    const err = dashboardDocStore.removeCard(card.id);
    if (err) onError(err);
  }

  type CardAction = "edit" | "duplicate" | "size" | "remove" | `move:${string}`;
  const menuOptions = $derived<SelectMenuOption<CardAction>[]>([
    { value: "edit", label: "Edit" },
    { value: "duplicate", label: "Duplicate" },
    // Phones and one-row cards have no corner handle; the size sheet replaces it.
    ...(phone || card.h === 1 ? [{ value: "size" as CardAction, label: "Size..." }] : []),
    ...sections
      .filter((s) => s.id !== sectionId)
      .map((s) => ({ value: `move:${s.id}` as CardAction, label: `Move to ${s.label}` })),
    { value: "remove", label: "Remove", danger: true },
  ]);

  function onAction(action: CardAction) {
    if (action === "edit") onEdit(card.id);
    else if (action === "duplicate") duplicate();
    else if (action === "size") onSize(card.id);
    else if (action === "remove") remove();
    else {
      const target = sections.find((s) => `move:${s.id}` === action);
      if (target) void moveTo(target.id, target.count);
    }
  }
</script>

<div class="db-card" class:edit={editing} class:short={editing && card.h === 1} data-card={card.id} data-type={card.type} style:grid-column="span {card.w}" style:grid-row="span {card.h}">
  <div class="db-body">
    {#if editing}
      <div class="db-face">
        <strong>{faceName}</strong>
        <span>{phone ? label : `${label} · ${card.w} × ${card.h}`}</span>
      </div>
    {:else}
      <Body {card} />
    {/if}
  </div>

  {#if editing}
    <span class="db-live" role="status" aria-live="polite">{announcement}</span>
    <button
      type="button"
      class="db-grip"
      use:dragHandle
      aria-label="Move {faceName}. Arrow keys move it in the section. Shift with up or down moves it to the next section."
      onkeydown={gripKey}
    >
      <svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true">
        <circle cx="9" cy="6" r="1.8" /><circle cx="15" cy="6" r="1.8" />
        <circle cx="9" cy="12" r="1.8" /><circle cx="15" cy="12" r="1.8" />
        <circle cx="9" cy="18" r="1.8" /><circle cx="15" cy="18" r="1.8" />
      </svg>
    </button>

    <div class="db-menu-wrap">
      <SelectMenu
        trigger="kebab"
        options={menuOptions}
        ariaLabel="Card menu, {faceName}"
        onChange={onAction}
      />
    </div>

    {#if card.h === 1}
      <!-- One-row cards drop the size handle (mockup); Size... is in the card menu. -->
    {:else if phone}
      <button type="button" class="db-sizechip" onclick={() => onSize(card.id)} aria-label="Size of {faceName}: {plural(card.w, "column")} by {plural(card.h, "row")}. Change">{card.w} × {card.h}</button>
    {:else}
      <button
        type="button"
        class="db-rz"
        aria-label="Resize {faceName}. Now {plural(card.w, "column")} by {plural(card.h, "row")}. Drag, or use arrow keys."
        onpointerdown={resizeDown}
        onpointermove={resizeMove}
        onpointerup={resizeUp}
        onpointercancel={resizeUp}
        onkeydown={resizeKey}
      >
        <svg viewBox="0 0 24 24" width="24" height="24" aria-hidden="true"><path d="M20 9 9 20M20 15l-5 5" /></svg>
      </button>
    {/if}
  {/if}
</div>

<style>
  .db-card {
    position: relative;
    display: flex;
    flex-direction: column;
    min-width: 0;
    border-radius: 22px;
    background: var(--card);
  }
  .db-card:not(.edit) {
    overflow: hidden;
  }
  .db-body {
    flex: 1;
    min-width: 0;
    min-height: 0;
    overflow: hidden;
    padding-top: 0;
  }
  /* Solid, so the column guides of the grid behind do not show through the card. */
  .db-card.edit {
    background: var(--float-solid);
    outline: 1px solid var(--hair);
    outline-offset: -1px;
  }
  .db-face {
    display: flex;
    flex-direction: column;
    justify-content: flex-end;
    gap: 1px;
    height: 100%;
    min-width: 0;
    padding: 44px 48px 8px 12px;
    color: var(--text);
    font-size: 0.8rem;
    line-height: 1.25;
  }
  .db-face strong {
    overflow: hidden;
    font-weight: 800;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .db-face span {
    overflow: hidden;
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  /* One-row cards put the handles at the sides, on one line with the name (mockup: edit face, one row). */
  .db-card.short .db-face {
    justify-content: center;
    padding: 0 44px;
  }
  .db-card.short .db-face span {
    display: none;
  }

  .db-card.short .db-grip,
  .db-card.short .db-menu-wrap {
    top: 50%;
    transform: translateY(-50%);
  }
  .db-card.short .db-grip {
    left: 0;
  }
  .db-card.short .db-menu-wrap {
    right: 0;
  }
  .db-live {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip-path: inset(50%);
    white-space: nowrap;
  }
  .db-grip,
  .db-rz,
  .db-sizechip {
    position: absolute;
    touch-action: none;
    user-select: none;
    -webkit-user-select: none;
    font: inherit;
    color: var(--muted);
  }
  .db-grip,
  .db-rz {
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    margin: 0;
    padding: 0;
    border: 0;
    background: none;
  }
  .db-grip {
    top: 0;
    left: 0;
    cursor: grab;
  }
  .db-menu-wrap {
    position: absolute;
    top: 0;
    right: 0;
  }
  .db-rz {
    right: 0;
    bottom: 0;
    cursor: nwse-resize;
  }
  .db-grip:focus-visible,
  .db-rz:focus-visible,
  .db-sizechip:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
  }
  .db-grip svg circle {
    fill: currentColor;
  }
  .db-rz svg path {
    fill: none;
    stroke: currentColor;
    stroke-width: 2.5;
    stroke-linecap: round;
  }
  .db-sizechip {
    right: 6px;
    bottom: 6px;
    min-height: 36px;
    min-width: 44px;
    padding: 0 10px;
    border: 0;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--ctl-text);
    font-size: 0.8rem;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    cursor: pointer;
  }
</style>
