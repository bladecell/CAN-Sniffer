<script lang="ts">
  import { PRIORITY_LABELS, levelToPriority, priorityToLevel } from "$lib/editor/pollRate";

  /** Five queue levels, highest on the left. Only the chosen level shows its name, so no numbers appear. Saved as 2 to 6. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: number | undefined;
    disabled?: boolean;
  }

  let { id, labelId, describedBy, value = $bindable(), disabled = false }: Props = $props();

  const level = $derived(priorityToLevel(value));

  // A press on the level already chosen writes nothing, so a stored 0, 1 or above 6 is only rewritten by a real change.
  function pick(index: number) {
    if (disabled || index === level) return;
    value = levelToPriority(Math.min(PRIORITY_LABELS.length - 1, Math.max(0, index)));
  }

  function onKey(e: KeyboardEvent) {
    if (disabled) return;
    const moves: Record<string, number> = {
      ArrowRight: level + 1,
      ArrowUp: level + 1,
      ArrowLeft: level - 1,
      ArrowDown: level - 1,
      Home: 0,
      End: PRIORITY_LABELS.length - 1,
    };
    const next = moves[e.key];
    if (next !== undefined) {
      e.preventDefault();
      pick(next);
    }
  }

  // A press picks the cell under the pointer as drawn. Nothing else reacts to the pointer (no click, no drag),
  // so the cells growing after the press cannot redirect it.
  function onDown(e: PointerEvent) {
    const cell = (e.target as HTMLElement).closest<HTMLElement>("[data-i]");
    if (e.button === 0 && cell) pick(Number(cell.dataset.i));
  }
</script>

<div
  {id}
  class="pick"
  class:off={disabled}
  role="slider"
  tabindex={disabled ? -1 : 0}
  aria-labelledby={labelId}
  aria-describedby={describedBy}
  aria-valuemin={1}
  aria-valuemax={PRIORITY_LABELS.length}
  aria-valuenow={level + 1}
  aria-valuetext={PRIORITY_LABELS[level]}
  aria-disabled={disabled}
  onkeydown={onKey}
  onpointerdown={onDown}
>
  {#each PRIORITY_LABELS as label, i (label)}
    <span class="cell" data-k={i < level ? "ahead" : i === level ? "sel" : "behind"} data-i={i}>
      {#if i === level}<span class="name">{label}</span>{/if}
    </span>
  {/each}
</div>

<style>
  .pick {
    position: relative;
    display: flex;
    gap: 4px;
    box-sizing: border-box;
    height: var(--ctl-h);
    padding: 4px;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    touch-action: none;
    user-select: none;
    cursor: pointer;
  }
  .pick:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .cell {
    flex: 1 1 0;
    min-width: 44px;
    display: grid;
    place-items: center;
    border-radius: calc(var(--r-ctl) - 4px);
    font-weight: 800;
    font-size: 0.9375rem;
    white-space: nowrap;
    overflow: hidden;
    transition: flex-grow 0.16s;
  }
  .cell[data-k="ahead"] {
    background: var(--ctl-sub);
  }
  .cell[data-k="behind"] {
    background: transparent;
    box-shadow: inset 0 0 0 2px var(--ctl-sub);
    opacity: 0.7;
  }
  .cell[data-k="sel"] {
    flex-grow: 2.6;
    background: var(--accent);
    color: var(--on-accent);
  }
  .cell::before {
    content: "";
    width: 14px;
    height: 8px;
    border-radius: 4px;
    background: currentColor;
    opacity: 0.5;
  }
  .cell[data-k="sel"]::before {
    display: none;
  }
  .name {
    padding: 0 4px;
  }
  .pick.off {
    opacity: 0.55;
    cursor: not-allowed;
  }
  .pick.off .cell {
    cursor: not-allowed;
  }
  @media (prefers-reduced-motion: reduce) {
    .cell {
      transition: none;
    }
  }
</style>
