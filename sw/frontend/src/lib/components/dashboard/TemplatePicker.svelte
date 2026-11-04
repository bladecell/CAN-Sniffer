<script lang="ts">
  import { TEMPLATES, type DashboardTemplate, type TemplateId } from "$lib/dashboardTemplates";

  interface Props {
    value: TemplateId;
    onChange: (id: TemplateId) => void;
  }

  let { value, onChange }: Props = $props();

  // Arrow keys move the choice, as in any radio group.
  function onKey(e: KeyboardEvent, index: number) {
    const dir = e.key === "ArrowRight" || e.key === "ArrowDown" ? 1 : e.key === "ArrowLeft" || e.key === "ArrowUp" ? -1 : 0;
    if (!dir) return;
    e.preventDefault();
    const next = TEMPLATES[(index + dir + TEMPLATES.length) % TEMPLATES.length];
    onChange(next.id);
    (e.currentTarget as HTMLElement).parentElement?.querySelector<HTMLElement>(`[data-value="${next.id}"]`)?.focus();
  }

  const shape = (t: DashboardTemplate) => t.sections.map((s) => s.cards);
</script>

<div class="tp" role="radiogroup" aria-label="Starting layout">
  {#each TEMPLATES as t, i (t.id)}
    <button
      type="button"
      role="radio"
      class="tp-tile"
      data-value={t.id}
      aria-checked={value === t.id}
      tabindex={value === t.id ? 0 : -1}
      onclick={() => onChange(t.id)}
      onkeydown={(e) => onKey(e, i)}
    >
      <span class="tp-mini" aria-hidden="true">
        {#each shape(t) as cards, si (si)}
          <span class="tp-sec">
            {#each cards as c, ci (ci)}
              <i style:grid-column="span {c.w}" style:grid-row="span {c.h}"></i>
            {/each}
          </span>
        {/each}
      </span>
      <span class="tp-name">{t.name}{#if t.id === TEMPLATES[0].id}<span class="tp-tag">Default</span>{/if}</span>
      <span class="tp-sum">{t.summary}</span>
    </button>
  {/each}
</div>

<style>
  .tp {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(min(100%, 200px), 1fr));
    gap: 10px;
  }
  .tp-tile {
    display: grid;
    gap: 8px;
    align-content: start;
    min-height: 44px;
    padding: 12px;
    border: 0;
    border-radius: 18px;
    background: var(--ctl);
    color: var(--ctl-text);
    text-align: left;
    cursor: pointer;
  }
  .tp-tile[aria-checked="true"] {
    background: var(--float-solid);
    box-shadow: inset 0 -3px 0 var(--accent);
  }
  .tp-tile:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .tp-mini {
    display: grid;
    gap: 4px;
    min-width: 0;
  }
  .tp-sec {
    display: grid;
    grid-template-columns: repeat(6, minmax(0, 1fr));
    grid-auto-rows: 6px;
    gap: 2px;
  }
  .tp-sec i {
    border-radius: 3px;
    background: var(--track);
    opacity: 0.55;
  }
  .tp-name {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 800;
    font-size: 0.95rem;
  }
  .tp-tag {
    padding: 0 8px;
    border-radius: 99px;
    background: var(--ctl);
    color: var(--ctl-sub);
    font-size: 0.72rem;
    font-weight: 800;
    line-height: 1.7;
  }
  .tp-sum {
    color: var(--muted);
    font-size: 0.85rem;
    font-weight: 600;
    line-height: 1.35;
  }
</style>
