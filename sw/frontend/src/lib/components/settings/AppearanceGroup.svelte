<script lang="ts">
  import { ACCENTS, CONTROL_FILLS, THEMES, nextOption, type ControlFill, type Theme } from "$lib/appearance";
  import { appearance, setAppearance } from "$lib/appearance.svelte";
  import { GROUP, ROW, rowShown, type RowMeta } from "$lib/settingsModel";
  import SegControl from "./SegControl.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.look, row);

  const themeOptions: { value: Theme; label: string }[] = THEMES.map((t) => ({ value: t, label: t[0]!.toUpperCase() + t.slice(1) }));
  const fillOptions: { value: ControlFill; label: string }[] = CONTROL_FILLS.map((c) => ({ value: c, label: c[0]!.toUpperCase() + c.slice(1) }));
  const accentLabel = $derived(ACCENTS.find((a) => a.id === appearance.accent)?.label ?? "Orange");

  function swatchKey(e: KeyboardEvent, id: string) {
    const next = nextOption(
      ACCENTS.map((a) => a.id),
      id as (typeof ACCENTS)[number]["id"],
      e.key,
    );
    if (next === undefined) return;
    e.preventDefault();
    setAppearance("accent", next);
    (e.currentTarget as HTMLElement).parentElement?.querySelector<HTMLElement>(`[data-value="${next}"]`)?.focus();
  }
</script>

<div class="st-sec">
  <h2>Theme and colour</h2>
  <div class="st-list">
    {#if show(ROW.theme)}
      <SettingRow title={ROW.theme.title} hint="System follows your device setting.">
        <SegControl label="Theme" options={themeOptions} value={appearance.theme} onChange={(v) => setAppearance("theme", v)} />
      </SettingRow>
    {/if}
    {#if show(ROW.accent)}
      <SettingRow title={ROW.accent.title} hint={`${accentLabel}. Status colours stay fixed.`} stack>
        <div class="st-sws" role="radiogroup" aria-label="Accent">
          {#each ACCENTS as a (a.id)}
            <button
              type="button"
              class="st-sw"
              role="radio"
              data-value={a.id}
              aria-label={a.label}
              aria-checked={appearance.accent === a.id}
              tabindex={appearance.accent === a.id ? 0 : -1}
              style="--h: {a.hue}; --c: {a.chroma}"
              onclick={() => setAppearance("accent", a.id)}
              onkeydown={(e) => swatchKey(e, a.id)}
            >
              <!-- Glyph bounds (x 4-20, y 6-17) fitted to the box, so the tick is centred, not the 24-unit icon box. -->
              <svg class="st-tick" viewBox="2.5 4.5 19 14" overflow="visible" aria-hidden="true"><polyline points="20 6 9 17 4 12" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round" /></svg>
            </button>
          {/each}
        </div>
      </SettingRow>
    {/if}
    {#if show(ROW.ctl)}
      <SettingRow title={ROW.ctl.title} hint="Strong gives buttons and fields a darker fill for more contrast.">
        <SegControl label="Control fill" options={fillOptions} value={appearance.ctl} onChange={(v) => setAppearance("ctl", v)} />
      </SettingRow>
    {/if}
  </div>
</div>
