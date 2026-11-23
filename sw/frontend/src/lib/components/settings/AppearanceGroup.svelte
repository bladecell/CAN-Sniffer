<script lang="ts">
  import { ACCENTS, BACKGROUNDS, BG_VAR_MAX, BG_VAR_MIN, CONTROL_FILLS, GLASS_MAX, GLASS_MIN, THEMES, nextOption, type ControlFill, type Theme } from "$lib/appearance";
  import { appearance, setAppearance } from "$lib/appearance.svelte";
  import { GROUP, ROW, rowShown, type RowMeta } from "$lib/settingsModel";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import RangeSlider from "$lib/components/RangeSlider.svelte";
  import SegControl from "./SegControl.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.look, row);

  const themeOptions: { value: Theme; label: string }[] = THEMES.map((t) => ({ value: t, label: t[0]!.toUpperCase() + t.slice(1) }));
  const fillOptions: { value: ControlFill; label: string }[] = CONTROL_FILLS.map((c) => ({ value: c, label: c[0]!.toUpperCase() + c.slice(1) }));
  const accentLabel = $derived(ACCENTS.find((a) => a.id === appearance.accent)?.label ?? "Orange");

  // The first stretch of the track is a hatched stop for 0 (Random); the rest runs 1 to 100.
  const BG_STOP = 0.1;
  const bgVarToPos = (v: number) => (v <= 0 ? 0 : BG_STOP + ((v - 1) / (BG_VAR_MAX - 1)) * (1 - BG_STOP));
  const posToBgVar = (x: number) => (x < BG_STOP ? 0 : Math.round(1 + ((x - BG_STOP) / (1 - BG_STOP)) * (BG_VAR_MAX - 1)));
  const bgOptions = BACKGROUNDS.map((b) => ({ value: b.id, label: b.label }));

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
      <SettingRow title={ROW.accent.title} hint={`${accentLabel}. Status colours stay fixed.`}>
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
    {#if show(ROW.glass)}
      <SettingRow title={ROW.glass.title} hint="Lower is more solid. Cards change the most, menus and dialogs stay readable.">
        <span class="sr-only" id="glass-label">{ROW.glass.title}</span>
        <RangeSlider
          labelIds={["glass-label"]}
          values={[appearance.glass]}
          min={GLASS_MIN}
          max={GLASS_MAX}
          step={1}
          keyStep={5}
          page={20}
          fit
          parse={(t) => (/^\d{1,3}\s*%?$/.test(t.trim()) ? Number(t.replace(/\D/g, "")) : null)}
          format={(v) => `${v}%`}
          onChange={([v]) => setAppearance("glass", v)}
        />
      </SettingRow>
    {/if}
    {#if show(ROW.background)}
      <SettingRow title={ROW.background.title} hint="A generated pattern behind the app, drawn in your accent colour.">
        <SelectMenu options={bgOptions} value={appearance.bg} ariaLabel={ROW.background.title} onChange={(v) => setAppearance("bg", v)} />
      </SettingRow>
    {/if}
    {#if appearance.bg === "contour" && show(ROW.bgVariation)}
      <SettingRow title={ROW.bgVariation.title} hint="Slide to reshape the lines. The hatched start of the slider is Random, a new pattern each day. Line count follows your screen width.">
        <span class="sr-only" id="bgvar-label">{ROW.bgVariation.title}</span>
        <RangeSlider
          labelIds={["bgvar-label"]}
          values={[appearance.bgVar]}
          min={BG_VAR_MIN}
          max={BG_VAR_MAX}
          keyStep={1}
          page={10}
          fit
          readouts={false}
          toPos={bgVarToPos}
          fromPos={posToBgVar}
          format={(v) => (v === 0 ? "Random" : String(v))}
          valueText={(v) => (v === 0 ? "Random, a new pattern each day" : `Pattern ${v}`)}
          stop={{ side: "left", share: BG_STOP, on: appearance.bgVar === 0 }}
          onChange={([v]) => setAppearance("bgVar", v)}
        />
      </SettingRow>
    {/if}
  </div>
</div>
