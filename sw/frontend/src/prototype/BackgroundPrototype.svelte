<script lang="ts">
  import { contourSvg, type ContourOpts } from './contour';
  import { dashWaveSvg } from './dashwave';

  const PATTERNS = { contour: contourSvg, 'dash wave': dashWaveSvg };
  let pattern = $state<keyof typeof PATTERNS>('contour');

  // One user-facing control: `variation` morphs the pattern. Everything else derives from the screen.
  let variation = $state(12);
  // Sized from the physical screen, never the window: resizing the window only crops the pattern.
  const side = Math.max(screen.width, screen.height), short = Math.min(screen.width, screen.height);
  let advanced = $state(false);
  let tune = $state({ seed: 7, scaleMul: 0.28, octaves: 3 });

  // Colours come from the app's accent (--ah hue, --ac chroma) and the active theme.
  let accent = $state('orange');
  let theme = $state<'light' | 'dark'>(matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
  let hue = $state(55), chroma = $state(0.17);
  $effect(() => {
    document.documentElement.dataset.accent = accent;
    document.documentElement.dataset.theme = theme;
    const cs = getComputedStyle(document.documentElement);
    hue = parseFloat(cs.getPropertyValue('--ah'));
    chroma = parseFloat(cs.getPropertyValue('--ac'));
  });
  const style = $derived(theme === 'dark'
    ? { lineColor: `oklch(0.78 ${chroma} ${hue})`, bgColor: `oklch(0.17 ${chroma * 0.15} ${hue})`, bgColor2: `oklch(0.27 ${chroma * 0.5} ${hue})` }
    : { lineColor: `oklch(0.4 ${chroma} ${hue})`, bgColor: `oklch(0.95 ${chroma * 0.1} ${hue})`, bgColor2: `oklch(0.88 ${chroma * 0.35} ${hue})` });

  const opts = $derived.by<ContourOpts>(() => {
    const scale = Math.max(80, short * tune.scaleMul);
    return {
      width: side, height: side, seed: tune.seed, phase: variation / 4,
      scale, octaves: tune.octaves, lo: 0.25, hi: 0.75,
      levels: Math.max(4, Math.min(11, Math.round(screen.width / 170))),
      cell: Math.max(5, Math.round(scale / 22)),
      stroke: screen.width < 700 ? 1.2 : 1.5, lineOpacity: 0.25, gradient: true, ...style,
    };
  });
  const svg = $derived(PATTERNS[pattern](opts));
  const url = $derived(`url("data:image/svg+xml,${encodeURIComponent(svg)}")`);
</script>

<div class="stage" style:background-image={url}></div>
<aside>
  <label>Pattern
    <select bind:value={pattern}>{#each Object.keys(PATTERNS) as p}<option>{p}</option>{/each}</select>
    <input type="range" min="0" max="100" step="0.5" bind:value={variation} /></label>
  <button type="button" class="link" onclick={() => (advanced = !advanced)}>{advanced ? 'Hide' : 'More'} options</button>
  {#if advanced}
    <label>Seed <input type="number" bind:value={tune.seed} /></label>
    <label>Feature size {tune.scaleMul}<input type="range" min="0.1" max="0.8" step="0.02" bind:value={tune.scaleMul} /></label>
    <div class="row">
      <label>Accent <select bind:value={accent}>{#each ['orange', 'yellow', 'green', 'teal', 'blue', 'violet', 'rose', 'neutral'] as a}<option>{a}</option>{/each}</select></label>
      <label>Theme <select bind:value={theme}><option>light</option><option>dark</option></select></label>
    </div>
  {/if}
  <small>{side}px · {(svg.length / 1024).toFixed(0)} KB</small>
</aside>

<style>
  .stage { position: fixed; inset: 0; background-repeat: no-repeat; background-size: auto; background-position: 0 0; }
  aside { position: fixed; left: 50%; bottom: 16px; transform: translateX(-50%); width: min(360px, calc(100vw - 24px)); padding: 12px 14px; display: grid; gap: 8px;
    background: color-mix(in srgb, canvas 85%, transparent); backdrop-filter: blur(10px); border-radius: 12px; color: canvasText; }
  label { display: grid; gap: 4px; font-size: 0.8rem; }
  .row { display: flex; gap: 8px; }
  .row label { flex: 1; }
  .link { all: unset; font-size: 0.75rem; cursor: pointer; text-decoration: underline; justify-self: start; }
  input[type='number'] { width: 100%; }
</style>
