<script lang="ts">
  import Field from "$lib/components/telemetry/editor/Field.svelte";

  interface Props {
    /** Inclusive columns and rows the card can take. Values outside the range are not offered. */
    min: [number, number];
    max: [number, number];
    w: number;
    h: number;
    onChange: (w: number, h: number) => void;
  }

  let { min, max, w, h, onChange }: Props = $props();

  const range = (lo: number, hi: number) => Array.from({ length: hi - lo + 1 }, (_, i) => lo + i);
  const cols = $derived(range(min[0], max[0]));
  const rows = $derived(range(min[1], max[1]));
</script>

<Field id="db-size-w" label="Width, columns">
  <div class="db-seg eq" role="radiogroup" aria-label="Width, columns">
    {#each cols as v (v)}
      <button type="button" role="radio" aria-checked={w === v} tabindex={w === v ? 0 : -1} onclick={() => onChange(v, h)}>{v}</button>
    {/each}
  </div>
</Field>
<Field id="db-size-h" label="Height, rows">
  <div class="db-seg eq" role="radiogroup" aria-label="Height, rows">
    {#each rows as v (v)}
      <button type="button" role="radio" aria-checked={h === v} tabindex={h === v ? 0 : -1} onclick={() => onChange(w, v)}>{v}</button>
    {/each}
  </div>
</Field>
