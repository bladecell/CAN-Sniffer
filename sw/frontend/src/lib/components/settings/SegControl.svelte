<script lang="ts" generics="T extends string">
  import { nextOption } from "$lib/appearance";

  interface Props {
    label: string;
    options: readonly { value: T; label: string }[];
    value: T;
    onChange: (value: T) => void;
  }

  let { label, options, value, onChange }: Props = $props();

  // Radio group keys: arrows move and select, as in the WAI-ARIA radio pattern.
  function onKey(e: KeyboardEvent) {
    const next = nextOption(
      options.map((o) => o.value),
      value,
      e.key,
    );
    if (next === undefined) return;
    e.preventDefault();
    onChange(next);
    (e.currentTarget as HTMLElement).parentElement?.querySelector<HTMLElement>(`[data-value="${next}"]`)?.focus();
  }
</script>

<div class="st-seg" role="radiogroup" aria-label={label}>
  {#each options as o (o.value)}
    <button
      type="button"
      role="radio"
      data-value={o.value}
      aria-checked={value === o.value}
      tabindex={value === o.value ? 0 : -1}
      onclick={() => onChange(o.value)}
      onkeydown={onKey}>{o.label}</button
    >
  {/each}
</div>
