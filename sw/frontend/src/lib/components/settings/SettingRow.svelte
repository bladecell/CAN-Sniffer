<script lang="ts">
  import type { Snippet } from "svelte";
  import SidebarHint from "$lib/components/telemetry/SidebarHint.svelte";

  interface Props {
    title: string;
    hint?: string;
    /** Small pill next to the title, e.g. "HW v2". */
    tag?: string;
    disabled?: boolean;
    /** Control goes under the text (text fields, segmented controls). */
    stack?: boolean;
    /** Switch and fact rows keep the control on the right on phones too. */
    inline?: boolean;
    /** Keeps its own box on phones, where other rows sit as plain text in the group's container. */
    boxed?: boolean;
    error?: string;
    children?: Snippet;
  }

  let { title, hint, tag, disabled = false, stack = false, inline = false, boxed = false, error, children }: Props = $props();

  // The subtitle is one line. When it does not fit, it is hidden (still measured) and the title gets the i popup instead.
  let hintEl: HTMLElement | undefined = $state();
  let over = $state(false);
  $effect(() => {
    const el = hintEl;
    const box = el?.parentElement;
    if (!el || !box) return;
    const measure = () => (over = el.scrollWidth > el.clientWidth);
    measure();
    const ro = new ResizeObserver(measure);
    ro.observe(box);
    return () => ro.disconnect();
  });
</script>

<div class="st-row" class:stack class:dis={disabled} class:inl={inline} class:boxed aria-disabled={disabled || undefined}>
  <div class="st-text">
    <span class="st-lab">{title}{#if tag}<span class="tag">{tag}</span>{/if}{#if hint && over}<SidebarHint inline text={hint} label="About {title}" />{/if}</span>
    {#if hint}<span class="st-hint" class:over bind:this={hintEl}>{hint}</span>{/if}
    {#if error}<span class="st-bad" role="alert">{error}</span>{/if}
  </div>
  {#if children}<div class="st-ctl">{@render children()}</div>{/if}
</div>
