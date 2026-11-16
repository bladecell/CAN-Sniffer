<script lang="ts">
  import type { Snippet } from "svelte";

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
</script>

<div class="st-row" class:stack class:dis={disabled} class:inl={inline} class:boxed aria-disabled={disabled || undefined}>
  <div class="st-text">
    <span class="st-lab">{title}{#if tag}<span class="tag">{tag}</span>{/if}</span>
    {#if hint}<span class="st-hint">{hint}</span>{/if}
    {#if error}<span class="st-bad" role="alert">{error}</span>{/if}
  </div>
  {#if children}<div class="st-ctl">{@render children()}</div>{/if}
</div>
