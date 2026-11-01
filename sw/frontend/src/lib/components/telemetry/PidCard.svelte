<script lang="ts">
  import type { Snippet } from "svelte";
  import type { Attachment } from "svelte/attachments";

  /**
   * One PID card, shared by the PIDs sidebar and the SD import sheet, so both look the same. Styles are the global
   * sidebar classes (telemetry.css: sb-row, sb-slide, sb-main, sb-sw, sb-nm, sb-hx). The extras are snippets:
   * tags (status tags, after the name), value (live value, right), actions (a control beside the card), behind
   * (phone swipe layers, under the card) and after (a panel under the card).
   */
  interface Props {
    selected: boolean;
    /** The PID colour as #rrggbb, shown as the swatch. */
    color: string;
    name: string;
    /** The PID as shown (0x0C). Empty hides it. */
    hex?: string;
    phone?: boolean;
    /** The panel under the card is open: the header and the panel read as one card. */
    open?: boolean;
    onToggle: () => void;
    /** Swipe handling for the card (phone). Nothing by default. */
    swipe?: Attachment<HTMLElement>;
    tags?: Snippet;
    value?: Snippet;
    actions?: Snippet;
    behind?: Snippet;
    after?: Snippet;
  }

  let { selected, color, name, hex = "", phone = false, open = false, onToggle, swipe, tags, value, actions, behind, after }: Props = $props();

  const noSwipe: Attachment<HTMLElement> = () => {};
</script>

<li class="sb-row prow" class:sel={selected} class:phone class:open>
  {@render behind?.()}
  <div class="sb-slide" {@attach swipe ?? noSwipe}>
    <button type="button" class="sb-main" aria-pressed={selected} onclick={onToggle}>
      <i class="sb-sw" style="--sw: {color}"></i>
      <span class="sb-nm">
        <span class="sb-nl">
          <b>{name}</b>
          {@render tags?.()}
        </span>
        {#if hex}<span class="sb-hx">{hex}</span>{/if}
      </span>
      {@render value?.()}
    </button>
    {@render actions?.()}
  </div>
  {#if after}<div class="pc-after">{@render after()}</div>{/if}
</li>

<style>
  /* Open: the header keeps its top corners and the panel its bottom ones, with the header's fill, so there is no notch.
     Closed cards are unchanged. The panel copies the header's hover and selected fills, in the same order. */
  .sb-row.open > .sb-slide {
    border-radius: var(--r-ctl) var(--r-ctl) 0 0;
  }
  .sb-row.open > .pc-after {
    border-radius: 0 0 var(--r-ctl) var(--r-ctl);
    background: var(--row);
  }
  @media (hover: hover) {
    .sb-row.open:hover > .pc-after {
      background: var(--hov);
    }
  }
  .sb-row.open.sel > .pc-after {
    background: color-mix(in oklch, var(--accent) 18%, var(--row));
  }
</style>
