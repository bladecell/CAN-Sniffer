<script lang="ts">
  import type { Snippet } from "svelte";
  import { measureHeight } from "$lib/layers";

  /**
   * One PID sidebar tab. The header and footer are float layers that sit beside the scroller, not inside it,
   * so their blur holds in Gecko (DESIGN.md layering rule). The wrapper has no overflow and no blur.
   */
  interface Props {
    header: Snippet;
    children: Snippet;
    footer?: Snippet;
  }

  let { header, children, footer }: Props = $props();
</script>

<div class="sb-panel">
  <div class="sb-top float-layer" {@attach measureHeight("--sb-head")}>
    {@render header()}
  </div>
  <div class="sb-body">
    {@render children()}
  </div>
  {#if footer}
    <div class="sb-foot" {@attach measureHeight("--sb-foot")}>
      {@render footer()}
    </div>
  {/if}
</div>
