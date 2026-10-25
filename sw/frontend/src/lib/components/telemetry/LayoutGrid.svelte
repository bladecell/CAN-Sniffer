<script lang="ts">
  interface Props {
    rows: number;
    cols: number;
    onPick: (rows: number, cols: number) => void;
  }

  let { rows, cols, onPick }: Props = $props();

  // Preview the hovered or focused size; starts at the current layout.
  let hoverRows = $state(0);
  let hoverCols = $state(0);
  const previewRows = $derived(hoverRows || rows);
  const previewCols = $derived(hoverCols || cols);
  const cells = Array.from({ length: 16 }, (_, i) => ({ r: Math.floor(i / 4) + 1, c: (i % 4) + 1 }));
</script>

<div class="lp" role="group" aria-label="Chart layout" onmouseleave={() => { hoverRows = 0; hoverCols = 0; }}>
  {#each cells as { r, c } (r * 4 + c)}
    <button
      type="button"
      class="cell"
      class:on={r <= previewRows && c <= previewCols}
      aria-pressed={r === rows && c === cols}
      aria-label="{r} {r > 1 ? 'rows' : 'row'} by {c} {c > 1 ? 'columns' : 'column'}"
      onmouseenter={() => { hoverRows = r; hoverCols = c; }}
      onfocus={() => { hoverRows = r; hoverCols = c; }}
      onclick={() => onPick(r, c)}
    ></button>
  {/each}
</div>
<p class="cap">Pick up to 4 &times; 4. Phones show one column.</p>

<style>
  .lp {
    display: grid;
    grid-template-columns: repeat(4, 40px);
    gap: 6px;
  }
  .cell {
    width: 40px;
    height: 40px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: 8px;
    background: var(--ctl);
    cursor: pointer;
  }
  .cell.on {
    background: var(--accent);
  }
  .cap {
    max-width: 220px;
    margin: 10px 0 0;
    color: var(--muted);
    font-size: 0.85rem;
  }
  @media (pointer: coarse) {
    .lp {
      grid-template-columns: repeat(4, 44px);
    }
    .cell {
      width: 44px;
      height: 44px;
    }
  }
</style>
