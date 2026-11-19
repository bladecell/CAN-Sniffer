<script lang="ts">
  // One data byte as a tile, shared by the frame table and the ID details. Height comes from --tile-h on the parent.
  interface Props {
    value: number;
    changed?: boolean;
  }

  let { value, changed = false }: Props = $props();
  const hex = $derived(value.toString(16).toUpperCase().padStart(2, "0"));
</script>

<div class="tile" class:z={value === 0} class:chg={changed}>{hex}</div>

<style>
  /* Changed on the frame: tint and corner mark. 00: dimmed. */
  .tile {
    position: relative;
    display: grid;
    place-items: center;
    height: var(--tile-h, 28px);
    min-width: 0;
    border-radius: 7px;
    background: var(--card2);
    font-family: var(--mono);
    font-weight: 700;
    font-size: 0.84rem;
    font-variant-numeric: tabular-nums;
  }
  .tile.z {
    color: var(--muted);
    font-weight: 500;
    opacity: 0.7;
  }
  .tile.chg {
    background: color-mix(in srgb, var(--accent) 26%, transparent);
    color: var(--text);
  }
  .tile.chg::after {
    content: "";
    position: absolute;
    top: 0;
    right: 0;
    width: 8px;
    height: 8px;
    border-top-right-radius: 7px;
    background: linear-gradient(225deg, var(--accent) 50%, transparent 50%);
  }
</style>
