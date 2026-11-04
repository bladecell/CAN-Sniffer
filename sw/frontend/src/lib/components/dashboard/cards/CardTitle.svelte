<script lang="ts">
  import type { Level, ReadState } from "$lib/dashboardCardView";

  interface Props {
    name: string;
    unit?: string;
    state?: ReadState;
    level?: Level | null;
    /** Whether a high value or a low value is the warning. */
    direction?: "high" | "low";
    /** Word for a warning level when High or Low does not fit the card (DTCs). */
    warnWord?: string;
    /** Word and check mark for a healthy reading that has a state of its own (battery: Charging). */
    okWord?: string;
    /** Narrow cards show the icon only; the word stays for screen readers. */
    iconOnly?: boolean;
    /** One-row cards keep the state on the name line. Taller cards give it a reserved line under the name. */
    inline?: boolean;
  }

  let { name, unit, state = "value", level = null, direction = "high", warnWord, okWord, iconOnly = false, inline = false }: Props = $props();

  const word = $derived(
    state === "nodata" ? "No data"
    : state === "off" ? "Polling off"
    : level === "crit" ? "Critical"
    : level === "warn" ? (warnWord ?? (direction === "low" ? "Low" : "High"))
    : okWord && state === "value" ? okWord
    : "",
  );
  const icon = $derived(
    state === "nodata" ? "nodata"
    : state === "off" ? "off"
    : level === "warn" ? "warn"
    : level === "crit" ? "crit"
    : okWord && state === "value" ? "ok"
    : null,
  );
  const onTop = $derived(inline || iconOnly);
</script>

<div class="ct" class:stacked={!onTop}>
  <div class="top">
    <span class="nm">{name}</span>
    {#if unit}<span class="un">{unit}</span>{/if}
    {#if onTop}
      <span class="ls" class:warn={level === "warn" && state === "value"} class:crit={level === "crit" && state === "value"} class:ok={icon === "ok"} class:dim={state !== "value"}>
        {#if icon}<svg viewBox="0 0 24 24" width="14" height="14" aria-hidden="true"><path d={ICONS[icon]} /></svg>{/if}
        {#if iconOnly}<span class="vh">{word}</span>{:else}{word}{/if}
      </span>
    {/if}
  </div>
  <!-- The state line is always rendered below the name so the value never moves when the state changes. -->
  {#if !onTop}
    <span class="ls" class:warn={level === "warn" && state === "value"} class:crit={level === "crit" && state === "value"} class:ok={icon === "ok"} class:dim={state !== "value"}>
      {#if icon}<svg viewBox="0 0 24 24" width="14" height="14" aria-hidden="true"><path d={ICONS[icon]} /></svg>{/if}
      {word}
    </span>
  {/if}
</div>

<script module lang="ts">
  // Icon paths: no data (dash in circle), polling off (pause), warning (triangle), critical (circle with mark), ok (check).
  const ICONS: Record<string, string> = {
    nodata: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-5 9h10v2H7z",
    off: "M7 5h4v14H7zM13 5h4v14h-4z",
    warn: "M12 3 22 20H2zm-1 6v5h2V9zm0 7v2h2v-2z",
    crit: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-1 5h2v6h-2zm0 8h2v2h-2z",
    ok: "M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20zm-1.2 14.4-4-4 1.4-1.4 2.6 2.6 5.6-5.6 1.4 1.4z",
  };
</script>

<style>
  .ct {
    display: flex;
    flex-direction: column;
    min-width: 0;
    font-size: 0.8rem;
    line-height: 1.2;
    font-weight: 700;
    color: var(--text);
  }
  .top {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
  }
  .nm {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .un {
    flex: none;
    color: var(--muted);
    font-weight: 600;
  }
  .ls {
    display: inline-flex;
    flex: none;
    align-items: center;
    gap: 4px;
    min-height: 1.2em;
    margin-left: auto;
    font-size: 0.75rem;
    font-weight: 800;
    white-space: nowrap;
    color: var(--muted);
  }
  .stacked .ls {
    margin-left: 0;
    min-height: 1.3em;
  }
  .ls.warn {
    color: var(--warn);
  }
  .ls.crit {
    color: var(--crit);
  }
  .ls.ok {
    color: var(--ok);
  }
  .ls.dim {
    color: var(--muted);
    font-weight: 600;
  }
  .ls svg {
    flex: none;
    fill: currentColor;
  }
  .vh {
    position: absolute;
    width: 1px;
    height: 1px;
    overflow: hidden;
    clip: rect(0 0 0 0);
    white-space: nowrap;
  }
</style>
