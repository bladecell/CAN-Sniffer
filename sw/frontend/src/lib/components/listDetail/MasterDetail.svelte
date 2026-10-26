<script module lang="ts">
  let seq = 0;
</script>

<script lang="ts" generics="T">
  import { tick, type Snippet } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import "./listDetail.css";

  interface Props {
    items: T[];
    keyOf: (item: T) => string;
    /** Short name for the status line announced on selection (e.g. "P0301"). */
    label: (item: T) => string;
    /** Key of the row shown in the detail pane (wide screens). Defaults to the first row. */
    selected?: string | null;
    listTitle: string;
    listHint?: string;
    detailLabel: string;
    row: Snippet<[T]>;
    /** `full` is true in the detail pane (title included), false in the phone accordion. */
    detail: Snippet<[T, boolean]>;
  }

  let {
    items,
    keyOf,
    label,
    selected = $bindable(null),
    listTitle,
    listHint,
    detailLabel,
    row,
    detail,
  }: Props = $props();

  // Phone (single column): each row expands in place and the detail pane is hidden.
  let phone = $state(false);
  $effect(() => {
    const query = matchMedia("(max-width: 859px)");
    const update = () => (phone = query.matches);
    update();
    query.addEventListener("change", update);
    return () => query.removeEventListener("change", update);
  });

  let openKey = $state<string | null>(null);
  const uid = `md${++seq}`;
  let announced = $state("");

  const active = $derived(items.find((item) => keyOf(item) === selected) ?? items[0] ?? null);
  const activeKey = $derived(active === null ? null : keyOf(active));

  // Phone: opening a row moves focus into its panel; closing returns nothing (the row keeps focus).
  async function pick(item: T, index: number) {
    const key = keyOf(item);
    selected = key;
    if (!phone) {
      announced = `${label(item)} selected`;
      return;
    }
    const opening = openKey !== key;
    openKey = opening ? key : null;
    announced = `${label(item)} ${opening ? "expanded" : "collapsed"}`;
    if (opening) {
      await tick();
      document.getElementById(`${uid}-panel-${index}`)?.focus();
    }
  }
</script>

<div class="rv-split">
  <section class="rv-list" aria-label={listTitle}>
    <header class="rv-list-head">
      <h2>{listTitle}</h2>
      {#if listHint}<span>{listHint}</span>{/if}
    </header>
    <ul>
      {#each items as item, i (keyOf(item))}
        {@const key = keyOf(item)}
        {@const expanded = phone && openKey === key}
        <li>
          <button
            type="button"
            id="{uid}-row-{i}"
            class="rv-row"
            aria-current={phone ? undefined : key === activeKey}
            aria-expanded={phone ? expanded : undefined}
            aria-controls={expanded ? `${uid}-panel-${i}` : undefined}
            onclick={() => pick(item, i)}
          >
            {@render row(item)}
            <Icon name="chevron-down" size={18} class="rv-row-chev" />
          </button>
          {#if expanded}
            <div
              id="{uid}-panel-{i}"
              class="rv-inline"
              role="region"
              tabindex="-1"
              aria-labelledby="{uid}-row-{i}"
            >
              {@render detail(item, false)}
            </div>
          {/if}
        </li>
      {/each}
    </ul>
  </section>

  {#if active}
    <section class="card rv-pane" aria-label={detailLabel}>
      {@render detail(active, true)}
    </section>
  {/if}
  <p class="rv-sr" role="status">{announced}</p>
</div>
