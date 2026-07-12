<script lang="ts">
  import type { DataTableProps } from "$lib/types";
  import Switch from "$lib/components/Switch.svelte";

  let {
    columns,
    data,
    selectedRowIndex = $bindable(-1),
  }: DataTableProps = $props();

  // --- UI STATE ---
  let activePopover = $state<"none" | "filter" | "view">("none");
  let toolbarRef: HTMLElement;

  // --- SORTING STATE ---
  let sortKey = $state<string | null>(null);
  let sortDirection = $state<"asc" | "desc">("asc");

  // --- VIEW (COLUMN VISIBILITY) STATE ---
  let hiddenColumns = $state<Set<string>>(
    new Set(columns.filter((c) => c.hidden).map((c) => c.key)),
  );

  let activeColumns = $derived(
    columns.filter((c) => !hiddenColumns.has(c.key)),
  );

  // --- FILTER STATE ---
  type FilterOperator = "contains" | "equals" | "starts_with" | "gt" | "lt";
  type FilterRule = {
    id: string;
    column: string;
    operator: FilterOperator;
    value: string;
  };

  let filterRules = $state<FilterRule[]>([]);

  // --- DERIVED FILTERED & SORTED DATA ---
  let processedData = $derived.by(() => {
    let result = data;

    const validFilters = filterRules.filter(
      (rule) => rule.column && rule.value.trim() !== "",
    );

    if (validFilters.length > 0) {
      result = result.filter((row) => {
        return validFilters.every((rule) => {
          const rawVal = row[rule.column];
          const strVal = String(rawVal ?? "").toLowerCase();
          const searchVal = rule.value.toLowerCase().trim();

          switch (rule.operator) {
            case "contains":
              return strVal.includes(searchVal);
            case "equals":
              return strVal === searchVal;
            case "starts_with":
              return strVal.startsWith(searchVal);
            case "gt":
              return Number(rawVal) > Number(rule.value);
            case "lt":
              return Number(rawVal) < Number(rule.value);
            default:
              return true;
          }
        });
      });
    }

    if (!sortKey) return result;

    const columnDef = columns.find((c) => c.key === sortKey);
    const isNumeric = columnDef?.type === "number";

    return [...result].sort((a, b) => {
      let valA = a[sortKey!];
      let valB = b[sortKey!];

      const isEmptyA = valA == null || valA === "N/A" || valA === "";
      const isEmptyB = valB == null || valB === "N/A" || valB === "";

      if (isEmptyA && isEmptyB) return 0;
      if (isEmptyA) return 1;
      if (isEmptyB) return -1;

      if (isNumeric) {
        const numA = Number(valA);
        const numB = Number(valB);
        if (!isNaN(numA) && !isNaN(numB)) {
          return sortDirection === "asc" ? numA - numB : numB - numA;
        }
      }

      const strA = String(valA).toLowerCase();
      const strB = String(valB).toLowerCase();

      if (strA < strB) return sortDirection === "asc" ? -1 : 1;
      if (strA > strB) return sortDirection === "asc" ? 1 : -1;
      return 0;
    });
  });

  // --- EVENT HANDLERS ---
  function selectRow(index: number) {
    selectedRowIndex = selectedRowIndex === index ? -1 : index;
  }

  function handleSort(key: string) {
    if (sortKey === key) {
      sortDirection = sortDirection === "asc" ? "desc" : "asc";
    } else {
      sortKey = key;
      sortDirection = "asc";
    }
    selectedRowIndex = -1;
  }

  function togglePopover(popover: "filter" | "view") {
    activePopover = activePopover === popover ? "none" : popover;
  }

  // Handle clicking outside the popover to close it
  function handleWindowClick(e: MouseEvent) {
    if (
      activePopover !== "none" &&
      toolbarRef &&
      !toolbarRef.contains(e.target as Node)
    ) {
      activePopover = "none";
    }
  }

  function addFilter() {
    filterRules = [
      ...filterRules,
      {
        id: crypto.randomUUID(),
        column: columns[0]?.key || "",
        operator: "contains",
        value: "",
      },
    ];
  }

  function removeFilter(id: string) {
    filterRules = filterRules.filter((r) => r.id !== id);
  }

  function resetFilters() {
    filterRules = [];
  }

  function toggleColumnVisibility(key: string) {
    const newSet = new Set(hiddenColumns);
    if (newSet.has(key)) {
      newSet.delete(key);
    } else {
      newSet.add(key);
    }
    hiddenColumns = newSet;
  }

  // Dynamically calculate the minimum width the table needs
  let minTableWidth = $derived(
    activeColumns.reduce((sum, col) => {
      if (col.width_px) {
        return sum + parseInt(col.width_px, 10);
      }
      // Give auto-scaling columns a sensible minimum so they don't squish to nothing
      return sum + 150;
    }, 0),
  );

  function formatNumber(value: any, format?: string) {
    if (value == null || value === "N/A" || value === "") return value;
    const num = Number(value);
    if (isNaN(num)) return value;

    if (format === "hex") {
      return "0x" + num.toString(16).toUpperCase();
    } else if (format === "bin") {
      return "0b" + num.toString(2);
    }
    return value;
  }
</script>

<!-- Global click listener for closing menus -->
<svelte:window onclick={handleWindowClick} />

<div class="data-grid-wrapper">
  <!-- TOP TOOLBAR -->
  <div class="toolbar">
    <div class="toolbar-actions" bind:this={toolbarRef}>
      <button class="toolbar-btn" disabled>
        <svg
          width="14"
          height="14"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          stroke-width="2"
          ><path
            d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4M7 10l5 5 5-5M12 15V3"
          /></svg
        >
        Export
      </button>

      <!-- FILTER MENU WRAPPER -->
      <div class="popover-container">
        <button
          class="toolbar-btn"
          class:active={activePopover === "filter" || filterRules.length > 0}
          onclick={() => togglePopover("filter")}
        >
          <svg
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            ><polygon
              points="22 3 2 3 10 12.46 10 19 14 21 14 12.46 22 3"
            /></svg
          >
          Filter {#if filterRules.length > 0}<span class="badge-count"
              >{filterRules.length}</span
            >{/if}
        </button>

        {#if activePopover === "filter"}
          <div class="popover-menu filter-menu blur-background">
            <div class="popover-header">Filter by</div>

            <div class="filter-rules">
              {#each filterRules as rule, i (rule.id)}
                <div class="filter-row">
                  <span class="filter-label">{i === 0 ? "Where" : "And"}</span>

                  <select class="filter-select" bind:value={rule.column}>
                    {#each columns as col}
                      <option value={col.key}>{col.label}</option>
                    {/each}
                  </select>

                  <select class="filter-select" bind:value={rule.operator}>
                    <option value="contains">contains</option>
                    <option value="equals">equals</option>
                    <option value="starts_with">starts with</option>
                    <option value="gt">&gt; greater than</option>
                    <option value="lt">&lt; less than</option>
                  </select>

                  <input
                    class="filter-input-text"
                    type="text"
                    placeholder="Value"
                    bind:value={rule.value}
                  />

                  <button
                    class="icon-btn"
                    onclick={() => removeFilter(rule.id)}
                    title="Remove rule"
                  >
                    <svg
                      width="14"
                      height="14"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      stroke-width="2"
                      ><polyline points="3 6 5 6 21 6" /><path
                        d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"
                      /></svg
                    >
                  </button>
                </div>
              {/each}

              {#if filterRules.length === 0}
                <p class="empty-state">No active filters.</p>
              {/if}
            </div>

            <div class="popover-footer">
              <button class="btn-primary" onclick={addFilter}>Add filter</button
              >
              {#if filterRules.length > 0}
                <button class="btn-ghost" onclick={resetFilters}
                  >Reset filters</button
                >
              {/if}
            </div>
          </div>
        {/if}
      </div>

      <!-- VIEW MENU WRAPPER -->
      <div class="popover-container">
        <button
          class="toolbar-btn"
          class:active={activePopover === "view" || hiddenColumns.size > 0}
          onclick={() => togglePopover("view")}
        >
          <svg
            width="14"
            height="14"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            stroke-width="2"
            ><circle cx="12" cy="12" r="3" /><path
              d="M2 12s3-7 10-7 10 7 10 7-3 7-10 7-10-7-10-7Z"
            /></svg
          >
          View
          {#if hiddenColumns.size > 0}
            <span class="badge-count">{hiddenColumns.size}</span>
          {/if}
        </button>

        {#if activePopover === "view"}
          <div class="popover-menu view-menu blur-background">
            <div class="column-list">
              {#each columns as col}
                <button
                  class="column-toggle-btn"
                  onclick={() => toggleColumnVisibility(col.key)}
                >
                  <span>{col.label}</span>
                  {#if !hiddenColumns.has(col.key)}
                    <svg
                      width="14"
                      height="14"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      stroke-width="2"><polyline points="20 6 9 17 4 12" /></svg
                    >
                  {/if}
                </button>
              {/each}
            </div>
          </div>
        {/if}
      </div>
    </div>
  </div>

  <!-- TABLE -->
  <div class="data-grid-container">
    <table style="min-width: calc({minTableWidth}px * var(--table-scale, 1));">
      <colgroup>
        {#each activeColumns as col}
          <col
            style="width: {(col.type === 'checkbox' || col.type === 'toggle') &&
            col.width_px
              ? `calc(${col.width_px}px * var(--control-col-scale, 1))`
              : col.width_px ? `${col.width_px}px` : 'auto'};"
          />
        {/each}
      </colgroup>
      <thead>
        <tr>
          {#each activeColumns as col}
            <th
              scope="col"
              class="sortable-header"
              onclick={() => handleSort(col.key)}
            >
              <div class="header-content">
                <span>{col.label}</span>
                <span class="sort-indicator" class:active={sortKey === col.key}>
                  {#if sortKey === col.key && sortDirection === "desc"}▼{:else}▲{/if}
                </span>
              </div>
            </th>
          {/each}
        </tr>
      </thead>
      <tbody>
        {#if processedData.length > 0}
          {#each processedData as row, rowIndex}
            <tr
              class:selected={selectedRowIndex === rowIndex}
              onclick={(e) => {
                e.stopPropagation();
                selectRow(rowIndex);
              }}
              tabindex="0"
              aria-selected={selectedRowIndex === rowIndex}
            >
              {#each activeColumns as col, colIndex}
                <td>
                  {#if col.type === "code"}
                    <span class="type-code">{row[col.key]}</span>
                  {:else if col.type === "checkbox"}
                    <div class="control-wrapper">
                      <input
                        type="checkbox"
                        bind:checked={row[col.key]}
                        onclick={(e) => e.stopPropagation()}
                      />
                    </div>
                  {:else if col.type === "toggle"}
                    <div class="control-wrapper">
                      <Switch
                        bind:checked={row[col.key]}
                        onclick={(e) => e.stopPropagation()}
                      />
                    </div>
                  {:else if col.type === "badge"}
                    <span
                      class="badge"
                      style:--badge-color={col.colorKey && row[col.colorKey]
                        ? row[col.colorKey]
                        : "rgba(255, 255, 255, 0.5)"}
                    >
                      {row[col.key]}
                    </span>
                  {:else if col.type === "number"}
                    {#if col.tooltipKey && row[col.tooltipKey]}
                      <span
                        data-tooltip={row[col.tooltipKey]}
                        data-placement={colIndex < activeColumns.length / 2 ? "right" : "left"}
                        class="tooltip-text"
                      >
                        {formatNumber(row[col.key], col.formatKey ? row[col.formatKey] : undefined)}
                        {#if col.unitKey && row[col.unitKey]}
                          <small style="opacity: 0.6; margin-left: 4px;">{row[col.unitKey]}</small>
                        {:else if col.unit}
                          <small style="opacity: 0.6; margin-left: 4px;">{col.unit}</small>
                        {/if}
                      </span>
                    {:else if col.showTooltip}
                      <span
                        data-tooltip={row[col.key]}
                        data-placement={colIndex < activeColumns.length / 2 ? "right" : "left"}
                        class="tooltip-text"
                      >
                        {formatNumber(row[col.key], col.formatKey ? row[col.formatKey] : undefined)}
                        {#if col.unitKey && row[col.unitKey]}
                          <small style="opacity: 0.6; margin-left: 4px;">{row[col.unitKey]}</small>
                        {:else if col.unit}
                          <small style="opacity: 0.6; margin-left: 4px;">{col.unit}</small>
                        {/if}
                      </span>
                    {:else}
                      {formatNumber(row[col.key], col.formatKey ? row[col.formatKey] : undefined)}
                      {#if col.unitKey && row[col.unitKey]}
                        <small style="opacity: 0.6; margin-left: 4px;">{row[col.unitKey]}</small>
                      {:else if col.unit}
                        <small style="opacity: 0.6; margin-left: 4px;">{col.unit}</small>
                      {/if}
                    {/if}
                  {:else if col.tooltipKey && row[col.tooltipKey]}
                    <span
                      data-tooltip={row[col.tooltipKey]}
                      data-placement={colIndex < activeColumns.length / 2
                        ? "right"
                        : "left"}
                      class="tooltip-text"
                    >
                      {row[col.key]}
                    </span>
                  {:else if col.showTooltip}
                    <span
                      data-tooltip={row[col.key]}
                      data-placement={colIndex < activeColumns.length / 2
                        ? "right"
                        : "left"}
                      class="tooltip-text"
                    >
                      {row[col.key]}
                    </span>
                  {:else}
                    {row[col.key]}
                  {/if}
                </td>
              {/each}
            </tr>
          {/each}
        {:else}
          <tr>
            <td
              colspan={activeColumns.length}
              style="text-align: center; padding: 2rem; color: var(--pico-muted-color);"
            >
              No matching data found
            </td>
          </tr>
        {/if}
      </tbody>
    </table>
  </div>
</div>

<style>
  .data-grid-wrapper {
    display: flex;
    flex-direction: column;
    gap: 0;
    position: relative;
    max-width: 100%;
    min-width: 0;
  }

  .control-wrapper {
    transform: scale(var(--control-scale, 1));
    transform-origin: left center;
    display: inline-flex;
    align-items: center;
  }

  /* --- TOOLBAR & BUTTONS --- */
  .toolbar {
    display: flex;
    padding: 0.75rem 1rem;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    background: transparent;
  }

  .toolbar-actions {
    display: flex;
    gap: 0.5rem;
  }

  .toolbar-btn {
    display: inline-flex;
    align-items: center;
    gap: 0.4rem;
    background: transparent;
    border: 1px solid rgba(255, 255, 255, 0.1);
    color: var(--pico-color);
    padding: 0.35rem 0.75rem;
    border-radius: 6px;
    font-size: 0.75rem;
    font-weight: 500;
    cursor: pointer;
    transition: all 0.2s ease;
  }

  .toolbar-btn:hover:not(:disabled) {
    background: rgba(255, 255, 255, 0.05);
    border-color: rgba(255, 255, 255, 0.2);
  }

  .toolbar-btn.active {
    background: rgba(255, 255, 255, 0.1);
    border-color: rgba(255, 255, 255, 0.25);
  }

  .toolbar-btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .badge-count {
    background: rgba(255, 255, 255, 0.15);
    padding: 0.1rem 0.4rem;
    border-radius: 10px;
    font-size: 0.65rem;
    margin-left: 0.2rem;
  }

  /* --- POPOVERS --- */
  .popover-container {
    position: relative;
  }

  .popover-menu {
    position: absolute;
    top: calc(100% + 0.5rem);
    left: 0;
    z-index: 50;
    color: #fff;
    font-size: 0.8rem;
  }

  /* Filter Menu Specific */
  .filter-menu {
    width: max-content;
    /* Use the smaller of 500px OR the screen width minus some padding */
    min-width: min(500px, calc(100vw - 2rem));
    max-width: calc(100vw - 2rem);
    padding: 1.25rem;
  }

  .popover-header {
    font-size: 0.95rem;
    font-weight: 600;
    margin-bottom: 1.25rem;
    color: rgba(255, 255, 255, 0.9);
  }

  .filter-rules {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    margin-bottom: 1.25rem;
  }

  .filter-row {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    width: 100%;
    /* Allow wrapping ONLY when forced by the media query */
    flex-wrap: wrap;
  }

  .filter-label {
    width: 50px;
    flex: 0 0 auto;
    color: rgba(255, 255, 255, 0.5);
    font-weight: 500;
    white-space: nowrap;
    margin: 0; /* Ensure no stray margins push the text around */
    line-height: 1;
  }

  .filter-select,
  .filter-input-text {
    flex: 1 1 0;
    background: rgba(255, 255, 255, 0.05);
    border: 1px solid rgba(255, 255, 255, 0.1);
    color: #fff;
    padding: 0 0.6rem;
    height: 34px;
    border-radius: 4px;
    font-size: 0.75rem;
    font-family: var(--pico-font-family-monospace);
    outline: none;
    box-sizing: border-box;
    transition: border-color 0.2s;
    margin: 0 !important; /* Overrides Pico CSS default bottom margin! */
  }

  .filter-select {
    cursor: pointer;
  }

  .filter-select:focus,
  .filter-input-text:focus {
    border-color: var(--pico-primary);
  }

  .icon-btn {
    --color: 238, 64, 46;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    background: rgba(var(--color), 0.15);
    border: none;
    color: rgba(var(--color));
    cursor: pointer;

    /* Force exact height matching the inputs and make it a perfect square */
    height: 34px;
    width: 34px;
    box-sizing: border-box;
    padding: 0;

    margin: 0;
    border-radius: 4px;
    flex: 0 0 auto;
    transition: all 0.2s ease;
  }

  .icon-btn:hover,
  .btn-primary:hover {
    background: rgba(var(--color), 0.2);
  }

  .empty-state {
    color: rgba(255, 255, 255, 0.4);
    font-style: italic;
    margin: 0;
  }

  .popover-footer {
    display: flex;
    align-items: center;
    gap: 0.75rem;
    padding-top: 1.25rem;
    border-top: 1px solid rgba(255, 255, 255, 0.05);
  }

  .btn-primary {
    --color: 57, 241, 166;
    background-color: rgba(var(--color), 0.1);
    border: none;
    color: rgb(var(--color));
    padding: 0.45rem 0.9rem;
    border-radius: 4px;
    font-size: 0.75rem;
    font-weight: 600;
    cursor: pointer;
  }

  .toolbar-btn:focus,
  .icon-btn:focus,
  .column-toggle-btn:focus,
  .btn-primary:focus,
  .btn-ghost:focus {
    outline: none;
    box-shadow: none;
  }

  .btn-ghost {
    background: transparent;
    color: rgba(255, 255, 255, 0.6);
    border: none;
    padding: 0.45rem 0.9rem;
    border-radius: 4px;
    font-size: 0.75rem;
    cursor: pointer;
  }

  .btn-ghost:hover {
    color: #fff;
    background: rgba(255, 255, 255, 0.05);
  }

  /* View Menu Specific */
  .view-menu {
    width: 220px;
    padding: 0.5rem;
  }

  .column-list {
    display: flex;
    flex-direction: column;
    max-height: 250px;
    overflow-y: auto;
  }

  .column-toggle-btn {
    display: flex;
    justify-content: space-between;
    align-items: center;
    background: transparent;
    border: none;
    color: rgba(255, 255, 255, 0.8);
    padding: 0.6rem 0.5rem;
    border-radius: 4px;
    cursor: pointer;
    font-size: 0.75rem;
    text-align: left;
  }

  .column-toggle-btn:hover {
    background: rgba(255, 255, 255, 0.05);
    color: #fff;
  }

  /* --- TABLE STYLES --- */
  .data-grid-container {
    overflow-x: auto;
    overflow-y: hidden;
    max-width: 100%;
    min-width: 0;
    --table-scale: 1;
    --control-col-scale: 1;
    --control-scale: 1;
    /* This prevents the table from adding a scrollbar if it's 100% wide */
    display: block;
  }

  @media (max-width: 768px) {
    .data-grid-container {
      --table-scale: 0.85; /* Squish columns slightly on tablets */
      --control-col-scale: 0.75;
      --control-scale: 0.85;
    }
  }

  @media (max-width: 480px) {
    .data-grid-container {
      --table-scale: 0.7; /* Squish columns more on mobile */
      --control-col-scale: 0.6;
      --control-scale: 0.7;
    }
  }

  table {
    box-sizing: border-box;
    font-size: 0.85rem;
    width: 100%;
    table-layout: fixed;
    border-collapse: separate;
    border-spacing: 0;
    margin: 0;
    background: transparent !important;
  }

  th,
  td {
    box-sizing: border-box;
  }

  th {
    text-transform: uppercase;
    font-size: 0.7rem;
    color: var(--pico-muted-color);
    border-bottom: 1px solid rgba(255, 255, 255, 0.1);
    padding: 0.75rem;
    text-align: left;
  }

  .sortable-header {
    cursor: pointer;
    user-select: none;
    transition:
      color 0.2s ease,
      background-color 0.2s ease;
  }

  .sortable-header:hover {
    color: var(--pico-color);
    background-color: rgba(255, 255, 255, 0.02);
  }

  .header-content {
    display: flex;
    align-items: center;
    gap: 6px;
  }

  .sort-indicator {
    font-size: 0.55rem;
    opacity: 0;
    transform: translateY(1px);
    transition: opacity 0.2s ease;
  }

  .sort-indicator.active {
    opacity: 1;
    color: var(--pico-primary);
  }

  .sortable-header:hover .sort-indicator:not(.active) {
    opacity: 0.3;
  }

  td {
    padding: 0.75rem !important;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    word-break: break-word;
    overflow-wrap: break-word;
    vertical-align: middle;
  }

  tbody tr {
    cursor: pointer;
  }

  tbody td {
    transition:
      background-color 0.15s ease,
      box-shadow 0.15s ease;
  }

  tbody tr:not(.selected):hover td {
    background-color: rgba(255, 255, 255, 0.05) !important;
  }

  tbody tr.selected td {
    background-color: color-mix(
      in srgb,
      var(--pico-primary) 15%,
      transparent
    ) !important;
  }

  tbody tr.selected td:first-child {
    box-shadow: inset 4px 0 0 0 var(--pico-primary) !important;
  }

  /* --- CELL FORMATTING --- */
  .tooltip-text {
    cursor: help;
    border-bottom: 1px dotted var(--pico-muted-color);
  }

  .type-code {
    --code-color: 255, 158, 100;
    font-family: var(--pico-font-family-monospace, monospace);
    background: rgba(var(--code-color), 0.1);
    color: rgb(var(--code-color));

    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    max-width: 100%;

    display: inline-block;
    vertical-align: middle;
    padding: 0.2rem 0.5rem;
    border-radius: 4px;
    border: none;
  }

  .badge {
    padding: 0.2rem 0.6rem;
    border-radius: 12px;
    font-size: 0.7rem;
    font-weight: 500;
    background: color-mix(in srgb, var(--badge-color) 15%, transparent);
    border: 1px solid color-mix(in srgb, var(--badge-color) 30%, transparent);
    color: var(--badge-color);
  }

  td input[type="checkbox"]:focus {
    outline: none !important;
    box-shadow: none !important;
  }

  table {
    --pico-tooltip-background-color: var(--backdrop-filter-background);
    --pico-tooltip-color: #ffffff;
  }

  .control-wrapper {
    display: flex;
    align-items: center;
    /* Optional: justify-content: center; if you want it horizontally centered too */
    height: 100%;
  }

  /* Reaches inside the wrapper and destroys any framework margins causing the offset */
  .control-wrapper :global(*) {
    margin: 0 !important;
  }

  :global(td [data-tooltip]::before) {
    backdrop-filter: var(--backdrop-filter) !important;
    -webkit-backdrop-filter: var(--backdrop-filter) !important;
    border: 1px solid rgba(255, 255, 255, 0.15) !important;
    color: #ffffff !important;
    font-weight: 500 !important;
  }

  @media (max-width: 768px) {
    .popover-menu {
      /* On mobile, center the popover on the screen instead of hanging it off the button */
      position: fixed;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      width: 90vw;
      box-shadow:
        0 0 0 100vw rgba(0, 0, 0, 0.5),
        0 10px 30px rgba(0, 0, 0, 0.5);
    }

    .filter-label {
      width: 100%;
      margin-bottom: 0.25rem;
    }

    .filter-select,
    .filter-input-text {
      /* Force inputs to take up 100% of the row on mobile */
      flex: 1 1 100%;
      margin-bottom: 0.25rem !important;
    }

    .icon-btn {
      /* Push the trash can to the far right on mobile */
      margin-left: auto;
      padding: 0.5rem;
    }
  }
</style>
