<script lang="ts">
  import type { DataTableProps } from "$lib/types";

  let {
    columns,
    data,
    selectedRowIndex = $bindable(-1),
  }: DataTableProps = $props();

  // Helper function to handle row clicks
  function selectRow(index: number) {
    // Clicking the already selected row deselects it
    if (selectedRowIndex === index) {
      selectedRowIndex = -1;
    } else {
      selectedRowIndex = index;
    }
  }
</script>

<div class="data-grid-container overflow-auto">
  <table>
    <thead>
      <tr>
        {#each columns as col}
          <th scope="col" style="width: {col.width || 'auto'}">
            {col.label}
          </th>
        {/each}
      </tr>
    </thead>
    <tbody>
      {#if data.length > 0}
        {#each data as row, rowIndex}
          <tr
            class:selected={selectedRowIndex === rowIndex}
            onclick={() => selectRow(rowIndex)}
            tabindex="0"
            aria-selected={selectedRowIndex === rowIndex}
          >
            {#each columns as col, colIndex}
              <td style="width: {col.width || 'auto'}">
                {#if col.type === "code"}
                  <span class="type-code">{row[col.key]}</span>
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
                  {row[col.key]}

                  {#if col.unitKey && row[col.unitKey]}
                    <small style="opacity: 0.6; margin-left: 4px;"
                      >{row[col.unitKey]}</small
                    >
                  {:else if col.unit}
                    <small style="opacity: 0.6; margin-left: 4px;"
                      >{col.unit}</small
                    >
                  {/if}
                {:else if col.tooltipKey && row[col.tooltipKey]}
                  <span
                    data-tooltip={row[col.tooltipKey]}
                    data-placement={colIndex === 0
                      ? "right"
                      : colIndex === columns.length - 1
                        ? "left"
                        : "bottom"}
                    class="tooltip-text"
                  >
                    {row[col.key]}
                  </span>
                {:else if col.showTooltip}
                  <span
                    data-tooltip={row[col.key]}
                    data-placement={colIndex === 0
                      ? "right"
                      : colIndex === columns.length - 1
                        ? "left"
                        : "bottom"}
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
          <td colspan={columns.length} style="text-align: center;">
            No data found
          </td>
        </tr>
      {/if}
    </tbody>
  </table>
</div>

<style>
  /* --- 1. BASIC TABLE LAYOUT --- */
  .data-grid-container {
    padding: 0;
    margin: 0;
  }

  table {
    font-size: 0.85rem;
    width: 100%;
    table-layout: fixed; /* Forces proportional widths to be respected */
    border-collapse: collapse;
    background: transparent !important;
  }

  th {
    text-transform: uppercase;
    font-size: 0.7rem;
    color: var(--pico-muted-color);
    border-bottom: 1px solid rgba(255, 255, 255, 0.1);
    padding: 0.75rem;
    text-align: left;
  }

  td {
    padding: 0.75rem !important;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  }

  /* --- 2. ROW INTERACTIVITY & HOVER --- */
  /* Make rows look clickable */
  tbody tr {
    cursor: pointer;
  }

  /* Move the transition to the CELLS, because that is what changes color */
  tbody td {
    transition:
      background-color 0.15s ease,
      box-shadow 0.15s ease;
  }

  /* Hover effect ONLY for unselected rows (Target the TD to override Pico!) */
  tbody tr:not(.selected):hover td {
    background-color: rgba(255, 255, 255, 0.05) !important;
  }

  /* --- 3. SELECTED ROW STATES --- */
  /* The selected row stays exactly the same color, even when hovered */
  tbody tr.selected td {
    background-color: color-mix(
      in srgb,
      var(--pico-primary) 15%,
      transparent
    ) !important;
  }

  /* The vertical highlight bar for the selected row */
  tbody tr.selected td:first-child {
    box-shadow: inset 4px 0 0 0 var(--pico-primary) !important;
  }

  .tooltip-text {
    cursor: help;
    border-bottom: 1px dotted var(--pico-muted-color);
  }

  /* --- 4. CUSTOM CELL FORMATTING --- */
  .type-code {
    font-family: monospace;
    background: rgba(0, 0, 0, 0.3);
    padding: 0.2rem 0.5rem;
    border-radius: 4px;
    border: 1px solid rgba(255, 255, 255, 0.1);
    color: var(--pico-primary);
  }

  .badge {
    padding: 0.2rem 0.6rem;
    border-radius: 12px;
    font-size: 0.7rem;
    font-weight: 500;

    /* Automatically tints the background and border based on the injected variable */
    background: color-mix(in srgb, var(--badge-color) 15%, transparent);
    border: 1px solid color-mix(in srgb, var(--badge-color) 30%, transparent);
    color: var(--badge-color);
  }

  table {
    /* A lighter frosty white looks much better as glass on dark themes! */
    --pico-tooltip-background-color: var(--backdrop-filter-background);

    /* Force the text to be pure white */
    --pico-tooltip-color: #ffffff;
  }

  /* 2. Apply the glass blur and enforce the text color */
  :global(td [data-tooltip]::before) {
    backdrop-filter: var(--backdrop-filter) !important;
    -webkit-backdrop-filter: var(--backdrop-filter) !important;
    border: 1px solid rgba(255, 255, 255, 0.15) !important;

    /* Backup force to ensure text is white and readable */
    color: #ffffff !important;
    font-weight: 500 !important; /* Slightly bolder text reads better on glass */
  }
</style>
