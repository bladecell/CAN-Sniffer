<script lang="ts">
  import type { Column } from "$lib/types";
  import type { DataTableProps } from "$lib/types";

  let {
    columns,
    data,
    selectedRowIndex = $bindable(-1),
  }: DataTableProps = $props();

  // Helper function to handle row clicks
  function selectRow(index: number) {
    // Optional: Clicking the already selected row deselects it
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
          <th scope="col" style="min-width: {col.width || 'auto'}">
            {col.label}
          </th>
        {/each}
      </tr>
    </thead>
    <tbody>
      {#if data.length > 0}
        <!-- Notice we added `rowIndex` to the loop here -->
        {#each data as row, rowIndex}
          <tr
            class:selected={selectedRowIndex === rowIndex}
            onclick={() => selectRow(rowIndex)}
            tabindex="0"
            aria-selected={selectedRowIndex === rowIndex}
          >
            {#each columns as col}
              <td style="min-width: {col.width || 'auto'}">
                {#if col.type === "code"}
                  <span class="type-code">{row[col.key]}</span>
                {:else if col.type === "badge"}
                  <span class="badge">{row[col.key]}</span>
                {:else if col.type === "number"}
                  {row[col.key]}
                  {#if col.unit}
                    <small style="opacity: 0.6; margin-left: 4px;"
                      >{col.unit}</small
                    >
                  {/if}
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
    background: rgba(255, 255, 255, 0.05);
    font-size: 0.7rem;
    border: 1px solid rgba(255, 255, 255, 0.1);
  }
</style>
