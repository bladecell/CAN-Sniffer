<script lang="ts">
  import type { DTCFaultProps } from "$lib/types";
  import Icon from "$lib/Icon.svelte";

  let {
    code = "P----",
    description = "No description provided",
    mode = 3,
    priority: override,
    ...rest
  }: DTCFaultProps = $props();

  const safeCode = $derived(code ? String(code).trim() : "P----");

  const searchUrl = $derived(
    safeCode === "P----"
      ? "#"
      : `https://www.dtclookup.com/obd-ii/${safeCode.toLowerCase()}/`,
  );

  const MODE_MAP: Record<string, { label: string; color: string }> = {
    "3": { label: "Confirmed", color: "#f87171" },
    "7": { label: "Pending", color: "#fbbf24" },
    "10": { label: "Permanent", color: "#d4d4d8" },
    "0x03": { label: "Confirmed", color: "#f87171" },
    "0x07": { label: "Pending", color: "#fbbf24" },
    "0x0a": { label: "Permanent", color: "#d4d4d8" },
  };

  const modeData = $derived(
    MODE_MAP[String(mode).toLowerCase()] ?? {
      label: `Mode ${mode}`,
      color: "#9ca3af",
    },
  );

  const priorityData = $derived.by(() => {
    if (override) return { label: override, color: "#d27a01" };

    const c = safeCode.toUpperCase();

    if (c === "P0524" || c === "P0217")
      return { label: "Stop Safely", color: "#ef4444" };
    if (c.startsWith("P030")) return { label: "Urgent", color: "#f97316" };
    if (c.startsWith("P07") || c.startsWith("P08"))
      return { label: "Service Trans", color: "#eab308" };
    if (c.startsWith("P01") || c.startsWith("P04") || c.startsWith("P02"))
      return { label: "Attend Soon", color: "rgb(210, 122, 1)" };
    if (c.startsWith("U")) return { label: "Network", color: "#a855f7" };
    if (c.startsWith("B") || c.startsWith("C"))
      return { label: "Inspect", color: "#3b82f6" };

    return { label: "Logged", color: "#94a3b8" };
  });
</script>

<article
  class="dashboard-card dtc-card"
  style="--module-accent: {modeData.color};"
  {...rest}
>
  <header class="dashboard-card-header">
    <div class="dashboard-card-icon">
      <Icon name="engine" size={24} />
    </div>

    <div class="dashboard-card-titles">
      <span class="dashboard-card-label">{safeCode}</span>
      <span class="dashboard-card-subtitle">Fault Code</span>
    </div>

    <div class="badge-group">
      <span
        class="badge dynamic-priority"
        style="--p-color: {priorityData.color};"
      >
        {priorityData.label}
      </span>

      <span class="badge mode-status" style="--mode-color: {modeData.color};">
        {modeData.label}
      </span>
    </div>
  </header>

  <div class="dashboard-card-body">
    <div class="dtc-description-well" style="--well-accent: {modeData.color};">
      <p class="dtc-description" title={description}>{description}</p>
    </div>
  </div>

  <footer class="dtc-footer">
    <a
      href={searchUrl}
      target="_blank"
      rel="noreferrer"
      class="search-btn"
      title="Search Web"
    >
      <Icon name="magnifying-glass" size={16} />
      <span class="search-text">Search Web</span>
    </a>
  </footer>
</article>

<style>
  .dtc-card {
    gap: 8px;
    container-type: inline-size;
    container-name: dtc;
    width: 100%;
    min-width: 140px;
  }

  .dashboard-card-header {
    margin-bottom: 0;
    align-items: center;
  }

  .badge-group {
    display: flex;
    gap: 4px;
    align-items: center;
  }

  .badge {
    font-size: 0.65rem;
    padding: 2px 10px;
    border-radius: 100px;
    font-weight: 600;
    text-transform: uppercase;
    white-space: nowrap;
  }

  .dynamic-priority {
    color: var(--p-color);
    background: color-mix(in srgb, var(--p-color) 20%, transparent);
  }

  .mode-status {
    color: var(--mode-color);
    background: color-mix(in srgb, var(--mode-color) 20%, transparent);
  }

  .dtc-description-well {
    background: rgba(0, 0, 0, 0.15);
    border-left: 3px solid var(--well-accent);
    padding: 8px 10px;
    border-radius: 0 6px 6px 0;
    margin: 2px 0;
  }

  .dtc-description {
    margin: 0;
    font-size: 0.85rem;
    font-weight: 500;
    line-height: 1.4;
    color: #f3f4f6; /* High contrast off-white */
    display: -webkit-box;
    -webkit-line-clamp: 3;
    -webkit-box-orient: vertical;
    overflow: hidden;
  }

  .dtc-footer {
    margin-top: auto;
    background: none;
    border: none;
  }

  .search-btn {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 6px 12px;
    background: rgba(15, 56, 136, 0.25);
    color: rgb(189, 188, 188);
    border-radius: 20px;
    text-decoration: none;
    font-size: 0.8rem;
    font-weight: 500;
    width: fit-content;
  }

  /* Adaptive UI logic */
  @container dtc (max-width: 320px) {
    .dashboard-card-icon {
      width: 38px;
      height: 38px;
    }
    .badge {
      padding: 2px 6px;
      font-size: 0.6rem;
    }
    .dtc-description-well {
      padding: 6px 8px;
      border-left-width: 2px;
    }
    .dtc-description {
      font-size: 0.8rem;
      -webkit-line-clamp: 2;
    }
  }

  @container dtc (max-width: 240px) {
    .dashboard-card-subtitle {
      display: none;
    }
    .badge-group {
      flex-direction: column;
      align-items: flex-end;
      gap: 2px;
    }
    .search-text {
      display: none;
    }
    .search-btn {
      padding: 6px;
      border-radius: 50%;
      width: 28px;
      height: 28px;
      justify-content: center;
    }
  }
</style>
