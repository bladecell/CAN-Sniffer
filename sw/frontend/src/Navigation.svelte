<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { dashboardStore } from "$lib/dashboardStore.svelte.ts";

  let { activeTab = $bindable() } = $props();

  const tabs = [
    { id: "dashboard", icon: "gauge", label: "Dashboard" },
    { id: "diagnostics", icon: "engine", label: "Diagnostics" },
    { id: "recorder", icon: "route", label: "Recorder" },
    { id: "telemetry", icon: "chart-fft", label: "Telemetry" },
    { id: "can-logging", icon: "waves-sine", label: "CAN Bus" },
    { id: "settings", icon: "sliders", label: "Settings" },
  ];

  let tabList: HTMLUListElement | undefined = $state();

  function go(id: string) {
    if (!document.startViewTransition) {
      activeTab = id;
      return;
    }
    document.startViewTransition(() => {
      activeTab = id;
    });
  }

  // Slides the accent bar under the active tab. Position comes from the DOM, so it re-runs on route change and resize.
  function moveBar() {
    const current = tabList?.querySelector<HTMLElement>("[aria-current='page']");
    if (!tabList || !current) return;
    tabList.style.setProperty("--ix", `${current.offsetLeft}px`);
    tabList.style.setProperty("--iw", `${current.offsetWidth}px`);
  }

  $effect(() => {
    if (tabList && activeTab) moveBar();
  });
</script>

<svelte:window onresize={moveBar} />

<div class="nav-wrapper">
  <nav class="dock" aria-label="Main">
    {#if dashboardStore.isEditMode}
      <div class="hud">
        <span class="live-dot" aria-hidden="true"></span>
        <span>Canvas unlocked</span>
        <button class="hud-lock-btn" onclick={() => (dashboardStore.isEditMode = false)}>
          Done editing
        </button>
      </div>
    {:else}
      <ul class="tabs" bind:this={tabList}>
        {#each tabs as tab (tab.id)}
          <li>
            <a
              class="tab"
              href="#{tab.id}"
              aria-current={activeTab === tab.id ? "page" : undefined}
              onclick={(e) => {
                if (e.metaKey || e.ctrlKey || e.shiftKey) return;
                e.preventDefault();
                go(tab.id);
              }}
            >
              <span class="ic"><Icon name={tab.icon} size={24} /></span>
              <span class="label">{tab.label}</span>
            </a>
          </li>
        {/each}
      </ul>
    {/if}
  </nav>
</div>

<style>
  .nav-wrapper {
    position: fixed;
    top: 14px;
    left: 14px;
    right: 14px;
    display: flex;
    justify-content: center;
    z-index: 1000;
    pointer-events: none;
  }

  /* Floating layer: the only place blur and float tokens are used for page chrome (DESIGN.md, Elevation) */
  .dock {
    pointer-events: auto;
    display: flex;
    align-items: center;
    margin: 0;
    padding: 8px 10px;
    border-radius: 28px;
    color: var(--text);
  }

  .tabs {
    position: relative;
    display: flex;
    gap: 4px;
    list-style: none;
    margin: 0;
    padding: 0;
  }

  .tabs::after {
    content: "";
    position: absolute;
    bottom: 0;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
    left: calc(var(--ix, 0px) + var(--iw, 0px) * 0.2);
    width: calc(var(--iw, 0px) * 0.6);
    transition:
      left 0.45s var(--ease-out),
      width 0.45s var(--ease-out);
  }

  .tabs li {
    margin: 0;
    padding: 0;
    display: flex;
  }

  /* Pico nav rules add a negative margin to the first link; reset it */
  .tabs .tab {
    margin: 0;
  }

  .tab {
    position: relative;
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 11px 16px 13px;
    border-radius: 20px;
    color: var(--muted);
    font-weight: 800;
    text-decoration: none;
    white-space: nowrap;
    transition:
      color 0.2s var(--ease-out),
      background-color 0.2s var(--ease-out);
  }

  .tab[aria-current="page"],
  .tab[aria-current="page"]:hover {
    color: var(--accent-text);
  }

  .ic {
    display: flex;
    transition: transform 0.35s var(--ease-out);
  }

  .label {
    line-height: 1.2;
  }

  .hud {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 12px;
    min-height: 48px;
    padding: 0 4px;
    color: var(--text);
    font-weight: 700;
    font-size: 0.9rem;
    white-space: nowrap;
  }

  .live-dot {
    flex: none;
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--accent);
  }

  .hud-lock-btn {
    display: block;
    margin: 0;
    padding: 8px 18px;
    border: none;
    border-radius: 16px;
    background: var(--accent);
    color: var(--on-accent);
    font-family: inherit;
    font-weight: 800;
    font-size: 0.85rem;
    cursor: pointer;
  }

  .hud-lock-btn:active {
    filter: brightness(0.94);
  }

  @media (hover: hover) {
    .tab:hover {
      background: var(--hov);
    }
  }

  @media (pointer: coarse) {
    .tab,
    .hud-lock-btn {
      min-height: 44px;
    }
  }

  /* Desktop: floating dock at the top, icon and label side by side */
  @media (min-width: 860px) and (max-width: 1060px) {
    .ic {
      display: none;
    }
  }

  /* Phone: bottom bar, six equal slots, icon over label */
  @media (max-width: 859px) {
    .nav-wrapper {
      top: auto;
      bottom: 0;
      left: 0;
      right: 0;
      padding: 0 6px calc(10px + env(safe-area-inset-bottom));
    }

    .dock {
      width: 100%;
      padding: 7px 2px 9px;
      border-radius: 32px;
    }

    .tabs {
      width: 100%;
    }

    .tabs li {
      flex: 1 1 0;
      min-width: 0;
    }

    .tabs::after {
      bottom: 1px;
    }

    .tab {
      flex-direction: column;
      justify-content: center;
      gap: 2px;
      width: 100%;
      min-height: 58px;
      padding: 6px 0 8px;
      border-radius: 24px;
      font-size: 0.64rem;
      font-weight: 700;
      letter-spacing: -0.01em;
    }

    .tab[aria-current="page"] .ic {
      transform: translateY(-1px) scale(1.12);
    }

    .hud {
      min-height: 58px;
      gap: 8px;
    }

    .hud-lock-btn {
      padding: 8px 12px;
    }
  }

  @media (max-width: 374px) {
    .tab {
      font-size: 0.58rem;
      letter-spacing: -0.02em;
    }
  }
</style>
