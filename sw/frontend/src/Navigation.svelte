<script lang="ts">
  import Icon from "$lib/Icon.svelte";

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

  // Phone bottom bar: hide on scroll down, show on scroll up. Needs a deliberate 40px scroll up to return; always shown at the top.
  let hidden = $state(false);
  let lastY = 0;

  function onScroll() {
    const y = window.scrollY;
    const dy = y - lastY;
    if (y <= 8) hidden = false;
    else if (dy > 6) {
      hidden = true;
      lastY = y;
    } else if (dy < -40) {
      hidden = false;
      lastY = y;
    }
    armIdle();
  }

  // Idle timeout: hide 2.5s after the last scroll or dock touch. Only away from the page top, so scrolling up can always bring it back.
  let idle: ReturnType<typeof setTimeout> | undefined;

  function armIdle() {
    clearTimeout(idle);
    idle = setTimeout(() => {
      if (window.scrollY > 8 && !tabList?.closest(".dock")?.matches(":focus-within")) hidden = true;
    }, 2500);
  }

  $effect(() => () => clearTimeout(idle));
</script>

<svelte:window onresize={moveBar} onscroll={onScroll} onpointerdown={(e) => (e.target as Element).closest?.(".dock") && armIdle()} />

<div class="nav-wrapper">
  <nav class="dock float-layer" class:hidden aria-label="Main">
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

  /* Float layer (app.css .float-layer). */
  .dock {
    pointer-events: auto;
    display: flex;
    align-items: center;
    margin: 0;
    padding: 8px 10px;
    border-radius: var(--r-float);
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
    border-radius: calc(var(--r-float) - 8px);
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

  @media (hover: hover) {
    .tab:hover {
      background: var(--hov);
    }
  }

  @media (pointer: coarse) {
    .tab {
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
      border-radius: calc(var(--r-float) + 4px);
    }

    .dock {
      transition:
        transform 0.3s var(--ease-out),
        opacity 0.2s var(--ease-out),
        visibility 0s 0.3s;
    }

    /* Fades while sliding: the backdrop blur drops during a transform, so the content behind would show through. */
    .dock.hidden {
      transform: translateY(calc(100% + 24px + env(safe-area-inset-bottom)));
      opacity: 0;
      visibility: hidden;
    }

    .dock:focus-within {
      transform: none;
      opacity: 1;
      visibility: visible;
    }

    .dock:not(.hidden) {
      transition-delay: 0s;
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
      border-radius: calc(var(--r-float) - 4px);
      font-size: 0.64rem;
      font-weight: 700;
      letter-spacing: -0.01em;
    }

    .tab[aria-current="page"] .ic {
      transform: translateY(-1px) scale(1.12);
    }
  }

  @media (max-width: 374px) {
    .tab {
      font-size: 0.58rem;
      letter-spacing: -0.02em;
    }
  }
</style>
