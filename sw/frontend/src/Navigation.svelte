<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { dashboardStore } from "$lib/dashboardStore.svelte.ts";

  let { activeTab = $bindable() } = $props();

  const tabs = [
    { id: "dashboard", icon: "gauge", label: "Dashboard" },
    { id: "telemetry", icon: "chart", label: "Telemetry" },
    { id: "diagnostics", icon: "engine", label: "Diagnostics" },
    { id: "can-logging", icon: "timeline-arrow", label: "CAN Logging" },
    { id: "settings", icon: "gear", label: "Settings" },
  ];
</script>

<div class="nav-wrapper">
  <nav class="pill-nav" class:is-editing={dashboardStore.isEditMode}>
    {#if dashboardStore.isEditMode}
      <!-- EDIT MODE OVERRIDE -->
      <div class="edit-mode-hud">
        <span class="live-dot"></span>
        <span class="label-text">CANVAS UNLOCKED</span>
        <button
          class="hud-lock-btn"
          onclick={() => (dashboardStore.isEditMode = false)}
        >
          Done Editing
        </button>
      </div>
    {:else}
      <!-- STANDARD NAVIGATION -->
      <ul>
        {#each tabs as tab}
          <li>
            <button
              class:active={activeTab === tab.id}
              onclick={() => {
                if (!document.startViewTransition) {
                  activeTab = tab.id;
                  return;
                }

                document.startViewTransition(() => {
                  activeTab = tab.id;
                });
              }}
            >
              <div class="icon">
                <Icon class="icon" name={tab.icon} size={26} />
              </div>
              <span class="label">{tab.label}</span>
            </button>
          </li>
        {/each}
      </ul>
    {/if}
  </nav>
</div>

<style>
  .nav-wrapper {
    position: fixed;
    left: 0;
    right: 0;
    display: flex;
    justify-content: center;
    z-index: 1000;
    pointer-events: none;
  }

  .pill-nav {
    pointer-events: auto;
    background: var(--backdrop-filter-background) !important;
    border: 1px solid var(--pico-muted-border-color);
    border-radius: 26px;
    padding: 0px 20px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
    backdrop-filter: var(--backdrop-filter);
    -webkit-backdrop-filter: var(--backdrop-filter);
    transition:
      border-color 0.3s ease,
      box-shadow 0.3s ease;
  }

  /* When editing, the entire nav bar glows orange/primary */
  .pill-nav.is-editing {
    border-color: var(--pico-primary);
    box-shadow: 0 10px 30px
      color-mix(in srgb, var(--pico-primary) 15%, transparent);
  }

  ul {
    list-style: none;
    padding: 0;
    margin: 0;
    display: flex;
    gap: 8px;
  }

  li {
    margin: 0;
  }

  button {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-items: center;
    gap: 4px;
    padding: 8px 16px;
    border: none;
    border-radius: 26px;
    color: var(--pico-secondary);
    background: transparent;
    cursor: pointer;
    transition: all 0.2s ease;
    outline: none;
    box-shadow: none;
  }

  button:hover {
    background: rgba(255, 255, 255, 0.05);
  }

  button.active {
    color: var(--pico-primary);
    border: none;
    outline: none;
    box-shadow: none;
  }

  .icon {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 48px;
    width: 48px;
  }

  /* --- NEW: EDIT MODE HUD STYLES --- */
  .edit-mode-hud {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 12px;
    padding: 10px 0;
    color: var(--pico-color);
    font-family: var(--pico-font-family-monospace);
    font-size: 0.85rem;
    font-weight: 700;
  }

  .live-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--pico-primary);
    animation: pulse-glow 1.5s infinite alternate;
  }

  .hud-lock-btn {
    background: var(--pico-primary);
    color: var(--pico-primary-inverse, #1a1a1a);
    border: none;
    padding: 8px 18px;
    border-radius: 20px;
    font-family: var(--pico-font-family);
    font-weight: 800;
    font-size: 0.75rem;
    cursor: pointer;
    text-transform: uppercase;
    transition: transform 0.15s ease;
    display: block; /* Overrides the flex-column of normal nav buttons */
  }

  .hud-lock-btn:active {
    transform: scale(0.95);
  }
  .hud-lock-btn:hover {
    background: var(--pico-primary);
    color: var(--pico-primary-inverse, #1a1a1a);
  }

  @keyframes pulse-glow {
    0% {
      opacity: 1;
      box-shadow: 0 0 1px var(--pico-primary);
    }
    50% {
      opacity: 0.85;
      box-shadow: 0 0 6px var(--pico-primary);
    }
    100% {
      opacity: 1;
      box-shadow: 0 0 8px var(--pico-primary);
    }
  }

  @media (min-width: 992px) {
    .nav-wrapper {
      top: 20px;
    }
    .icon {
      display: none;
    }
    button {
      display: block;
    }
  }

  @media (max-width: 991px) {
    .nav-wrapper {
      bottom: 20px;
    }
    .label {
      display: none;
    }

    button {
      height: 56px;
      width: 64px;
      min-height: auto;
    }

    ul {
      height: 80px;
    }
    .pill-nav {
      padding: 0px 10px;
    }

    /* Prevents the Edit 'Done' button from inheriting the 64x56 square shape on mobile */
    .edit-mode-hud .hud-lock-btn {
      width: auto;
      height: auto;
      display: block;
    }

    .edit-mode-hud {
      height: 80px; /* Matches the ul height to prevent the navbar from shrinking */
    }

    .edit-mode-hud .label-text {
      font-size: 0.75rem; /* Scales text slightly down to ensure it fits next to the button */
    }
  }
</style>
