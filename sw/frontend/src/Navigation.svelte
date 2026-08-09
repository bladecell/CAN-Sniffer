<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { dashboardStore } from "$lib/dashboardStore.svelte.ts";

  let { activeTab = $bindable() } = $props();

  const tabs = [
    { id: "dashboard", icon: "gauge", label: "Dashboard" },
    { id: "recorder", icon: "route", label: "Recorder" },
    { id: "diagnostics", icon: "engine", label: "Diagnostics" },
    { id: "telemetry", icon: "chart", label: "Telemetry" },
    { id: "can-logging", icon: "timeline-arrow", label: "CAN Logging" },
    { id: "settings", icon: "gear", label: "Settings" },
  ];
</script>

<div class="nav-wrapper">
  <nav class="pill-nav blur-background" class:is-editing={dashboardStore.isEditMode}>
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
    border-radius: 26px;
    padding: 0px 20px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
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
      padding: 0 10px; /* Gives a safe zone on the extreme edges of tiny screens */
      box-sizing: border-box;
    }

    .pill-nav {
      padding: 0px 6px; /* Reduced padding on the left/right of the pill */
      width: 100%;
      max-width: 450px; /* Keeps it looking nice on medium tablets, but shrinks on phones */
      box-sizing: border-box;
    }

    ul {
      height: 70px; /* Slightly shorter to maintain nice proportions */
      gap: 2px; /* Tiny gap to save space */
      width: 100%;
    }

    li {
      flex: 1; /* THE MAGIC FIX: Forces all 6 list items to share the width equally */
      display: flex;
    }

    button {
      height: 100%;
      width: 100%; /* Stretches to fill the flexible li */
      min-height: auto;
      padding: 0; /* Remove padding to maximize touch area */
    }

    .icon {
      /* Shrink the icon wrapper slightly so it fits inside the compressed button */
      width: 40px;
      height: 40px;
    }

    .label {
      display: none;
    }

    /* --- EDIT MODE ADJUSTMENTS --- */
    .edit-mode-hud {
      height: 70px; /* Match the new UL height */
      width: 100%;
      gap: 8px; /* Tighter spacing */
    }

    .edit-mode-hud .hud-lock-btn {
      width: auto;
      height: auto;
      display: block;
      padding: 8px 12px;
    }

    .edit-mode-hud .label-text {
      font-size: 0.7rem;
      white-space: nowrap; /* Prevents text from awkwardly stacking */
    }
  }
</style>
