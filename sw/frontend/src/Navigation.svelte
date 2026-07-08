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

<div
  class="fixed left-0 right-0 z-50 flex justify-center pointer-events-none top-5 max-lg:top-auto max-lg:bottom-5 max-lg:px-2.5"
>
  <nav
    class="pointer-events-auto px-5 max-lg:px-1.5 py-0 max-lg:h-30 rounded-[26px] bg-base-200/60 backdrop-blur-md border border-base-content/10 shadow-xl transition-all duration-300 w-auto max-lg:w-full max-lg:max-w-[450px] {dashboardStore.isEditMode
      ? 'border-primary shadow-primary/15'
      : ''}"
  >
    {#if dashboardStore.isEditMode}
      <div
        class="flex items-center justify-center gap-3 py-2.5 h-full font-mono text-sm font-bold tracking-wider text-base-content"
      >
        <span
          class="w-2.5 h-2.5 rounded-full bg-primary animate-pulse shadow-[0_0_8px_rgba(var(--p),1)]"
        ></span>
        <span class="max-lg:text-[0.7rem] whitespace-nowrap"
          >CANVAS UNLOCKED</span
        >
        <button
          class="btn btn-primary btn-sm rounded-full font-extrabold text-[0.75rem] uppercase tracking-normal"
          onclick={() => (dashboardStore.isEditMode = false)}
        >
          Done Editing
        </button>
      </div>
    {:else}
      <ul class="flex gap-2 max-lg:gap-0.5 h-full w-full m-0 p-0 list-none">
        {#each tabs as tab}
          <li class="m-0 p-0 h-full max-lg:flex-1 flex">
            <button
              class="rounded-[26px] h-full w-full flex flex-col items-center justify-center transition-all duration-200 cursor-pointer outline-none border-none select-none lg:btn lg:btn-ghost lg:h-auto lg:py-5 lg:px-4 lg:min-h-0 {activeTab ===
              tab.id
                ? 'text-primary bg-white/5'
                : 'bg-transparent'}"
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
              <div
                class="hidden max-lg:flex items-center justify-center h-12 w-12"
              >
                <Icon name={tab.icon} size={26} />
              </div>
              <span class="max-lg:hidden text-sm font-medium">{tab.label}</span>
            </button>
          </li>
        {/each}
      </ul>
    {/if}
  </nav>
</div>
