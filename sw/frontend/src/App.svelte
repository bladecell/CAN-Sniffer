<script>
import { connectionStore } from "$lib/connectionStore.svelte";


  import Navigation from "./Navigation.svelte";
  import Dashboard from "./Dashboard.svelte";
  import Recorder from "./Recorder.svelte";
  import Settings from "./Settings.svelte";
  import Diagnostics from "./Diagnostics.svelte";
  import Telemetry from "./Telemetry.svelte";
  import CanLogging from "./CanLogging.svelte";
  import { onMount } from "svelte";
  import Toast from "./lib/components/Toast.svelte";
  import AuthDialog from "./lib/components/AuthDialog.svelte";
  import ApOpenBanner from "./lib/components/ApOpenBanner.svelte";
  import DevBypassBanner from "./lib/components/DevBypassBanner.svelte";
  import { auth } from "$lib/auth.svelte";

  onMount(() => {
    connectionStore.connect();
    auth.refresh();

    // 3. Listen for manual URL changes (Back/Forward button)
    const handleHashChange = () => {
      const hash = window.location.hash.slice(1);
      if (hash) activeTab = hash;
    };
    window.addEventListener("hashchange", handleHashChange);

    const cleanup = () => {
      connectionStore.disconnect();
    };

    window.addEventListener("beforeunload", cleanup);

    return () => {
      cleanup();
      window.removeEventListener("beforeunload", cleanup);
      // Clean up the hash listener too
      window.removeEventListener("hashchange", handleHashChange);
    };
  });

  $effect(() => {
    window.location.hash = activeTab;
  });

  let activeTab = $state(window.location.hash.slice(1) || "dashboard");
</script>

<Navigation bind:activeTab />

<main class="main-content">
  <DevBypassBanner />
  <ApOpenBanner />
  {#if activeTab === "dashboard"}
    <Dashboard />
  {:else if activeTab === "recorder"}
    <Recorder />
  {:else if activeTab === "settings"}
    <Settings />
  {:else if activeTab === "diagnostics"}
    <Diagnostics />
  {:else if activeTab === "can-logging"}
    <CanLogging />
  {:else if activeTab === "telemetry"}
    <Telemetry />
  {/if}
</main>

<Toast />
<AuthDialog />

<style>
  .main-content {
    margin-top: 102px; /* Below the floating dock (DESIGN.md, Layout) */
    padding: 1.5rem; /* DESIGN.md page-pad 24px */
    width: 100vw;
    max-width: 100vw;
    margin-left: 0;
    margin-right: 0;
    padding-bottom: 56px;
  }

  @media (max-width: 859px) {
    .main-content {
      margin-top: 20px; /* The nav is a bottom bar here */
      padding-left: 0.875rem; /* page-pad 14px on phone */
      padding-right: 0.875rem;
      padding-bottom: calc(118px + env(safe-area-inset-bottom)); /* Clears the bottom bar */
    }
  }

  :global(:root) {
    --card-min-width: 200px;
    --card-pref-width: 280px;
    --card-gap: 1.25rem;
    --card-height: 160px;
    --card-max-width: calc((2 * var(--card-min-width)) + var(--card-gap));

    --backdrop-filter: blur(var(--blur)) saturate(1.35);
    --backdrop-filter-background: var(--float);

    --normal-color: var(--ok);
    --warning-color: var(--warn);
    --error-color: var(--crit);

    --grid-border-radius: 12px;
  }

  :global(html) {
    /* Reserve scrollbar space always to prevent shift */
    scrollbar-gutter: stable;
  }

  :global(body.no-scroll) {
    overflow: hidden;
    touch-action: none;
  }

  ::view-transition-old(root),
  ::view-transition-new(root) {
    animation-duration: 0.25s;
    animation-timing-function: var(--ease-out);
  }

  @media (prefers-reduced-motion: reduce) {
    ::view-transition-old(root),
    ::view-transition-new(root) {
      animation-duration: 0.12s;
    }
  }
</style>
