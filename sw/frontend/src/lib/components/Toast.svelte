<script lang="ts">
  import { alertStore } from "$lib/alertStore.svelte";
  import { escapeTarget } from "$lib/toast";
  import { fly } from "svelte/transition";
  import { flip } from "svelte/animate";
  import type { Alert } from "$lib/types";
  import Icon from "$lib/Icon.svelte";

  const reduceMotion = typeof matchMedia === "function" && matchMedia("(prefers-reduced-motion: reduce)").matches;

  const icons: Record<Alert["type"], string> = {
    success: "circle-check",
    error: "circle-xmark",
    warning: "circle-exclamation",
    info: "circle-info",
  };

  // Esc dismisses the newest toast, unless a dialog or popover is open (they handle Esc first).
  function onKeydown(e: KeyboardEvent) {
    if (e.key !== "Escape") return;
    const overlayOpen = document.querySelector("dialog[open], :popover-open") !== null;
    const id = escapeTarget(
      alertStore.alerts.map((a) => a.id),
      overlayOpen,
    );
    if (id !== null) alertStore.remove(id);
  }
</script>

<svelte:window onkeydown={onKeydown} />

<div class="toasts">
  {#each alertStore.alerts as alert (alert.id)}
    <div
      class="toast float-layer {alert.type}"
      role={alert.type === "error" ? "alert" : "status"}
      in:fly={{ y: reduceMotion ? 0 : 16, duration: reduceMotion ? 0 : 300 }}
      out:fly={{ y: reduceMotion ? 0 : 8, duration: reduceMotion ? 0 : 200 }}
      animate:flip
      onpointerenter={() => alertStore.pause(alert.id)}
      onpointerleave={(e) => {
        if (!e.currentTarget.contains(document.activeElement)) alertStore.resume(alert.id);
      }}
      onfocusin={() => alertStore.pause(alert.id)}
      onfocusout={(e) => {
        if (!e.currentTarget.contains(e.relatedTarget as Node | null)) alertStore.resume(alert.id);
      }}
    >
      <span class="ic"><Icon name={icons[alert.type]} size={20} /></span>
      <span class="msg">{alert.message}</span>
      <button type="button" class="close" aria-label="Dismiss notification" onclick={() => alertStore.remove(alert.id)}>
        <Icon name="x" size={18} />
      </button>
    </div>
  {/each}
</div>

<style>
  /* Centred with auto margins, not a transform: a transformed box would drop the blur of the toasts inside it. */
  .toasts {
    position: fixed;
    left: 0;
    right: 0;
    bottom: 30px;
    z-index: 100;
    display: grid;
    justify-items: center;
    gap: 8px;
    width: max-content;
    max-width: calc(100vw - 24px);
    margin: 0 auto;
    pointer-events: none;
  }
  /* Above the bottom nav bar (DESIGN.md, Toast). */
  @media (max-width: 859px) {
    .toasts {
      bottom: calc(100px + env(safe-area-inset-bottom));
    }
  }

  .toast {
    pointer-events: auto;
    display: flex;
    align-items: center;
    gap: 10px;
    flex: none;
    width: max-content;
    max-width: min(480px, calc(100vw - 24px));
    padding: 8px 8px 8px 16px;
    border-radius: var(--r-float);
    color: var(--text);
    font-weight: 700;
  }
  .ic {
    display: inline-flex;
    flex: none;
  }
  .success .ic {
    color: var(--ok);
  }
  .error .ic {
    color: var(--crit);
  }
  .warning .ic {
    color: var(--warn);
  }
  .info .ic {
    color: var(--accent-text);
  }
  .msg {
    min-width: 0;
    overflow-wrap: anywhere;
  }
  .close {
    flex: none;
    display: grid;
    place-items: center;
    width: 36px;
    height: 36px;
    margin: 0;
    padding: 0;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    color: var(--muted);
    cursor: pointer;
  }
  .close:hover {
    background: var(--hov);
  }
  @media (pointer: coarse) {
    .close {
      width: 44px;
      height: 44px;
    }
  }
</style>
