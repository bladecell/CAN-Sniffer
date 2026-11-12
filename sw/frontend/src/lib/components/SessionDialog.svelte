<script lang="ts">
  import { connectionStore } from "$lib/connectionStore.svelte";
  import Dialog from "./Dialog.svelte";
  import { untrack } from "svelte";
  import { alertStore } from "$lib/alertStore.svelte";

  // The device serves one session. `inUse`: this page was refused. `replaced`: another page took over.
  const session = $derived(connectionStore.session);
  const asking = $derived(session === "inUse" || session === "replaced");
  const replaced = $derived(session === "replaced");

  // Parked: a persistent alert (same toast as "WebSocket Disconnected") until the user takes over.
  $effect(() => {
    if (session !== "parked") return;
    // untrack: add() and remove() read the alerts list, which would make this effect depend on what it writes.
    const id = untrack(() =>
      alertStore.add("Not connected: another tab or device is using it.", "warning", Infinity, {
        label: "Take over",
        run: () => connectionStore.takeOver(),
      }),
    );
    return () => untrack(() => alertStore.remove(id));
  });
</script>

<Dialog
  open={asking}
  title={replaced ? "Session moved" : "Device in use"}
  message={replaced
    ? "Another tab or device is connected now. Live data stopped here."
    : "Another tab or device is connected. Take over to disconnect it."}
  confirmLabel={replaced ? "Use here" : "Take over"}
  cancelLabel={replaced ? "Close" : "Not now"}
  onConfirm={() => connectionStore.takeOver()}
  onCancel={() => connectionStore.parkSession()}
/>
