// The polling and recorder switches, shared by their own cards and the Status card. Called from a component's setup.
import { auth } from "$lib/auth.svelte";
import { connectionStore } from "$lib/connectionStore.svelte";
import { recordingStore } from "$lib/recordingStore.svelte";
import { vehicleStore } from "$lib/vehicleStore.svelte";
import { controlReason } from "$lib/dashboardCardView";
import { loadWhileMounted } from "./cardSources.svelte";

export interface Control {
  readonly on: boolean;
  /** On, Off, Recording, Stopping, Error or No data. */
  readonly word: string;
  /** Why the switch is disabled, or null when it can run. */
  readonly reason: string | null;
  toggle(): void | Promise<void>;
}

const linked = () => ({ linked: connectionStore.connected, busUp: connectionStore.wsCanStatus.state === "CAN Connected" });

/** Polling is switched only by this control. The dashboard never starts it. */
export function pollingControl(): Control {
  loadWhileMounted(() => vehicleStore.getObd2Status());
  let busy = $state(false);
  const status = $derived(vehicleStore.obd2Status);
  const on = $derived(status?.auto_poll ?? false);
  const reason = $derived(
    controlReason({ ...linked(), sdPresent: null, needsLogin: false, loggedIn: auth.canUseGated }) ??
      (status === null ? "No data" : busy ? "Working" : null),
  );
  return {
    get on() {
      return on;
    },
    get word() {
      return status === null ? "No data" : on ? "On" : "Off";
    },
    get reason() {
      return reason;
    },
    async toggle() {
      if (busy || reason !== null) return;
      busy = true;
      try {
        await vehicleStore.setAutoPoll(!on);
      } finally {
        busy = false;
      }
    },
  };
}

/** Recording needs the SD card and a login. */
export function recorderControl(): Control {
  let sdRead = $state(false);
  loadWhileMounted(async () => {
    void recordingStore.refresh();
    await vehicleStore.requestSystem();
    sdRead = true;
  });
  const rec = $derived(recordingStore.status);
  const on = $derived(rec?.state === "recording");
  const stopping = $derived(rec?.state === "stopping" || recordingStore.busy);
  const word = $derived(
    rec === null ? "No data"
    : stopping ? "Stopping"
    : rec.state === "recording" ? "Recording"
    : rec.state === "error" ? "Error"
    : "Off",
  );
  const reason = $derived(
    controlReason({ ...linked(), sdPresent: sdRead ? vehicleStore.system.sd_card_detected : null, needsLogin: true, loggedIn: auth.canUseGated }) ??
      (rec === null ? "No data" : recordingStore.coolingDown ? `Wait ${recordingStore.cooldownLeft} s` : stopping ? "Working" : null),
  );
  return {
    get on() {
      return on;
    },
    get word() {
      return word;
    },
    get reason() {
      return reason;
    },
    toggle() {
      if (reason !== null) return;
      if (on) void recordingStore.stop();
      else void recordingStore.start();
    },
  };
}
