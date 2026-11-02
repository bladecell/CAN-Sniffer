// Reactive state for GET/PATCH /settings (docs/api.md). The screen decides when to load: the endpoint is gated.
import { parseJsonResponse } from "./api";
import { alertStore } from "./alertStore.svelte";
import { firmwareApi } from "./firmwareApi.svelte";
import { parseSettingsDoc, type Patch, type SettingsDoc } from "./settingsModel";

const errorText = (e: unknown) => (e instanceof Error ? e.message : String(e));

class SettingsStore {
  doc = $state.raw<SettingsDoc | null>(null);
  loading = $state(false);
  loadError = $state<string | null>(null);
  saving = $state(false);
  /** Set after any successful save: values take effect after a reboot (docs/api.md, PATCH /settings). */
  restartRequired = $state(false);

  async load(): Promise<void> {
    this.loading = true;
    this.loadError = null;
    try {
      const doc = parseSettingsDoc(await parseJsonResponse<unknown>(await firmwareApi.getSettings()));
      if (doc === null) throw new Error("Unexpected settings reply");
      this.doc = doc;
    } catch (e) {
      this.loadError = errorText(e);
    } finally {
      this.loading = false;
    }
  }

  /** One section per call. Returns true when the firmware saved it. */
  async save(section: "wifi" | "can" | "system", fields: Patch): Promise<boolean> {
    if (Object.keys(fields).length === 0) return true;
    this.saving = true;
    try {
      await parseJsonResponse<unknown>(await firmwareApi.patchSettings({ [section]: fields }));
      this.restartRequired = true;
      alertStore.add("Saved. Restart the device to apply.", "success");
      await this.load();
      return true;
    } catch (e) {
      alertStore.add(`Not saved: ${errorText(e)}`, "error");
      return false;
    } finally {
      this.saving = false;
    }
  }
}

export const settingsStore = new SettingsStore();
