import { untrack } from "svelte";
import { auth } from "./auth.svelte";
import { createDashboardController } from "./dashboardController";
import { createDashboardPersistence } from "./dashboardPersistence";
import { firmwareApi } from "./firmwareApi.svelte";

// Reactive wrapper around the dashboard controller.
const persistence = createDashboardPersistence({
  storage: { getItem: (k) => localStorage.getItem(k), setItem: (k, v) => localStorage.setItem(k, v) },
  sd: {
    download: (path) => firmwareApi.downloadSdFile(path),
    upload: (path, blob) => firmwareApi.uploadSdFile(path, blob),
    copy: (source, destination) => firmwareApi.copySdFile(source, destination),
    remove: (path) => firmwareApi.deleteSdFile(path),
  },
  now: () => Date.now(),
});

let version = $state(0);
const controller = createDashboardController({
  persistence,
  isLoggedIn: () => auth.view.loggedIn,
  now: () => Date.now(),
  onChange: () => {
    // Untracked: a change fired from inside an effect (sync after login) must not make that effect depend on `version`.
    untrack(() => {
      version += 1;
    });
  },
});

/** Read `view` inside an effect or template; it re-runs on every change. Call `sync()` once the user is logged in. */
export const dashboardDocStore = {
  ...controller,
  get view() {
    void version;
    return controller.view();
  },
};
