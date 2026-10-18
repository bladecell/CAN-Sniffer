import { createAuth, initialAuthView, type AuthView } from "./auth";

// Reactive view of the access-control state. The state machine lives in auth.ts (tested without Svelte).
let view = $state.raw<AuthView>(initialAuthView());

const core = createAuth({
  fetch: (url, init) => fetch(url, init),
  now: () => Date.now(),
  sleep: (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  onChange: (next) => {
    view = next;
  },
});

export const auth = {
  get view() {
    return view;
  },
  refresh: core.refresh,
  gatedFetch: core.gatedFetch,
  ensureLogin: core.ensureLogin,
  closeDialog: core.closeDialog,
  login: core.login,
  startClaim: core.startClaim,
  retryClaim: core.retryClaim,
  changePin: core.changePin,
  logout: core.logout,
};
