import { createAuth, initialAuthView, isPin, type AuthView, type PinStore } from "./auth";

// Reactive view of the access-control state. The state machine lives in auth.ts (tested without Svelte).
// The PIN is kept in localStorage only after the user opts in ("Remember PIN on this device").
const PIN_KEY = "can-sniffer.auth.pin";

const pinStorage: PinStore = {
  load() {
    try {
      const value = localStorage.getItem(PIN_KEY);
      return value !== null && isPin(value) ? value : null;
    } catch {
      return null;
    }
  },
  save(pin) {
    try {
      if (pin === null) localStorage.removeItem(PIN_KEY);
      else localStorage.setItem(PIN_KEY, pin);
    } catch {
      // Storage unavailable (private mode, quota): the PIN is simply not remembered.
    }
  },
};

let view = $state.raw<AuthView>(initialAuthView(pinStorage.load() !== null));

const core = createAuth({
  fetch: (url, init) => fetch(url, init),
  now: () => Date.now(),
  sleep: (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  onChange: (next) => {
    view = next;
  },
  savedPin: pinStorage,
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
  forgetPin: core.forgetPin,
  startClaim: core.startClaim,
  retryClaim: core.retryClaim,
  changePin: core.changePin,
  logout: core.logout,
};
