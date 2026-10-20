// Client for the firmware access control (docs/api.md, Authentication).
// Plain TypeScript so the state machine runs under `bun test`; auth.svelte.ts holds the reactive view.
// The session token lives in memory only: no cookie, localStorage or sessionStorage.

export const AUTH_ERRORS = [
  "BAD_PIN",
  "BAD_PIN_FORMAT",
  "LOGIN_BACKOFF",
  "UNAUTHORIZED",
  "NOT_CLAIMED",
  "ALREADY_CLAIMED",
  "CLAIM_CONFLICT",
  "BUTTON_REQUIRED",
  "AUTH_UNAVAILABLE",
  "AUTH_INTERNAL",
  "NETWORK",
  "UNKNOWN",
] as const;
export type AuthError = (typeof AUTH_ERRORS)[number];

export type AuthState = "unknown" | "unclaimed" | "pending" | "claimed" | "unavailable";
export type AuthDialog = "none" | "login" | "claim";
/** idle: nothing running. posting: POST in flight. waiting: our claim, BOOT press pending. other: another client's claim. final: outcome shown, user must retry. */
export type ClaimPhase = "idle" | "posting" | "waiting" | "other" | "final";
export type ClaimOutcome = "committed" | "expired" | "conflict" | "failed" | "unknown";
export type AuthNotice = "session_ended" | "pin_changed";

export interface AuthView {
  state: AuthState;
  apOpen: boolean;
  /** Development firmware only: the device accepts every gated request without a token. */
  devBypass: boolean;
  loggedIn: boolean;
  dialog: AuthDialog;
  error: AuthError | null;
  /** Clock ms before which logins are refused (Retry-After). 0 when a login may be tried. */
  loginWaitUntil: number;
  claimPhase: ClaimPhase;
  /** Clock ms when the pending claim window closes (ours or another client's). */
  claimExpiresAt: number;
  claimOutcome: ClaimOutcome | null;
  notice: AuthNotice | null;
}

export interface AuthDeps {
  fetch(url: string, init?: RequestInit): Promise<Response>;
  now(): number;
  sleep(ms: number): Promise<void>;
  onChange(view: AuthView): void;
}

const API = "/api/v1/auth";
const PIN = /^[0-9]{6}$/;
/** Same rule as isPin, for the HTML pattern attribute. Svelte attributes read `{...}` as an expression, so the pattern must come from here. */
export const PIN_PATTERN = "[0-9]{6}";
const CLAIM_POLL_MS = 1000;
/** Past the claim window, a claim with no final answer is given up after this grace. */
const CLAIM_GRACE_MS = 10_000;
const DEFAULT_IDLE_S = 1800;
const DEFAULT_MAX_S = 43200;
const CLAIM_WINDOW_S = 60;
const JSON_HEADERS = { "Content-Type": "application/json" };

export function initialAuthView(): AuthView {
  return {
    state: "unknown",
    apOpen: false,
    devBypass: false,
    loggedIn: false,
    dialog: "none",
    error: null,
    loginWaitUntil: 0,
    claimPhase: "idle",
    claimExpiresAt: 0,
    claimOutcome: null,
    notice: null,
  };
}

export function isPin(value: string): boolean {
  return PIN.test(value);
}

/** Short plain-language text for an error code. */
export function errorText(error: AuthError): string {
  switch (error) {
    case "BAD_PIN":
      return "Wrong PIN.";
    case "BAD_PIN_FORMAT":
      return "The PIN is 6 digits.";
    case "LOGIN_BACKOFF":
      return "Too many wrong PINs. Wait before trying again.";
    case "UNAUTHORIZED":
      return "Log in to continue.";
    case "NOT_CLAIMED":
      return "This device has no PIN yet. Set one first.";
    case "ALREADY_CLAIMED":
      return "This device already has a PIN. Log in with it.";
    case "CLAIM_CONFLICT":
      return "Another PIN setup is running. Wait for it to finish.";
    case "BUTTON_REQUIRED":
      return "The BOOT button on the device is not working, so the PIN cannot be confirmed.";
    case "AUTH_UNAVAILABLE":
      return "The device cannot read its PIN store. Hold BOOT for 10 seconds to reset it.";
    case "AUTH_INTERNAL":
      return "The device could not save the PIN. Try again.";
    case "NETWORK":
      return "No answer from the device. Check the connection.";
    case "UNKNOWN":
      return "The device returned an unexpected answer.";
  }
}

const isRecord = (v: unknown): v is Record<string, unknown> => typeof v === "object" && v !== null;
const num = (v: unknown, fallback: number): number => (typeof v === "number" && Number.isFinite(v) ? v : fallback);

async function readJson(res: Response): Promise<unknown> {
  try {
    return await res.json();
  } catch {
    return undefined;
  }
}

async function codeOf(res: Response): Promise<string> {
  const body = await readJson(res.clone());
  return isRecord(body) && typeof body.code === "string" ? body.code : "";
}

function parseState(v: unknown): AuthState {
  return v === "unclaimed" || v === "pending" || v === "claimed" || v === "unavailable" ? v : "unknown";
}

/** Token, idle and absolute lifetimes from `POST /auth/login` or a committed claim. */
function parseSession(raw: unknown): { token: string; idleS: number; maxS: number } | null {
  if (!isRecord(raw) || typeof raw.token !== "string" || !/^[0-9a-f]{32}$/i.test(raw.token)) return null;
  return { token: raw.token, idleS: num(raw.idle_timeout_s, DEFAULT_IDLE_S), maxS: num(raw.max_age_s, DEFAULT_MAX_S) };
}

const jsonPost = (body: unknown, method = "POST"): RequestInit => ({ method, headers: JSON_HEADERS, body: JSON.stringify(body) });

function refusedResponse(): Response {
  return new Response(JSON.stringify({ status: "error", reason: errorText("UNAUTHORIZED"), code: "UNAUTHORIZED" }), {
    status: 401,
    headers: JSON_HEADERS,
  });
}

export function createAuth(deps: AuthDeps) {
  let view = initialAuthView();
  let session: { token: string; loginAt: number; lastActivity: number; idleMs: number; maxMs: number } | null = null;
  let waiters: Array<(ok: boolean) => void> = [];
  // Bumped to stop this client's own claim poll.
  let generation = 0;
  // Auth status is only authoritative for the session state under which its request began.
  let authRevision = 0;
  // Ignore out-of-order responses without making an in-flight later request block this one.
  let refreshGeneration = 0;
  let appliedRefreshGeneration = 0;

  function set(patch: Partial<AuthView>) {
    view = { ...view, ...patch };
    deps.onChange(view);
  }

  function startSession(token: string, idleS: number, maxS: number) {
    authRevision++;
    const t = deps.now();
    session = { token, loginAt: t, lastActivity: t, idleMs: idleS * 1000, maxMs: maxS * 1000 };
    set({ loggedIn: true });
  }

  function dropSession(notice: AuthNotice | null = null) {
    if (!session) return;
    authRevision++;
    session = null;
    set({ loggedIn: false, notice: notice ?? view.notice });
  }

  /** The token if the session is still inside its idle and absolute limits; otherwise drops it. */
  function liveToken(): string | null {
    if (!session) return null;
    const deadline = Math.min(session.lastActivity + session.idleMs, session.loginAt + session.maxMs);
    if (deps.now() >= deadline) {
      dropSession("session_ended");
      return null;
    }
    return session.token;
  }

  function touch() {
    if (session) session.lastActivity = deps.now();
  }

  function settleWaiters(ok: boolean) {
    const pending = waiters;
    waiters = [];
    for (const resolve of pending) resolve(ok);
  }

  function reconcileLoginWaiters() {
    if (waiters.length === 0 || view.dialog === "none") return;
    if (view.devBypass || liveToken()) {
      set({ dialog: "none" });
      settleWaiters(true);
    } else {
      set({ dialog: dialogFor(view.state), error: null });
    }
  }

  function dialogFor(state: AuthState): AuthDialog {
    return state === "unclaimed" || state === "pending" ? "claim" : "login";
  }

  /** Resolves true once logged in (also at once if already logged in), false when the dialog is closed. */
  function ensureLogin(): Promise<boolean> {
    if (view.devBypass || liveToken()) return Promise.resolve(true);
    return new Promise((resolve) => {
      waiters.push(resolve);
      if (view.dialog !== "none") return;
      // Show a prompt from cached state; a slow status request must not hide the gated route's next step.
      set({ dialog: dialogFor(view.state), error: null });
      // Re-read the state because a BOOT hold may have cleared or set the PIN since the last request.
      void refresh();
    });
  }

  function closeDialog() {
    generation++;
    const claimPhase = view.claimPhase === "other" ? "other" : "idle";
    set({ dialog: "none", error: null, claimPhase, claimOutcome: null });
    settleWaiters(false);
  }

  function finishLogin() {
    generation++;
    set({
      loggedIn: true,
      state: "claimed",
      dialog: "none",
      error: null,
      loginWaitUntil: 0,
      claimPhase: "idle",
      claimOutcome: null,
      notice: null,
    });
    settleWaiters(true);
  }

  /** Maps a failed response to an error code and applies the side effects that go with it. */
  function failure(res: Response, code: string): AuthError {
    const retry = Number(res.headers.get("Retry-After"));
    const retryMs = Number.isFinite(retry) && retry > 0 ? retry * 1000 : 0;
    if (retryMs > 0 && (res.status === 429 || code === "BAD_PIN")) set({ loginWaitUntil: deps.now() + retryMs });
    if (code === "AUTH_UNAVAILABLE") set({ state: "unavailable" });
    if (code === "NOT_CLAIMED") set({ state: "unclaimed", dialog: view.dialog === "login" ? "claim" : view.dialog });
    return (AUTH_ERRORS as readonly string[]).includes(code) ? (code as AuthError) : "UNKNOWN";
  }

  function send(url: string, init: RequestInit | undefined, token: string | null): Promise<Response> {
    const headers = new Headers(init?.headers);
    if (token) headers.set("Authorization", `Bearer ${token}`);
    return deps.fetch(url, { ...init, headers });
  }

  /** Applies a `GET /auth` body (also the body of a claim poll). Starts the watch on another client's claim. */
  function applyStatus(body: Record<string, unknown>, sentToken: boolean) {
    if (sentToken && session) {
      if (body.authenticated === true) touch();
      else dropSession("session_ended");
    }
    const state = parseState(body.state);
    const pending = state === "pending";
    const prevPhase = view.claimPhase;
    let phase = prevPhase;
    let dialog = view.dialog;
    if (pending && phase === "idle") phase = "other";
    if (!pending && phase === "other") phase = "idle";
    if (state === "claimed" && dialog === "claim" && phase === "idle") dialog = "login";
    if (state === "unclaimed" && dialog === "login") dialog = "claim";
    if (pending && dialog === "login") dialog = "claim";

    const patch: Partial<AuthView> = { state, apOpen: body.ap_open === true, devBypass: body.dev_bypass === true, claimPhase: phase, dialog };
    if (phase === "other" && prevPhase !== "other") patch.claimExpiresAt = deps.now() + num(body.claim_expires_in_s, 0) * 1000;
    const retry = num(body.retry_after_s, 0);
    if (retry > 0) patch.loginWaitUntil = deps.now() + retry * 1000;
    set(patch);
    if (phase === "other" && prevPhase !== "other") void watchOther();
  }

  async function refresh(): Promise<void> {
    const gen = ++refreshGeneration;
    const token = liveToken();
    const revision = authRevision;
    let res: Response;
    try {
      res = await send(API, undefined, token);
    } catch {
      return;
    }
    if (!res.ok) return;
    const body = await readJson(res);
    if (gen > appliedRefreshGeneration && revision === authRevision && isRecord(body)) {
      appliedRefreshGeneration = gen;
      applyStatus(body, token !== null);
      reconcileLoginWaiters();
    }
  }

  /** While another client holds a claim, re-read GET /auth once a second until it ends. */
  async function watchOther() {
    while (view.claimPhase === "other") {
      await deps.sleep(CLAIM_POLL_MS);
      if (view.claimPhase !== "other") return;
      if (deps.now() > view.claimExpiresAt + CLAIM_GRACE_MS) {
        set({ claimPhase: "idle" });
        return;
      }
      await refresh();
    }
  }

  function finalizeClaim(gen: number, outcome: ClaimOutcome) {
    if (gen !== generation) return;
    set({ claimPhase: "final", claimOutcome: outcome });
  }

  async function pollClaim(id: string, gen: number) {
    while (gen === generation) {
      if (deps.now() > view.claimExpiresAt + CLAIM_GRACE_MS) return finalizeClaim(gen, "unknown");
      await deps.sleep(CLAIM_POLL_MS);
      if (gen !== generation) return;
      let res: Response;
      try {
        res = await deps.fetch(`${API}?claim_id=${encodeURIComponent(id)}`);
      } catch {
        continue;
      }
      if (gen !== generation) return;
      const body = res.ok ? await readJson(res) : undefined;
      if (!isRecord(body)) continue;
      applyStatus(body, false);
      const claim = isRecord(body.claim) ? body.claim : {};
      switch (claim.result) {
        case "pending":
          // The window is set once, at the POST. A server countdown must never extend it, or a stuck answer never ends.
          continue;
        case "committed": {
          const session = parseSession(claim);
          if (!session) return finalizeClaim(gen, "unknown");
          startSession(session.token, session.idleS, session.maxS);
          finishLogin();
          return;
        }
        case "expired":
        case "conflict":
        case "failed":
        case "unknown":
          return finalizeClaim(gen, claim.result);
        default:
          return finalizeClaim(gen, "unknown");
      }
    }
  }

  /** Step 1 of the claim: post the PIN, then poll until the BOOT press commits it or the window ends. */
  async function startClaim(pin: string): Promise<void> {
    if (view.claimPhase !== "idle") return;
    if (!isPin(pin)) return set({ error: "BAD_PIN_FORMAT" });
    const gen = ++generation;
    set({ claimPhase: "posting", error: null, claimOutcome: null });
    let res: Response;
    try {
      res = await deps.fetch(`${API}/claim`, jsonPost({ pin }));
    } catch {
      if (gen === generation) set({ claimPhase: "idle", error: "NETWORK" });
      return;
    }
    if (gen !== generation) return;
    if (res.status === 202) {
      const body = await readJson(res);
      const id = isRecord(body) && typeof body.claim_id === "string" ? body.claim_id : "";
      if (!id) return set({ claimPhase: "idle", error: "UNKNOWN" });
      const windowS = isRecord(body) ? num(body.expires_in_s, CLAIM_WINDOW_S) : CLAIM_WINDOW_S;
      set({ claimPhase: "waiting", claimExpiresAt: deps.now() + windowS * 1000 });
      void pollClaim(id, gen);
      return;
    }
    const code = await codeOf(res);
    if (code === "CLAIM_CONFLICT") return set({ claimPhase: "final", claimOutcome: "conflict" });
    if (code === "ALREADY_CLAIMED") return set({ claimPhase: "idle", state: "claimed", dialog: "login", error: "ALREADY_CLAIMED" });
    set({ claimPhase: "idle", error: failure(res, code) });
  }

  async function retryClaim(): Promise<void> {
    set({ claimPhase: "idle", claimOutcome: null, error: null });
    await refresh();
  }

  async function login(pin: string): Promise<void> {
    if (!isPin(pin)) return set({ error: "BAD_PIN_FORMAT" });
    if (deps.now() < view.loginWaitUntil) return set({ error: "LOGIN_BACKOFF" });
    let res: Response;
    try {
      res = await deps.fetch(`${API}/login`, jsonPost({ pin }));
    } catch {
      return set({ error: "NETWORK" });
    }
    if (res.ok) {
      const session = parseSession(await readJson(res));
      if (!session) return set({ error: "UNKNOWN" });
      startSession(session.token, session.idleS, session.maxS);
      finishLogin();
      return;
    }
    set({ error: failure(res, await codeOf(res)) });
  }

  /**
   * Fetch for a gated route. Without a live session it first asks for a login and sends nothing.
   * A 401 UNAUTHORIZED (session ended) or a 403 NOT_CLAIMED mid-session prompts once, then retries the request.
   * A 401 BAD_PIN is an answer to the request (wrong old PIN), not a session end, and is returned as is.
   */
  async function gatedFetch(url: string, init?: RequestInit): Promise<Response> {
    if (!(await ensureLogin())) return refusedResponse();
    const sentToken = liveToken();
    let res = await send(url, init, sentToken);
    if (view.devBypass) return res;
    if (res.status !== 401 && res.status !== 403) {
      touch();
      return res;
    }
    const code = await codeOf(res);
    if (code !== "UNAUTHORIZED" && code !== "NOT_CLAIMED") return res;
    failure(res, code);
    // A late 401 for a token that has since been replaced must not end the newer session.
    if (session?.token === sentToken) dropSession("session_ended");
    if (!(await ensureLogin())) return res;
    res = await send(url, init, liveToken());
    if (res.status !== 401 && res.status !== 403) touch();
    return res;
  }

  /** Returns null on success, else the error code to show. */
  async function changePin(oldPin: string, newPin: string): Promise<AuthError | null> {
    if (!isPin(oldPin) || !isPin(newPin)) return "BAD_PIN_FORMAT";
    if (deps.now() < view.loginWaitUntil) return "LOGIN_BACKOFF";
    let res: Response;
    try {
      res = await gatedFetch(`${API}/pin`, jsonPost({ old_pin: oldPin, new_pin: newPin }, "PUT"));
    } catch {
      return "NETWORK";
    }
    if (res.status === 204) {
      // The change ends every session, the caller's too.
      dropSession();
      set({ notice: "pin_changed" });
      void ensureLogin();
      return null;
    }
    return failure(res, await codeOf(res));
  }

  async function logout(): Promise<void> {
    const token = liveToken();
    if (token) {
      try {
        await send(`${API}/logout`, { method: "POST" }, token);
      } catch {
        // The local session ends either way.
      }
    }
    dropSession();
  }

  return {
    get view() {
      return view;
    },
    refresh,
    gatedFetch,
    ensureLogin,
    closeDialog,
    login,
    startClaim,
    retryClaim,
    changePin,
    logout,
  };
}

export type Auth = ReturnType<typeof createAuth>;
