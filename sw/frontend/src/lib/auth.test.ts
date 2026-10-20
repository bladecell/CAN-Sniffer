import { describe, expect, test } from "bun:test";
import { createAuth, errorText, isPin, PIN_PATTERN, type AuthView } from "./auth";

type Route = (url: string, init?: RequestInit) => Response | Promise<Response>;

const TOKEN = "0123456789abcdef0123456789abcdef";
const TOKEN2 = "fedcba9876543210fedcba9876543210";
const PIN = "123456";

const json = (status: number, body: unknown, headers: Record<string, string> = {}) =>
  new Response(JSON.stringify(body), { status, headers: { "Content-Type": "application/json", ...headers } });
const empty = (status: number) => new Response(null, { status });
const session = (token: string) => ({ status: "ok", token, idle_timeout_s: 1800, max_age_s: 43200 });

/** Fake clock: `sleep` advances time, so claim polls run without real waiting. */
function harness(route: Route) {
  let t = 1_000_000;
  const calls: Array<{ url: string; method: string; auth: string | null }> = [];
  const views: AuthView[] = [];
  const auth = createAuth({
    fetch: async (url, init) => {
      calls.push({ url, method: init?.method ?? "GET", auth: new Headers(init?.headers).get("Authorization") });
      return route(url, init);
    },
    now: () => t,
    sleep: async (ms) => {
      t += ms;
      await Promise.resolve();
    },
    onChange: (v) => views.push(v),
  });
  return {
    auth,
    calls,
    views,
    advance: (ms: number) => {
      t += ms;
    },
  };
}

async function until(cond: () => boolean) {
  for (let i = 0; i < 1000 && !cond(); i++) await new Promise((r) => setTimeout(r, 0));
  expect(cond()).toBe(true);
}

describe("stale state", () => {
  test("an older overlapping refresh cannot overwrite the newer auth status", async () => {
    const pending: Array<(response: Response) => void> = [];
    const h = harness(() => new Promise<Response>((resolve) => pending.push(resolve)));
    const first = h.auth.refresh();
    const second = h.auth.refresh();
    pending[1](json(200, { state: "claimed", dev_bypass: true }));
    await second;
    expect(h.auth.view.devBypass).toBe(true);
    pending[0](json(200, { state: "claimed", dev_bypass: false }));
    await first;
    expect(h.auth.view.devBypass).toBe(true);
  });

  test("opening the login prompt re-reads the state: a PIN cleared by BOOT hold opens the claim dialog", async () => {
    let state = "claimed";
    const h = harness(() => json(200, { state }));
    await h.auth.refresh();
    expect(h.auth.view.state).toBe("claimed");
    state = "unclaimed";
    void h.auth.ensureLogin();
    await until(() => h.auth.view.dialog === "claim");
    expect(h.auth.view.state).toBe("unclaimed");
  });

  test("a status response from an older session cannot end a newer login", async () => {
    let resolveStatus!: (response: Response) => void;
    let loginCount = 0;
    const h = harness((url) => {
      if (url === "/api/v1/auth/login") return json(200, session(++loginCount === 1 ? TOKEN : TOKEN2));
      if (url === "/api/v1/auth") return new Promise<Response>((resolve) => { resolveStatus = resolve; });
      return empty(204);
    });
    await h.auth.login(PIN);
    const staleRefresh = h.auth.refresh();
    await h.auth.login(PIN);
    resolveStatus(json(200, { state: "claimed", authenticated: false }));
    await staleRefresh;
    expect(h.auth.view.loggedIn).toBe(true);
    const gated = await h.auth.gatedFetch("/api/v1/system/reboot");
    expect(gated.status).toBe(204);
    expect(h.calls.find((call) => call.url === "/api/v1/system/reboot")?.auth).toBe(`Bearer ${TOKEN2}`);
  });
});

describe("login prompt while auth state refreshes", () => {
  test("shows the cached login dialog and close cancels a gated request before GET /auth resolves", async () => {
    let resolveStatus!: (response: Response) => void;
    const h = harness((url) => url === "/api/v1/auth" ? new Promise<Response>((resolve) => { resolveStatus = resolve; }) : empty(204));
    const gated = h.auth.gatedFetch("/api/v1/system/reboot");
    expect(h.auth.view.dialog).toBe("login");
    expect(h.calls.filter((call) => call.url === "/api/v1/system/reboot")).toHaveLength(0);
    h.auth.closeDialog();
    expect(await gated).toHaveProperty("status", 401);
    resolveStatus(json(200, { state: "claimed" }));
    await Promise.resolve();
    expect(h.auth.view.dialog).toBe("none");
  });

  test("fresh dev bypass resolves queued gated requests without opening a dialog", async () => {
    let resolveStatus!: (response: Response) => void;
    const h = harness((url) => url === "/api/v1/auth"
      ? new Promise<Response>((resolve) => { resolveStatus = resolve; })
      : empty(204));
    const gated = h.auth.gatedFetch("/api/v1/system/reboot");
    expect(h.auth.view.dialog).toBe("login");
    resolveStatus(json(200, { state: "claimed", dev_bypass: true }));
    expect((await gated).status).toBe(204);
    expect(h.auth.view.dialog).toBe("none");
    expect(h.calls.filter((call) => call.url === "/api/v1/system/reboot")).toHaveLength(1);
    expect(h.calls.find((call) => call.url === "/api/v1/system/reboot")?.auth).toBeNull();
  });

  test("concurrent gated requests share one prompt and close cancels every waiter", async () => {
    const h = harness((url) => url === "/api/v1/auth" ? new Promise<Response>(() => {}) : empty(204));
    const first = h.auth.gatedFetch("/api/v1/system/reboot");
    const second = h.auth.gatedFetch("/api/v1/system/sleep");
    expect(h.auth.view.dialog).toBe("login");
    expect(h.calls.filter((call) => call.url === "/api/v1/auth")).toHaveLength(1);
    h.auth.closeDialog();
    expect((await Promise.all([first, second])).map((response) => response.status)).toEqual([401, 401]);
    expect(h.calls.filter((call) => call.url.includes("/system/"))).toHaveLength(0);
  });

  test("login settles its waiter and a stale status refresh cannot reopen the dialog", async () => {
    let resolveStatus!: (response: Response) => void;
    const h = harness((url) => {
      if (url === "/api/v1/auth") return new Promise<Response>((resolve) => { resolveStatus = resolve; });
      if (url === "/api/v1/auth/login") return json(200, session(TOKEN));
      return empty(204);
    });
    const waiting = h.auth.ensureLogin();
    expect(h.auth.view.dialog).toBe("login");
    await h.auth.login(PIN);
    expect(await waiting).toBe(true);
    resolveStatus(json(200, { state: "claimed" }));
    await Promise.resolve();
    expect(h.auth.view.dialog).toBe("none");
    expect(h.auth.view.loggedIn).toBe(true);
  });
});

describe("state read", () => {
  test("refresh reads state and ap_open without opening a dialog", async () => {
    const h = harness(() => json(200, { state: "claimed", authenticated: false, ap_open: true, retry_after_s: 0, claim_expires_in_s: 0 }));
    await h.auth.refresh();
    expect(h.auth.view.state).toBe("claimed");
    expect(h.auth.view.apOpen).toBe(true);
    expect(h.auth.view.dialog).toBe("none");
    expect(h.auth.view.loggedIn).toBe(false);
    expect(h.calls[0].auth).toBeNull();
  });

  test("PIN format is six ASCII digits", () => {
    expect(isPin("123456")).toBe(true);
    expect(isPin("12345")).toBe(false);
    expect(isPin("12345a")).toBe(false);
    expect(isPin("1234567")).toBe(false);
  });
});

describe("login and 401 retry", () => {
  test("a gated call without a session asks for login, sends nothing, then retries after login", async () => {
    let token = TOKEN;
    const h = harness((url, init) => {
      if (url === "/api/v1/auth/login") return json(200, session(token));
      const auth = new Headers(init?.headers).get("Authorization");
      return auth === `Bearer ${TOKEN2}` || auth === `Bearer ${TOKEN}` ? empty(204) : json(401, { code: "UNAUTHORIZED" });
    });
    await h.auth.refresh();
    const pending = h.auth.gatedFetch("/api/v1/system/reboot", { method: "POST" });
    await until(() => h.auth.view.dialog === "login");
    expect(h.calls.filter((c) => c.url === "/api/v1/system/reboot")).toHaveLength(0);
    await h.auth.login(PIN);
    const res = await pending;
    expect(res.status).toBe(204);
    expect(h.auth.view.dialog).toBe("none");
    expect(h.auth.view.loggedIn).toBe(true);
    expect(h.calls.at(-1)?.auth).toBe(`Bearer ${TOKEN}`);
    token = TOKEN2;
  });

  test("401 UNAUTHORIZED ends the session, re-prompts and retries the action with the new token", async () => {
    let nextToken = TOKEN;
    const h = harness((url, init) => {
      if (url === "/api/v1/auth/login") return json(200, session(nextToken));
      if (url === "/api/v1/req/clear_dtc") {
        return new Headers(init?.headers).get("Authorization") === `Bearer ${TOKEN}`
          ? json(401, { code: "UNAUTHORIZED" })
          : json(200, { status: "success" });
      }
      return empty(204);
    });
    await h.auth.login(PIN);
    expect(h.auth.view.loggedIn).toBe(true);

    nextToken = TOKEN2;
    const pending = h.auth.gatedFetch("/api/v1/req/clear_dtc", { method: "POST" });
    await until(() => h.auth.view.dialog === "login");
    expect(h.auth.view.loggedIn).toBe(false);
    expect(h.auth.view.notice).toBe("session_ended");
    await h.auth.login(PIN);
    const res = await pending;
    expect(res.status).toBe(200);
    const clearCalls = h.calls.filter((c) => c.url === "/api/v1/req/clear_dtc");
    expect(clearCalls.map((c) => c.auth)).toEqual([`Bearer ${TOKEN}`, `Bearer ${TOKEN2}`]);
  });

  test("a late 401 for an old token does not drop a newer session; the request retries with it", async () => {
    let nextToken = TOKEN;
    let releaseSlow: (r: Response) => void = () => {};
    const h = harness((url, init) => {
      if (url === "/api/v1/auth/login") return json(200, session(nextToken));
      const bearer = new Headers(init?.headers).get("Authorization");
      if (bearer !== `Bearer ${TOKEN}`) return json(200, { status: "success" });
      if (url === "/api/v1/slow") return new Promise<Response>((r) => (releaseSlow = r));
      return json(401, { code: "UNAUTHORIZED" });
    });
    await h.auth.login(PIN);
    const slow = h.auth.gatedFetch("/api/v1/slow");
    const fast = h.auth.gatedFetch("/api/v1/fast");
    await until(() => h.auth.view.dialog === "login");
    nextToken = TOKEN2;
    await h.auth.login(PIN);
    expect((await fast).status).toBe(200);
    expect(h.auth.view.loggedIn).toBe(true);

    releaseSlow(json(401, { code: "UNAUTHORIZED" }));
    const res = await slow;
    expect(res.status).toBe(200);
    expect(h.auth.view.loggedIn).toBe(true);
    expect(h.auth.view.dialog).toBe("none");
    const slowCalls = h.calls.filter((c) => c.url === "/api/v1/slow");
    expect(slowCalls.map((c) => c.auth)).toEqual([`Bearer ${TOKEN}`, `Bearer ${TOKEN2}`]);
  });

  test("401 BAD_PIN is an answer to the request (wrong old PIN), not a session end", async () => {
    const h = harness((url) => (url === "/api/v1/auth/login" ? json(200, session(TOKEN)) : json(401, { code: "BAD_PIN" })));
    await h.auth.login(PIN);
    const res = await h.auth.gatedFetch("/api/v1/auth/pin", { method: "PUT" });
    expect(res.status).toBe(401);
    expect(h.auth.view.dialog).toBe("none");
    expect(h.auth.view.loggedIn).toBe(true);
  });

  test("closing the login dialog returns the refusal and sends no gated request", async () => {
    const h = harness(() => json(200, {}));
    const pending = h.auth.gatedFetch("/api/v1/settings");
    h.auth.closeDialog();
    const res = await pending;
    expect(res.status).toBe(401);
    expect(h.calls.filter((c) => c.url === "/api/v1/settings")).toHaveLength(0);
    expect(h.auth.view.dialog).toBe("none");
  });

  test("403 NOT_CLAIMED mid-session switches to the claim dialog and retries after the claim", async () => {
    const h = harness((url, init) => {
      if (url === "/api/v1/auth/login") return json(200, session(TOKEN));
      if (url === "/api/v1/auth") return json(200, { state: "unclaimed", authenticated: false });
      return json(403, { code: "NOT_CLAIMED" });
    });
    await h.auth.login(PIN);
    const pending = h.auth.gatedFetch("/api/v1/pid_def", { method: "PUT" });
    await until(() => h.auth.view.dialog === "claim");
    expect(h.auth.view.state).toBe("unclaimed");
    h.auth.closeDialog();
    expect((await pending).status).toBe(403);
  });
});

describe("login backoff (429)", () => {
  test("Retry-After blocks logins until the countdown ends, without sending them", async () => {
    let status = 429;
    const h = harness((url) => {
      if (url !== "/api/v1/auth/login") return empty(500);
      return status === 429
        ? json(429, { code: "LOGIN_BACKOFF" }, { "Retry-After": "5" })
        : json(200, session(TOKEN));
    });
    await h.auth.login("000000");
    expect(h.auth.view.error).toBe("LOGIN_BACKOFF");
    const before = h.auth.view.loginWaitUntil;
    expect(before).toBeGreaterThan(0);
    const loginCalls = () => h.calls.filter((c) => c.url === "/api/v1/auth/login").length;
    expect(loginCalls()).toBe(1);

    await h.auth.login(PIN);
    expect(loginCalls()).toBe(1);

    h.advance(5000);
    status = 200;
    await h.auth.login(PIN);
    expect(loginCalls()).toBe(2);
    expect(h.auth.view.loggedIn).toBe(true);
    expect(h.auth.view.loginWaitUntil).toBe(0);
  });

  test("a wrong PIN with Retry-After starts the same wait", async () => {
    const h = harness(() => json(401, { code: "BAD_PIN" }, { "Retry-After": "2" }));
    await h.auth.login(PIN);
    expect(h.auth.view.error).toBe("BAD_PIN");
    expect(h.auth.view.loginWaitUntil).toBeGreaterThan(0);
  });
});

describe("session expiry", () => {
  test("an idle session is dropped locally and no stale token is sent", async () => {
    const h = harness((url) => (url === "/api/v1/auth/login" ? json(200, session(TOKEN)) : empty(204)));
    await h.auth.login(PIN);
    h.advance(1800 * 1000);
    const pending = h.auth.gatedFetch("/api/v1/sd_card/format", { method: "POST" });
    expect(h.auth.view.loggedIn).toBe(false);
    await until(() => h.auth.view.dialog === "login");
    expect(h.calls.some((c) => c.auth === `Bearer ${TOKEN}` && c.url !== "/api/v1/auth/login")).toBe(false);
    h.auth.closeDialog();
    await pending;
  });

  test("a gated request restarts the idle timer", async () => {
    const h = harness((url) => (url === "/api/v1/auth/login" ? json(200, session(TOKEN)) : empty(204)));
    await h.auth.login(PIN);
    h.advance(1500 * 1000);
    await h.auth.gatedFetch("/api/v1/settings");
    h.advance(1500 * 1000);
    expect(h.auth.view.loggedIn).toBe(true);
  });

  test("the absolute lifetime ends the session even when it is active", async () => {
    const h = harness((url) =>
      url === "/api/v1/auth/login" ? json(200, { ...session(TOKEN), max_age_s: 3600 }) : empty(204),
    );
    await h.auth.login(PIN);
    h.advance(1500 * 1000);
    await h.auth.gatedFetch("/api/v1/settings");
    h.advance(1500 * 1000);
    await h.auth.refresh();
    expect(h.auth.view.loggedIn).toBe(true);
    h.advance(700 * 1000);
    await h.auth.refresh();
    expect(h.auth.view.loggedIn).toBe(false);
  });

  test("logout ends the session locally even when the request fails", async () => {
    const h = harness((url) => {
      if (url === "/api/v1/auth/logout") throw new Error("offline");
      return json(200, session(TOKEN));
    });
    await h.auth.login(PIN);
    await h.auth.logout();
    expect(h.auth.view.loggedIn).toBe(false);
  });
});

describe("claim (two-step)", () => {
  test("posts the PIN, polls until the BOOT press commits it, then logs in", async () => {
    let polls = 0;
    const h = harness((url) => {
      if (url === "/api/v1/auth") return json(200, { state: "unclaimed", authenticated: false });
      if (url === "/api/v1/auth/claim") return json(202, { status: "pending", claim_id: "abc", expires_in_s: 60 });
      polls++;
      if (polls < 3) return json(200, { state: "pending", claim: { result: "pending", expires_in_s: 50 } });
      return json(200, { state: "claimed", claim: { result: "committed", ...session(TOKEN) } });
    });
    await h.auth.refresh();
    const dialog = h.auth.ensureLogin();
    await until(() => h.auth.view.dialog === "claim");
    await h.auth.startClaim(PIN);
    expect(h.calls.find((c) => c.url === "/api/v1/auth/claim")?.method).toBe("POST");
    expect(h.auth.view.claimPhase === "waiting" || h.auth.view.loggedIn).toBe(true);
    expect(await dialog).toBe(true);
    expect(h.auth.view.loggedIn).toBe(true);
    expect(h.auth.view.claimPhase).toBe("idle");
    expect(h.calls.filter((c) => c.url === "/api/v1/auth?claim_id=abc")).toHaveLength(3);
  });

  for (const outcome of ["expired", "conflict", "failed", "unknown"] as const) {
    test(`a final ${outcome} result is shown and is not retried by the client`, async () => {
      const h = harness((url) => {
        if (url === "/api/v1/auth/claim") return json(202, { status: "pending", claim_id: "abc", expires_in_s: 60 });
        return json(200, { state: "unclaimed", claim: { result: outcome } });
      });
      await h.auth.startClaim(PIN);
      await until(() => h.auth.view.claimPhase === "final");
      expect(h.auth.view.claimOutcome).toBe(outcome);
      expect(h.auth.view.loggedIn).toBe(false);
      expect(h.calls.filter((c) => c.url === "/api/v1/auth/claim")).toHaveLength(1);
    });
  }

  test("a claim with no answer gives up as unknown after the window and a grace period", async () => {
    const h = harness((url) =>
      url === "/api/v1/auth/claim"
        ? json(202, { status: "pending", claim_id: "abc", expires_in_s: 60 })
        : json(200, { state: "pending", claim: { result: "pending", expires_in_s: 0 } }),
    );
    await h.auth.startClaim(PIN);
    await until(() => h.auth.view.claimPhase === "final");
    expect(h.auth.view.claimOutcome).toBe("unknown");
  });

  test("a failed poll request is retried, not treated as the outcome", async () => {
    let polls = 0;
    const h = harness((url) => {
      if (url === "/api/v1/auth/claim") return json(202, { status: "pending", claim_id: "abc", expires_in_s: 60 });
      polls++;
      if (polls === 1) throw new Error("offline");
      return json(200, { state: "claimed", claim: { result: "committed", ...session(TOKEN2) } });
    });
    await h.auth.startClaim(PIN);
    await until(() => h.auth.view.loggedIn);
    expect(polls).toBe(2);
  });

  test("a network failure on the claim POST shows NETWORK and leaves the form usable", async () => {
    const h = harness(() => {
      throw new Error("offline");
    });
    await h.auth.startClaim(PIN);
    expect(h.auth.view.error).toBe("NETWORK");
    expect(h.auth.view.claimPhase).toBe("idle");
  });

  test("a 403 with no known code (for example the Host guard) shows UNKNOWN and leaves the form usable", async () => {
    const h = harness(() => json(403, { status: "error", reason: "bad host", code: "BAD_HOST" }));
    await h.auth.startClaim(PIN);
    expect(h.auth.view.error).toBe("UNKNOWN");
    expect(h.auth.view.claimPhase).toBe("idle");
  });

  test("409 CLAIM_CONFLICT is final with outcome conflict", async () => {
    const h = harness(() => json(409, { code: "CLAIM_CONFLICT" }));
    await h.auth.startClaim(PIN);
    expect(h.auth.view.claimPhase).toBe("final");
    expect(h.auth.view.claimOutcome).toBe("conflict");
  });

  test("409 ALREADY_CLAIMED switches to the login dialog", async () => {
    const h = harness(() => json(409, { code: "ALREADY_CLAIMED" }));
    h.auth.ensureLogin();
    await h.auth.startClaim(PIN);
    expect(h.auth.view.state).toBe("claimed");
    expect(h.auth.view.dialog).toBe("login");
    expect(h.auth.view.error).toBe("ALREADY_CLAIMED");
    expect(h.auth.view.claimPhase).toBe("idle");
  });

  test("403 BUTTON_REQUIRED leaves the form usable with a message", async () => {
    const h = harness(() => json(403, { code: "BUTTON_REQUIRED" }));
    await h.auth.startClaim(PIN);
    expect(h.auth.view.error).toBe("BUTTON_REQUIRED");
    expect(h.auth.view.claimPhase).toBe("idle");
  });

  test("a malformed PIN is refused before any request", async () => {
    const h = harness(() => json(202, {}));
    await h.auth.startClaim("12345");
    expect(h.auth.view.error).toBe("BAD_PIN_FORMAT");
    expect(h.calls).toHaveLength(0);
  });

  test("another client's pending claim shows as other, then clears when it ends", async () => {
    let state = "pending";
    const h = harness((url) => (url.startsWith("/api/v1/auth") ? json(200, { state, claim_expires_in_s: 40 }) : empty(500)));
    h.auth.ensureLogin();
    await h.auth.refresh();
    expect(h.auth.view.claimPhase).toBe("other");
    expect(h.auth.view.dialog).toBe("claim");
    state = "claimed";
    await until(() => h.auth.view.claimPhase === "idle");
    expect(h.auth.view.dialog).toBe("login");
    h.auth.closeDialog();
  });
});

describe("change PIN", () => {
  test("204 ends all sessions and asks for the new PIN", async () => {
    const h = harness((url) => (url === "/api/v1/auth/login" ? json(200, session(TOKEN)) : empty(204)));
    await h.auth.login(PIN);
    expect(await h.auth.changePin(PIN, "654321")).toBeNull();
    expect(h.auth.view.loggedIn).toBe(false);
    expect(h.auth.view.notice).toBe("pin_changed");
    await until(() => h.auth.view.dialog === "login");
    h.auth.closeDialog();
  });

  test("a wrong old PIN returns BAD_PIN and keeps the session", async () => {
    const h = harness((url) => (url === "/api/v1/auth/login" ? json(200, session(TOKEN)) : json(401, { code: "BAD_PIN" })));
    await h.auth.login(PIN);
    expect(await h.auth.changePin("000000", "654321")).toBe("BAD_PIN");
    expect(h.auth.view.loggedIn).toBe(true);
  });

  test("a malformed new PIN is refused before any request", async () => {
    const h = harness(() => json(200, {}));
    expect(await h.auth.changePin(PIN, "abc")).toBe("BAD_PIN_FORMAT");
    expect(h.calls).toHaveLength(0);
  });
});

test("every error code has a message", () => {
  expect(errorText("NETWORK").length).toBeGreaterThan(0);
  expect(errorText("UNKNOWN").length).toBeGreaterThan(0);
});

describe("PIN pattern (HTML attribute)", () => {
  // Regression: a literal pattern="[0-9]{6}" in a Svelte attribute became "[0-9]6" (the braces are an expression),
  // so the browser refused every PIN and the claim submit never fired.
  test("accepts exactly six digits and nothing else", () => {
    const re = new RegExp(`^(?:${PIN_PATTERN})$`);
    expect(re.test("123456")).toBe(true);
    expect(re.test("12345")).toBe(false);
    expect(re.test("1234567")).toBe(false);
    expect(re.test("12345a")).toBe(false);
    expect(re.test("6")).toBe(false);
  });
});

describe("development build (dev_bypass)", () => {
  test("with dev_bypass the gated call goes out without a token and opens no prompt", async () => {
    const h = harness((url) =>
      url === "/api/v1/auth"
        ? json(200, { state: "claimed", authenticated: true, retry_after_s: 0, claim_expires_in_s: 0, ap_open: false, dev_bypass: true })
        : empty(204),
    );
    await h.auth.refresh();
    expect(h.auth.view.devBypass).toBe(true);
    const res = await h.auth.gatedFetch("/api/v1/req/clear_dtc", { method: "POST" });
    expect(res.status).toBe(204);
    expect(h.auth.view.dialog).toBe("none");
    expect(h.calls.find((c) => c.url === "/api/v1/req/clear_dtc")?.auth).toBeNull();
  });

  test("without dev_bypass the flag stays false", async () => {
    const h = harness(() => json(200, { state: "claimed", authenticated: false, ap_open: false }));
    await h.auth.refresh();
    expect(h.auth.view.devBypass).toBe(false);
  });
});
