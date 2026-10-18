<script lang="ts">
  import { auth } from "$lib/auth.svelte";
  import { errorText, isPin, PIN_PATTERN, type ClaimOutcome } from "$lib/auth";
  import { dialogFocus } from "$lib/dialogFocus";

  // Local form state lives inside the {#if} blocks below, so it resets when the dialog closes.
  let clock = $state(Date.now());

  $effect(() => {
    if (auth.view.dialog === "none") return;
    const id = setInterval(() => (clock = Date.now()), 500);
    return () => clearInterval(id);
  });

  const waitLeft = $derived(Math.max(0, Math.ceil((auth.view.loginWaitUntil - clock) / 1000)));
  const claimLeft = $derived(Math.max(0, Math.ceil((auth.view.claimExpiresAt - clock) / 1000)));

  const notices = { session_ended: "Your session ended. Log in to continue.", pin_changed: "PIN changed. Log in with the new PIN." };

  let loginPin = $state("");
  let newPin = $state("");
  let confirmPin = $state("");
  let claimMismatch = $derived(confirmPin.length === 6 && newPin !== confirmPin);

  async function submitLogin(e: SubmitEvent) {
    e.preventDefault();
    await auth.login(loginPin);
  }

  async function submitClaim(e: SubmitEvent) {
    e.preventDefault();
    if (newPin !== confirmPin) return;
    await auth.startClaim(newPin);
    newPin = "";
    confirmPin = "";
  }

  const finalText: Record<ClaimOutcome, string> = {
    committed: "The PIN is set.",
    expired: "No BOOT press within 60 seconds, so no PIN was set.",
    conflict: "Another PIN setup started, so both were cancelled.",
    failed: "The device could not store the PIN. Nothing changed.",
    unknown: "The device gave no answer. If the PIN was set, log in with it.",
  };

  const title = $derived(
    auth.view.dialog === "login"
      ? "Log in"
      : auth.view.claimPhase === "other"
        ? "Setup in progress"
        : auth.view.claimPhase === "waiting"
          ? "Confirm on the device"
          : auth.view.claimPhase === "final"
            ? "PIN not set"
            : "Set a PIN",
  );
</script>

{#if auth.view.dialog !== "none"}
  <div
    class="auth-backdrop"
    role="presentation"
    onclick={(e) => {
      if (e.target === e.currentTarget) auth.closeDialog();
    }}
  >
    <div
      class="auth-dialog"
      role="dialog"
      aria-modal="true"
      aria-labelledby="auth-title"
      tabindex="-1"
      {@attach dialogFocus(() => auth.closeDialog())}
    >
      <header>
        <h2 id="auth-title">{title}</h2>
        <button type="button" class="auth-close" aria-label="Close" onclick={() => auth.closeDialog()}>×</button>
      </header>

      {#if auth.view.dialog === "login"}
        {#if auth.view.notice}
          <p class="auth-note">{notices[auth.view.notice]}</p>
        {/if}
        <form onsubmit={submitLogin}>
          <label for="auth-login-pin">PIN (6 digits)</label>
          <input
            id="auth-login-pin"
            type="password"
            inputmode="numeric"
            autocomplete="off"
            maxlength="6"
            pattern={PIN_PATTERN}
            bind:value={loginPin}
            disabled={waitLeft > 0 || auth.view.state === "unavailable"}
            aria-describedby="auth-login-msg"
            required
          />
          <p id="auth-login-msg" class="auth-error" role="alert">
            {#if auth.view.state === "unavailable"}
              {errorText("AUTH_UNAVAILABLE")}
            {:else if waitLeft > 0}
              {errorText("LOGIN_BACKOFF")} Try again in {waitLeft} s.
            {:else if auth.view.error}
              {errorText(auth.view.error)}
            {/if}
          </p>
          <div class="auth-actions">
            <button type="button" class="secondary" onclick={() => auth.closeDialog()}>Cancel</button>
            <button type="submit" disabled={!isPin(loginPin) || waitLeft > 0 || auth.view.state === "unavailable"}>Log in</button>
          </div>
        </form>
        <p class="auth-hint">Forgot the PIN? Hold the BOOT button on the device for 10 seconds. This clears the PIN, and you can set a new one.</p>
      {:else if auth.view.claimPhase === "other"}
        <p>Another client is setting the PIN. Wait for it to finish, or for its window to close in {claimLeft} s.</p>
        <div class="auth-actions">
          <button type="button" class="secondary" onclick={() => auth.closeDialog()}>Close</button>
        </div>
      {:else if auth.view.claimPhase === "waiting"}
        <p class="auth-countdown" aria-live="polite">Press the BOOT button on the device. {claimLeft} s left.</p>
        <p class="auth-hint">Nothing is set until you press BOOT. The window closes after 60 seconds.</p>
        <div class="auth-actions">
          <button type="button" class="secondary" onclick={() => auth.closeDialog()}>Cancel</button>
        </div>
      {:else if auth.view.claimPhase === "posting"}
        <p aria-live="polite">Sending the PIN to the device.</p>
      {:else if auth.view.claimPhase === "final"}
        <p class="auth-error" role="alert">{finalText[auth.view.claimOutcome ?? "unknown"]}</p>
        <div class="auth-actions">
          <button type="button" class="secondary" onclick={() => auth.closeDialog()}>Close</button>
          <button type="button" onclick={() => auth.retryClaim()}>Start again</button>
        </div>
      {:else}
        <form onsubmit={submitClaim}>
          <p>This device has no PIN yet. Choose 6 digits. The PIN is set after you press BOOT on the device.</p>
          <label for="auth-new-pin">New PIN (6 digits)</label>
          <input id="auth-new-pin" type="password" inputmode="numeric" autocomplete="new-password" maxlength="6" pattern={PIN_PATTERN} bind:value={newPin} required />
          <label for="auth-confirm-pin">Repeat the PIN</label>
          <input
            id="auth-confirm-pin"
            type="password"
            inputmode="numeric"
            autocomplete="new-password"
            maxlength="6"
            pattern={PIN_PATTERN}
            bind:value={confirmPin}
            aria-describedby="auth-claim-msg"
            required
          />
          <p id="auth-claim-msg" class="auth-error" role="alert">
            {#if claimMismatch}
              The PINs do not match.
            {:else if auth.view.error}
              {errorText(auth.view.error)}
            {/if}
          </p>
          <div class="auth-actions">
            <button type="button" class="secondary" onclick={() => auth.closeDialog()}>Cancel</button>
            <button type="submit" disabled={!isPin(newPin) || newPin !== confirmPin}>Set PIN</button>
          </div>
        </form>
      {/if}
    </div>
  </div>
{/if}

<style>
  .auth-backdrop {
    position: fixed;
    inset: 0;
    z-index: 10000;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 1rem;
    background: rgb(0 0 0 / 0.5);
  }
  .auth-dialog {
    width: min(100%, 420px);
    max-height: 90vh;
    overflow-y: auto;
    padding: 1.25rem 1.5rem;
    border-radius: var(--pico-border-radius);
    background: var(--pico-background-color);
    border: 1px solid var(--pico-border-color);
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 1rem;
  }
  h2 {
    margin: 0 0 0.75rem;
    font-size: 1.15rem;
  }
  .auth-close {
    width: 44px;
    min-height: 44px;
    margin: -0.5rem -0.75rem 0 0;
    padding: 0;
    background: transparent;
    border: none;
    color: var(--pico-muted-color);
    font-size: 1.4rem;
    cursor: pointer;
  }
  .auth-error {
    min-height: 1.25em;
    margin: 0.25rem 0 0.75rem;
    color: var(--pico-del-color, #b91c1c);
  }
  .auth-note,
  .auth-countdown {
    font-weight: 600;
  }
  .auth-hint {
    margin-top: 1rem;
    font-size: 0.875rem;
    color: var(--pico-muted-color);
  }
  .auth-actions {
    display: flex;
    flex-wrap: wrap;
    justify-content: flex-end;
    gap: 0.5rem;
  }
  .auth-actions button {
    min-height: 44px;
    width: auto;
    margin: 0;
  }
  input {
    margin-bottom: 0.75rem;
    letter-spacing: 0.3em;
  }
</style>
