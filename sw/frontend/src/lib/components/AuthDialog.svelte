<script lang="ts">
  import { auth } from "$lib/auth.svelte";
  import { errorText, isPin, type ClaimOutcome } from "$lib/auth";
  import Icon from "$lib/Icon.svelte";
  import Sheet from "./Sheet.svelte";
  import PinField from "./PinField.svelte";
  import PinPad from "./PinPad.svelte";

  // Local form state lives inside the {#if} blocks below, so it resets when the dialog closes.
  let clock = $state(Date.now());

  $effect(() => {
    if (auth.view.dialog === "none") return;
    const id = setInterval(() => (clock = Date.now()), 500);
    return () => clearInterval(id);
  });

  const waitLeft = $derived(Math.max(0, Math.ceil((auth.view.loginWaitUntil - clock) / 1000)));
  const claimLeft = $derived(Math.max(0, Math.ceil((auth.view.claimExpiresAt - clock) / 1000)));

  const notices = {
    session_ended: "Your session ended. Log in to continue.",
    pin_changed: "PIN changed. Log in with the new PIN.",
    saved_pin_removed: "The saved PIN was rejected and removed. Log in with the PIN.",
  };

  let loginPin = $state("");
  let reveal = $state(false);
  // null until the user touches the checkbox: then it follows whether a PIN is already remembered.
  let rememberChoice = $state<boolean | null>(null);
  let newPin = $state("");
  let confirmPin = $state("");
  // The pad types into the claim field that has focus (or was tapped last).
  let claimField = $state<"new" | "repeat">("new");
  let claimMismatch = $derived(confirmPin.length === 6 && newPin !== confirmPin);

  async function submitLogin(e: SubmitEvent) {
    e.preventDefault();
    await auth.login(loginPin, rememberChoice ?? auth.view.pinSaved);
  }

  async function submitClaim(e: SubmitEvent) {
    e.preventDefault();
    if (newPin !== confirmPin) return;
    await auth.startClaim(newPin);
    newPin = "";
    confirmPin = "";
    claimField = "new";
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

<Sheet open={auth.view.dialog !== "none"} {title} onClose={() => auth.closeDialog()}>
  {#if auth.view.dialog !== "none"}
    {#if auth.view.dialog === "login"}
      {#if auth.view.notice}
        <p class="auth-note">{notices[auth.view.notice]}</p>
      {/if}
      <form class="pin-col" onsubmit={submitLogin}>
        <PinField
          id="auth-login-pin"
          label="PIN (6 digits)"
          value={loginPin}
          onchange={(next) => (loginPin = next)}
          reveal={reveal}
          disabled={waitLeft > 0 || auth.view.state === "unavailable"}
          describedby="auth-login-msg"
          autocomplete="current-password"
          autofocus
        />
        <div class="auth-reveal">
          <button type="button" class="btn sm" onclick={() => (reveal = !reveal)}>
            <Icon name={reveal ? "eye-slash" : "eye"} size={18} />
            {reveal ? "Hide PIN" : "Show PIN"}
          </button>
        </div>
        <PinPad value={loginPin} onchange={(next) => (loginPin = next)} disabled={waitLeft > 0 || auth.view.state === "unavailable"} />
        <label class="auth-remember">
          <input
            type="checkbox"
            checked={rememberChoice ?? auth.view.pinSaved}
            onchange={(e) => (rememberChoice = e.currentTarget.checked)}
          />
          Remember PIN on this device
        </label>
        <p class="auth-hint auth-remember-hint">Stored in this browser only. Forget it in Settings.</p>
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
          <button type="button" class="btn" onclick={() => auth.closeDialog()}>Cancel</button>
          <button type="submit" class="btn pri" disabled={!isPin(loginPin) || waitLeft > 0 || auth.view.state === "unavailable"}>Log in</button>
        </div>
      </form>
      <p class="auth-hint pin-col">Forgot the PIN? Hold the BOOT button on the device for 10 seconds. This clears the PIN, and you can set a new one.</p>
    {:else if auth.view.claimPhase === "other"}
      <p>Another client is setting the PIN. Wait for it to finish, or for its window to close in {claimLeft} s.</p>
      <div class="auth-actions">
        <button type="button" class="btn" onclick={() => auth.closeDialog()}>Close</button>
      </div>
    {:else if auth.view.claimPhase === "waiting"}
      <p class="auth-countdown" aria-live="polite">Press the BOOT button on the device. {claimLeft} s left.</p>
      <p class="auth-hint">Nothing is set until you press BOOT. The window closes after 60 seconds.</p>
      <div class="auth-actions">
        <button type="button" class="btn" onclick={() => auth.closeDialog()}>Cancel</button>
      </div>
    {:else if auth.view.claimPhase === "posting"}
      <p aria-live="polite">Sending the PIN to the device.</p>
    {:else if auth.view.claimPhase === "final"}
      <p class="auth-error" role="alert">{finalText[auth.view.claimOutcome ?? "unknown"]}</p>
      <div class="auth-actions">
        <button type="button" class="btn" onclick={() => auth.closeDialog()}>Close</button>
        <button type="button" class="btn pri" onclick={() => auth.retryClaim()}>Start again</button>
      </div>
    {:else}
      <form class="pin-col" onsubmit={submitClaim}>
        <p>This device has no PIN yet. Choose 6 digits. The PIN is set after you press BOOT on the device.</p>
        <PinField
          id="auth-new-pin"
          label="New PIN (6 digits)"
          value={newPin}
          onchange={(next) => (newPin = next)}
          active={claimField === "new"}
          onfocus={() => (claimField = "new")}
          autocomplete="new-password"
          autofocus
        />
        <div class="auth-gap"></div>
        <PinField
          id="auth-confirm-pin"
          label="Repeat PIN (6 digits)"
          value={confirmPin}
          onchange={(next) => (confirmPin = next)}
          active={claimField === "repeat"}
          onfocus={() => (claimField = "repeat")}
          invalid={claimMismatch}
          describedby="auth-claim-msg"
          autocomplete="new-password"
        />
        <PinPad
          value={claimField === "new" ? newPin : confirmPin}
          onchange={(next) => (claimField === "new" ? (newPin = next) : (confirmPin = next))}
        />
        <p id="auth-claim-msg" class="auth-error" role="alert">
          {#if claimMismatch}
            The PINs do not match.
          {:else if auth.view.error}
            {errorText(auth.view.error)}
          {/if}
        </p>
        <div class="auth-actions">
          <button type="button" class="btn" onclick={() => auth.closeDialog()}>Cancel</button>
          <button type="submit" class="btn pri" disabled={!isPin(newPin) || newPin !== confirmPin}>Set PIN</button>
        </div>
      </form>
    {/if}
  {/if}
</Sheet>

<style>
  .auth-error {
    min-height: 1.25em;
    margin: 0.25rem 0 0.75rem;
    color: var(--crit);
  }
  .auth-note,
  .auth-countdown {
    font-weight: 700;
  }
  .auth-hint {
    margin-top: 1rem;
    font-size: 0.82rem;
    color: var(--muted);
  }
  .auth-remember {
    display: flex;
    align-items: center;
    gap: 8px;
    margin: 0.5rem 0 0;
    font-weight: 400;
    cursor: pointer;
  }
  .pin-col {
    max-width: 360px;
    margin-inline: auto;
  }
  .auth-remember input {
    flex: none;
    width: 22px;
    height: 22px;
    margin: 0;
    accent-color: var(--accent);
  }
  .auth-actions .btn {
    flex: 1;
  }
  .auth-remember-hint {
    margin: 0.25rem 0 0;
  }
  .auth-actions {
    display: flex;
    flex-wrap: wrap;
    justify-content: flex-end;
    gap: 10px;
    margin-top: 8px;
  }
  .auth-reveal {
    display: flex;
    justify-content: flex-end;
    margin: 6px 0 12px;
  }
  .auth-gap {
    height: 12px;
  }
</style>
