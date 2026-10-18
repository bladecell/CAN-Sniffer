<script lang="ts">
  import { auth } from "$lib/auth.svelte";
  import { errorText, isPin, PIN_PATTERN, type AuthError } from "$lib/auth";

  let oldPin = $state("");
  let newPin = $state("");
  let repeatPin = $state("");
  let pinError = $state<AuthError | "MISMATCH" | null>(null);
  let pinBusy = $state(false);

  const canChange = $derived(isPin(oldPin) && isPin(newPin) && newPin === repeatPin && !pinBusy);

  async function submitPinChange(e: SubmitEvent) {
    e.preventDefault();
    if (newPin !== repeatPin) {
      pinError = "MISMATCH";
      return;
    }
    pinBusy = true;
    pinError = await auth.changePin(oldPin, newPin);
    pinBusy = false;
    if (pinError === null) {
      oldPin = "";
      newPin = "";
      repeatPin = "";
    }
  }

  const pinMessage = $derived(pinError === null ? "" : pinError === "MISMATCH" ? "The new PINs do not match." : errorText(pinError));
</script>

<section>
  <h2>System Settings</h2>
  <label for="baud">
    Baudrate
    <select id="baud">
      <option>500 kbps</option>
      <option>250 kbps</option>
    </select>
  </label>
  <button onclick={() => alert("Saved!")}>Apply Changes</button>
</section>

<section class="access">
  <h2>Access</h2>
  {#if auth.view.loggedIn}
    <p>You are logged in.</p>
    <button type="button" class="secondary" onclick={() => auth.logout()}>Log out</button>

    <h3>Change PIN</h3>
    <p class="hint">Changing the PIN logs out every session, including this one.</p>
    <form onsubmit={submitPinChange}>
      <label for="old-pin">Current PIN</label>
      <input id="old-pin" type="password" inputmode="numeric" autocomplete="current-password" maxlength="6" pattern={PIN_PATTERN} bind:value={oldPin} required />
      <label for="new-pin">New PIN (6 digits)</label>
      <input id="new-pin" type="password" inputmode="numeric" autocomplete="new-password" maxlength="6" pattern={PIN_PATTERN} bind:value={newPin} required />
      <label for="repeat-pin">Repeat the new PIN</label>
      <input id="repeat-pin" type="password" inputmode="numeric" autocomplete="new-password" maxlength="6" pattern={PIN_PATTERN} bind:value={repeatPin} aria-describedby="pin-msg" required />
      <p id="pin-msg" class="pin-error" role="alert">{pinMessage}</p>
      <button type="submit" disabled={!canChange}>Change PIN</button>
    </form>
  {:else}
    <p>Log in to change settings, manage SD card files, or clear DTCs.</p>
    <button type="button" onclick={() => auth.ensureLogin()}>Log in</button>
  {/if}
  <p class="hint">Forgot the PIN? Hold the BOOT button on the device for 10 seconds. This clears the PIN, and you can set a new one.</p>
</section>

<style>
  .access {
    margin-top: 2rem;
  }
  .hint {
    font-size: 0.875rem;
    color: var(--pico-muted-color);
  }
  .pin-error {
    min-height: 1.25em;
    color: var(--pico-del-color, #b91c1c);
  }
  input {
    letter-spacing: 0.3em;
  }
</style>
