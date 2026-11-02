<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { auth } from "$lib/auth.svelte";
  import { errorText, isPin, PIN_PATTERN, type AuthError } from "$lib/auth";
  import { GROUP, ROW, anyShown, rowShown, type RowMeta } from "$lib/settingsModel";
  import SettingRow from "./SettingRow.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.access, row);

  let oldPin = $state("");
  let newPin = $state("");
  let repeatPin = $state("");
  let pinError = $state<AuthError | "MISMATCH" | null>(null);
  let pinBusy = $state(false);

  const canChange = $derived(isPin(oldPin) && isPin(newPin) && newPin === repeatPin && !pinBusy);
  const pinMessage = $derived(pinError === null ? "" : pinError === "MISMATCH" ? "The new PINs do not match." : errorText(pinError));

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
</script>

{#if anyShown(q, GROUP.access, [ROW.session, ROW.savedPin])}
<div class="st-sec">
  <h2>This browser</h2>
  <div class="st-list">
    {#if show(ROW.session)}
      <SettingRow
        title={ROW.session.title}
        hint={auth.view.devBypass ? "Development firmware: no login needed." : auth.view.loggedIn ? "You are logged in." : "Log in to change settings, manage SD card files or clear DTCs."}
        inline
      >
        {#if auth.view.loggedIn}
          <button type="button" class="st-btn sm" onclick={() => auth.logout()}>Log out</button>
        {:else if !auth.view.devBypass}
          <button type="button" class="st-btn sm pri" onclick={() => auth.ensureLogin()}>Log in</button>
        {/if}
      </SettingRow>
    {/if}
    {#if show(ROW.savedPin)}
      <SettingRow
        title={ROW.savedPin.title}
        hint={auth.view.pinSaved ? "Kept in this browser so it is not asked again. The PIN on the device is unchanged." : "No PIN is saved in this browser."}
        inline
      >
        {#if auth.view.pinSaved}
          <button type="button" class="st-btn sm" onclick={() => auth.forgetPin()}>Forget saved PIN</button>
        {/if}
      </SettingRow>
    {/if}
  </div>
</div>
{/if}

{#if auth.view.loggedIn && show(ROW.changePin)}
  <form class="st-sec" onsubmit={submitPinChange}>
    <h2>Change PIN</h2>
    <div class="st-list">
      <SettingRow title="Current PIN" hint="6 digits">
        <input class="st-in mono" type="password" inputmode="numeric" autocomplete="current-password" maxlength="6" pattern={PIN_PATTERN} aria-label="Current PIN" bind:value={oldPin} required />
      </SettingRow>
      <SettingRow title="New PIN" hint="6 digits">
        <input class="st-in mono" type="password" inputmode="numeric" autocomplete="new-password" maxlength="6" pattern={PIN_PATTERN} aria-label="New PIN" bind:value={newPin} required />
      </SettingRow>
      <SettingRow title="Repeat new PIN" hint="Changing the PIN logs out every session, including this one." error={pinMessage || undefined}>
        <input
          class="st-in mono"
          type="password"
          inputmode="numeric"
          autocomplete="new-password"
          maxlength="6"
          pattern={PIN_PATTERN}
          aria-label="Repeat new PIN"
         
          bind:value={repeatPin}
          required
        />
      </SettingRow>
      <SettingRow title="Forgot the PIN?" hint="Hold the BOOT button on the device for 10 seconds. This clears the PIN, and you can set a new one.">
        <button type="submit" class="st-btn pri" disabled={!canChange}>Change PIN</button>
      </SettingRow>
    </div>
  </form>
{/if}
