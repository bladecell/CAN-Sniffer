<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import { auth } from "$lib/auth.svelte";
  import { GROUP, ROW, anyShown, rowShown, type RowMeta } from "$lib/settingsModel";
  import SettingRow from "./SettingRow.svelte";
  import ChangePinSheet from "./ChangePinSheet.svelte";

  let { q }: { q: string } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.access, row);

  let pinOpen = $state(false);
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
          <button type="button" class="btn sm" onclick={() => auth.logout()}>Log out</button>
        {:else if !auth.view.devBypass}
          <button type="button" class="btn sm pri" onclick={() => auth.ensureLogin()}>Log in</button>
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
          <button type="button" class="btn sm" onclick={() => auth.forgetPin()}>Forget saved PIN</button>
        {/if}
      </SettingRow>
    {/if}
  </div>
</div>
{/if}

{#if auth.view.loggedIn && show(ROW.changePin)}
  <div class="st-sec">
    <h2>Change PIN</h2>
    <div class="st-list">
      <SettingRow title="Change PIN" hint="Changing the PIN logs out every session, including this one.">
        <button type="button" class="btn sm pri" onclick={() => (pinOpen = true)}>Change PIN</button>
      </SettingRow>
      <SettingRow title="Forgot the PIN?" hint="Hold the BOOT button on the device for 10 seconds. This clears the PIN, and you can set a new one." />
    </div>
  </div>
  <ChangePinSheet open={pinOpen} onClose={() => (pinOpen = false)} />
{/if}
