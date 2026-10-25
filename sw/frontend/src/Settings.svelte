<script lang="ts">
  import { onMount } from "svelte";
  import { auth } from "$lib/auth.svelte";
  import { errorText, isPin, PIN_PATTERN, type AuthError } from "$lib/auth";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import Icon from "$lib/Icon.svelte";
  import { ACCENTS, CONTROL_FILLS, THEMES, nextOption, type ControlFill, type Theme } from "$lib/appearance";
  import { appearance, setAppearance } from "$lib/appearance.svelte";

  onMount(() => void auth.refresh());

  const themeLabels: Record<Theme, string> = { system: "System", light: "Light", dark: "Dark" };
  const fillLabels: Record<ControlFill, string> = { clear: "Clear", strong: "Strong" };

  // Arrow keys move the selection and focus within a radio group (WAI-ARIA radio pattern).
  function radioKey<T extends string>(e: KeyboardEvent, options: readonly T[], current: T, set: (v: T) => void) {
    const next = nextOption(options, current, e.key);
    if (next === undefined) return;
    e.preventDefault();
    set(next);
    (e.currentTarget as HTMLElement).parentElement?.querySelector<HTMLElement>(`[data-value="${next}"]`)?.focus();
  }

  let baudrate = $state("500 kbps");

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

<section class="card appearance" aria-labelledby="appearance-title">
  <h2 id="appearance-title">Appearance</h2>

  <div class="row">
    <span class="lab" id="theme-label">Theme</span>
    <div class="seg" role="radiogroup" aria-labelledby="theme-label">
      {#each THEMES as t (t)}
        <button
          type="button"
          role="radio"
          data-value={t}
          aria-checked={appearance.theme === t}
          tabindex={appearance.theme === t ? 0 : -1}
          onclick={() => setAppearance("theme", t)}
          onkeydown={(e) => radioKey(e, THEMES, appearance.theme, (v) => setAppearance("theme", v))}>{themeLabels[t]}</button>
      {/each}
    </div>
    <span class="hint">System follows your device setting.</span>
  </div>

  <div class="row">
    <span class="lab" id="accent-label">Accent</span>
    <div class="swatches" role="radiogroup" aria-labelledby="accent-label">
      {#each ACCENTS as a (a.id)}
        <button
          type="button"
          class="swatch"
          role="radio"
          data-value={a.id}
          aria-label={a.label}
          aria-checked={appearance.accent === a.id}
          tabindex={appearance.accent === a.id ? 0 : -1}
          style="--ah: {a.hue}; --ac: {a.chroma}"
          onclick={() => setAppearance("accent", a.id)}
          onkeydown={(e) => radioKey(e, ACCENTS.map((x) => x.id), appearance.accent, (v) => setAppearance("accent", v))}>
          {#if appearance.accent === a.id}<Icon name="check" size={22} />{/if}
        </button>
      {/each}
    </div>
    <span class="hint">{ACCENTS.find((a) => a.id === appearance.accent)?.label}</span>
  </div>

  <div class="row">
    <span class="lab" id="ctl-label">Control fill</span>
    <div class="seg" role="radiogroup" aria-labelledby="ctl-label">
      {#each CONTROL_FILLS as c (c)}
        <button
          type="button"
          role="radio"
          data-value={c}
          aria-checked={appearance.ctl === c}
          tabindex={appearance.ctl === c ? 0 : -1}
          onclick={() => setAppearance("ctl", c)}
          onkeydown={(e) => radioKey(e, CONTROL_FILLS, appearance.ctl, (v) => setAppearance("ctl", v))}>{fillLabels[c]}</button>
      {/each}
    </div>
    <span class="hint">Strong gives buttons and fields a darker fill for more contrast.</span>
  </div>
</section>

<section>
  <h2>System Settings</h2>
  <label for="baud">Baudrate</label>
  <SelectMenu
    id="baud"
    bind:value={baudrate}
    options={[
      { value: "500 kbps", label: "500 kbps" },
      { value: "250 kbps", label: "250 kbps" },
    ]}
  />
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
  .appearance {
    padding: var(--pad-card);
    display: grid;
    gap: 18px;
    margin-bottom: 2rem;
  }
  .appearance h2 {
    margin: 0;
  }
  .row {
    display: grid;
    gap: 8px;
    justify-items: start;
  }
  .lab {
    font-weight: 700;
    font-size: 0.85rem;
  }
  .swatches {
    display: flex;
    flex-wrap: wrap;
    gap: 6px 10px;
    padding: 2px 0 8px;
  }
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
