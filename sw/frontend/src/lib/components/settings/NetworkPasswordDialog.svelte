<script lang="ts">
  import Dialog from "$lib/components/Dialog.svelte";
  import { checkJoinPassword } from "$lib/settingsModel";

  // Mounted only while it is needed, so every open starts with an empty, hidden password.
  let { ssid, onJoin, onCancel }: { ssid: string; onJoin: (password: string) => void; onCancel: () => void } = $props();
  let password = $state("");
  let shown = $state(false);
  let error = $state<string | null>(null);

  function join() {
    error = checkJoinPassword(password);
    if (error === null) onJoin(password);
  }
</script>

<Dialog
  open={true}
  title={`Join ${ssid}`}
  message="This network is locked. Enter its password. It is kept on the device after you press Apply."
  confirmLabel="Join"
  onConfirm={join}
  onCancel={onCancel}
>
  <div class="pw">
    <input
      type={shown ? "text" : "password"}
      autocomplete="off"
      autocapitalize="off"
      spellcheck="false"
      aria-label="Network password"
      aria-invalid={error ? "true" : undefined}
      bind:value={password}
      onkeydown={(e) => {
        if (e.key === "Enter") {
          e.preventDefault();
          join();
        }
      }}
    />
    <button type="button" aria-pressed={shown} onclick={() => (shown = !shown)}>{shown ? "Hide" : "Show"}</button>
  </div>
  {#if error}<p class="bad" role="alert">{error}</p>{/if}
</Dialog>

<style>
  .pw {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-top: 14px;
  }
  /* The overlay action moves the dialog out of .st-root, so the settings rules do not reach it: style the row here.
     overlay.css gives every overlay input a 14px bottom margin (for sheet forms); this row is centred, so it must be 0. */
  .pw input,
  .pw button {
    box-sizing: border-box;
    flex: none;
    height: var(--ctl-h);
    margin: 0 !important;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
    font-weight: 700;
  }
  .pw input {
    flex: 1 1 auto;
    min-width: 0;
    padding: 0 14px;
    font-family: var(--mono);
    font-weight: 400;
  }
  .pw input::placeholder {
    color: var(--ctl-sub);
  }
  .pw button {
    padding: 0 18px;
    cursor: pointer;
  }
  .bad {
    margin: 8px 0 0;
    font-weight: 700;
    font-size: 0.9rem;
    color: var(--crit);
  }
</style>
