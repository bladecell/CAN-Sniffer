<script lang="ts">
  import { auth } from "$lib/auth.svelte";
  import { errorText, isPin, type AuthError } from "$lib/auth";
  import Sheet from "$lib/components/Sheet.svelte";
  import PinField from "$lib/components/PinField.svelte";
  import PinPad from "$lib/components/PinPad.svelte";

  let { open, onClose }: { open: boolean; onClose: () => void } = $props();

  const steps = [
    { title: "Current PIN", hint: "Enter the PIN you use now." },
    { title: "New PIN", hint: "Choose 6 digits." },
    { title: "Repeat new PIN", hint: "Enter it once more. Changing the PIN logs out every session, including this one." },
  ];
  const last = steps.length - 1;

  let step = $state(0);
  // [current, new, repeat]. The form state is reset on close, because the sheet unmounts its body when closed.
  let pins = $state(["", "", ""]);
  let pinError = $state<AuthError | null>(null);
  let busy = $state(false);

  const mismatch = $derived(step === last && isPin(pins[last]) && pins[last] !== pins[1]);
  const message = $derived(mismatch ? "The new PINs do not match." : pinError ? errorText(pinError) : "");
  const canChange = $derived(isPin(pins[0]) && isPin(pins[1]) && pins[last] === pins[1] && !busy);

  function reset() {
    step = 0;
    pins = ["", "", ""];
    pinError = null;
    busy = false;
  }

  function close() {
    reset();
    onClose();
  }

  function back() {
    pinError = null;
    step -= 1;
  }

  async function submit(e: SubmitEvent) {
    e.preventDefault();
    if (step < last) {
      if (isPin(pins[step])) step += 1;
      return;
    }
    if (!canChange) return;
    busy = true;
    pinError = await auth.changePin(pins[0], pins[1]);
    busy = false;
    if (pinError === null) close();
  }
</script>

<Sheet {open} title="Change PIN" onClose={close}>
  <form class="pin-col" onsubmit={submit}>
    <div class="steps" aria-hidden="true">
      {#each steps as _, i (i)}
        <i class="step" class:done={i < step} class:cur={i === step}></i>
      {/each}
    </div>
    <p class="count">Step {step + 1} of {steps.length}</p>
    {#key step}
      <PinField
        id="change-pin-field"
        label={steps[step].title}
        value={pins[step]}
        onchange={(next) => (pins[step] = next)}
        describedby="change-pin-msg"
        invalid={mismatch}
        autocomplete={step === 0 ? "current-password" : "new-password"}
        autofocus
      />
    {/key}
    <p class="hint">{steps[step].hint}</p>
    <PinPad value={pins[step]} onchange={(next) => (pins[step] = next)} disabled={busy} />
    <p id="change-pin-msg" class="err" role="alert">{message}</p>
    <div class="actions">
      {#if step > 0}
        <button type="button" class="btn" onclick={back}>Back</button>
      {:else}
        <button type="button" class="btn" onclick={close}>Cancel</button>
      {/if}
      {#if step < last}
        <button type="submit" class="btn pri" disabled={!isPin(pins[step])}>Next</button>
      {:else}
        <button type="submit" class="btn pri" disabled={!canChange}>Change PIN</button>
      {/if}
    </div>
  </form>
</Sheet>

<style>
  .pin-col {
    max-width: 360px;
    margin-inline: auto;
  }
  .steps {
    display: flex;
    gap: 6px;
    margin: 0 0 10px;
  }
  .step {
    flex: 1;
    height: 5px;
    border-radius: 3px;
    background: var(--ctl);
  }
  .step.done,
  .step.cur {
    background: var(--accent);
  }
  .step.cur {
    opacity: 0.55;
  }
  .count {
    margin: 0 0 6px;
    font-size: 0.82rem;
    font-weight: 700;
    color: var(--muted);
  }
  .hint {
    margin: 6px 0 12px;
    font-size: 0.875rem;
    color: var(--muted);
  }
  .err {
    min-height: 1.25em;
    margin: 8px 0 0;
    color: var(--crit);
  }
  .actions {
    display: flex;
    gap: 10px;
    margin-top: 14px;
  }
  .actions .btn {
    flex: 1;
  }
</style>
