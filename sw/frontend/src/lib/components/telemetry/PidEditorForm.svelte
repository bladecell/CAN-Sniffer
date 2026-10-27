<script lang="ts">
  import { telemetryStore, handleSavePid, modes, units } from "$lib/telemetryStore.svelte";
  import {
    isValidHex,
    isValidName,
    isValidDescription,
    isModeValidForPid,
    isLengthValidForPid,
    isIdValidForMode,
    isValidFormula,
  } from "$lib/pidInputRules";
  import { activeSectionIndex } from "$lib/editor/scrollSpy";
  import Icon from "$lib/Icon.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import Field from "./editor/Field.svelte";
  import NumberStepper from "./editor/NumberStepper.svelte";
  import ColorPopover from "./editor/ColorPopover.svelte";
  import IconPicker from "./editor/IconPicker.svelte";
  import FormulaWorkbench from "./editor/FormulaWorkbench.svelte";

  interface Props {
    onDone: () => void;
    formId?: string;
  }

  let { onDone, formId = "pid-editor" }: Props = $props();

  // Load the selected row, or clear for a new PID. Runs once per open: the sheet is a modal, so the selection cannot change under it.
  const initial = telemetryStore.local_piddef[telemetryStore.activeIndex];
  if (telemetryStore.activeIndex >= 0 && initial?.def) {
    const def = initial.def;
    telemetryStore.pidInput = "0x" + Number(def.pid).toString(16).toUpperCase().padStart(2, "0");
    telemetryStore.lastValidPidForLength = telemetryStore.pidInput;
    telemetryStore.nameInput = def.name || "";
    telemetryStore.descInput = def.description || "";
    telemetryStore.unitInput = def.unit || "";
    telemetryStore.minInput = def.minValue;
    telemetryStore.maxInput = def.maxValue;
    telemetryStore.modeInput = "0x" + Number(def.mode).toString(16).padStart(2, "0");
    telemetryStore.lastValidModeForId = telemetryStore.modeInput;
    telemetryStore.formulaInput = def.formula || "";
    telemetryStore.lengthInput = def.length;
    telemetryStore.idInput = def.id ? "0x" + Number(def.id).toString(16).toUpperCase() : "";
    telemetryStore.priorityInput = def.priority;
    telemetryStore.updateIntervalInput = def.update_interval_ms;
    telemetryStore.colorInput = "#" + Number(def.color).toString(16).padStart(6, "0");
    telemetryStore.iconInput = def.icon || "";
  } else {
    telemetryStore.clearForm();
  }
  const isEdit = telemetryStore.activeIndex >= 0 && !!initial?.def;

  // Derived fields: a PID sets its length, and the 0x01 and 0x45 modes fix the CAN ID.
  $effect(() => {
    if (isValidHex(telemetryStore.pidInput) && telemetryStore.pidInput !== telemetryStore.lastValidPidForLength) {
      telemetryStore.lastValidPidForLength = telemetryStore.pidInput;
      const pidVal = parseInt(telemetryStore.pidInput.replace(/^0x/i, ""), 16);
      telemetryStore.lengthInput = pidVal <= 0xff ? 2 : 3;
    }
  });

  $effect(() => {
    if (telemetryStore.modeInput && telemetryStore.modeInput !== telemetryStore.lastValidModeForId) {
      telemetryStore.lastValidModeForId = telemetryStore.modeInput;
      if (telemetryStore.modeInput === "0x01" || telemetryStore.modeInput === "0x45") {
        telemetryStore.idInput = "0x7DF";
      }
      if (telemetryStore.modeInput === "0x45") {
        telemetryStore.lengthInput = 0;
      }
    }
  });

  const modeOptions = modes.map((m) => ({ value: m.value, label: m.label }));
  const unitOptions = units.map((u) => ({ value: u, label: u }));
  const derivedMode = $derived(telemetryStore.modeInput === "0x45");
  const unitStep = $derived(({ rpm: 100, V: 0.1, bar: 0.1 } as Record<string, number>)[telemetryStore.unitInput] ?? 1);
  const minMaxBad = $derived(
    telemetryStore.minInput !== undefined && telemetryStore.maxInput !== undefined && telemetryStore.minInput > telemetryStore.maxInput,
  );
  const priorityBad = $derived(
    telemetryStore.priorityInput !== undefined && (telemetryStore.priorityInput < 0 || telemetryStore.priorityInput > 255),
  );
  const intervalBad = $derived(
    telemetryStore.updateIntervalInput !== undefined &&
      telemetryStore.updateIntervalInput !== 0 &&
      (telemetryStore.updateIntervalInput < 16 || telemetryStore.updateIntervalInput > 65535),
  );

  // Field messages. A blank field shows its hint; a filled, invalid field shows what to fix.
  const err = $derived({
    id: telemetryStore.idInput && !isIdValidForMode(telemetryStore.modeInput, telemetryStore.idInput) ? "Mode 0x01 and 0x45 use 0x7DF, mode 0x22 uses 0x700 to 0x7FF" : "",
    pid: telemetryStore.pidInput && !isValidHex(telemetryStore.pidInput) ? "Use 0x and up to 4 hex digits" : "",
    mode: telemetryStore.modeInput && !isModeValidForPid(telemetryStore.pidInput, telemetryStore.modeInput) ? "This mode does not fit this PID" : "",
    length: !derivedMode && telemetryStore.lengthInput !== undefined && !isLengthValidForPid(telemetryStore.pidInput, telemetryStore.lengthInput, telemetryStore.modeInput) ? "PIDs up to 0xFF need 2 bytes, others 3" : "",
    name: telemetryStore.nameInput && !isValidName(telemetryStore.nameInput) ? "Letters, digits and - ( ) _ . , / * + or space" : "",
    desc: telemetryStore.descInput && !isValidDescription(telemetryStore.descInput) ? "Same characters as Name, 256 at most" : "",
    minmax: minMaxBad ? "Min must be below max" : "",
    priority: priorityBad ? "Use a whole number from 0 to 255" : "",
    formula: telemetryStore.formulaInput && !isValidFormula(telemetryStore.modeInput, telemetryStore.formulaInput) ? "Formula does not parse, or uses unknown names" : "",
    interval: intervalBad ? "Use 0 (polling off) or 16 to 65535" : "",
  });

  function save() {
    if (!telemetryStore.isFormValid) return;
    handleSavePid();
    onDone();
  }

  // Test bytes live in the editor only: the browser has no raw bytes for a PID yet (see the Test bytes hint).
  let testBytes = $state("");

  // Section tabs and scroll-spy. The form scrolls inside the sheet body, so the spy listens there.
  const sections = [
    { id: "request", label: "Request" },
    { id: "signal", label: "Signal" },
    { id: "display", label: "Display" },
    { id: "polling", label: "Polling" },
  ];
  const PROBE = 64;
  let root: HTMLElement | undefined = $state();
  let active = $state(0);
  let lockUntil = 0;
  const reduceMotion = typeof matchMedia === "function" && matchMedia("(prefers-reduced-motion: reduce)").matches;

  function scroller(): HTMLElement | null {
    return root?.closest<HTMLElement>(".body") ?? null;
  }
  function sectionTops(box: HTMLElement): number[] {
    const origin = box.getBoundingClientRect().top - box.scrollTop;
    return sections.map((s) => {
      const el = document.getElementById(`${formId}-${s.id}`);
      return el ? el.getBoundingClientRect().top - origin : 0;
    });
  }
  function onScroll() {
    const box = scroller();
    if (!box || performance.now() < lockUntil) return;
    const atEnd = box.scrollTop + box.clientHeight >= box.scrollHeight - 2;
    active = activeSectionIndex(sectionTops(box), box.scrollTop, PROBE, atEnd);
  }
  function goTo(i: number) {
    const box = scroller();
    active = i;
    if (!box) return;
    lockUntil = performance.now() + 700;
    const top = sectionTops(box)[i];
    box.scrollTo({ top: box.scrollTop + top - 60, behavior: reduceMotion ? "auto" : "smooth" });
  }

  $effect(() => {
    const box = scroller();
    if (!box) return;
    box.addEventListener("scroll", onScroll, { passive: true });
    onScroll();
    return () => box.removeEventListener("scroll", onScroll);
  });
</script>

<div class="ed" bind:this={root}>
  <nav class="tabs float-layer" aria-label="Editor sections">
    {#each sections as s, i (s.id)}
      <button type="button" class="tab" aria-current={active === i ? "true" : undefined} onclick={() => goTo(i)}>{s.label}</button>
    {/each}
  </nav>

  <form id={formId} class="form" onsubmit={(e) => { e.preventDefault(); save(); }} novalidate>
    {#if telemetryStore.validationErrors.length > 0}
      <div class="errsum" role="alert">
        <strong><Icon name="circle-exclamantion" size={16} />Please fix the following errors</strong>
        <ul>
          {#each telemetryStore.validationErrors as error, i (i)}
            <li>{error}</li>
          {/each}
        </ul>
      </div>
    {/if}

    <section id="{formId}-request" class="sec" aria-labelledby="{formId}-request-h">
      <h3 id="{formId}-request-h" class="sh">Request</h3>
      <div class="grid">
        <Field id="ed-id" label="CAN ID" hint="ECU CAN identifier" error={err.id}>
          {#snippet children(labelId, msgId)}
            <input
              id="ed-id"
              class="in mono"
              placeholder="0x7DF"
              autocomplete="off"
              spellcheck="false"
              aria-labelledby={labelId}
              aria-describedby={msgId}
              aria-invalid={err.id ? "true" : undefined}
              bind:value={telemetryStore.idInput}
            />
          {/snippet}
        </Field>
        <Field id="ed-mode" label="Mode" hint="OBD2 service mode" error={err.mode}>
          {#snippet children(labelId, msgId)}
            <SelectMenu
              bind:value={telemetryStore.modeInput}
              options={modeOptions}
              placeholder="Select a mode..."
              ariaLabel="Mode"
             
              ariaDescribedby={msgId}
              ariaInvalid={!!err.mode}
            />
          {/snippet}
        </Field>
        <Field id="ed-pid" label="PID" hint="Parameter ID" error={err.pid}>
          {#snippet children(labelId, msgId)}
            <input
              id="ed-pid"
              class="in mono"
              placeholder="0x0C"
              autocomplete="off"
              spellcheck="false"
              aria-labelledby={labelId}
              aria-describedby={msgId}
              aria-invalid={err.pid ? "true" : undefined}
              bind:value={telemetryStore.pidInput}
            />
          {/snippet}
        </Field>
        <Field id="ed-len" label="Length" hint={derivedMode ? "Not used in Derived Data" : "Response bytes"} error={err.length}>
          {#snippet children(labelId, msgId)}
            <NumberStepper
              id="ed-len"
              labelId={labelId}
              describedBy={msgId}
              bind:value={telemetryStore.lengthInput}
              min={0}
              max={8}
              invalid={!!err.length}
              disabled={derivedMode}
            />
          {/snippet}
        </Field>
      </div>
    </section>

    <section id="{formId}-signal" class="sec" aria-labelledby="{formId}-signal-h">
      <h3 id="{formId}-signal-h" class="sh">Signal</h3>
      <FormulaWorkbench
        id="ed-fx"
        bind:formula={telemetryStore.formulaInput}
        bind:testBytes
        unit={telemetryStore.unitInput}
        error={err.formula}
        {isEdit}
      />
      <div class="grid">
        <Field id="ed-unit" label="Unit" hint="Shown after the value">
          {#snippet children(labelId, msgId)}
            <SelectMenu bind:value={telemetryStore.unitInput} options={unitOptions} placeholder="Select a unit..." ariaLabel="Unit" ariaDescribedby={msgId} />
          {/snippet}
        </Field>
        <Field id="ed-min" label="Min value" hint="Chart floor" error={err.minmax}>
          {#snippet children(labelId, msgId)}
            <NumberStepper id="ed-min" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.minInput} step={unitStep} invalid={minMaxBad} />
          {/snippet}
        </Field>
        <Field id="ed-max" label="Max value" hint="Chart ceiling" error={err.minmax}>
          {#snippet children(labelId, msgId)}
            <NumberStepper id="ed-max" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.maxInput} step={unitStep} invalid={minMaxBad} />
          {/snippet}
        </Field>
      </div>
    </section>

    <section id="{formId}-display" class="sec" aria-labelledby="{formId}-display-h">
      <h3 id="{formId}-display-h" class="sh">Display</h3>
      <div class="grid">
        <Field id="ed-name" label="Name" hint="Shown in lists and charts" error={err.name}>
          {#snippet children(labelId, msgId)}
            <input
              id="ed-name"
              class="in"
              placeholder="Coolant temperature"
              autocomplete="off"
              aria-labelledby={labelId}
              aria-describedby={msgId}
              aria-invalid={err.name ? "true" : undefined}
              bind:value={telemetryStore.nameInput}
            />
          {/snippet}
        </Field>
        <Field id="ed-desc" label="Description" hint="Optional" error={err.desc}>
          {#snippet children(labelId, msgId)}
            <input
              id="ed-desc"
              class="in"
              placeholder="Optional"
              autocomplete="off"
              aria-labelledby={labelId}
              aria-describedby={msgId}
              aria-invalid={err.desc ? "true" : undefined}
              bind:value={telemetryStore.descInput}
            />
          {/snippet}
        </Field>
        <Field id="ed-color" label="Color" hint="Line and swatch">
          {#snippet children(labelId, msgId)}
            <ColorPopover id="ed-color" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.colorInput} />
          {/snippet}
        </Field>
        <Field id="ed-icon" label="Icon" hint="Shown on the row">
          {#snippet children(labelId, msgId)}
            <IconPicker id="ed-icon" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.iconInput} />
          {/snippet}
        </Field>
      </div>
    </section>

    <section id="{formId}-polling" class="sec" aria-labelledby="{formId}-polling-h">
      <h3 id="{formId}-polling-h" class="sh">Polling</h3>
      <div class="grid">
        <Field id="ed-interval" label="Update interval (ms)" hint={telemetryStore.updateIntervalInput === 0 ? "Polling disabled" : "Milliseconds between polls"} error={err.interval}>
          {#snippet children(labelId, msgId)}
            <NumberStepper id="ed-interval" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.updateIntervalInput} step={10} min={0} max={65535} invalid={intervalBad} />
          {/snippet}
        </Field>
        <Field id="ed-prio" label="Priority" hint="0 to 255" error={err.priority}>
          {#snippet children(labelId, msgId)}
            <NumberStepper id="ed-prio" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.priorityInput} min={0} max={255} invalid={priorityBad} />
          {/snippet}
        </Field>
      </div>
    </section>
  </form>
</div>

<style>
  .ed {
    display: grid;
    gap: 6px;
    min-width: 0;
    container-type: inline-size;
  }
  .tabs {
    position: sticky;
    top: var(--sheet-head, 0px);
    z-index: 2;
    display: flex;
    gap: 4px;
    margin: 0 -22px;
    padding: 6px 22px;
    overflow-x: auto;
    scrollbar-width: none;
  }
  .tab {
    flex: 1 0 auto;
    min-height: 44px;
    padding: 0 12px;
    border: 0;
    border-radius: var(--r-ctl);
    background: transparent;
    color: var(--muted);
    font: inherit;
    font-weight: 700;
    white-space: nowrap;
    cursor: pointer;
    position: relative;
  }
  .tab:hover {
    color: var(--text);
    background: var(--hov);
  }
  .tab[aria-current="true"] {
    color: var(--accent-text);
    font-weight: 800;
  }
  .tab::after {
    content: "";
    position: absolute;
    left: 22%;
    right: 22%;
    bottom: 4px;
    height: 3px;
    border-radius: 3px;
    background: var(--accent);
    transform: scaleX(0);
    transition: transform 0.2s cubic-bezier(0.16, 1, 0.3, 1);
  }
  .tab[aria-current="true"]::after {
    transform: scaleX(1);
  }
  .tab:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: -3px;
  }
  .form {
    display: grid;
    gap: 6px;
    min-width: 0;
  }
  .sec {
    display: grid;
    gap: 12px;
    padding: 14px 0 6px;
    scroll-margin-top: 0;
  }
  .sh {
    margin: 0;
    font-size: 0.95rem;
    font-weight: 800;
  }
  .grid {
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 6px 12px;
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 12px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
  }
  .in::placeholder {
    color: var(--ctl-sub);
    opacity: 1;
  }
  .in[aria-invalid="true"] {
    outline: 2px solid var(--crit);
    outline-offset: 0;
  }
  .in:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .mono {
    font-family: var(--mono);
  }
  .errsum {
    padding: 12px 14px;
    border-radius: var(--r-ctl);
    background: var(--crit-f);
    color: var(--text);
  }
  .errsum strong {
    display: flex;
    align-items: center;
    gap: 8px;
    color: var(--crit);
  }
  .errsum ul {
    margin: 6px 0 0;
    padding-left: 22px;
    font-size: 0.9rem;
  }
  @container (min-width: 560px) {
    .grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
  }
  @media (hover: hover) and (min-width: 600px) {
    .ed {
      --ctl-h: 40px;
    }
  }
</style>
