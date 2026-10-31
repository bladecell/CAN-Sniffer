<script lang="ts">
  import { telemetryStore, handleSavePid } from "$lib/telemetryStore.svelte";
  import { EDITOR_MODES } from "$lib/pidModes";
  import { describeInterval, intervalError, savePriority, priorityEnabled } from "$lib/editor/pollRate";
  import {
    isValidHex,
    isValidName,
    isValidDescription,
    isModeValidForPid,
    isIdValidForMode,
    isValidFormula,
    lengthError,
  } from "$lib/pidInputRules";
  import { activeSectionIndex } from "$lib/editor/scrollSpy";
  import { EDITOR_SECTIONS, editorNav } from "./editorNav.svelte";
  import Icon from "$lib/Icon.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import Field from "./editor/Field.svelte";
  import NumberStepper from "./editor/NumberStepper.svelte";
  import ColorPopover from "./editor/ColorPopover.svelte";
  import IconPicker from "./editor/IconPicker.svelte";
  import FormulaWorkbench from "./editor/FormulaWorkbench.svelte";
  import GridPicker, { type GridOption } from "./editor/GridPicker.svelte";
  import PriorityPicker from "./editor/PriorityPicker.svelte";
  import IntervalSlider from "./editor/IntervalSlider.svelte";
  import { unitsWith } from "./editor/units";

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

  const modeOptions = EDITOR_MODES.map((m) => ({ value: m.value, label: m.label }));
  // Unit tiles: the list in editor/units.ts, symbol only. A unit saved outside the list still shows as a tile.
  const unitOptions = $derived<GridOption[]>(
    unitsWith(telemetryStore.unitInput).map((u) => (u === "" ? { value: "", label: "None" } : { value: u, sym: u, label: "" })),
  );
  const derivedMode = $derived(telemetryStore.modeInput === "0x45");
  const unitStep = $derived(({ rpm: 100, V: 0.1, bar: 0.1 } as Record<string, number>)[telemetryStore.unitInput] ?? 1);
  const minMaxBad = $derived(
    telemetryStore.minInput !== undefined && telemetryStore.maxInput !== undefined && telemetryStore.minInput > telemetryStore.maxInput,
  );
  const staticPoll = $derived((telemetryStore.updateIntervalInput ?? 0) === 0);

  // Field messages. A filled, invalid field shows what to fix.
  const err = $derived({
    id: telemetryStore.idInput && !isIdValidForMode(telemetryStore.modeInput, telemetryStore.idInput) ? "Mode 0x01 and 0x45 use 0x7DF, mode 0x22 uses 0x700 to 0x7FF" : "",
    pid: telemetryStore.pidInput && !isValidHex(telemetryStore.pidInput) ? "Use 0x and up to 4 hex digits" : "",
    mode: telemetryStore.modeInput && !isModeValidForPid(telemetryStore.pidInput, telemetryStore.modeInput) ? "This mode does not fit this PID" : "",
    length: lengthError(telemetryStore.pidInput, telemetryStore.lengthInput, telemetryStore.modeInput),
    name: telemetryStore.nameInput && !isValidName(telemetryStore.nameInput) ? "Letters, digits and - ( ) _ . , / * + or space" : "",
    desc: telemetryStore.descInput && !isValidDescription(telemetryStore.descInput) ? "Same characters as Name, 256 at most" : "",
    minmax: minMaxBad ? "Min must be below max" : "",
    interval: intervalError(telemetryStore.updateIntervalInput),
    formula: telemetryStore.formulaInput && !isValidFormula(telemetryStore.modeInput, telemetryStore.formulaInput) ? "Formula does not parse, or uses unknown names" : "",
  });

  function save() {
    if (!telemetryStore.isFormValid) return;
    telemetryStore.priorityInput = savePriority(telemetryStore.priorityInput ?? 4);
    handleSavePid();
    onDone();
  }

  // Test bytes live in the editor only: the browser has no raw bytes for a PID yet.
  let testBytes = $state("");

  // Section tabs and scroll-spy. The form scrolls inside the sheet body, so the spy listens there.
  const PROBE = 64;
  const sections = EDITOR_SECTIONS;
  let root: HTMLElement | undefined = $state();
  // Opens on Request. The spy only moves when the user scrolls, so the open sheet cannot read "at end" from its start.
  editorNav.active = 0;
  let lockUntil = 0;
  const reduceMotion = typeof matchMedia === "function" && matchMedia("(prefers-reduced-motion: reduce)").matches;

  // The sheet body is the scroller. Its top layer and footer sit beside it (Sheet.svelte), so it pads past them.
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
    editorNav.active = activeSectionIndex(sectionTops(box), box.scrollTop, PROBE, atEnd);
  }
  /** Scrolls so the section's heading sits just under the sticky top layer. Reduced motion jumps. */
  function goTo(i: number) {
    const box = scroller();
    editorNav.active = i;
    if (!box) return;
    // The spy ignores scroll events while the smooth scroll runs, so the clicked tab stays active.
    lockUntil = performance.now() + 1000;
    const headHeight = parseFloat(root?.closest("dialog")?.style.getPropertyValue("--sheet-head") ?? "") || 0;
    // sectionTops is already in scroll-content coordinates, so it is the target scrollTop, not an offset from it.
    const target = sectionTops(box)[i] - headHeight - 12;
    box.scrollTo({ top: Math.max(0, target), behavior: reduceMotion ? "auto" : "smooth" });
  }

  $effect(() => {
    const box = scroller();
    if (!box) return;
    box.addEventListener("scroll", onScroll, { passive: true });
    return () => box.removeEventListener("scroll", onScroll);
  });

  editorNav.goTo = goTo;
</script>

<div class="ed" bind:this={root}>
  <form id={formId} class="form" onsubmit={(e) => { e.preventDefault(); save(); }} novalidate>
    {#if telemetryStore.validationErrors.length > 0}
      <div class="errsum" role="alert">
        <strong><Icon name="circle-exclamation" size={16} />Please fix the following errors</strong>
        <ul>
          {#each telemetryStore.validationErrors as error, i (i)}
            <li>{error}</li>
          {/each}
        </ul>
      </div>
    {/if}

    <section id="{formId}-request" class="sec" aria-labelledby="{formId}-request-h">
      <div class="sh"><h3 id="{formId}-request-h">Request</h3></div>
      <div class="grid four">
        <Field id="ed-mode" hint="Request mode" label="Mode" error={err.mode}>
          {#snippet children(labelId, msgId)}
            <SelectMenu
              bind:value={telemetryStore.modeInput}
              options={modeOptions}
              placeholder="Select mode"
              ariaLabel="Mode"
              ariaDescribedby={msgId}
              ariaInvalid={!!err.mode}
            />
          {/snippet}
        </Field>
        <Field id="ed-id" hint="Number the ECU answers to" label="CAN ID" error={err.id}>
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
        <Field id="ed-pid" hint="Code for this value" label="PID" error={err.pid}>
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
        <Field id="ed-len" hint={derivedMode ? "Not used in Derived Data" : "Response bytes"} label="Length" error={err.length}>
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
      <div class="sh"><h3 id="{formId}-signal-h">Signal</h3></div>
      <FormulaWorkbench
        id="ed-fx"
        bind:formula={telemetryStore.formulaInput}
        bind:testBytes
        unit={telemetryStore.unitInput}
        error={err.formula}
      />
      <div class="grid three">
        <Field id="ed-unit" hint="Shown after the value" label="Unit">
          {#snippet children(labelId, msgId)}
            <GridPicker id="ed-unit" {labelId} describedBy={msgId} bind:value={telemetryStore.unitInput} options={unitOptions} columns={6} />
          {/snippet}
        </Field>
        <Field id="ed-min" hint="Bottom of the chart" label="Min" error={err.minmax}>
          {#snippet children(labelId, msgId)}
            <NumberStepper id="ed-min" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.minInput} step={unitStep} invalid={minMaxBad} />
          {/snippet}
        </Field>
        <Field id="ed-max" hint="Top of the chart" label="Max" error={err.minmax}>
          {#snippet children(labelId, msgId)}
            <NumberStepper id="ed-max" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.maxInput} step={unitStep} invalid={minMaxBad} />
          {/snippet}
        </Field>
      </div>
    </section>
    <section id="{formId}-display" class="sec" aria-labelledby="{formId}-display-h">
      <div class="sh"><h3 id="{formId}-display-h">Display</h3></div>
      <div class="grid">
        <Field id="ed-name" hint="Shown in lists and charts" label="Name" error={err.name}>
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
        <Field id="ed-desc" hint="Optional note" label="Description" error={err.desc}>
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
        <Field id="ed-color" hint="Used for the chart line" label="Color">
          {#snippet children(labelId, msgId)}
            <ColorPopover id="ed-color" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.colorInput} />
          {/snippet}
        </Field>
        <Field id="ed-icon" hint="Shown next to the name" label="Icon">
          {#snippet children(labelId, msgId)}
            <IconPicker id="ed-icon" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.iconInput} />
          {/snippet}
        </Field>
      </div>
    </section>
    <section id="{formId}-polling" class="sec" aria-labelledby="{formId}-polling-h">
      <div class="sh"><h3 id="{formId}-polling-h">Polling</h3></div>
      <div class="grid">
        <Field id="ed-interval" hint={describeInterval(telemetryStore.updateIntervalInput ?? 0)} label="Update interval" error={err.interval}>
          {#snippet children(labelId, msgId)}
            <!-- Keyed on the PID, so the slider forgets its last polled value when another PID is edited. -->
            {#key telemetryStore.activeIndex}
              <IntervalSlider
                id="ed-interval"
                labelId={labelId}
                describedBy={msgId}
                bind:value={telemetryStore.updateIntervalInput}
                invalid={!!err.interval}
              />
            {/key}
          {/snippet}
        </Field>
        <Field id="ed-prio" hint={staticPoll ? "Static PIDs are never queued, so priority does not apply" : "Which request goes first when two are due"} label="Priority">
          {#snippet children(labelId, msgId)}
            <PriorityPicker id="ed-prio" labelId={labelId} describedBy={msgId} bind:value={telemetryStore.priorityInput} disabled={staticPoll} />
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
  .form {
    display: grid;
    gap: 0;
    min-width: 0;
  }
  .sec {
    display: grid;
    gap: 14px;
    margin: 0;
    padding: 16px 0 28px;
    scroll-margin-top: 0;
  }
  .sh {
    display: flex;
    align-items: baseline;
    flex-wrap: wrap;
    gap: 4px 12px;
    margin-bottom: 0;
    padding-bottom: 10px;
    border-bottom: 1px solid var(--hair);
  }
  .sh h3 {
    margin: 0;
    font-size: 1.2rem;
    font-weight: 800;
    line-height: 1.3;
  }
  .grid {
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 4px 16px;
  }
  /* Phone: Mode and CAN ID, then PID and Length, as in the mockup. */
  .grid.four {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
  /* The message row shows only an error. */
  .grid :global(.msg:not(:empty)) {
    padding-top: 6px;
    font-size: 0.8125rem;
    line-height: 1.35;
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 14px;
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
  /* Name and Description share a row, as in the mockup. Below 500px they stack. */
  @container (min-width: 500px) {
    .grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
    /* Unit, Min and Max share one row, as in the mockup. */
    .grid.three {
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }
  }
  @container (min-width: 640px) {
    /* Mode, CAN ID, PID and Length share one row, as in the mockup. */
    .grid.four {
      grid-template-columns: repeat(4, minmax(0, 1fr));
    }
  }
  /* Controls are 48px everywhere, as in the mockup. */
</style>
