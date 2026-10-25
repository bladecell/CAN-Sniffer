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
  import Icon from "$lib/Icon.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";

  interface Props {
    onDone: () => void;
  }

  let { onDone }: Props = $props();

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

  const iconNames = import.meta.glob("$lib/icons_svg/*.svg");
  const iconOptions = Object.keys(iconNames).map((path) => {
    const name = path.split("/").pop()!.replace(".svg", "");
    const label = name.split("-").map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join(" ");
    return { value: name, label, icon: name };
  });
  const modeOptions = modes.map((m) => ({ value: m.value, label: m.label }));
  const unitOptions = units.map((u) => ({ value: u, label: u }));

  const minMaxBad = $derived(
    telemetryStore.minInput !== undefined &&
      telemetryStore.maxInput !== undefined &&
      telemetryStore.minInput > telemetryStore.maxInput,
  );
  const derivedMode = $derived(telemetryStore.modeInput === "0x45");

  function save() {
    if (!telemetryStore.isFormValid) return;
    handleSavePid();
    onDone();
  }
</script>

<form class="edform" onsubmit={(e) => { e.preventDefault(); save(); }} novalidate>
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

  <div class="fgrid">
    <div class="fld">
      <label class="lab" for="ed-id">CAN ID</label>
      <input
        id="ed-id"
        class="mono"
        placeholder="CAN ID (e.g., 0x7DF)"
        aria-describedby="h-id"
        autocomplete="off"
        spellcheck="false"
        bind:value={telemetryStore.idInput}
        aria-invalid={!isIdValidForMode(telemetryStore.modeInput, telemetryStore.idInput)}
      />
      <small class="hint" id="h-id">ECU CAN identifier</small>
    </div>

    <div class="fld">
      <span class="lab" id="l-mode">Mode</span>
      <SelectMenu
        bind:value={telemetryStore.modeInput}
        options={modeOptions}
        placeholder="Select a mode..."
        ariaLabel="Mode"
        ariaDescribedby="h-mode"
        ariaInvalid={!isModeValidForPid(telemetryStore.pidInput, telemetryStore.modeInput)}
      />
      <small class="hint" id="h-mode">OBD2 service mode</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-pid">PID</label>
      <input
        id="ed-pid"
        class="mono"
        placeholder="PID (e.g., 0x0C)"
        aria-describedby="h-pid"
        autocomplete="off"
        spellcheck="false"
        bind:value={telemetryStore.pidInput}
        aria-invalid={!isValidHex(telemetryStore.pidInput)}
      />
      <small class="hint" id="h-pid">Parameter ID</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-len">Length</label>
      <input
        id="ed-len"
        type="number"
        placeholder="Length (bytes)"
        aria-describedby="h-len"
        bind:value={telemetryStore.lengthInput}
        aria-invalid={!isLengthValidForPid(telemetryStore.pidInput, telemetryStore.lengthInput, telemetryStore.modeInput)}
        disabled={derivedMode}
      />
      <small class="hint" id="h-len">{derivedMode ? "Not used in Derived Data" : "Response bytes"}</small>
    </div>

    <div class="fld w2">
      <label class="lab" for="ed-name">Name</label>
      <input
        id="ed-name"
        placeholder="Name"
        aria-describedby="h-name"
        autocomplete="off"
        bind:value={telemetryStore.nameInput}
        aria-invalid={!isValidName(telemetryStore.nameInput)}
      />
      <small class="hint" id="h-name">Name of the parameter</small>
    </div>

    <div class="fld w2">
      <label class="lab" for="ed-desc">Description (optional)</label>
      <input
        id="ed-desc"
        placeholder="Description (optional)"
        aria-describedby="h-desc"
        autocomplete="off"
        bind:value={telemetryStore.descInput}
        aria-invalid={telemetryStore.descInput.length > 0 ? !isValidDescription(telemetryStore.descInput) : undefined}
      />
      <small class="hint" id="h-desc">Detailed description</small>
    </div>

    <div class="fld">
      <span class="lab">Unit</span>
      <SelectMenu bind:value={telemetryStore.unitInput} options={unitOptions} placeholder="Select a unit..." ariaLabel="Unit" ariaDescribedby="h-unit" />
      <small class="hint" id="h-unit">Unit of measurement</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-min">Min value</label>
      <input
        id="ed-min"
        type="number"
        placeholder="Min value"
        aria-describedby="h-min"
        bind:value={telemetryStore.minInput}
        aria-invalid={minMaxBad ? "true" : undefined}
      />
      <small class="hint" id="h-min">Minimum expected</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-max">Max value</label>
      <input
        id="ed-max"
        type="number"
        placeholder="Max value"
        aria-describedby="h-max"
        bind:value={telemetryStore.maxInput}
        aria-invalid={minMaxBad ? "true" : undefined}
      />
      <small class="hint" id="h-max">Maximum expected</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-prio">Priority</label>
      <input
        id="ed-prio"
        type="number"
        min="0"
        max="255"
        placeholder="Priority (0-255)"
        aria-describedby="h-prio"
        bind:value={telemetryStore.priorityInput}
        aria-invalid={telemetryStore.priorityInput !== undefined && (telemetryStore.priorityInput < 0 || telemetryStore.priorityInput > 255) ? "true" : undefined}
      />
      <small class="hint" id="h-prio">0 to 255</small>
    </div>

    <div class="fld w2">
      <label class="lab" for="ed-formula">Formula</label>
      <input
        id="ed-formula"
        class="mono"
        placeholder="Formula (e.g., A * 100 / 255)"
        aria-describedby="h-formula"
        autocomplete="off"
        spellcheck="false"
        bind:value={telemetryStore.formulaInput}
        aria-invalid={!isValidFormula(telemetryStore.modeInput, telemetryStore.formulaInput)}
      />
      <small class="hint" id="h-formula">Expression to compute the final value</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-interval">Update interval (ms)</label>
      <input
        id="ed-interval"
        type="number"
        min="0"
        max="65535"
        placeholder="Interval (ms)"
        aria-describedby="h-interval"
        bind:value={telemetryStore.updateIntervalInput}
        aria-invalid={telemetryStore.updateIntervalInput !== undefined &&
          telemetryStore.updateIntervalInput !== 0 &&
          (telemetryStore.updateIntervalInput < 16 || telemetryStore.updateIntervalInput > 65535)
          ? "true"
          : undefined}
      />
      <small class="hint" id="h-interval">{telemetryStore.updateIntervalInput === 0 ? "Polling disabled" : "Interval in ms"}</small>
    </div>

    <div class="fld">
      <label class="lab" for="ed-color">Color</label>
      <input id="ed-color" class="tel-color" type="color" bind:value={telemetryStore.colorInput} aria-describedby="h-color" />
      <small class="hint" id="h-color">{telemetryStore.colorInput.toUpperCase()}</small>
    </div>

    <div class="fld">
      <span class="lab">Icon</span>
      <SelectMenu bind:value={telemetryStore.iconInput} options={iconOptions} placeholder="Select an icon..." ariaLabel="Icon" ariaDescribedby="h-icon" />
      <small class="hint" id="h-icon">Display icon</small>
    </div>

    <div class="acts">
      <button type="button" class="tel-btn" onclick={() => telemetryStore.clearForm()}>Clear form</button>
      <button type="submit" class="tel-btn pri" disabled={!telemetryStore.isFormValid} aria-disabled={!telemetryStore.isFormValid}>
        {telemetryStore.activeIndex === -1 ? "Add PID definition" : "Save changes"}
      </button>
    </div>
  </div>
</form>

<style>
  .edform {
    display: grid;
    gap: 10px;
    padding: 2px 0 0;
  }
  @media (hover: hover) and (min-width: 760px) {
    .edform {
      --ctl-h: 40px;
    }
  }
  .fgrid {
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 8px 12px;
  }
  .fld {
    display: grid;
    min-width: 0;
  }
  .lab {
    display: block;
    margin-bottom: 3px;
    font-size: 0.8rem;
    font-weight: 700;
  }
  .hint {
    display: block;
    margin-top: 3px;
    color: var(--muted);
    font-size: 0.76rem;
    line-height: 1.25;
  }
  .mono {
    font-family: var(--mono);
  }
  .tel-color {
    width: 100%;
    min-height: var(--ctl-h);
    padding: 3px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    cursor: pointer;
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
  .acts {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
    justify-content: flex-end;
    padding-top: 4px;
  }
  .acts .tel-btn {
    flex: 1 1 140px;
  }
  @media (min-width: 760px) {
    .fgrid {
      grid-template-columns: repeat(4, minmax(0, 1fr));
    }
    .w2 {
      grid-column: span 2;
    }
    .acts {
      grid-column: span 4;
    }
  }
</style>
