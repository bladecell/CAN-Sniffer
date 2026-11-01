import type { PidDefinition } from "$lib/types";
import { getModeLabel } from "$lib/pidHelpers.svelte.ts";
import {
  MAX_PID_FORMULA_LENGTH,
  MAX_PID_INTERVAL,
  MAX_PID_PRIORITY,
  MIN_PID_INTERVAL,
  MIN_PID_PRIORITY,
} from "$lib/pidValidation";
import { isValidHex, isValidName, isValidDescription, isModeValidForPid, isLengthValidForPid, isIdValidForMode, isValidFormula } from "$lib/pidInputRules";
import { connectionStore } from "./connectionStore.svelte";
import { vehicleStore } from "./vehicleStore.svelte";
import { SvelteSet } from "svelte/reactivity";
import { DEFAULT_PLOT_COUNT, type PlotCount } from "$lib/telemetryChart/layout";

/** A PID object from a file or the SD card: long or short key names, nothing guaranteed. */
type RawPidDef = Partial<PidDefinition> & { desc?: string; len?: number; minV?: number; maxV?: number; interval?: number };

export class TelemetryStore {
  constructor() {
    $effect.root(() => {
      $effect(() => {
        const piddef = vehicleStore.pidDefinitions;
        if (piddef && piddef !== this.lastPidDef) {
          const onDeviceBefore = new Set((this.lastPidDef ?? []).map((def) => Number(def.pid)));
          this.lastPidDef = piddef;

          const newDefMap = new Map();
          for (const def of piddef) {
            newDefMap.set(Number(def.pid), def);
          }

          const updatedPids = [];

          // 1. Process existing elements to retain their selected/loaded states
          for (const existing of this.local_piddef) {
            const rawPid = parseInt(existing.pid, 16);
            const updatedDef = newDefMap.get(rawPid);

            if (updatedDef) {
              // A row with local edits keeps them; the device copy is only the baseline. Otherwise take the device's copy.
              updatedPids.push(
                existing.isDirty ? existing : createRowObject(updatedDef, existing.loaded, existing.selected),
              );
              newDefMap.delete(rawPid);
            } else {
              // A PID the device had is now gone, so it is not loaded. A PID never sent to the device stays as an addition.
              if (onDeviceBefore.has(rawPid)) existing.loaded = false;
              updatedPids.push(existing);
            }
          }

          // 2. Add brand new elements
          for (const newDef of newDefMap.values()) {
            updatedPids.push(createRowObject(newDef, true, false));
          }

          this.local_piddef = updatedPids;
        }
      });
    });
  }

  local_piddef = $state<PidRow[]>([]);
  lastPidDef = $state<PidDefinition[] | null>(null);
  /** Changes tab: PIDs whose pending change is not ticked for Apply. Every pending change is ticked by default. */
  deselectedChanges = new SvelteSet<number>();

  /**
   * Puts the rows for these PIDs back to the device's copy: a Revert, or the state after Apply. A PID the device no
   * longer has is dropped (an added PID that was reverted, or a removed one that was applied).
   */
  restoreFromDevice(pids: ReadonlySet<number>) {
    const devices = new Map(vehicleStore.pidDefinitions.map((def) => [Number(def.pid), def]));
    const next: PidRow[] = [];
    for (const row of this.local_piddef) {
      const pid = parseInt(row.pid, 16);
      if (!pids.has(pid)) next.push(row);
      else {
        const def = devices.get(pid);
        if (def) next.push(createRowObject(def, true, row.selected));
      }
    }
    this.local_piddef = next;
  }

  /** Plots on the page: 1, 2, 4 or 6. Plot n shows chartPids[n - 1]. */
  plotCount = $state<PlotCount>(DEFAULT_PLOT_COUNT);
  activeChartIndex = $state(0);
  chartPids = $state<number[][]>(Array.from({ length: DEFAULT_PLOT_COUNT }, () => []));
  activeIndex = $state(-1);

  // --- FORM STATE ---
  pidInput = $state("");
  nameInput = $state("");
  descInput = $state("");
  unitInput = $state("");
  minInput = $state<number | undefined>(undefined);
  maxInput = $state<number | undefined>(undefined);
  modeInput = $state("");
  lengthInput = $state<number | undefined>(undefined);
  lastValidPidForLength = $state<string>("");
  idInput = $state("");
  lastValidModeForId = $state<string>("");
  formulaInput = $state("");
  priorityInput = $state<number | undefined>(4);
  colorInput = $state("#01AAFF");
  updateIntervalInput = $state<number | undefined>(512);
  iconInput = $state("");

  colorUint32 = $derived(parseInt(this.colorInput.replace("#", ""), 16));

  validationErrors = $derived.by(() => {
    let errors: string[] = [];
    if (this.pidInput && !isValidHex(this.pidInput)) errors.push("PID must be a valid hex format (e.g., 0x0C).");
    if (this.nameInput && !isValidName(this.nameInput)) errors.push("Name can only contain letters, numbers, and basic symbols (maximum 64 characters).");
    if (this.descInput && !isValidDescription(this.descInput)) errors.push("Description contains invalid characters.");
    if (this.formulaInput.length > MAX_PID_FORMULA_LENGTH) errors.push(`Formula must be at most ${MAX_PID_FORMULA_LENGTH} characters.`);
    if ((this.minInput !== undefined && !Number.isFinite(this.minInput)) || (this.maxInput !== undefined && !Number.isFinite(this.maxInput))) errors.push("Minimum and maximum values must be finite numbers.");
    if (this.minInput !== undefined && this.maxInput !== undefined && this.minInput > this.maxInput) errors.push("Minimum value cannot be greater than maximum value.");
    if (this.pidInput && this.modeInput && !isModeValidForPid(this.pidInput, this.modeInput)) errors.push("Mode is invalid for this PID (PIDs 0x00-0xFF require 'Current Data').");
    if (this.formulaInput && !isValidFormula(this.modeInput, this.formulaInput)) {
      if (this.modeInput === "0x45") errors.push("Formula syntax invalid: Derived mode only allows getPID, getPIDRaw, getBit, bitMask, hex values, and math.");
      else errors.push("Formula syntax invalid: Standard modes only allow A, B, C, D, a, b, c, d, numbers, and basic math.");
    }
    if (this.lengthInput !== undefined && this.pidInput && !isLengthValidForPid(this.pidInput, this.lengthInput, this.modeInput)) errors.push("Length is invalid (PIDs <= 0xFF require 2 bytes, others require 3 bytes).");
    if (this.idInput && this.modeInput && !isIdValidForMode(this.modeInput, this.idInput)) errors.push("CAN ID is invalid for the selected mode (Current/Derived require 0x7DF, Read By Identifier requires 0x700-0x7FF).");
    if (this.priorityInput !== undefined && (!Number.isFinite(this.priorityInput) || !Number.isInteger(this.priorityInput) || this.priorityInput < MIN_PID_PRIORITY || this.priorityInput > MAX_PID_PRIORITY)) errors.push(`Priority must be an integer between ${MIN_PID_PRIORITY} and ${MAX_PID_PRIORITY}.`);
    if (this.updateIntervalInput !== undefined && (!Number.isFinite(this.updateIntervalInput) || !Number.isInteger(this.updateIntervalInput) || this.updateIntervalInput < 0 || this.updateIntervalInput > MAX_PID_INTERVAL || (this.updateIntervalInput !== 0 && this.updateIntervalInput < MIN_PID_INTERVAL))) errors.push(`Update Interval must be an integer from ${MIN_PID_INTERVAL} to ${MAX_PID_INTERVAL}ms (or 0 to disable).`);
    return errors;
  });

  isFormValid = $derived.by(() => {
    if (this.validationErrors.length > 0) return false;
    if (!isValidHex(this.pidInput)) return false;
    if (!isValidName(this.nameInput)) return false;
    if (!this.modeInput) return false;
    if (!isValidFormula(this.modeInput, this.formulaInput)) return false;
    if (this.modeInput !== "0x45" && !isLengthValidForPid(this.pidInput, this.lengthInput, this.modeInput)) return false;
    if (!isIdValidForMode(this.modeInput, this.idInput)) return false;
    if (this.priorityInput === undefined || !Number.isFinite(this.priorityInput) || !Number.isInteger(this.priorityInput) || this.priorityInput < MIN_PID_PRIORITY || this.priorityInput > MAX_PID_PRIORITY) return false;
    if (this.updateIntervalInput === undefined) return false;
    return true;
  });

  clearForm() {
    this.pidInput = "";
    this.nameInput = "";
    this.descInput = "";
    this.unitInput = "";
    this.minInput = undefined;
    this.maxInput = undefined;
    this.modeInput = "";
    this.formulaInput = "";
    this.lengthInput = undefined;
    this.idInput = "";
    this.priorityInput = 4;
    this.updateIntervalInput = 512;
    this.colorInput = "#01AAFF";
    this.iconInput = "";
    this.lastValidPidForLength = "";
    this.lastValidModeForId = "";
  }


  importPidDefinitions(defs: RawPidDef[]) {
    const newRows = [];
    for (let def of defs) {
      // Validate or map minimally
      if (!def || typeof def !== "object") continue;
      const rawPid = Number(def.pid);
      if (isNaN(rawPid)) continue;

      // Normalize short keys from SD card JSON to frontend expected keys
      const normalizedDef = {
        ...def,
        description: def.description !== undefined ? def.description : def.desc,
        length: def.length !== undefined ? def.length : def.len,
        minValue: def.minValue !== undefined ? def.minValue : def.minV,
        maxValue: def.maxValue !== undefined ? def.maxValue : def.maxV,
        update_interval_ms: def.update_interval_ms !== undefined ? def.update_interval_ms : def.interval,
      };
      
      const newRow = createRowObject(normalizedDef as PidDefinition, true, false);
      newRows.push(newRow);
    }
    
    // Merge or replace
    const current = [...this.local_piddef];
    for (const row of newRows) {
      const existingIdx = current.findIndex(r => parseInt(r.pid, 16) === parseInt(row.pid, 16));
      if (existingIdx >= 0) {
        current[existingIdx] = row;
      } else {
        current.push(row);
      }
    }
    
    this.local_piddef = current;
    return newRows.length;
  }

}

export const telemetryStore = new TelemetryStore();

export type PidRow = ReturnType<typeof createRowObject>;

export function createRowObject(def: PidDefinition, loaded: boolean, selected: boolean) {
  const rawPid = Number(def.pid);
  return {
    def: def,
    // Static fields
    name: def.name,
    moduleDescription: def.description,
    pid: "0x" + rawPid.toString(16).toUpperCase().padStart(2, "0"),
    metricUnit: def.unit,
    formula: def.formula,
    priority: def.priority,
    mode: def.mode,
    modeDisplayFormat: "hex",
    modeDescription: getModeLabel(def.mode),
    min_val: def.minValue,
    max_val: def.maxValue,
    len: def.length,
    updateInterval: def.update_interval_ms,

    // Stateful editable fields
    loaded: loaded,
    selected: selected,

    // Dynamic reactive getters
    get value() {
      return connectionStore.pids.get(rawPid)?.value ?? "N/A";
    },
    get isSupported() {
      return (
        vehicleStore.obd2Status?.supported_pids.groups.get(rawPid) ??
        connectionStore.pids.get(rawPid)?.isSupported ??
        false
      );
    },
    get supported() {
      return this.isSupported ? "Yes" : "No";
    },
    get badgeColor() {
      return this.isSupported ? "var(--normal-color)" : "var(--error-color)";
    },
    get isValid() {
      return connectionStore.pids.get(rawPid)?.isValid ?? false;
    },
    get validDisplay() {
      return this.isValid ? "Yes" : "No";
    },
    get validBadgeColor() {
      return this.isValid ? "var(--normal-color)" : "var(--error-color)";
    },
    get isDirty() {
      const original = vehicleStore.pidDefinitions.find((d) => Number(d.pid) === rawPid);
      const originalLoaded = !!original;

      if (this.loaded !== originalLoaded) return true;
      if (!original) return false;

      // Object.is: NaN (unset min/max) equals itself
      return (
        def.name !== original.name ||
        def.description !== original.description ||
        def.formula !== original.formula ||
        def.unit !== original.unit ||
        !Object.is(def.minValue, original.minValue) ||
        !Object.is(def.maxValue, original.maxValue) ||
        def.mode !== original.mode ||
        def.length !== original.length ||
        def.update_interval_ms !== original.update_interval_ms ||
        def.id !== original.id ||
        def.priority !== original.priority ||
        def.color !== original.color ||
        def.icon !== original.icon
      );
    },
    get pendingChanges() {
      return this.isDirty ? "Yes" : "No";
    },
    get pendingChangesColor() {
      return this.isDirty ? "var(--pico-primary)" : "rgba(255, 255, 255, 0.2)";
    },
  };
}

export function handleSavePid() {
  if (!telemetryStore.isFormValid) return;

  const rawPid = parseInt(telemetryStore.pidInput.replace(/^0x/i, ""), 16);
  const newDef = {
    pid: rawPid,
    name: telemetryStore.nameInput,
    description: telemetryStore.descInput,
    unit: telemetryStore.unitInput,
    minValue: telemetryStore.minInput,
    maxValue: telemetryStore.maxInput,
    mode: parseInt(telemetryStore.modeInput.replace(/^0x/i, ""), 16),
    length:
      telemetryStore.modeInput === "0x45" ? 0 : telemetryStore.lengthInput,
    formula: telemetryStore.formulaInput,
    id: parseInt(telemetryStore.idInput.replace(/^0x/i, ""), 16),
    priority: telemetryStore.priorityInput,
    update_interval_ms: telemetryStore.updateIntervalInput,
    color: parseInt(telemetryStore.colorInput.replace("#", ""), 16),
    icon: telemetryStore.iconInput,
  } as PidDefinition; // min, max and length may be unset (undefined) while the form is partial

  const localIndex = telemetryStore.local_piddef.findIndex(
    (row) => parseInt(row.pid, 16) === rawPid,
  );

  if (localIndex >= 0) {
    const loaded = telemetryStore.local_piddef[localIndex].loaded;
    const selected = telemetryStore.local_piddef[localIndex].selected;
    telemetryStore.local_piddef[localIndex] = createRowObject(
      newDef,
      loaded,
      selected,
    );
  } else {
    telemetryStore.local_piddef = [
      ...telemetryStore.local_piddef,
      createRowObject(newDef, true, false),
    ];
  }

  // Force reactivity on the array if needed (though localIndex assignment might suffice, it's safer)
  telemetryStore.local_piddef = [...telemetryStore.local_piddef];
}


