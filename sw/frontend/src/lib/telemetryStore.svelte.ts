export class TelemetryStore {
  local_piddef = $state<any[]>([]);
  lastPidDef = $state<any[] | null>(null);

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
  priorityInput = $state<number | undefined>(5);
  colorInput = $state("#01AAFF");
  updateIntervalInput = $state<number | undefined>(512);
  iconInput = $state("");

  colorUint32 = $derived(parseInt(this.colorInput.replace("#", ""), 16));

  validationErrors = $derived.by(() => {
    let errors: string[] = [];
    if (this.pidInput && !isValidHex(this.pidInput)) errors.push("PID must be a valid hex format (e.g., 0x0C).");
    if (this.nameInput && !isValidName(this.nameInput)) errors.push("Name can only contain letters, numbers, and basic symbols.");
    if (this.descInput && !isValidDescription(this.descInput)) errors.push("Description contains invalid characters.");
    if (this.minInput !== undefined && this.maxInput !== undefined && this.minInput > this.maxInput) errors.push("Minimum value cannot be greater than maximum value.");
    if (this.pidInput && this.modeInput && !isModeValidForPid(this.pidInput, this.modeInput)) errors.push("Mode is invalid for this PID (PIDs 0x00-0xFF require 'Current Data').");
    if (this.formulaInput && !isValidFormula(this.modeInput, this.formulaInput)) {
      if (this.modeInput === "0x45") errors.push("Formula syntax invalid: Derived mode only allows getPID, getPIDRaw, getBit, bitMask, hex values, and math.");
      else errors.push("Formula syntax invalid: Standard modes only allow A, B, C, D, a, b, c, d, numbers, and basic math.");
    }
    if (this.lengthInput !== undefined && this.pidInput && !isLengthValidForPid(this.pidInput, this.lengthInput)) errors.push("Length is invalid (PIDs <= 0xFF require 2 bytes, others require 3 bytes).");
    if (this.idInput && this.modeInput && !isIdValidForMode(this.modeInput, this.idInput)) errors.push("CAN ID is invalid for the selected mode (Current/Derived require 0x7DF, Read By Identifier requires 0x700-0x7FF).");
    if (this.priorityInput !== undefined && (this.priorityInput < 1 || this.priorityInput > 255)) errors.push("Priority must be between 1 and 255.");
    if (this.updateIntervalInput !== undefined && this.updateIntervalInput !== 0 && (this.updateIntervalInput < 16 || this.updateIntervalInput > 4294967295)) errors.push("Update Interval must be at least 16ms (or 0 to disable).");
    return errors;
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
    this.priorityInput = 5;
    this.updateIntervalInput = 512;
    this.colorInput = "#01AAFF";
    this.iconInput = "";
    this.lastValidPidForLength = "";
    this.lastValidModeForId = "";
  }
}

export const telemetryStore = new TelemetryStore();

export function isValidHex(input: string): boolean {
  // 1. Must strictly start with "0x" or "0X"
  if (!/^0x/i.test(input)) return false;

  // 2. Remove "0x" prefix to validate the rest
  const cleanHex = input.replace(/^0x/i, "");

  // 4. Regex: Ensure it contains only valid hex characters (0-9, a-f)
  // and is not empty.
  const hexRegex = /^[0-9a-fA-F]+$/;
  if (!hexRegex.test(cleanHex)) return false;

  // 5. Size check: Max 4 chars (2 bytes)
  if (cleanHex.length > 4) return false;

  // 6. Value check: Ensure it's not negative
  const val = parseInt(cleanHex, 16);
  if (isNaN(val) || val < 0) return false;

  return true;
}

export function isValidName(input: string): boolean {
  if (!input) return false;
  if (input.length > 128) return false;
  return /^[0-9A-Za-z\-_.,\/*+ ]+$/.test(input);
}

export function isValidDescription(input: string): boolean {
  if (!input) return true;
  if (input.length > 256) return false;
  return /^[0-9A-Za-z\-\(\)_.,\/*+ ]+$/.test(input);
}

export function isModeValidForPid(pidInput: string, modeInput: string): boolean {
  if (!modeInput) return false; // Mode is strictly required

  // If PID hasn't been validly entered yet, just assume mode is fine for now
  if (!isValidHex(pidInput)) return true;

  const cleanHex = pidInput.replace(/^0x/i, "");
  const pidVal = parseInt(cleanHex, 16);

  // PIDs 0x00-0xFF -> ONLY Current Data (0x01)
  if (pidVal <= 0xFF) {
    return modeInput === "0x01";
  } else {
    // Larger PIDs -> Everything EXCEPT Current Data
    return modeInput !== "0x01";
  }
}

export function isLengthValidForPid(pidInput: string, lengthInput: number | undefined): boolean {
  if (lengthInput === undefined) return false;
  if (!isValidHex(pidInput)) return true;

  const cleanHex = pidInput.replace(/^0x/i, "");
  const pidVal = parseInt(cleanHex, 16);

  if (pidVal <= 0xFF) {
    return lengthInput === 2;
  } else {
    return lengthInput === 3;
  }
}

export function isIdValidForMode(modeInput: string, idInput: string): boolean {
  if (!idInput) return false;
  if (!isValidHex(idInput)) return false;

  const cleanHex = idInput.replace(/^0x/i, "");
  const idVal = parseInt(cleanHex, 16);

  if (modeInput === "0x01" || modeInput === "0x45") {
    return idVal === 0x7DF;
  } else if (modeInput === "0x22") {
    return idVal >= 0x700 && idVal <= 0x7FF;
  }

  return true;
}

export function isValidFormula(modeInput: string, formula: string): boolean {
  if (!formula) return false;

  // Default to standard math validation if mode is not yet selected
  if (!modeInput || modeInput === "0x01" || modeInput === "0x22") {
    // Current Data & Read Data By Identifier: Uses A, B, C, D, numbers, and basic math
    const hasValidChars = /^[A-Da-d0-9\s\.\+\-\*\/\(\)]+$/.test(formula);
    if (!hasValidChars) return false;

    // Use JavaScript's native parser to validate the structural syntax of the math equation!
    try {
      new Function("A", "B", "C", "D", "a", "b", "c", "d", "return " + formula + ";");
      return true;
    } catch (e) {
      return false;
    }
  } else if (modeInput === "0x45") {
    // Derived Data: Uses specific functions, commas, hex strings, numbers, and basic math
    const allowedFunctions = ["getPIDRaw", "getPID", "getBit", "bitMask"];

    let temp = formula;
    // Strip out the allowed functions
    for (const func of allowedFunctions) {
      temp = temp.split(func).join("");
    }

    // Completely strip out full hex values (e.g. 0x0C, 0x1A) so their inner A-F letters don't bleed out
    temp = temp.replace(/0x[0-9a-fA-F]+/gi, "");

    // What remains should purely be numbers (for constants), spaces, commas, dots, and math operators.
    // Any stray letters (like A, B, C, D, or an invalid function) will instantly trigger a validation failure.
    const hasValidChars = /^[0-9\s\.,\+\-\*\/\(\)]+$/.test(temp);
    if (!hasValidChars) return false;

    // Validate structural math syntax by mocking the allowed functions
    try {
      new Function("getPIDRaw", "getPID", "getBit", "bitMask", "return " + formula + ";");
      return true;
    } catch (e) {
      return false;
    }
  }

  return false;
}
