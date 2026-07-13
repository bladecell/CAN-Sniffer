export class TelemetryStore {
  local_piddef = $state<any[]>([]);
  lastPidDef = $state<any[] | null>(null);
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
  return /^[0-9A-Za-z\-_.,\/*+ ]+$/.test(input);
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
