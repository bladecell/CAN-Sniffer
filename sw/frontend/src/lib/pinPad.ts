// Digit entry for the PIN pad and the PIN fields. Pure, so the rules are tested without Svelte.
export const PIN_MAX = 6;

/** Keeps only the digits of pasted or typed text, capped at `max`. */
export function pinFromText(text: string, max = PIN_MAX): string {
  return text.replace(/\D/g, "").slice(0, max);
}

/** Appends one digit. Anything that is not a single 0-9 character, or a value already at `max`, is ignored. */
export function pressDigit(value: string, digit: string, max = PIN_MAX): string {
  if (!/^[0-9]$/.test(digit) || value.length >= max) return value;
  return value + digit;
}

export function pressBackspace(value: string): string {
  return value.slice(0, -1);
}
