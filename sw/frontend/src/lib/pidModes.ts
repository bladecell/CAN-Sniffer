/** Modes the PID editor offers. 0x09 (Vehicle Info) is not a PID mode here. Pure, so it is unit-tested (pidModes.test.ts). */
export const EDITOR_MODES = [
  { label: "Current Data", value: "0x01" },
  { label: "Read Data By Identifier", value: "0x22" },
  { label: "Derived Data", value: "0x45" },
];
