/** Short "how this page works" hints for the sidebar tabs. Edit the copy here. */
export const SIDEBAR_HINTS = {
  pids: "Tick a PID to chart it. On a phone, swipe a card left to edit or right to remove. The bottom button edits or removes the selected PIDs.",
  changes: "Ticked changes are applied: they are written to the device and saved. Revert puts a PID back to the device's copy. On a phone, swipe left to apply a change, right to revert it.",
  sd: "Open lists a file's PIDs. Import the ones you pick, then review them in Changes. Save selected PIDs overwrites the chosen file. To save PIDs, select them on the PIDs tab.",
  settings: "The SD folder is stored in this browser only.",
} as const;
