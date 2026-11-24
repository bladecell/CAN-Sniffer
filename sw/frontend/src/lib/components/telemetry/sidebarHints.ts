/** Short "how this page works" hints for the sidebar tabs. Edit the copy here. */
export const SIDEBAR_HINTS = {
  pids: "Select a plot, then tick PIDs to chart them. With no plot selected, a tap selects one PID to edit. On a phone, swipe a card left to edit or right to remove. The bottom button edits or removes the selected PIDs.",
  changes: "Ticked changes are applied: they are written to the device and saved. Revert puts a PID back to the device's copy. On a phone, swipe left to apply a change, right to revert it.",
  sd: "Open lists a file's PIDs. Import the ones you pick, then review them in Changes. Save selected PIDs overwrites the chosen file. To save PIDs, select them on the PIDs tab.",
  replays: "Pick a trip recorded on the SD card to plot its PIDs and play it back. Leave this tab to go back to the live view.",
  settings: "The SD folder is stored in this browser only.",
} as const;
