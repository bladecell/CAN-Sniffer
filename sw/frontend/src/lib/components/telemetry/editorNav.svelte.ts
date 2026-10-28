/** Editor sections and the active one. The tab bar sits in the sheet header; the form owns scrolling and sets goTo. */
export const EDITOR_SECTIONS = [
  { id: "request", label: "Request" },
  { id: "signal", label: "Signal" },
  { id: "display", label: "Display" },
  { id: "polling", label: "Polling" },
] as const;

export const editorNav = $state({ active: 0, goTo: (_index: number) => {} });
