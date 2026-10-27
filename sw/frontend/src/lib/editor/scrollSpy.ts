/**
 * Scroll-spy: which section is in view. tops are section offsets inside the scroll container,
 * scrollTop is the container's scroll position, probe is the distance below the top edge
 * where a section counts as reached. atEnd forces the last section, because it may never reach the probe.
 */
export function activeSectionIndex(tops: number[], scrollTop: number, probe: number, atEnd = false): number {
  if (tops.length === 0) return 0;
  if (atEnd) return tops.length - 1;
  const line = scrollTop + probe;
  let active = 0;
  for (let i = 0; i < tops.length; i++) {
    if (tops[i] <= line) active = i;
  }
  return active;
}
