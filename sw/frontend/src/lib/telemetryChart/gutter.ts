// Shared plot-area insets in px. Every plot (overlay, single PID, lanes) and the overview strip use the same left and
// right x, so "now" sits under "now" on every plot and on the strip. Fixed, sized from measured text.
//
// Left: the widest y tick label ("10,000", measured in the app font) plus a gap.
// Right: only what keeps the line end, the "now" label and the cursor dot inside the plot. End labels are gone; lane
// values sit in the lane header band.

/** Measured in the app font (11.5 px, weight 600, tabular): the widest overlay y tick label, "100%". A wider single-PID
 *  label such as "10,000" (37.3 px) overhangs the gutter into the card padding; it is not clipped. */
export const MEASURED_TICK_PX = 31.7;
/** Gap between the tick text and the plot edge (the tick labels sit at left - 4). */
export const GUTTER_GAP_PX = 4;
/** Room past the plot's right edge for the line end and the cursor dot. */
export const GUTTER_RIGHT_PX = 8;

export interface Gutters {
  left: number;
  right: number;
}

export const DEFAULT_GUTTERS: Gutters = {
  left: Math.ceil(MEASURED_TICK_PX) + GUTTER_GAP_PX,
  right: GUTTER_RIGHT_PX,
};

export const GUTTER_LEFT = DEFAULT_GUTTERS.left;
export const GUTTER_RIGHT = DEFAULT_GUTTERS.right;
