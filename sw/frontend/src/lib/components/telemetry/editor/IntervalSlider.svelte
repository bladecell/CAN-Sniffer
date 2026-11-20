<script lang="ts">
  import RangeSlider from "$lib/components/RangeSlider.svelte";
  import {
    SLIDER_MAX_MS,
    STATIC_MS,
    brief,
    intervalToPos,
    pageDown,
    pageUp,
    parseInterval,
    posToInterval,
    stepDown,
    stepUp,
  } from "$lib/editor/pollRate";

  /** Log slider from Static (far left) to 10 s. Arrow keys step, PageUp and PageDown jump, Home and End go to the ends. The readout can be typed. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: number | undefined;
    invalid?: boolean;
  }

  let { id, labelId, describedBy, value = $bindable(), invalid = false }: Props = $props();

  const ms = $derived(value ?? STATIC_MS);

  // Moving off Static returns to the last polled interval, not a default.
  let lastPolled = $state(512);
  $effect(() => {
    if (ms > 0) lastPolled = ms;
  });

  function keyFor(_: number, v: number, key: string): number | undefined {
    const next: Record<string, number> = {
      ArrowUp: stepUp(v, lastPolled),
      ArrowRight: stepUp(v, lastPolled),
      ArrowDown: stepDown(v),
      ArrowLeft: stepDown(v),
      PageUp: pageUp(v),
      PageDown: pageDown(v),
      Home: STATIC_MS,
      End: SLIDER_MAX_MS,
    };
    return next[key];
  }
</script>

<RangeSlider
  {id}
  labelIds={[labelId]}
  {describedBy}
  {invalid}
  values={[ms]}
  min={STATIC_MS}
  max={SLIDER_MAX_MS}
  toPos={intervalToPos}
  fromPos={posToInterval}
  {keyFor}
  format={brief}
  valueText={(v) => (v === 0 ? "Static, read once and never polled" : `${v} milliseconds`)}
  parse={parseInterval}
  ticks={[100, 500, 1000, 5000]}
  scale={[
    { value: 0, text: "Static", at: 0.06 },
    { value: 100, text: "100 ms" },
    { value: 1000, text: "1 s" },
    { value: SLIDER_MAX_MS, text: "10 s", edge: "right" },
  ]}
  stop={{ side: "left", share: 0.12, on: ms === 0 }}
  onChange={([v]) => (value = v)}
/>
