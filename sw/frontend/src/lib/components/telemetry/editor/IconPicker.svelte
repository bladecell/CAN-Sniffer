<script lang="ts">
  import GridPicker, { type GridOption } from "./GridPicker.svelte";
  import { iconLabel } from "./pidIcons";

  /** Icon field: the shared grid popover, filled with the PID icons. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: string;
  }

  let { id, labelId, describedBy, value = $bindable() }: Props = $props();

  const iconFiles = import.meta.glob("$lib/icons_svg/*.svg");
  // PID icons only: the UI glyphs (check, chevron, plus...) are not offered. A saved icon outside this list still shows.
  const pidIcons = ["thermometer", "gauge", "droplet", "battery-full", "gas-pump", "fan", "stopwatch", "bolt", "engine", "dial", "clock", "route", "timeline-arrow", "chart"];
  const available = new Set(Object.keys(iconFiles).map((path) => path.split("/").pop()!.replace(".svg", "")));
  const names = $derived(value && !pidIcons.includes(value) && available.has(value) ? [value, ...pidIcons] : pidIcons);
  // The first tile clears the icon: an empty icon saves as "" (docs/pid-config.md).
  const options = $derived<GridOption[]>([
    { value: "", label: "None", icon: "x" },
    ...names
      .filter((name) => available.has(name))
      .map((name) => ({
        value: name,
        icon: name,
        label: iconLabel(name),
      })),
  ]);

</script>

<GridPicker {id} {labelId} {describedBy} bind:value {options} showLabels={false} />
