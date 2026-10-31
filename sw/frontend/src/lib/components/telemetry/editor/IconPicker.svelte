<script lang="ts">
  import GridPicker, { type GridOption } from "./GridPicker.svelte";

  /** Icon field: the shared grid popover, filled with the PID icons. */
  interface Props {
    id: string;
    labelId: string;
    describedBy: string;
    value: string;
  }

  let { id, labelId, describedBy, value = $bindable() }: Props = $props();

  const iconFiles = import.meta.glob("$lib/icons_svg/*.svg");
  // Friendly names for the icons a PID usually shows. Other icons use their file name.
  const friendly: Record<string, string> = {
    thermometer: "Thermometer",
    gauge: "Gauge",
    droplet: "Fluid",
    "battery-full": "Battery",
    "gas-pump": "Fuel",
    fan: "Fan",
    stopwatch: "Timing",
    bolt: "Voltage",
    engine: "Engine",
  };
  // PID icons only: the UI glyphs (check, chevron, plus...) are not offered. A saved icon outside this list still shows.
  const pidIcons = ["thermometer", "gauge", "droplet", "battery-full", "gas-pump", "fan", "stopwatch", "bolt", "engine", "dial", "clock", "route", "timeline-arrow", "chart"];
  const available = new Set(Object.keys(iconFiles).map((path) => path.split("/").pop()!.replace(".svg", "")));
  const names = $derived(value && !pidIcons.includes(value) && available.has(value) ? [value, ...pidIcons] : pidIcons);
  const options = $derived<GridOption[]>(
    names
      .filter((name) => available.has(name))
      .map((name) => ({
        value: name,
        icon: name,
        label: friendly[name] ?? name.split("-").map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join(" "),
      })),
  );

</script>

<GridPicker {id} {labelId} {describedBy} bind:value {options} showLabels={false} />
