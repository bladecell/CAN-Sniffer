<script lang="ts">
  import SmallCardChart from "./SmallCardChart.svelte";
  import { carBatteryFeed, BATTERY_HISTORY_MS } from "$lib/carBatteryFeed.svelte";
  import { BATTERY_SCALE_MAX_V, BATTERY_SCALE_MIN_V } from "$lib/carBattery";

  interface Props {
    color: string;
  }

  let { color }: Props = $props();
  const uid = $props.id();

  const chartData = $derived.by(() => {
    const { t, v } = carBatteryFeed.snapshot();
    return t.map((time, i) => ({ date: new Date(time), value: v[i] }));
  });

  const xDomain = $derived(
    chartData.length > 0
      ? [
          new Date(chartData[chartData.length - 1].date.getTime() - BATTERY_HISTORY_MS),
          chartData[chartData.length - 1].date,
        ]
      : undefined,
  );
</script>

<SmallCardChart
  data={chartData}
  {xDomain}
  yDomain={[BATTERY_SCALE_MIN_V, BATTERY_SCALE_MAX_V]}
  {color}
  gradientId="battery-area-{uid}"
  minHeight="120px"
/>
