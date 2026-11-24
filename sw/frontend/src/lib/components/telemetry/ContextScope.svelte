<script lang="ts">
  import { setContext, type Snippet } from "svelte";
  import { PID_DEFS } from "$lib/telemetryChart/pidDefs";
  import type { PidDefinition } from "$lib/types";
  import { TELEMETRY_SOURCE, type SeriesSource } from "$lib/telemetryChart/feed.svelte";
  import { TELEMETRY_VIEW, type TelemetryView } from "$lib/telemetryChart/viewState.svelte";

  // Plots find their time window and sample source in context. This puts a different pair under one part of the page
  // (the recorded trip) without a second copy of the plot markup. The values are read once, so a change remounts it.
  let { view, source, defs, children }: { view: TelemetryView; source?: SeriesSource; defs?: () => readonly PidDefinition[]; children: Snippet } = $props();
  // svelte-ignore state_referenced_locally
  setContext(TELEMETRY_VIEW, view);
  // svelte-ignore state_referenced_locally
  if (source) setContext(TELEMETRY_SOURCE, source);
  // svelte-ignore state_referenced_locally
  if (defs) setContext(PID_DEFS, defs);
</script>

{@render children()}
