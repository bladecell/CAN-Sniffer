<script lang="ts">
  import { canStore } from "$lib/canStore.svelte";
  import Icon from "$lib/Icon.svelte";
  import ToggleSwitch from "$lib/components/ToggleSwitch.svelte";

  let statusColor = "oklch(1 0 0)";
  let isRecording = $derived(canStore.isRecording);
</script>

<div class="card bg-base-200/50 backdrop-blur-md border border-base-300 mb-8 overflow-hidden shadow-sm" style="--module-accent: {statusColor};">
  <div class="card-body p-4 md:p-6 flex flex-col gap-6">
    <div class="flex flex-col md:flex-row items-baseline gap-2 md:gap-6 flex-wrap">
      <h2 class="text-2xl font-bold m-0">Recorder</h2>
      <p class="text-xs md:text-sm font-bold uppercase tracking-wider m-0" style="color: var(--module-accent)">{isRecording ? "Recording" : "Not Recording"}</p>
    </div>

    <div class="flex flex-wrap items-center gap-4 w-full">
      <button
        type="button"
        class="btn btn-success btn-outline flex-[1_1_250px] h-[54px] rounded-lg font-medium text-base gap-2 hover:bg-success/20 hover:text-success hover:border-success/50"
        onclick={() => (canStore.isRecording = !canStore.isRecording)}
      >
        <Icon name={isRecording ? "square" : "circle-solid"} size={16} />
        {isRecording ? "Stop Recording" : "Record"}
      </button>

      <button
        type="button"
        class="btn btn-outline flex-[1_1_250px] h-[54px] rounded-lg font-medium text-base gap-2 opacity-80"
        // onclick={}
      >
        <Icon name="stopwatch" size={16} />
        Performance test – 0-100 km/h
      </button>

      <div class="w-full mt-2">
        <ToggleSwitch
          label="Auto Record"
          subLabel="Start recording when car is started"
          disabled={canStore.wsCanStatus?.state != "CAN Connected"}
        />
      </div>

      <div class="w-full mt-2">
        <ToggleSwitch
          label="Log Data"
          subLabel="Start polling and logging data from the car"
          checked={canStore.obd2Status?.continuous_running}
          onchange={(newState: boolean) => {
            canStore.setContinuousPolling(newState);
          }}
          disabled={canStore.wsCanStatus?.state != "CAN Connected"}
        />
      </div>
    </div>
  </div>
</div>
