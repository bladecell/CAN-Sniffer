<script lang="ts">
  import SidebarHint from "$lib/components/telemetry/SidebarHint.svelte";
  import type { EcuRow } from "$lib/canBus/canModel";
  import { ageText, hzText } from "$lib/canBus/canFormat";
  import { formatId } from "$lib/canBus/canIds";

  interface Props {
    ecus: EcuRow[];
    now: number;
  }

  let { ecus, now }: Props = $props();
</script>

<div class="ce">
  <div>
    <div class="h-hint"><h2>ECU IDs</h2><SidebarHint inline label="About ECU IDs" text="Lists IDs that answer on the OBD response pattern (7E8-7EF, 18DAF1xx), with the request ID each one replies to. Passive: this tab sends nothing." /></div>
    <p class="muted">IDs that answer on the OBD response pattern (7E8-7EF, 18DAF1xx). Passive: nothing is sent.</p>
  </div>

  {#if ecus.length === 0}
    <p class="muted">No OBD responses seen yet. Silent ECUs leave this list after a minute.</p>
  {:else}
    <ul class="ce-list">
      {#each ecus as e (`${e.ext}-${e.id}`)}
        <li>
          <span>
            <b class="mono">{formatId(e.id, e.ext)}</b>
            <small class="mono">request {formatId(e.request, e.ext)}</small>
          </span>
          <em>DLC {e.dlc} · {hzText(e.hz)} · {ageText(e.seenMs, now)}</em>
        </li>
      {/each}
    </ul>
  {/if}
</div>

<style>
  .ce {
    display: grid;
    gap: 12px;
    min-width: 0;
  }
  .ce h2 {
    margin: 0;
  }
  .ce p {
    margin: 2px 12px 0 0;
    font-size: 0.88rem;
  }
  .ce-list {
    display: grid;
    gap: 8px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .ce-list li {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 6px 12px;
    padding-right: 14px;
    padding: 10px 12px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .ce-list small {
    display: block;
    white-space: nowrap;
    color: var(--muted);
    font-weight: 600;
  }
  .ce-list b {
    white-space: nowrap;
  }
  .ce-list em {
    font-style: normal;
    color: var(--muted);
    font-weight: 700;
    text-align: left;
    white-space: nowrap;
  }
</style>
