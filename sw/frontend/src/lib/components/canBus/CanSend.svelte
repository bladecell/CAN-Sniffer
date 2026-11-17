<script lang="ts">
  import { untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { auth } from "$lib/auth.svelte";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import ItemList from "$lib/components/dashboard/ItemList.svelte";
  import { idHex, moveAt, parseIntIn, parseSendData, parseSendId, planBody, SEND_MAX_FRAMES, SEND_MAX_PERIOD_MS, SEND_MAX_REPEAT, SEND_MIN_PERIOD_MS, reorderBy, type SendFrame } from "$lib/canBus/canSend";
  import type { CanModeStatus, CanSendStatus } from "$lib/types";

  let idText = $state("7DF");
  let dataText = $state("02 01 0C 00 00 00 00 00");
  let periodText = $state("");
  let repeatText = $state("1");
  type Queued = SendFrame & { key: string };
  let frames = $state<Queued[]>([]);
  let nextKey = 0;
  let fc = $state(false);
  let confirming = $state(false);
  let busy = $state(false);
  let error = $state<string | null>(null);
  let sender = $state<CanSendStatus | null>(null);

  const raw = $derived(canBus.mode?.mode === "raw");

  async function readSender() {
    try {
      sender = await parseJsonResponse<CanSendStatus>(await firmwareApi.canSend());
    } catch {
      // Keep the last status; the next poll tries again.
    }
  }

  // The sender status is an open read: poll it while this tab is open.
  $effect(() =>
    untrack(() => {
      void readSender();
      const timer = setInterval(() => void readSender(), 1000);
      return () => clearInterval(timer);
    }),
  );

  // Leaving the tab (or the page) ends Raw mode and stops any running plan. The device also ends Raw on close.
  // Only a running plan and Raw mode need a call: both are gated, so an idle tab must not ask for a login.
  $effect(() => () => {
    untrack(() => {
      if (sender?.running) void firmwareApi.stopCanSend().catch(() => {});
      if (canBus.mode?.mode === "raw") void setMode({ mode: "monitor" });
    });
  });

  async function setMode(mode: { mode: "monitor" | "raw"; isotp_flow_control?: boolean }) {
    try {
      canBus.mode = await parseJsonResponse<CanModeStatus>(await firmwareApi.setCanMode(mode));
      error = null;
    } catch (e) {
      error = e instanceof Error ? e.message : "Could not change the mode";
    }
  }

  async function enterRaw() {
    confirming = false;
    busy = true;
    await setMode({ mode: "raw", isotp_flow_control: fc });
    busy = false;
  }

  async function toggleFc(on: boolean) {
    fc = on;
    if (raw) await setMode({ mode: "raw", isotp_flow_control: on });
  }

  /** The frame(s) to send: the sequence if there is one, else the draft frame. */
  function current(): { ok: true; list: SendFrame[] } | { ok: false; reason: string } {
    const id = parseSendId(idText);
    if (!id.ok) return { ok: false, reason: id.reason };
    const data = parseSendData(dataText);
    if (!data.ok) return { ok: false, reason: data.reason };
    const list = frames.length > 0 ? frames : [{ id: id.value, data: data.value }];
    return { ok: true, list };
  }

  function addFrame() {
    const c = current();
    if (!c.ok) {
      error = c.reason;
      return;
    }
    if (frames.length >= SEND_MAX_FRAMES) {
      error = `A sequence holds at most ${SEND_MAX_FRAMES} frames`;
      return;
    }
    const id = parseSendId(idText);
    const data = parseSendData(dataText);
    if (!id.ok || !data.ok) return;
    frames = [...frames, { id: id.value, data: data.value, key: String(nextKey++) }];
    error = null;
  }

  async function send() {
    if (!auth.canUseGated) return;
    error = null;
    const c = current();
    if (!c.ok) {
      error = c.reason;
      return;
    }
    const periodTrimmed = periodText.trim();
    const period = periodTrimmed === "" ? null : parseIntIn(periodTrimmed, SEND_MIN_PERIOD_MS, SEND_MAX_PERIOD_MS);
    if (periodTrimmed !== "" && period === null) {
      error = `Period must be ${SEND_MIN_PERIOD_MS} ms to ${SEND_MAX_PERIOD_MS} ms`;
      return;
    }
    const repeat = parseIntIn(repeatText, 0, SEND_MAX_REPEAT);
    if (repeat === null) {
      error = `Repeat must be 0 to ${SEND_MAX_REPEAT} (0 repeats until stopped)`;
      return;
    }
    busy = true;
    try {
      if (period === null && c.list.length === 1) {
        await parseJsonResponse(await firmwareApi.sendCanFrame(c.list[0]));
      } else if (period === null) {
        error = "A sequence needs a period";
      } else {
        sender = await parseJsonResponse<CanSendStatus>(await firmwareApi.sendCanPlan(planBody(c.list, period, repeat)));
      }
    } catch (e) {
      error = e instanceof Error ? e.message : "Not sent";
    } finally {
      busy = false;
      void readSender();
    }
  }

  async function stop() {
    busy = true;
    try {
      sender = await parseJsonResponse<CanSendStatus>(await firmwareApi.stopCanSend());
    } catch (e) {
      error = e instanceof Error ? e.message : "Could not stop";
    } finally {
      busy = false;
    }
  }
</script>

<div class="cs">
  <div>
    <h2>Send frames</h2>
    {#if !auth.canUseGated}
      <p class="muted">Sending needs a login.</p>
    {/if}
  </div>
  <p class="cs-warn"><Icon name="triangle-exclamation" size={18} /> This transmits on the car's bus, exactly as typed.</p>

  {#if !auth.canUseGated}
    <button class="btn pri" onclick={() => void auth.ensureLogin()}>Log in to send</button>
  {:else if !raw}
    <div class="cs-gate">
      <p>Sending needs Raw mode. Polling stops while it is on. Leaving this tab returns to Monitor.</p>
      {#if confirming}
        <p class="cs-q">Stop polling and switch to Raw mode?</p>
        <div class="cs-row">
          <button class="btn pri" disabled={busy} onclick={enterRaw}>Switch</button>
          <button class="btn" onclick={() => (confirming = false)}>Cancel</button>
        </div>
      {:else}
        <button class="btn pri" onclick={() => (confirming = true)}>Switch to Raw mode</button>
      {/if}
    </div>
  {:else}
    <div class="cs-form">
      <span class="chip c-warn">Raw mode, polling stopped</span>

      <div class="cs-row">
        <div>
          <label class="lab" for="cs-id">ID (hex, standard)</label>
          <input id="cs-id" class="in mono" bind:value={idText} />
        </div>
      </div>
      <label class="lab" for="cs-data">Data (hex, 1 to 8 bytes)</label>
      <input id="cs-data" class="in mono" bind:value={dataText} />

      <div class="cs-row">
        <div>
          <label class="lab" for="cs-period">Period (ms, min {SEND_MIN_PERIOD_MS})</label>
          <input id="cs-period" class="in mono" inputmode="numeric" bind:value={periodText} placeholder="single send" />
        </div>
        <div>
          <label class="lab" for="cs-repeat">Repeat (0 = until stopped)</label>
          <input id="cs-repeat" class="in mono" inputmode="numeric" bind:value={repeatText} />
        </div>
      </div>

      <ItemList
        title="Sequence"
        rows={frames.map((f) => ({ id: f.key, name: idHex(f.id), unit: "", detail: f.data }))}
        max={SEND_MAX_FRAMES}
        emptyHint="Empty. Send uses the frame above, or add frames to build a sequence."
        onRemove={(i) => (frames = frames.filter((_, j) => j !== i))}
        onMove={(i, d) => (frames = [...moveAt(frames, i, d)])}
        onReorder={(keys) => (frames = [...reorderBy(frames, (f) => f.key, keys)])}
      />
      <button class="btn" disabled={frames.length >= SEND_MAX_FRAMES} onclick={addFrame}>Add frame to sequence</button>

      <label class="cs-check"><input type="checkbox" checked={fc} onchange={(e) => toggleFc(e.currentTarget.checked)} /> ISO-TP flow control (answers first frames with 30 00 00)</label>

      {#if error}
        <p class="t-crit" role="alert">{error}</p>
      {/if}

      <div class="cs-row">
        <button class="btn pri" disabled={busy} onclick={send}>
          <Icon name="upload" size={16} /> Send
        </button>
        <button class="btn" disabled={!sender?.running} onclick={stop}>Stop</button>
      </div>
      <p class="muted">
        {#if sender?.running}
          Transmitting: {sender.sent} of {sender.total || "until stopped"} sent, every {sender.period_ms} ms. Stops on Stop, a mode change, or when this page closes.
        {:else if sender?.error}
          Last plan stopped: {sender.error}.
        {:else}
          Nothing is being sent.
        {/if}
      </p>
    </div>
  {/if}
</div>

<style>
  .cs {
    display: grid;
    gap: 12px;
    min-width: 0;
  }
  .cs h2 {
    margin: 0;
  }
  .cs p {
    margin: 0;
  }
  .cs-warn {
    display: flex;
    gap: 8px;
    align-items: flex-start;
    padding: 8px 12px;
    border-radius: var(--r-ctl);
    background: var(--warn-f, var(--ctl));
    color: var(--warn);
    font-size: 0.88rem;
    font-weight: 700;
  }
  .cs-gate,
  .cs-form {
    display: grid;
    gap: 10px;
  }
  .cs-row {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    align-items: end;
  }
  .cs-row > div {
    flex: 1 1 120px;
    min-width: 0;
  }
  .cs-form .in {
    width: 100%;
    min-width: 0;
  }
  .lab {
    display: block;
    font-size: 0.85rem;
    font-weight: 700;
  }
  .cs-check {
    display: flex;
    gap: 8px;
    align-items: center;
    font-weight: 600;
  }
  .cs-q {
    font-weight: 700;
  }
</style>
