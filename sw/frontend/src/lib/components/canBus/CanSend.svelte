<script lang="ts">
  import { untrack } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import SidebarHint from "$lib/components/telemetry/SidebarHint.svelte";
  import Checkbox from "$lib/components/Checkbox.svelte";
  import RangeSlider from "$lib/components/RangeSlider.svelte";
  import { auth } from "$lib/auth.svelte";
  import { firmwareApi } from "$lib/firmwareApi.svelte";
  import { parseJsonResponse } from "$lib/api";
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import ItemList from "$lib/components/dashboard/ItemList.svelte";
  import {
    PERIOD_TICKS, REPEAT_STOP, REPEAT_TICKS, SEND_MAX_PERIOD_MS, SEND_MAX_REPEAT, SEND_MIN_PERIOD_MS, parsePeriod, parseRepeat, SEND_MAX_FRAMES, SEQUENCE_MAX_BYTES, idHex, moveAt, parseSendData, parseSendId, parseSequence, periodBrief, periodKey, periodToPos, planBody, posToPeriod, posToRepeat,
    reorderBy, repeatBrief, repeatKey, repeatToPos, sequenceSummary, serializeSequence, type SendFrame, type Sequence,
  } from "$lib/canBus/canSend";
  import { SEQ_DIR, isSeqName, parseSeqList, seqPath } from "$lib/canBus/canSequenceFiles";
  import type { CanModeStatus, CanSendStatus } from "$lib/types";

  let idText = $state("7DF");
  let dataText = $state("02 01 0C 00 00 00 00 00");
  let period = $state(100);
  let repeat = $state(1);
  type Queued = SendFrame & { key: string };
  let frames = $state<Queued[]>([]);
  let nextKey = 0;
  let openKey = $state<string | null>(null);
  // The text of the open row's fields. Edits apply to the frame as soon as they are valid, so the text is kept apart from the frame.
  let rowText = $state({ id: "", data: "" });
  let rowError = $state<string | null>(null);
  let fc = $state(false);
  let confirming = $state(false);
  let busy = $state(false);
  let error = $state<string | null>(null);
  let quickNote = $state<string | null>(null);
  let sender = $state<CanSendStatus | null>(null);
  let fileNote = $state<{ ok: boolean; text: string } | null>(null);
  let fileInput = $state<HTMLInputElement>();
  let saved = $state<string[] | null>(null);
  let saveName = $state("");
  let sdNote = $state<{ ok: boolean; text: string } | null>(null);
  let sdBusy = $state(false);

  const raw = $derived(canBus.mode?.mode === "raw");
  const running = $derived(sender?.running === true);
  const draftId = $derived(parseSendId(idText));
  const draftData = $derived(parseSendData(dataText));
  const nameBad = $derived(saveName !== "" && !isSeqName(saveName));

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

  // The saved list is a gated read: ask for it only once sending is possible.
  $effect(() => {
    if (auth.canUseGated && raw) untrack(() => void refreshSaved());
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

  /** The quick frame, or why it is not valid. */
  function quickFrame(): { ok: true; frame: SendFrame } | { ok: false; reason: string } {
    if (!draftId.ok) return { ok: false, reason: draftId.reason };
    if (!draftData.ok) return { ok: false, reason: draftData.reason };
    return { ok: true, frame: { id: draftId.value, data: draftData.value } };
  }

  function addQuick() {
    quickNote = null;
    const q = quickFrame();
    if (!q.ok) {
      error = q.reason;
      return;
    }
    if (frames.length >= SEND_MAX_FRAMES) {
      error = `A sequence holds at most ${SEND_MAX_FRAMES} frames`;
      return;
    }
    error = null;
    addRow(q.frame);
  }

  function addRow(f: SendFrame) {
    const key = String(nextKey++);
    frames = [...frames, { ...f, key }];
    openRow(key);
  }

  function addBlank() {
    if (frames.length >= SEND_MAX_FRAMES) return;
    addRow({ id: 0x7df, data: "00" });
  }

  function openRow(key: string | null) {
    openKey = key;
    rowError = null;
    const f = frames.find((q) => q.key === key);
    if (f) rowText = { id: idHex(f.id), data: f.data };
  }

  /** Edits one queued frame as it is typed. A bad value keeps the old frame and says why. */
  function editRow(key: string, field: "id" | "data") {
    const parsed = field === "id" ? parseSendId(rowText.id) : parseSendData(rowText.data);
    if (!parsed.ok) {
      rowError = parsed.reason;
      return;
    }
    rowError = null;
    frames = frames.map((f) => (f.key === key ? { ...f, [field]: parsed.value } : f));
  }

  const rowBad = (field: "id" | "data") => (field === "id" ? !parseSendId(rowText.id).ok : !parseSendData(rowText.data).ok);

  async function sendOne(frame: SendFrame, note: string) {
    if (!auth.canUseGated) return;
    busy = true;
    error = null;
    quickNote = null;
    try {
      await parseJsonResponse(await firmwareApi.sendCanFrame(frame));
      quickNote = note;
    } catch (e) {
      error = e instanceof Error ? e.message : "Not sent";
    } finally {
      busy = false;
      void readSender();
    }
  }

  function sendQuick() {
    const q = quickFrame();
    if (!q.ok) {
      error = q.reason;
      quickNote = null;
      return;
    }
    void sendOne(q.frame, `Sent ${idHex(q.frame.id)}`);
  }

  async function runSequence() {
    if (!auth.canUseGated || frames.length === 0) return;
    error = null;
    quickNote = null;
    busy = true;
    try {
      sender = await parseJsonResponse<CanSendStatus>(await firmwareApi.sendCanPlan(planBody(frames, period, repeat)));
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

  const current = (): Sequence => ({ frames, periodMs: period, repeat });

  /** Replaces the sequence, period and repeat with a checked file. */
  function applySequence(s: Sequence) {
    frames = s.frames.map((f) => ({ ...f, key: String(nextKey++) }));
    period = s.periodMs;
    repeat = s.repeat;
    openKey = null;
    rowError = null;
  }

  function exportFile() {
    const url = URL.createObjectURL(new Blob([serializeSequence(current())], { type: "application/json" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = "can-sequence.json";
    link.click();
    URL.revokeObjectURL(url);
  }

  async function importFile(e: Event) {
    const input = e.currentTarget as HTMLInputElement;
    const file = input.files?.[0];
    input.value = "";
    if (!file) return;
    if (file.size > SEQUENCE_MAX_BYTES) {
      fileNote = { ok: false, text: "File is larger than 16 KB. Nothing was loaded." };
      return;
    }
    const parsed = parseSequence(await file.text());
    if (!parsed.ok) {
      fileNote = { ok: false, text: `${parsed.reason}. Nothing was loaded.` };
      return;
    }
    applySequence(parsed.value);
    fileNote = { ok: true, text: `${parsed.value.frames.length} frames loaded` };
  }

  /** Throws the firmware's reason for a failed SD call. */
  async function must(res: Response) {
    if (!res.ok) await parseJsonResponse(res);
  }

  async function refreshSaved() {
    try {
      const res = await firmwareApi.sdTree(SEQ_DIR, { depth: 0, limit: 512 });
      // No folder yet is an empty list, not an error.
      saved = res.status === 404 ? [] : parseSeqList(await parseJsonResponse(res));
    } catch (e) {
      sdNote = { ok: false, text: e instanceof Error ? e.message : "Could not read the SD card" };
    }
  }

  async function sdRun(job: () => Promise<string | void>) {
    if (sdBusy) return;
    sdBusy = true;
    sdNote = null;
    try {
      const text = await job();
      if (text) sdNote = { ok: true, text };
    } catch (e) {
      sdNote = { ok: false, text: e instanceof Error ? e.message : "SD card error" };
    } finally {
      sdBusy = false;
    }
  }

  const saveToSd = () =>
    sdRun(async () => {
      if (!isSeqName(saveName)) throw new Error("Name is 1 to 32 letters, digits, - or _");
      if (frames.length === 0) throw new Error("Add at least one frame first");
      const replacing = saved?.includes(saveName);
      await must(await firmwareApi.createSdDirectory(SEQ_DIR));
      await must(await firmwareApi.uploadSdFile(seqPath(saveName), new Blob([serializeSequence(current())], { type: "application/octet-stream" })));
      await refreshSaved();
      return `${replacing ? "Replaced" : "Saved"} ${saveName}`;
    });

  const loadFromSd = (name: string) =>
    sdRun(async () => {
      const res = await firmwareApi.downloadSdFile(seqPath(name));
      await must(res);
      if (Number(res.headers.get("Content-Length")) > SEQUENCE_MAX_BYTES) throw new Error(`${name} is larger than 16 KB`);
      const parsed = parseSequence(await res.text());
      if (!parsed.ok) throw new Error(`${name}: ${parsed.reason}`);
      applySequence(parsed.value);
      return `Loaded ${name}, ${parsed.value.frames.length} frames`;
    });

  const deleteFromSd = (name: string) =>
    sdRun(async () => {
      await must(await firmwareApi.deleteSdFile(seqPath(name)));
      await refreshSaved();
      return `Deleted ${name}`;
    });
</script>

{#snippet body(row: { id: string })}
  {@const f = frames.find((q) => q.key === row.id)}
  {#if f}
    <div class="cs-edit">
      <div>
        <label class="lab" for="cs-fid-{row.id}">ID (hex)</label>
        <input id="cs-fid-{row.id}" class="in mono" aria-invalid={rowBad("id") || undefined} bind:value={rowText.id} oninput={() => editRow(row.id, "id")} />
      </div>
      <div>
        <label class="lab" for="cs-fd-{row.id}">Data (hex)</label>
        <input id="cs-fd-{row.id}" class="in mono" aria-invalid={rowBad("data") || undefined} bind:value={rowText.data} oninput={() => editRow(row.id, "data")} />
      </div>
      {#if rowError}
        <p class="hint bad" role="alert">{rowError}</p>
      {/if}
      <div class="brow">
        <button type="button" class="btn sm" disabled={busy || running || rowError !== null} onclick={() => void sendOne(f, `Sent ${idHex(f.id)}`)}>Send this frame</button>
      </div>
    </div>
  {/if}
{/snippet}

<div class="cs">
  <div class="hd">
    <h2>Send frames</h2>
    {#if raw && auth.canUseGated}
      <span class="chip c-warn">Raw mode</span>
    {/if}
  </div>
  {#if !auth.canUseGated}
    <p class="muted">Sending needs a login.</p>
  {/if}
  <p class="warn">
    <Icon name="triangle-exclamation" size={18} />
    <span>{raw ? "Transmits on the car's bus, exactly as typed. Polling is stopped." : "This transmits on the car's bus, exactly as typed."}</span>
  </p>

  {#if !auth.canUseGated}
    <button class="btn pri" onclick={() => void auth.ensureLogin()}>Log in to send</button>
  {:else if !raw}
    <div class="gate">
      <p>Sending needs Raw mode. Polling stops while it is on. Leaving this tab returns to Monitor.</p>
      <button
        type="button"
        class="btn pri"
        class:fill-crit={confirming}
        disabled={busy}
        onclick={() => (confirming ? void enterRaw() : (confirming = true))}
        onblur={() => (confirming = false)}
        onkeydown={(e) => {
          if (e.key === "Escape") confirming = false;
          else if (e.repeat) e.preventDefault();
        }}
      >
        {confirming ? "Confirm: stop polling" : "Switch to Raw mode"}
      </button>
      <span class="sr-only" role="status" aria-live="polite">{confirming ? "Press again to stop polling and switch to Raw mode. Cancels on Escape or when focus leaves." : ""}</span>
      {#if error}
        <p class="hint bad" role="alert">{error}</p>
      {/if}
    </div>
  {:else}
    <section class="pn" aria-labelledby="cs-qh">
      <h3 id="cs-qh">Quick send</h3>
      <div class="q">
        <div>
          <label class="lab" for="cs-id">ID (hex)</label>
          <input id="cs-id" class="in mono" aria-invalid={!draftId.ok || undefined} bind:value={idText} />
        </div>
        <div>
          <label class="lab" for="cs-data">Data (hex, 1 to 8 bytes)</label>
          <input id="cs-data" class="in mono" aria-invalid={!draftData.ok || undefined} bind:value={dataText} />
        </div>
      </div>
      <div class="brow">
        <button class="btn pri" disabled={busy} onclick={sendQuick}><Icon name="upload" size={16} /> Send once</button>
        <button class="btn" disabled={frames.length >= SEND_MAX_FRAMES} onclick={addQuick}>Add to sequence</button>
      </div>
      {#if quickNote}
        <p class="hint t-ok" role="status">{quickNote}</p>
      {/if}
    </section>

    <section aria-label="Sequence">
      <ItemList
        title="Sequence"
        rows={frames.map((f) => ({ id: f.key, name: idHex(f.id), unit: `${f.data.split(" ").length} B`, detail: f.data, mono: true }))}
        max={SEND_MAX_FRAMES}
        emptyHint="Empty. Add the quick frame above, or start a blank row."
        onRemove={(i) => (frames = frames.filter((_, j) => j !== i))}
        onMove={(i, d) => (frames = [...moveAt(frames, i, d)])}
        onReorder={(keys) => (frames = [...reorderBy(frames, (f) => f.key, keys)])}
        openId={openKey}
        onToggle={(key) => openRow(openKey === key ? null : key)}
        {body}
      />
      <button class="btn full" disabled={frames.length >= SEND_MAX_FRAMES} onclick={addBlank}>Add frame</button>
      <div class="brow">
        <button class="btn sm" disabled={frames.length === 0} onclick={exportFile}>Export</button>
        <button class="btn sm" onclick={() => fileInput?.click()}>Import</button>
        <input bind:this={fileInput} class="cs-file" type="file" accept="application/json,.json" onchange={importFile} />
      </div>
      {#if fileNote}
        <p class="hint" class:bad={!fileNote.ok} class:t-ok={fileNote.ok} role="status">{fileNote.text}</p>
      {/if}
    </section>

    <section class="pn" aria-labelledby="cs-th">
      <h3 id="cs-th">Timing</h3>
      <div>
        <span class="lab" id="cs-pl">Period between frames</span>
        <RangeSlider
          labelIds={["cs-pl"]}
          values={[period]}
          min={SEND_MIN_PERIOD_MS}
          max={SEND_MAX_PERIOD_MS}
          toPos={periodToPos}
          fromPos={posToPeriod}
          keyFor={(_, v, k) => periodKey(v, k)}
          format={periodBrief}
          valueText={(v) => `${v} milliseconds`}
          parse={parsePeriod}
          ticks={PERIOD_TICKS}
          scale={[
            { value: 10, text: "10 ms", edge: "left" },
            { value: 1000, text: "1 s" },
            { value: 60000, text: "1 min" },
            { value: 3600000, text: "1 h", edge: "right" },
          ]}
          onChange={([v]) => (period = v)}
        />
      </div>
      <div>
        <span class="lab" id="cs-rl">Repeat the sequence</span>
        <RangeSlider
          labelIds={["cs-rl"]}
          values={[repeat]}
          min={0}
          max={SEND_MAX_REPEAT}
          toPos={repeatToPos}
          fromPos={posToRepeat}
          keyFor={(_, v, k) => repeatKey(v, k)}
          format={repeatBrief}
          parse={parseRepeat}
          ticks={REPEAT_TICKS}
          scale={[
            { value: 1, text: "Once", edge: "left" },
            { value: 100, text: "100" },
            { value: 1000, text: "1k" },
            { value: 0, text: "Endless", edge: "right" },
          ]}
          stop={{ side: "right", share: REPEAT_STOP, on: repeat === 0 }}
          onChange={([v]) => (repeat = v)}
        />
      </div>
      <div class="chk">
        <span class="chk-l">
          ISO-TP flow control
          <SidebarHint inline text="Sends a flow-control frame (30 00 00) in reply to each first frame, so the ECU keeps sending the rest of a multi-frame message." label="About ISO-TP flow control" />
        </span>
        <Checkbox checked={fc} aria-label="ISO-TP flow control" onchange={toggleFc} />
      </div>
    </section>

    <section class="run" aria-label="Run">
      <p class="sum">{frames.length === 0 ? "Add at least one frame to run a sequence." : sequenceSummary(frames.length, period, repeat)}</p>
      {#if error}
        <p class="hint bad" role="alert">{error}</p>
      {/if}
      <div class="brow">
        <button class="btn pri" disabled={busy || running || frames.length === 0} onclick={runSequence}><Icon name="upload" size={16} /> Send sequence</button>
        <button class="btn" class:fill-crit={running} disabled={!running} onclick={stop}>Stop</button>
      </div>
      <div class="st" role="status">
        {#if running}
          <span class="chip c-ok">Transmitting</span>
          <span><b>{sender?.sent}</b> sent, every {periodBrief(sender?.period_ms ?? period)}</span>
        {:else if sender?.error}
          <span class="chip c-crit">Stopped</span>
          <span class="t-crit">Last plan stopped: {sender.error}.</span>
        {:else}
          <span class="chip c-plain">Idle</span>
          <span>Nothing is being sent.</span>
        {/if}
      </div>
    </section>

    <section class="pn" aria-labelledby="cs-sh">
      <h3 id="cs-sh">Saved on the SD card</h3>
      <div class="sv">
        <div>
          <label class="lab" for="cs-sn">Name</label>
          <input id="cs-sn" class="in mono" maxlength="32" placeholder="rpm-sweep" aria-invalid={nameBad || undefined} bind:value={saveName} />
        </div>
        <button class="btn pri" disabled={sdBusy || frames.length === 0 || !isSeqName(saveName)} onclick={saveToSd}>Save</button>
      </div>
      {#if nameBad}
        <p class="hint bad">Letters, digits, - and _ only, up to 32.</p>
      {/if}
      {#if saved && saved.length > 0}
        <ul class="sv-list" aria-label="Saved sequences">
          {#each saved as name (name)}
            <li>
              <button type="button" class="sv-name mono" disabled={sdBusy} onclick={() => loadFromSd(name)} title="Load {name}">{name}</button>
              <button type="button" class="btn icon danger sm" aria-label="Delete {name}" disabled={sdBusy} onclick={() => deleteFromSd(name)}>
                <Icon name="trash" size={18} />
              </button>
            </li>
          {/each}
        </ul>
      {:else if saved}
        <p class="hint">Nothing saved yet.</p>
      {/if}
      {#if sdNote}
        <p class="hint" class:bad={!sdNote.ok} class:t-ok={sdNote.ok} role="status">{sdNote.text}</p>
      {/if}
    </section>
  {/if}
</div>

<style>
  .cs {
    --ctl-h: 40px;
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 10px;
    min-width: 0;
  }
  .cs > * {
    min-width: 0;
  }
  .hd {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
  }
  h2 {
    margin: 0;
    font-size: 1.2rem;
    font-weight: 800;
  }
  h3 {
    display: flex;
    align-items: center;
    gap: 8px;
    margin: 0;
    font-size: 1rem;
    font-weight: 800;
  }
  p {
    margin: 0;
  }
  .warn {
    display: flex;
    gap: 8px;
    align-items: center;
    padding: 8px 12px;
    border-radius: var(--r-ctl);
    background: var(--warn-f, var(--ctl));
    color: var(--warn);
    font-size: 0.88rem;
    font-weight: 700;
  }
  .warn :global(svg) {
    flex: none;
  }
  .gate {
    display: grid;
    gap: 10px;
  }
  .pn {
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 8px;
    padding: 10px;
    border-radius: var(--r-track);
    background: var(--card2);
  }
  .q {
    display: grid;
    grid-template-columns: 64px minmax(0, 1fr);
    gap: 6px;
  }
  .cs-edit {
    display: grid;
    grid-template-columns: 64px minmax(0, 1fr);
    gap: 6px;
  }
  .cs-edit .brow,
  .cs-edit p {
    grid-column: 1 / -1;
  }
  .lab {
    display: block;
    margin-bottom: 4px;
    font-size: 0.78rem;
    font-weight: 700;
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-width: 0;
    height: var(--ctl-h);
    padding: 0 8px;
    font-size: 0.85rem;
  }
  .in[aria-invalid="true"] {
    outline: 2px solid var(--crit);
    outline-offset: 0;
  }
  .hint {
    font-size: 0.82rem;
    color: var(--muted);
    line-height: 1.35;
    overflow-wrap: anywhere;
  }
  .hint.bad {
    color: var(--crit);
    font-weight: 700;
  }
  .brow {
    display: flex;
    gap: 6px;
    margin-top: 8px;
  }
  .pn .brow,
  .run .brow,
  .cs-edit .brow {
    margin-top: 0;
  }
  .brow .btn {
    flex: 1 1 0;
    min-width: 0;
    padding: 0 8px;
    font-size: 0.92rem;
  }
  .cs .btn {
    min-height: var(--ctl-h);
    height: var(--ctl-h);
  }
  .cs .btn.sm {
    min-height: 32px;
    height: 32px;
  }
  .cs .btn.icon {
    flex: none;
    width: var(--ctl-h);
    padding: 0;
  }
  .cs .btn.icon.sm {
    width: 34px;
    height: 34px;
    min-height: 34px;
  }
  .run .brow .btn.pri {
    flex-grow: 2;
  }
  .full {
    width: 100%;
    margin-top: 8px;
  }
  .cs-file {
    display: none;
  }
  .chk {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    font-weight: 700;
  }
  .chk-l {
    display: flex;
    align-items: center;
    gap: 6px;
    line-height: 24px;
  }
  .chk-l :global(button) {
    margin: 0 !important;
  }
  .run {
    display: grid;
    gap: 10px;
  }
  .sum {
    color: var(--muted);
    font-size: 0.9rem;
    font-weight: 600;
  }
  .st {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 8px;
    color: var(--muted);
    font-size: 0.9rem;
    font-weight: 600;
  }
  .st b {
    color: var(--text);
    font-variant-numeric: tabular-nums;
  }
  .sv {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 6px;
    align-items: end;
  }
  .sv-list {
    display: grid;
    gap: 6px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .sv-list li {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 2px 6px 2px 4px;
    border-radius: var(--r-track);
    background: var(--row);
  }
  .sv-name {
    flex: 1 1 0;
    min-width: 0;
    min-height: 40px;
    padding: 0 8px;
    overflow: hidden;
    border: 0;
    background: none;
    color: inherit;
    font-weight: 800;
    text-align: left;
    text-decoration: underline dotted;
    text-underline-offset: 4px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
