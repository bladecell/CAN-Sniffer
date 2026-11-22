<script lang="ts">
  import SidebarHint from "$lib/components/telemetry/SidebarHint.svelte";
  import { tick, untrack } from "svelte";
  import { slide } from "svelte/transition";
  import { accordion } from "$lib/components/accordion";
  import Icon from "$lib/Icon.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import Checkbox from "$lib/components/Checkbox.svelte";
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import { newRuleId, rulesStore } from "$lib/canBus/canRulesStore.svelte";
  import { MAX_BYTE_FIELDS, emptyForm, formFromRule, fullMask, matchSummary, parseHex, ruleFromForm, type RuleForm } from "$lib/canBus/canRuleForm";
  import { compileFormula, FRAME_LETTERS } from "$lib/editor/formulaTest";
  import { hexBytes } from "$lib/canBus/canIds";
  import { helperFormula } from "$lib/canBus/canRuleHelper";
  import { RULE_CAP, type Rule } from "$lib/canBus/canRules";
  import { parseRules, RULES_MAX_BYTES } from "$lib/canBus/canRulesIo";

  let form = $state<RuleForm>(emptyForm());
  let openId = $state<string | null>(null);
  // The row of a new rule: it exists only in the editor until the form is valid, then it is saved under this id.
  let draftId = $state<string | null>(null);
  let helper = $state({ start: "0", length: "1", order: "be" as "be" | "le" });
  let storeError = $state<string | null>(null);
  let fileNote = $state<{ ok: boolean; text: string } | null>(null);
  let fileInput = $state<HTMLInputElement>();

  const full = $derived(rulesStore.list.length >= RULE_CAP);
  const built = $derived(ruleFromForm(form));
  const problem = $derived(storeError ?? (built.ok ? null : built.reason));
  const helperResult = $derived(helperFormula({ start: Number(helper.start), length: Number(helper.length), order: helper.order }));
  const blank: Rule = { id: "", name: "", match: { id: 0, ext: false, idMask: 0x7ff, bytes: [] }, formula: "" };
  const rows = $derived<Rule[]>(draftId && !rulesStore.list.some((r) => r.id === draftId) ? [...rulesStore.list, { ...blank, id: draftId }] : rulesStore.list);

  // Live value for the open rule: the draft formula on the newest frame its match accepts. Re-runs on every snapshot.
  const preview = $derived.by(() => {
    canBus.snapshot;
    if (!built.ok || built.value.formula === "") return null;
    const frame = canBus.lastMatchFor(built.value.match);
    if (!frame) return { frame: null, value: "..." };
    const compiled = compileFormula(built.value.formula, FRAME_LETTERS);
    const r = compiled.ok ? compiled.run(frame.data) : null;
    return { frame: hexBytes(frame.data), value: r?.ok ? String(Number(r.value.toFixed(2))) : "invalid" };
  });

  const badHex = (t: string, max: number, optional = false) => (optional && t.trim() === "" ? false : parseHex(t, max) === null);
  const badInt = (t: string, min: number, max: number) => !/^\d+$/.test(t.trim()) || Number(t) < min || Number(t) > max;
  const badDlc = $derived(form.dlc !== "" && badInt(form.dlc, 0, 8));

  async function focusName(id: string) {
    await tick();
    const el = document.getElementById(`cr-nm-${id}`);
    el?.scrollIntoView({ block: "nearest" });
    el?.focus();
  }

  function close() {
    openId = null;
    draftId = null;
    storeError = null;
  }

  function openRule(rule: Rule) {
    form = formFromRule(rule);
    helper = { start: "0", length: "1", order: "be" };
    draftId = null;
    storeError = null;
    openId = rule.id;
    void tick().then(() => document.getElementById(`cr-row-${rule.id}`)?.scrollIntoView({ block: "nearest" }));
  }

  function toggle(rule: Rule) {
    if (openId === rule.id) return close();
    openRule(rule);
  }

  /** A new row that exists only in the editor until the form is edited into a valid rule. */
  function startDraft(next: RuleForm, id: string) {
    form = next;
    helper = { start: "0", length: "1", order: "be" };
    storeError = null;
    draftId = openId = id;
    void focusName(id);
  }

  function startNew() {
    if (full) return;
    startDraft(emptyForm(), newRuleId());
  }

  // "Create rule" / "Open rule" from the ID details. takeRule clears the request, so a later visit to this tab does not repeat it.
  $effect(() => {
    if (!canBus.pendingRule) return;
    untrack(() => {
      const req = canBus.takeRule();
      if (req?.kind === "open") {
        const rule = rulesStore.list.find((r) => r.id === req.id);
        if (rule) openRule(rule);
      } else if (req?.kind === "draft") {
        if (full) storeError = `At most ${RULE_CAP} rules`;
        else startDraft(req.form, req.id);
      }
    });
  });

  /** Edits apply as they are typed. An invalid form keeps the last good rule and the fields say what is wrong. */
  function apply() {
    if (!openId || !built.ok) return;
    storeError = null;
    const cur = rulesStore.list.find((r) => r.id === openId);
    const draft = cur?.enabled === false ? { ...built.value, enabled: false } : built.value;
    const result = cur ? rulesStore.update(openId, draft) : rulesStore.add(draft, openId);
    if (!result.ok) storeError = result.reason;
  }

  function removeOpen() {
    if (openId) rulesStore.remove(openId);
    close();
  }

  function removeByte(i: number) {
    form.bytes = form.bytes.filter((_, j) => j !== i);
    apply();
  }

  function addByte() {
    const used = new Set(form.bytes.map((b) => Number(b.index)));
    const index = [0, 1, 2, 3, 4, 5, 6, 7].find((n) => !used.has(n)) ?? 0;
    form.bytes = [...form.bytes, { index: String(index), value: "00", mask: "FF" }];
    apply();
  }

  /** Start, length or order changed: write the formula for those bytes. Bad bytes leave the formula alone. */
  function writeFormula() {
    if (helperResult.ok) form.formula = helperResult.formula;
    apply();
  }

  function exportFile() {
    const url = URL.createObjectURL(new Blob([rulesStore.exportText()], { type: "application/json" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = "can-rules.json";
    link.click();
    URL.revokeObjectURL(url);
  }

  /** Import replaces the rules. One bad rule in the file saves nothing, and the reason says which one. */
  async function importFile(e: Event) {
    const input = e.currentTarget as HTMLInputElement;
    const file = input.files?.[0];
    input.value = "";
    if (!file) return;
    if (file.size > RULES_MAX_BYTES) {
      fileNote = { ok: false, text: "File is larger than 64 KB. Nothing was saved." };
      return;
    }
    const parsed = parseRules(await file.text());
    if (!parsed.ok) {
      fileNote = { ok: false, text: parsed.reason };
      return;
    }
    close();
    rulesStore.replaceAll(parsed.rules);
    fileNote = { ok: true, text: `${parsed.rules.length} rules imported` };
  }

  type Cell = { text: string; cls: "acc" | "crit" | "muted" };
  function ruleValue(rule: Rule): Cell {
    if (rule.enabled === false) return { text: "paused", cls: "muted" };
    if (rule.formula === "") return { text: "hides", cls: "muted" };
    const v = canBus.snapshot.ruleValues.get(rule.id);
    if (v === undefined) return { text: "no frame yet", cls: "muted" };
    return { text: v, cls: v === "invalid" ? "crit" : "acc" };
  }

  const nameOf = (rule: Rule) => (openId === rule.id ? form.name : rule.name) || "Unnamed";
  function summaryOf(rule: Rule): string {
    if (openId !== rule.id) return `${matchSummary(rule.match)}${rule.formula ? ` · ${rule.formula}` : ""}`;
    return built.ok ? `${matchSummary(built.value.match)}${built.value.formula ? ` · ${built.value.formula}` : ""}` : "Not valid yet";
  }
</script>

<!-- Escape closes the open editor, like Done. -->
<div class="cr" role="presentation" onkeydown={(e) => e.key === "Escape" && openId && close()}>
  <div class="cr-hd">
    <div class="h-hint"><h2>Rules</h2><SidebarHint inline label="About rules" text="A rule names an ID and can decode its bytes with a formula. Rules are stored on the device and can be exported and imported as a file." /></div>
    <span class="tag">{rulesStore.list.length} of {RULE_CAP}</span>
  </div>

  <div class="cr-brow">
    <button class="btn pri" aria-disabled={full} onclick={startNew}>New rule</button>
    <button class="btn" disabled={rulesStore.list.length === 0} onclick={exportFile}>Export</button>
    <button class="btn" onclick={() => fileInput?.click()}>Import</button>
    <input bind:this={fileInput} class="cr-file" type="file" accept="application/json,.json" onchange={importFile} />
  </div>
  {#if fileNote}
    <p class="hint" class:bad={!fileNote.ok} class:t-ok={fileNote.ok} role="status">{fileNote.text}</p>
  {/if}

  {#if rows.length === 0}
    <div class="cr-empty">No rules yet. New rule starts one, Import loads a file.</div>
  {:else}
    <ul class="rl" aria-label="Rules">
      {#each rows as rule (rule.id)}
        {@const open = openId === rule.id}
        {@const cell = open && !built.ok ? ({ text: "invalid", cls: "crit" } as Cell) : ruleValue(rule)}
        <li class="rl-i" class:on={open} class:off={rule.enabled === false} id="cr-row-{rule.id}">
          <button type="button" class="rl-open" aria-expanded={open} aria-controls="cr-body-{rule.id}" onclick={() => toggle(rule)}>
            <span class="nm">{nameOf(rule)}</span>
            <small>{summaryOf(rule)}</small>
          </button>
          <span class="rl-v {cell.cls}">{cell.text}</span>
          {#if rulesStore.list.some((r) => r.id === rule.id)}
            <Switch checked={rule.enabled !== false} aria-label="Enable {rule.name || 'rule'}" onchange={(on) => rulesStore.setEnabled(rule.id, on)} />
          {/if}
          {#if open}
            <div class="rl-body" id="cr-body-{rule.id}" role="presentation" oninput={apply} transition:slide={accordion()}>
              <div>
                <label class="lab" for="cr-nm-{rule.id}">Name</label>
                <input id="cr-nm-{rule.id}" class="in" maxlength="40" bind:value={form.name} />
              </div>

              <section class="sec" aria-labelledby="cr-sm-{rule.id}">
                <h4 id="cr-sm-{rule.id}">
                  Match
                  <span class="ext">Extended id <Checkbox checked={form.ext} aria-label="Extended id" onchange={(v) => { form.ext = v; apply(); }} /></span>
                </h4>
                <div class="r3m">
                  <div>
                    <label class="lab" for="cr-id-{rule.id}">ID (hex)</label>
                    <input id="cr-id-{rule.id}" class="in mono" aria-invalid={badHex(form.id, fullMask(form.ext)) || undefined} bind:value={form.id} placeholder="545" />
                  </div>
                  <div>
                    <label class="lab" for="cr-mk-{rule.id}">Mask</label>
                    <input id="cr-mk-{rule.id}" class="in mono" aria-invalid={badHex(form.idMask, fullMask(form.ext), true) || undefined} bind:value={form.idMask} placeholder="exact" />
                  </div>
                  <div>
                    <label class="lab" for="cr-dl-{rule.id}">DLC</label>
                    <input id="cr-dl-{rule.id}" class="in mono" inputmode="numeric" aria-invalid={badDlc || undefined} bind:value={form.dlc} placeholder="any" />
                  </div>
                </div>
              </section>

              <section class="sec" aria-labelledby="cr-sb-{rule.id}">
                <h4 id="cr-sb-{rule.id}">
                  Byte matches <span class="tag">{form.bytes.length} of {MAX_BYTE_FIELDS}</span>
                  <button type="button" class="btn sm add" disabled={form.bytes.length >= MAX_BYTE_FIELDS} onclick={addByte}>Add</button>
                </h4>
                {#if form.bytes.length === 0}
                  <p class="hint">None. Every frame with this ID matches.</p>
                {:else}
                  <div class="rb rbh" aria-hidden="true"><span>Byte</span><span>Mask</span><span>Value</span><span></span></div>
                  {#each form.bytes as b, i (i)}
                    <div class="rb">
                      <input class="in mono" aria-label="Byte {i + 1} index" inputmode="numeric" aria-invalid={badInt(b.index, 0, 7) || undefined} bind:value={b.index} />
                      <input class="in mono" aria-label="Byte {i + 1} mask" aria-invalid={badHex(b.mask, 0xff) || undefined} bind:value={b.mask} />
                      <input class="in mono" aria-label="Byte {i + 1} value" aria-invalid={badHex(b.value, 0xff) || undefined} bind:value={b.value} />
                      <button type="button" class="btn icon danger sm" aria-label="Remove byte match {i + 1}" onclick={() => removeByte(i)}>
                        <Icon name="trash" size={18} />
                      </button>
                    </div>
                  {/each}
                {/if}
              </section>

              <section class="sec" aria-labelledby="cr-sv-{rule.id}">
                <h4 id="cr-sv-{rule.id}">Value</h4>
                <div class="rv">
                  <div>
                    <label class="lab" for="cr-hs-{rule.id}">Start</label>
                    <input id="cr-hs-{rule.id}" class="in mono" inputmode="numeric" aria-invalid={!helperResult.ok || undefined} bind:value={helper.start} oninput={writeFormula} />
                  </div>
                  <div>
                    <label class="lab" for="cr-hl-{rule.id}">Bytes</label>
                    <input id="cr-hl-{rule.id}" class="in mono" inputmode="numeric" aria-invalid={!helperResult.ok || undefined} bind:value={helper.length} oninput={writeFormula} />
                  </div>
                  <div>
                    <span class="lab">Order</span>
                    <div class="seg fw" role="radiogroup" aria-label="Byte order">
                      <button type="button" role="radio" aria-checked={helper.order === "be"} onclick={() => { helper.order = "be"; writeFormula(); }}>Big</button>
                      <button type="button" role="radio" aria-checked={helper.order === "le"} onclick={() => { helper.order = "le"; writeFormula(); }}>Little</button>
                    </div>
                  </div>
                </div>
                {#if !helperResult.ok}
                  <p class="hint bad" role="alert">{helperResult.reason}</p>
                {/if}
                <div>
                  <label class="lab" for="cr-fm-{rule.id}">Formula (bytes A to H)</label>
                  <input id="cr-fm-{rule.id}" class="in mono" bind:value={form.formula} placeholder="empty = plain filter" />
                </div>
                <div class="live">
                  {#if form.formula.trim() === ""}
                    <span>No formula. Matching frames are hidden.</span>
                  {:else}
                    <span>Last match<br /><span class="mono">{preview?.frame ?? "no frame yet"}</span></span>
                    <b class:t-crit={preview?.value === "invalid"} class:t-acc={preview?.value !== "invalid"}>{preview?.value ?? "..."}</b>
                  {/if}
                </div>
              </section>

              {#if problem}
                <p class="hint bad" role="alert">{problem}</p>
              {/if}

              <div class="acts">
                <button type="button" class="btn danger" onclick={removeOpen}>Delete rule</button>
                <button type="button" class="btn pri" onclick={close}>Done</button>
              </div>
            </div>
          {/if}
        </li>
      {/each}
    </ul>
  {/if}

  <p class="muted cr-foot">The hardware filter is in <a href="#settings">Settings</a>.</p>
</div>

<style>
  .cr {
    --ctl-h: 40px;
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 10px;
    min-width: 0;
  }
  .cr > * {
    min-width: 0;
  }
  .cr-hd {
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
  .cr-brow,
  .acts {
    display: flex;
    gap: 6px;
  }
  .cr-brow .btn,
  .acts .btn {
    flex: 1 1 0;
    min-width: 0;
    padding: 0 10px;
  }
  .cr .btn {
    min-height: var(--ctl-h);
  }
  .cr .btn.sm {
    min-height: 32px;
    padding: 0 12px;
  }
  .cr .btn.icon {
    width: var(--ctl-h);
    padding: 0;
  }
  .cr .btn.icon.sm {
    width: 34px;
    min-height: 34px;
  }
  .cr-file {
    display: none;
  }
  .cr-empty {
    padding: 18px 14px;
    border-radius: var(--r-track);
    background: var(--row);
    color: var(--muted);
    font-weight: 600;
  }
  .cr-foot {
    margin: 0;
    font-size: 0.88rem;
  }
  .hint {
    display: block;
    margin: 0;
    font-size: 0.82rem;
    color: var(--muted);
    line-height: 1.35;
  }
  .hint.bad {
    color: var(--crit);
    font-weight: 700;
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
  }
  .in[aria-invalid="true"] {
    outline: 2px solid var(--crit);
    outline-offset: 0;
  }

  /* Rule tiles: the ItemList tile with a live value and a switch. */
  .rl {
    display: grid;
    gap: 6px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .rl-i {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 2px 10px;
    padding: 2px 10px 2px 12px;
    border-radius: var(--r-track);
    background: var(--row);
  }
  .rl-i.off .rl-open,
  .rl-i.off .rl-v {
    opacity: 0.55;
  }
  .rl-open {
    flex: 1 1 0;
    min-width: 0;
    min-height: 52px;
    margin: 0;
    padding: 6px 0 0;
    border: 0;
    background: none;
    color: inherit;
    font: inherit;
    font-size: 0.95rem;
    font-weight: 800;
    text-align: left;
    cursor: pointer;
  }
  .rl-open:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .nm {
    display: block;
    overflow: hidden;
    line-height: 1.3;
    text-decoration: underline dotted;
    text-underline-offset: 4px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .rl-open[aria-expanded="true"] .nm {
    color: var(--accent-text);
    text-decoration-style: solid;
  }
  small {
    display: block;
    margin-top: 1px;
    padding-bottom: 6px;
    overflow: hidden;
    color: var(--muted);
    font-family: var(--mono);
    font-size: 0.8rem;
    font-weight: 600;
    line-height: 1.3;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .rl-v {
    flex: none;
    max-width: 84px;
    overflow: hidden;
    font-weight: 800;
    font-variant-numeric: tabular-nums;
    text-align: right;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .rl-v.muted {
    color: var(--muted);
    font-size: 0.88rem;
    font-weight: 700;
  }
  .rl-v.acc {
    color: var(--accent-text);
  }
  .rl-v.crit {
    color: var(--crit);
  }

  .rl-body {
    flex: 1 0 100%;
    display: grid;
    gap: 8px;
    padding: 2px 0 8px;
  }
  .sec {
    display: grid;
    gap: 6px;
    padding-top: 8px;
    border-top: 1px solid var(--hair);
  }
  h4 {
    display: flex;
    align-items: center;
    gap: 8px;
    min-height: 32px;
    margin: 0;
    font-size: 0.95rem;
    font-weight: 800;
    white-space: nowrap;
  }
  h4 .add,
  h4 .ext {
    margin-left: auto;
  }
  h4 .ext {
    display: flex;
    align-items: center;
    gap: 8px;
    color: var(--muted);
    font-size: 0.82rem;
    font-weight: 700;
  }
  .r3m {
    display: grid;
    grid-template-columns: 1.1fr 1.1fr 0.8fr;
    gap: 6px;
  }
  .rv {
    display: grid;
    grid-template-columns: 58px 58px minmax(0, 1fr);
    gap: 6px;
    align-items: end;
  }
  .rb {
    display: grid;
    grid-template-columns: 52px minmax(0, 1fr) minmax(0, 1fr) 34px;
    gap: 6px;
    align-items: center;
  }
  .rbh {
    margin-bottom: -2px;
    color: var(--muted);
    font-size: 0.74rem;
    font-weight: 800;
  }
  .seg.fw {
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
  }
  .seg.fw button {
    flex: 1;
    justify-content: center;
    padding: 0 8px;
  }
  .live {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    padding: 6px 10px;
    border-radius: var(--r-ctl);
    background: var(--card2);
  }
  .live b {
    font-size: 1.05rem;
    font-variant-numeric: tabular-nums;
  }
  .live span {
    color: var(--muted);
    font-size: 0.8rem;
    font-weight: 600;
  }
  .live .mono {
    font-size: 0.78rem;
  }
</style>
