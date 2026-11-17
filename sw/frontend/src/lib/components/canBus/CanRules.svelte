<script lang="ts">
  import { canBus } from "$lib/canBus/canBusStore.svelte";
  import { rulesStore } from "$lib/canBus/canRulesStore.svelte";
  import { MAX_BYTE_FIELDS, emptyForm, formFromRule, matchSummary, ruleFromForm, type RuleForm } from "$lib/canBus/canRuleForm";
  import { compileFormula, FRAME_LETTERS } from "$lib/editor/formulaTest";
  import { hexBytes } from "$lib/canBus/canIds";
  import { helperFormula } from "$lib/canBus/canRuleHelper";
  import { RULE_CAP, type Rule } from "$lib/canBus/canRules";
  import { parseRules, RULES_MAX_BYTES } from "$lib/canBus/canRulesIo";

  let form = $state<RuleForm>(emptyForm());
  let editingId = $state<string | null>(null);
  let open = $state(false);
  let error = $state<string | null>(null);
  let helper = $state({ start: "0", length: "1", order: "be" as "be" | "le" });
  let helperError = $state<string | null>(null);
  let fileNote = $state<{ ok: boolean; text: string } | null>(null);

  const full = $derived(rulesStore.list.length >= RULE_CAP && editingId === null);

  // Preview: the draft formula on the newest frame its match accepts. Re-runs on every snapshot, so it follows the bus.
  const preview = $derived.by(() => {
    canBus.snapshot;
    const built = ruleFromForm(form);
    if (!built.ok || built.value.formula === "") return null;
    const frame = canBus.lastMatchFor(built.value.match);
    if (!frame) return { text: "No matching frame yet" };
    const compiled = compileFormula(built.value.formula, FRAME_LETTERS);
    const r = compiled.ok ? compiled.run(frame.data) : null;
    const value = r?.ok ? String(Number(r.value.toFixed(2))) : "invalid";
    return { text: `${value} from ${hexBytes(frame.data)}` };
  });

  function startNew() {
    form = emptyForm();
    editingId = null;
    error = null;
    open = true;
  }

  function startEdit(rule: Rule) {
    form = formFromRule(rule);
    editingId = rule.id;
    error = null;
    open = true;
  }

  function close() {
    open = false;
    editingId = null;
    error = null;
  }

  function submit(e: SubmitEvent) {
    e.preventDefault();
    const built = ruleFromForm(form);
    if (!built.ok) {
      error = built.reason;
      return;
    }
    const result = editingId === null ? rulesStore.add(built.value) : rulesStore.update(editingId, built.value);
    if (!result.ok) {
      error = result.reason;
      return;
    }
    close();
  }

  function removeByte(i: number) {
    form.bytes = form.bytes.filter((_, j) => j !== i);
  }

  function addByte() {
    const used = new Set(form.bytes.map((b) => Number(b.index)));
    const index = [0, 1, 2, 3, 4, 5, 6, 7].find((n) => !used.has(n)) ?? 0;
    form.bytes = [...form.bytes, { index: String(index), value: "00", mask: "FF" }];
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
    rulesStore.replaceAll(parsed.rules);
    fileNote = { ok: true, text: `${parsed.rules.length} rules imported` };
  }

  /** Writes the formula for the chosen bytes. The formula field stays editable. */
  function writeHelperFormula() {
    const spec = { start: Number(helper.start), length: Number(helper.length), order: helper.order };
    const written = helperFormula(spec);
    helperError = written.ok ? null : written.reason;
    if (written.ok) form.formula = written.formula;
  }

  function ruleValue(rule: Rule): string {
    if (rule.formula === "") return "hides";
    return canBus.snapshot.ruleValues.get(rule.id) ?? "no frame yet";
  }
</script>

<div class="cr">
  <div class="cr-head">
    <div>
      <h2>Rules</h2>
      <p class="muted">A rule matches frames and can parse a value. A match with no formula hides those frames. Every matching rule applies.</p>
    </div>
    <span class="muted">{rulesStore.list.length} of {RULE_CAP}</span>
  </div>

  <div class="cr-actions">
    <button class="btn sm" disabled={rulesStore.list.length === 0} aria-disabled={rulesStore.list.length === 0} onclick={exportFile}>Export</button>
    <label class="btn sm cr-import">Import<input type="file" accept="application/json,.json" onchange={importFile} /></label>
  </div>
  {#if fileNote}
    <p class:t-crit={!fileNote.ok} class:t-ok={fileNote.ok} role="status">{fileNote.text}</p>
  {/if}

  {#if rulesStore.list.length === 0 && !open}
    <p class="muted">No rules yet.</p>
  {/if}

  <ul class="cr-list">
    {#each rulesStore.list as rule (rule.id)}
      <li>
        <span>
          <b>{rule.name || "Unnamed"}</b>
          <small class="mono">{matchSummary(rule.match)}{rule.formula ? ` · ${rule.formula}` : ""}</small>
        </span>
        <em class:t-acc={rule.formula !== "" && ruleValue(rule) !== "invalid"} class:t-crit={ruleValue(rule) === "invalid"}>{ruleValue(rule)}</em>
        <span class="cr-actions">
          <button class="btn sm" onclick={() => startEdit(rule)}>Edit</button>
          <button class="btn sm" onclick={() => rulesStore.remove(rule.id)}>Delete</button>
        </span>
      </li>
    {/each}
  </ul>

  {#if open}
    <form class="cr-form" onsubmit={submit} novalidate>
      <h3>{editingId === null ? "New rule" : "Edit rule"}</h3>
      <label class="lab" for="cr-name">Name</label>
      <input id="cr-name" class="in" maxlength="40" bind:value={form.name} />

      <div class="cr-row">
        <div>
          <label class="lab" for="cr-id">ID (hex)</label>
          <input id="cr-id" class="in mono" bind:value={form.id} placeholder="545" />
        </div>
        <div>
          <label class="lab" for="cr-mask">Mask (hex)</label>
          <input id="cr-mask" class="in mono" bind:value={form.idMask} placeholder="exact" />
        </div>
      </div>

      <label class="cr-check"><input type="checkbox" bind:checked={form.ext} /> Extended id</label>

      <label class="lab" for="cr-dlc">DLC</label>
      <input id="cr-dlc" class="in mono" inputmode="numeric" bind:value={form.dlc} placeholder="any" />

      <div class="cr-bytes">
        {#each form.bytes as b, i (i)}
          <div class="cr-row">
            <div>
              <label class="lab" for="cr-bi-{i}">Byte</label>
              <input id="cr-bi-{i}" class="in mono" inputmode="numeric" bind:value={b.index} />
            </div>
            <div>
              <label class="lab" for="cr-bv-{i}">Value</label>
              <input id="cr-bv-{i}" class="in mono" bind:value={b.value} />
            </div>
            <div>
              <label class="lab" for="cr-bm-{i}">Mask</label>
              <input id="cr-bm-{i}" class="in mono" bind:value={b.mask} />
            </div>
            <button type="button" class="btn sm" aria-label="Remove byte match" onclick={() => removeByte(i)}>Remove</button>
          </div>
        {/each}
        <button type="button" class="btn sm" disabled={form.bytes.length >= MAX_BYTE_FIELDS} onclick={addByte}>Add byte match</button>
      </div>

      <div class="cr-row">
        <div>
          <label class="lab" for="cr-hs">Start byte</label>
          <input id="cr-hs" class="in mono" inputmode="numeric" bind:value={helper.start} />
        </div>
        <div>
          <label class="lab" for="cr-hl">Length (bytes)</label>
          <input id="cr-hl" class="in mono" inputmode="numeric" bind:value={helper.length} />
        </div>
      </div>
      <div class="seg" role="radiogroup" aria-label="Byte order">
        <button type="button" role="radio" aria-checked={helper.order === "be"} onclick={() => (helper.order = "be")}>Big endian</button>
        <button type="button" role="radio" aria-checked={helper.order === "le"} onclick={() => (helper.order = "le")}>Little endian</button>
      </div>
      <button type="button" class="btn sm" onclick={writeHelperFormula}>Write formula from bytes</button>
      {#if helperError}
        <p class="t-crit" role="alert">{helperError}</p>
      {/if}

      <label class="lab" for="cr-formula">Formula (bytes A to H)</label>
      <input id="cr-formula" class="in mono" bind:value={form.formula} placeholder="empty = plain filter" />
      {#if preview}
        <p class="muted">Preview on the last matched frame: <b class="t-acc mono">{preview.text}</b></p>
      {/if}

      {#if error}
        <p class="t-crit" role="alert">{error}</p>
      {/if}

      <div class="cr-actions">
        <button type="submit" class="btn pri" aria-disabled={full}>{editingId === null ? "Add rule" : "Save rule"}</button>
        <button type="button" class="btn" onclick={close}>Cancel</button>
      </div>
    </form>
  {:else}
    <button class="btn" aria-disabled={full} onclick={() => !full && startNew()}>New rule</button>
  {/if}
</div>

<style>
  .cr {
    display: grid;
    gap: 12px;
    min-width: 0;
  }
  .cr-head {
    display: flex;
    justify-content: space-between;
    gap: 10px;
  }
  .cr-head h2 {
    margin: 0;
  }
  .cr-head p {
    margin: 2px 0 0;
    font-size: 0.88rem;
  }
  .cr-list {
    display: grid;
    gap: 8px;
    margin: 0;
    padding: 0;
    list-style: none;
  }
  .cr-list li {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 6px 10px;
    padding: 10px 12px;
    border-radius: var(--r-ctl);
    background: var(--row);
  }
  .cr-list small {
    display: block;
    color: var(--muted);
    font-weight: 600;
    overflow-wrap: anywhere;
  }
  .cr-list em {
    font-style: normal;
    font-weight: 700;
    text-align: right;
    font-variant-numeric: tabular-nums;
  }
  .cr-list .cr-actions {
    grid-column: 1 / -1;
    justify-content: flex-end;
  }
  .cr-actions {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
  }
  .cr-form {
    display: grid;
    gap: 8px;
    min-width: 0;
  }
  .cr-form h3 {
    margin: 4px 0 0;
  }
  .cr-row {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 8px;
    align-items: end;
  }
  .cr-bytes {
    display: grid;
    gap: 8px;
  }
  .cr-import {
    position: relative;
    overflow: hidden;
    cursor: pointer;
  }
  .cr-import input {
    position: absolute;
    inset: 0;
    opacity: 0;
    cursor: pointer;
  }
  .cr-check {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 600;
  }
  .lab {
    display: block;
    font-size: 0.85rem;
    font-weight: 700;
  }
  .cr-form .in {
    width: 100%;
    min-width: 0;
  }
</style>
