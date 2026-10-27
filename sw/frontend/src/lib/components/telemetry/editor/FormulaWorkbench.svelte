<script lang="ts">
  import { tick } from "svelte";
  import Field from "./Field.svelte";
  import { evaluateFormula, parseTestBytes } from "$lib/editor/formulaTest";

  /**
   * Formula workbench: variable tokens A to D (square tiles), function tokens getBit and getPID (wide tiles),
   * a live test with hex bytes, and one explanatory line for the token that is hovered, focused or last inserted.
   */
  interface Props {
    id: string;
    formula: string;
    testBytes: string;
    unit: string;
    error?: string;
    isEdit: boolean;
  }

  let { id, formula = $bindable(), testBytes = $bindable(), unit, error, isEdit }: Props = $props();

  type Token = { key: string; insert: string; kind: "var" | "fn"; help: string };
  const tokens: Token[] = [
    { key: "A", insert: "A", kind: "var", help: "A: first response data byte. Bytes past the end of the frame read as 0." },
    { key: "B", insert: "B", kind: "var", help: "B: second response data byte. Bytes past the end of the frame read as 0." },
    { key: "C", insert: "C", kind: "var", help: "C: third response data byte. Bytes past the end of the frame read as 0." },
    { key: "D", insert: "D", kind: "var", help: "D: fourth response data byte. Bytes past the end of the frame read as 0." },
    {
      key: "getBit",
      insert: "getBit(",
      kind: "fn",
      help: "getBit(value, bit): bit number bit of value, 0 is the least significant. Returns 0 or 1, and 0 above bit 31. Example: getBit(A, 3)",
    },
    {
      key: "getPID",
      insert: "getPID(",
      kind: "fn",
      help: "getPID(pid): the last stored value of another PID, looked up by its pid, not its CAN ID. Undefined PID gives NaN. Example: getPID(0x0C) / 4",
    },
  ];
  const defaultHelp = "Tap A to D for a response byte, or getBit and getPID for logic. Hover or focus a token to read what it does.";

  let input: HTMLInputElement | undefined = $state();
  let hovered = $state<string | null>(null);
  let focused = $state<string | null>(null);
  let lastInserted = $state<string | null>(null);
  const activeKey = $derived(hovered ?? focused ?? lastInserted);
  const helpText = $derived(tokens.find((t) => t.key === activeKey)?.help ?? defaultHelp);

  async function insert(t: Token) {
    const at = input?.selectionStart ?? formula.length;
    const to = input?.selectionEnd ?? at;
    formula = formula.slice(0, at) + t.insert + formula.slice(to);
    lastInserted = t.key;
    await tick();
    input?.focus();
    input?.setSelectionRange(at + t.insert.length, at + t.insert.length);
  }

  const parsed = $derived(parseTestBytes(testBytes));
  const result = $derived.by(() => {
    if (!parsed.ok) return null;
    return evaluateFormula(formula, parsed.bytes);
  });
  const testError = $derived(
    !testBytes.trim() ? "" : !parsed.ok ? "Use hex pairs, up to 8 bytes: 0A 3C" : result && !result.ok ? result.reason : "",
  );
  const shown = $derived(result && result.ok ? `${Number(result.value.toFixed(3))}${unit ? ` ${unit}` : ""}` : "");
</script>

<div class="bench">
  <div class="tokens" role="group" aria-label="Insert into formula">
    {#each tokens as t (t.key)}
      <button
        type="button"
        class="tok {t.kind}"
        aria-label={t.kind === "var" ? `Byte ${t.key}` : `Function ${t.key}`}
        aria-describedby={`${id}-help`}
        onpointerenter={() => (hovered = t.key)}
        onpointerleave={() => (hovered = null)}
        onfocus={() => (focused = t.key)}
        onblur={() => (focused = null)}
        onclick={() => insert(t)}
      >
        {#if t.kind === "var"}{t.key}{:else}<span class="fx">fx</span>{t.key}{/if}
      </button>
    {/each}
  </div>
  <p class="help" id={`${id}-help`} aria-live="polite">{helpText}</p>

  <div class="pair">
    <Field id={`${id}-f`} label="Formula" hint="Uses A to D, getBit and getPID" error={error}>
      {#snippet children(labelId, msgId)}
        <input
          bind:this={input}
          id={`${id}-f`}
          class="in mono"
          type="text"
          autocomplete="off"
          spellcheck="false"
          aria-labelledby={labelId}
          aria-describedby={msgId}
          aria-invalid={error ? "true" : undefined}
          bind:value={formula}
        />
      {/snippet}
    </Field>

    <Field
      id={`${id}-t`}
      label="Test bytes"
      hint={isEdit && !testBytes.trim() ? "Raw bytes are not sent to the browser yet, type them here" : "Hex, first byte is A"}
      error={testError}
    >
      {#snippet children(labelId, msgId)}
        <input
          id={`${id}-t`}
          class="in mono"
          type="text"
          placeholder="0A 3C"
          autocomplete="off"
          spellcheck="false"
          autocapitalize="off"
          aria-labelledby={labelId}
          aria-describedby={msgId}
          aria-invalid={testError ? "true" : undefined}
          bind:value={testBytes}
        />
      {/snippet}
    </Field>
  </div>

  <div class="res" class:bad={!!testError} aria-live="polite">
    <span class="rl">Result</span>
    <span class="rv">{shown || (testError ? "Not computed" : "Enter test bytes")}</span>
  </div>
</div>

<style>
  .bench {
    display: grid;
    gap: 6px;
    min-width: 0;
  }
  .tokens {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
  }
  .tok {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 4px;
    min-width: 44px;
    min-height: 44px;
    padding: 0;
    border: 0;
    font: inherit;
    cursor: pointer;
  }
  /* Variables: square tiles in the accent tint, monospace letter. */
  .tok.var {
    width: 44px;
    border-radius: 10px;
    background: color-mix(in srgb, var(--accent) 22%, transparent);
    color: var(--accent-text);
    font-family: var(--mono);
    font-size: 1.05rem;
    font-weight: 800;
  }
  /* Functions: wide tiles with a dashed outline and an fx mark, neutral fill. */
  .tok.fn {
    padding: 0 14px;
    border-radius: 6px;
    background: var(--ctl);
    color: var(--ctl-text);
    box-shadow: inset 0 0 0 1.5px var(--ctl-sub);
    font-family: var(--mono);
    font-size: 0.95rem;
    font-weight: 700;
  }
  .fx {
    font-family: var(--font);
    font-size: 0.72rem;
    font-style: italic;
    color: var(--ctl-sub);
  }
  .tok:hover {
    filter: brightness(1.08);
  }
  .tok:focus-visible {
    outline: 3px solid var(--focus);
    outline-offset: 2px;
  }
  .help {
    min-height: 2.6em;
    margin: 0;
    color: var(--muted);
    font-size: 0.84rem;
    line-height: 1.3;
    overflow-wrap: anywhere;
  }
  .pair {
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: 0 12px;
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 12px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
  }
  .mono {
    font-family: var(--mono);
  }
  .res {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 10px;
    min-height: var(--ctl-h);
    padding: 0 14px;
    border-radius: var(--r-ctl);
    background: var(--card2);
    font-weight: 700;
    font-variant-numeric: tabular-nums;
  }
  .res.bad {
    color: var(--crit);
  }
  .rv {
    font-size: 1.15rem;
    font-weight: 800;
    overflow-wrap: anywhere;
    text-align: right;
  }
  @media (min-width: 560px) {
    .pair {
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
    }
  }
</style>
