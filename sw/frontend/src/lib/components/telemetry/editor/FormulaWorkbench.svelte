<script lang="ts">
  import { tick } from "svelte";
  import Field from "./Field.svelte";
  import { evaluateFormula, parseTestBytes } from "$lib/editor/formulaTest";

  /**
   * Formula workbench (plan/mockups/pid-v7.html, Signal): Bytes A to D as outlined square tiles, Functions getBit and
   * getPID as outlined pills, the help box, the formula and test bytes full width, and the result row.
   */
  interface Props {
    id: string;
    formula: string;
    testBytes: string;
    unit: string;
    error?: string;
  }

  let { id, formula = $bindable(), testBytes = $bindable(), unit, error }: Props = $props();

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
  const defaultHelp = "A to D are bytes of the reply. getBit and getPID are functions. Hover or focus a tile to see what it does.";

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
    <div class="tg">
      <span>Bytes</span>
      {#each tokens.filter((t) => t.kind === "var") as t (t.key)}
        <button
          type="button"
          class="tok var"
          class:on={activeKey === t.key}
          aria-label={`Byte ${t.key}`}
          aria-describedby={`${id}-help`}
          onpointerenter={() => (hovered = t.key)}
          onpointerleave={() => (hovered = null)}
          onfocus={() => (focused = t.key)}
          onblur={() => (focused = null)}
          onclick={() => insert(t)}
        >{t.key}</button>
      {/each}
    </div>
    <div class="tg">
      <span>Functions</span>
      {#each tokens.filter((t) => t.kind === "fn") as t (t.key)}
        <button
          type="button"
          class="tok fn"
          class:on={activeKey === t.key}
          aria-label={`Function ${t.key}`}
          aria-describedby={`${id}-help`}
          onpointerenter={() => (hovered = t.key)}
          onpointerleave={() => (hovered = null)}
          onfocus={() => (focused = t.key)}
          onblur={() => (focused = null)}
          onclick={() => insert(t)}
        ><span class="fx">fx</span>{t.key}</button>
      {/each}
    </div>
  </div>
  <p class="help" id={`${id}-help`} aria-live="polite">{helpText}</p>

  <Field id={`${id}-f`} label="Formula" hint="Use A to D, getBit and getPID" error={error}>
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

  <Field id={`${id}-t`} label="Test bytes" hint="Hex bytes, the first is A..." error={testError}>
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

  <div class="res" class:bad={!!testError} aria-live="polite">
    <span class="rl">Result</span>
    <span class="rv">{shown || (testError ? "Not computed" : "Add test bytes")}</span>
  </div>
</div>

<style>
  .bench {
    display: grid;
    gap: 16px;
    min-width: 0;
  }
  .tokens {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: flex-start;
    gap: 8px 18px;
  }
  .tg {
    flex: none;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .tg > span {
    margin-right: 2px;
    font-size: 0.8125rem;
    font-weight: 700;
    color: var(--muted);
  }
  /* Byte tiles: outlined squares. Function pills: outlined in the accent with an fx circle. */
  .tok {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    min-height: 44px;
    border: 0;
    font-family: var(--mono);
    font-weight: 800;
    cursor: pointer;
  }
  .tok.var {
    width: 48px;
    height: 48px;
    border-radius: 10px;
    background: #dde4f6;
    color: #27345c;
    box-shadow: inset 0 0 0 2px #6d7fb8;
    font-size: 1.1rem;
  }
  .tok.fn {
    height: 48px;
    padding: 0 16px 0 12px;
    border-radius: 24px;
    background: transparent;
    color: var(--accent-text);
    box-shadow: inset 0 0 0 2px var(--accent);
    font-size: 0.9375rem;
  }
  .tok.fn .fx {
    display: grid;
    place-items: center;
    width: 24px;
    height: 24px;
    border-radius: 50%;
    background: var(--accent);
    color: var(--on-accent);
    font-size: 0.8rem;
    font-style: italic;
    font-weight: 700;
  }
  .tok.on.var {
    background: #27345c;
    color: #dde4f6;
  }
  .tok.on.fn {
    background: color-mix(in oklch, var(--accent) 17%, transparent);
  }
  @media (prefers-color-scheme: dark) {
    :global(html:not([data-theme='light'])) .tok.var {
      background: #2d3550;
      color: #dbe2fa;
      box-shadow: inset 0 0 0 2px #7f90cc;
    }
  }
  :global(html[data-theme='dark']) .tok.var {
    background: #2d3550;
    color: #dbe2fa;
    box-shadow: inset 0 0 0 2px #7f90cc;
  }
  .tok:focus-visible {
    outline: none;
    background: var(--hov);
  }
  .help {
    min-height: 3.6em;
    margin: 0;
    padding: 10px 14px;
    border-radius: 12px;
    background: var(--ctl);
    font-size: 0.875rem;
    line-height: 1.45;
  }
  .in {
    box-sizing: border-box;
    width: 100%;
    min-height: var(--ctl-h);
    padding: 0 14px;
    border: 0;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
    font: inherit;
  }
  .in::placeholder {
    color: var(--ctl-sub);
    opacity: 1;
  }
  .in[aria-invalid='true'] {
    box-shadow: inset 0 0 0 2px var(--crit);
  }
  .in:focus-visible {
    outline: none;
    background: var(--ctl-h2);
  }
  .mono {
    font-family: var(--mono);
  }
  /* Result: a tinted row. The value is monospace in the accent text; an error turns the row red. */
  .res {
    display: flex;
    align-items: baseline;
    gap: 12px;
    min-height: 56px;
    padding: 10px 14px;
    border-radius: 12px;
    background: color-mix(in oklch, var(--accent) 17%, transparent);
  }
  .res.bad {
    background: var(--crit-f);
  }
  .rl {
    font-size: 0.875rem;
    font-weight: 700;
  }
  .rv {
    font-family: var(--mono);
    font-size: 1.4rem;
    font-weight: 800;
    color: var(--accent-text);
  }
  .res.bad .rv {
    color: var(--crit);
    font-size: 1rem;
  }
</style>
