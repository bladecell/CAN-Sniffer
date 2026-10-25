<script lang="ts">
  interface Props {
    value: number;
    min?: number;
    max?: number;
  }

  let { value = $bindable(), min = 1, max = 60 }: Props = $props();

  // Handle keyboard interaction safely since we shifted to structural divs
  function handleKeyDown(e: KeyboardEvent, type: "add" | "sub") {
    if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      if (type === "sub") {
        value = Math.max(min, value - 1);
      } else {
        value = Math.min(max, value + 1);
      }
    }
  }
</script>

<div class="minimal-ui-stepper">
  <div class="value-pane">{value}</div>

  <div
    role="button"
    tabindex={value <= min ? -1 : 0}
    class="stepper-target"
    class:is-disabled={value <= min}
    onclick={() => (value = Math.max(min, value - 1))}
    onkeydown={(e) => handleKeyDown(e, "sub")}
  >
    −
  </div>

  <div
    role="button"
    tabindex={value >= max ? -1 : 0}
    class="stepper-target"
    class:is-disabled={value >= max}
    onclick={() => (value = Math.min(max, value + 1))}
    onkeydown={(e) => handleKeyDown(e, "add")}
  >
    +
  </div>
</div>

<style>
  /* One control fill, no borders (DESIGN.md Controls). */
  .minimal-ui-stepper {
    display: flex;
    align-items: center;
    height: var(--ctl-h);
    overflow: hidden;
    border-radius: var(--r-ctl);
    background: var(--ctl);
    color: var(--ctl-text);
  }

  .value-pane {
    flex-grow: 1;
    padding-left: 16px;
    font-size: 1.15rem;
    font-weight: 500;
    color: var(--ctl-text);
    font-family: var(--pico-font-family-monospace);
    font-variant-numeric: tabular-nums;
    text-align: left;
    user-select: none;
  }

  .stepper-target {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100%;
    width: 44px;
    min-width: 44px;
    font-size: 1.25rem;
    font-weight: 500;
    /* Pico styles [role="button"] as a primary button, so the fill, border and radius are reset explicitly. */
    background: transparent;
    border: 0;
    border-radius: 0;
    box-shadow: none;
    color: var(--ctl-text);
    cursor: pointer;
    user-select: none;
    transition: background-color 0.15s, opacity 0.15s;
  }

  .stepper-target:hover {
    background: var(--ctl-h2);
  }

  .stepper-target:active {
    transition: background-color 0s;
  }

  .stepper-target.is-disabled {
    opacity: 0.55;
    cursor: not-allowed;
  }

  .stepper-target:focus:not(:focus-visible) {
    outline: none !important;
    box-shadow: none !important;
  }
</style>
