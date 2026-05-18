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
    class="stepper-target subtract-target"
    class:is-disabled={value <= min}
    onclick={() => (value = Math.max(min, value - 1))}
    onkeydown={(e) => handleKeyDown(e, "sub")}
  >
    −
  </div>

  <div
    role="button"
    tabindex={value >= max ? -1 : 0}
    class="stepper-target add-target"
    class:is-disabled={value >= max}
    onclick={() => (value = Math.min(max, value + 1))}
    onkeydown={(e) => handleKeyDown(e, "add")}
  >
    +
  </div>
</div>

<style>
  .minimal-ui-stepper {
    display: flex;
    align-items: center;
    background-color: var(--pico-card-background-color);
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);
    border-radius: var(--pico-border-radius);
    height: 44px;
    overflow: hidden;
    background-color: rgb(
      from var(--pico-form-element-background-color) r g b / 0.6
    );
  }

  .value-pane {
    flex-grow: 1;
    padding-left: 16px;
    font-size: 1.15rem;
    font-weight: 500;
    color: var(--pico-color);
    font-family: var(--pico-font-family-monospace);
    text-align: left;
    user-select: none;
  }

  /* Structural UI Div Tap Blocks */
  .stepper-target {
    display: flex;
    align-items: center;
    justify-content: center;
    height: 100%;
    width: 44px;
    min-width: 44px;
    font-size: 1.25rem;
    font-weight: 500;
    cursor: pointer;
    user-select: none;

    /* Clean, soft semi-transparent contrast background */
    background-color: var(--pico-form-element-disabled-background-color);
    color: var(--pico-color-orange-500);

    transition:
      background-color 0.15s,
      color 0.15s,
      opacity 0.15s,
      transform 0.1s ease-out;
  }

  /* Segment boundaries divider line styling */
  .subtract-target {
    background: none;
    border-left: var(--pico-border-width) solid
      var(--pico-form-element-border-color);
    border-right: var(--pico-border-width) solid
      var(--pico-form-element-border-color);
    border-top: none;
    border-bottom: none;
    border-radius: 0px;
  }

  .add-target {
    background: none;
    border: none;
  }

  .stepper-target:focus {
    outline: none !important;
    box-shadow: none !important;
  }

  .subtract-target:focus,
  .subtract-target:active,
  .subtract-target:hover {
    border: var(--pico-border-width) solid var(--pico-form-element-color);
  }

  .subtract-target:active {
    background-color: rgba(255, 255, 255, 0.1) !important;
    transition: background-color 0s !important;
  }

  .add-target:focus,
  .add-target:active,
  .add-target:hover {
    border: var(--pico-border-width) solid var(--pico-form-element-color);
    border-top-left-radius: 0px !important;
    border-bottom-left-radius: 0px !important;
  }

  .add-target:active {
    background-color: rgba(255, 255, 255, 0.1) !important;
    transition: background-color 0s !important;
  }
</style>
