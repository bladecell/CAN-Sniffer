<script module lang="ts">
  let seq = 0;
</script>

<script lang="ts" generics="T">
  import { tick } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { defaultIndex, navigate, type SelectMenuOption } from "$lib/selectMenu";

  const MENU_MAX_HEIGHT = 250;

  interface Props {
    options: SelectMenuOption<T>[];
    value?: T;
    placeholder?: string;
    disabled?: boolean;
    id?: string;
    ariaLabel?: string;
    ariaDescribedby?: string;
    ariaInvalid?: boolean | "true" | "false";
    onChange?: (value: T) => void;
  }

  let {
    options,
    value = $bindable(),
    placeholder = "Select...",
    disabled = false,
    id,
    ariaLabel,
    ariaDescribedby,
    ariaInvalid,
    onChange,
  }: Props = $props();

  const listId = `select-menu-${++seq}`;

  let open = $state(false);
  let activeIndex = $state(-1);
  let menuStyle = $state("");
  let rootEl = $state<HTMLDivElement>();
  let listEl = $state<HTMLDivElement>();
  let triggerEl = $state<HTMLButtonElement>();
  let optionEls: (HTMLButtonElement | undefined)[] = [];

  let selectedIndex = $derived(options.findIndex((o) => o.value === value));
  let selected = $derived(selectedIndex >= 0 ? options[selectedIndex] : undefined);

  /** Places the list against the trigger. Fixed to the viewport, so a scrolling dialog body cannot clip it. */
  function place() {
    if (!triggerEl) return;
    const rect = triggerEl.getBoundingClientRect();
    const spaceBelow = window.innerHeight - rect.bottom;
    const placement = spaceBelow < MENU_MAX_HEIGHT && rect.top > spaceBelow ? "up" : "down";
    const space = placement === "up" ? rect.top : spaceBelow;
    const menuMaxHeight = Math.max(80, Math.min(MENU_MAX_HEIGHT, space - 8));
    const edge =
      placement === "up"
        ? `bottom:${window.innerHeight - rect.top + 4}px`
        : `top:${rect.bottom + 4}px`;
    menuStyle = `left:${rect.left}px;width:${rect.width}px;max-height:${menuMaxHeight}px;${edge}`;
  }

  async function openMenu() {
    if (disabled || !triggerEl || options.length === 0) return;
    place();
    activeIndex = defaultIndex(options, selectedIndex);
    open = true;
    await tick();
    optionEls[activeIndex]?.focus();
  }

  function closeMenu(restoreFocus: boolean) {
    if (!open) return;
    open = false;
    if (restoreFocus) triggerEl?.focus();
  }

  function choose(index: number) {
    const option = options[index];
    if (!option || option.disabled) return;
    value = option.value;
    onChange?.(option.value);
    closeMenu(true);
  }

  function toggle() {
    if (open) closeMenu(true);
    else openMenu();
  }

  function onTriggerKeydown(e: KeyboardEvent) {
    if (e.key === "Escape" && open) {
      // Closes only the list: preventDefault stops the dialog's native Esc (cancel) from firing.
      e.preventDefault();
      e.stopPropagation();
      closeMenu(true);
      return;
    }
    if (e.key !== "ArrowDown" && e.key !== "ArrowUp") return;
    e.preventDefault();
    if (!open) openMenu();
  }

  function onListKeydown(e: KeyboardEvent) {
    if (e.key === "Escape") {
      e.preventDefault();
      closeMenu(true);
      return;
    }
    const next = navigate(options, activeIndex, e.key);
    if (next === undefined) return;
    e.preventDefault();
    activeIndex = next;
    optionEls[next]?.focus();
  }

  function onFocusOut(e: FocusEvent) {
    if (!rootEl?.contains(e.relatedTarget as Node | null)) closeMenu(false);
  }

  function onWindowPointerDown(e: PointerEvent) {
    if (open && !rootEl?.contains(e.target as Node)) closeMenu(false);
  }

  // Top layer: a fixed box inside the blurred dialog would be positioned against the dialog, not the viewport.
  $effect(() => {
    if (listEl?.showPopover && !listEl.matches(":popover-open")) listEl.showPopover();
  });

  // Follow the trigger when the page or a scroll container moves; the list's own scroll is ignored.
  $effect(() => {
    if (!open) return;
    const onMove = (e: Event) => {
      if (!(e.target instanceof Node && listEl?.contains(e.target))) place();
    };
    window.addEventListener("scroll", onMove, { capture: true, passive: true });
    window.addEventListener("resize", onMove);
    return () => {
      window.removeEventListener("scroll", onMove, { capture: true });
      window.removeEventListener("resize", onMove);
    };
  });
</script>

<svelte:window onpointerdown={onWindowPointerDown} />

<div class="select-menu" bind:this={rootEl} onfocusout={onFocusOut}>
  <button
    bind:this={triggerEl}
    type="button"
    {id}
    class="select-menu-trigger"
    role="combobox"
    {disabled}
    aria-haspopup="listbox"
    aria-expanded={open}
    aria-controls={open ? listId : undefined}
    aria-label={ariaLabel}
    aria-describedby={ariaDescribedby}
    aria-invalid={ariaInvalid}
    onclick={toggle}
    onkeydown={onTriggerKeydown}
  >
    {#if selected}
      <span class="select-menu-value">
        {#if selected.icon}<Icon name={selected.icon} size={18} />{/if}
        <span>{selected.label}</span>
      </span>
    {:else}
      <span class="select-menu-placeholder">{placeholder}</span>
    {/if}
    <Icon name="chevron-down" size={14} />
  </button>

  {#if open}
    <div
      bind:this={listEl}
      id={listId}
      popover="manual"
      class="select-menu-list blur-background"
      role="listbox"
      tabindex="-1"
      style={menuStyle}
      onkeydown={onListKeydown}
    >
      {#each options as option, i (i)}
        <button
          bind:this={optionEls[i]}
          type="button"
          role="option"
          tabindex="-1"
          class="select-menu-option"
          class:active={i === selectedIndex}
          aria-selected={i === selectedIndex}
          disabled={option.disabled}
          onclick={() => choose(i)}
        >
          {#if option.icon}<Icon name={option.icon} size={18} />{/if}
          <span>{option.label}</span>
        </button>
      {/each}
    </div>
  {/if}
</div>

<style>
  .select-menu {
    position: relative;
    width: 100%;
  }

  .select-menu-trigger {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    box-sizing: border-box;
    width: 100%;
    min-height: 48px;
    margin: 0;
    padding: var(--pico-form-element-spacing-vertical) var(--pico-form-element-spacing-horizontal);
    background-color: var(--pico-form-element-background-color);
    border: var(--pico-border-width) solid var(--pico-form-element-border-color);
    border-radius: var(--pico-border-radius);
    color: var(--pico-form-element-color);
    cursor: pointer;
    text-align: left;
    transition: background-color var(--pico-transition), border-color var(--pico-transition), color var(--pico-transition), box-shadow var(--pico-transition);
  }

  .select-menu-trigger:focus {
    box-shadow: var(--pico-form-element-focus-color) 0 0 0 0.125rem;
    border-color: var(--pico-form-element-active-border-color);
  }

  .select-menu-trigger:disabled {
    cursor: not-allowed;
    opacity: 0.5;
  }

  .select-menu-trigger :global(svg) {
    flex-shrink: 0;
  }

  .select-menu-value {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }

  .select-menu-value span,
  .select-menu-placeholder {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .select-menu-placeholder {
    color: var(--pico-muted-color);
  }

  .select-menu-list {
    position: fixed;
    inset: auto;
    margin: 0;
    box-sizing: border-box;
    overflow-y: auto;
    z-index: 100;
    padding: 4px 0;
    border: 1px solid var(--pico-form-element-border-color);
    border-radius: var(--pico-border-radius);
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
    animation: select-menu-in 0.15s ease-out;
  }

  .select-menu-option {
    display: flex;
    align-items: center;
    gap: 8px;
    box-sizing: border-box;
    width: 100%;
    margin: 0;
    padding: 8px 12px;
    background: none;
    border: none;
    border-radius: 0;
    color: var(--text);
    cursor: pointer;
    text-align: left;
    transition: background-color 0.2s;
  }

  .select-menu-option:not(:disabled):hover,
  .select-menu-option:not(:disabled):focus-visible {
    background-color: var(--overlay-1);
  }

  .select-menu-option:focus-visible {
    outline-offset: -2px;
  }

  .select-menu-option.active {
    background-color: rgba(from var(--pico-primary) r g b / 0.1);
    color: var(--pico-primary);
  }

  .select-menu-option:disabled {
    cursor: not-allowed;
    opacity: 0.45;
  }

  @keyframes select-menu-in {
    from {
      opacity: 0;
    }
  }

  @media (pointer: coarse) {
    .select-menu-trigger {
      min-height: 48px;
    }

    .select-menu-option {
      min-height: 44px;
    }
  }

  @media (prefers-reduced-motion: reduce) {
    .select-menu-list {
      animation: none;
    }

    .select-menu-option {
      transition: none;
    }
  }
</style>
