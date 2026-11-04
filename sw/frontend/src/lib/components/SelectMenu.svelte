<script module lang="ts">
  let seq = 0;
</script>

<script lang="ts" generics="T">
  import { tick } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { closeOnOutsideScroll, fieldBox } from "$lib/popoverAnchor";
  import { portal } from "$lib/layers";
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
    /** "kebab": an icon-only three-dots button that opens the list as a menu of actions. Options then run through onChange. */
    trigger?: "field" | "kebab";
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
    trigger = "field",
  }: Props = $props();

  const kebab = $derived(trigger === "kebab");

  const listId = `select-menu-${++seq}`;

  let open = $state(false);
  let activeIndex = $state(-1);
  let menuStyle = $state("");
  let rootEl = $state<HTMLDivElement>();
  let listEl = $state<HTMLDivElement>();
  let triggerEl = $state<HTMLButtonElement>();
  let optionEls = $state<(HTMLButtonElement | undefined)[]>([]);

  let selectedIndex = $derived(options.findIndex((o) => o.value === value));
  let selected = $derived(selectedIndex >= 0 ? options[selectedIndex] : undefined);

  /** Places the list against the trigger. Fixed to the viewport, so a scrolling dialog body cannot clip it. */
  function place() {
    if (!triggerEl) return;
    const rect = triggerEl.getBoundingClientRect();
    // A kebab menu is narrower than its button needs: it opens to the left of the button's right edge.
    const width = kebab ? Math.max(200, rect.width) : rect.width;
    const left = kebab ? Math.max(8, rect.right - width) : rect.left;
    // Up: above the field's label, so the list never covers it.
    const field = fieldBox(triggerEl);
    const spaceBelow = window.innerHeight - rect.bottom;
    const placement = spaceBelow < MENU_MAX_HEIGHT && field.top > spaceBelow ? "up" : "down";
    const space = placement === "up" ? field.top : spaceBelow;
    const menuMaxHeight = Math.max(80, Math.min(MENU_MAX_HEIGHT, space - 8));
    const edge =
      placement === "up"
        ? `bottom:${window.innerHeight - field.top + 4}px`
        : `top:${rect.bottom + 4}px`;
    menuStyle = `left:${left}px;width:${width}px;max-height:${menuMaxHeight}px;${edge}`;
  }

  async function openMenu() {
    if (disabled || !triggerEl || options.length === 0) return;
    place();
    activeIndex = defaultIndex(options, selectedIndex);
    open = true;
    await tick();
    optionEls[activeIndex]?.focus({ preventScroll: true });
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

  // The list is portalled out of rootEl (lib/layers.ts), so focus and pointers in it count as inside the menu.
  const inMenu = (node: EventTarget | null) => !!node && (!!rootEl?.contains(node as Node) || !!listEl?.contains(node as Node));

  function onFocusOut(e: FocusEvent) {
    if (!inMenu(e.relatedTarget)) closeMenu(false);
  }

  function onWindowPointerDown(e: PointerEvent) {
    if (open && !inMenu(e.target)) closeMenu(false);
  }

  // Top layer: showPopover puts the fixed list above the page, not against the dialog it sits in.
  $effect(() => {
    if (listEl?.showPopover && !listEl.matches(":popover-open")) listEl.showPopover();
  });

  // Scrolling the page or the sheet closes the list: it never drifts from its field.
  $effect(() => {
    if (!open) return;
    return closeOnOutsideScroll(listEl, () => closeMenu(false));
  });
</script>

<svelte:window onpointerdown={onWindowPointerDown} />

<div class="select-menu" bind:this={rootEl} onfocusout={onFocusOut}>
  {#if kebab}
    <button
      bind:this={triggerEl}
      type="button"
      {id}
      class="select-menu-kebab"
      aria-haspopup="menu"
      aria-expanded={open}
      aria-controls={open ? listId : undefined}
      aria-label={ariaLabel}
      {disabled}
      onclick={toggle}
      onkeydown={onTriggerKeydown}
    >
      <Icon name="ellipsis" size={20} />
    </button>
  {:else}
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
  {/if}

  {#if open}
    <div
      bind:this={listEl}
      use:portal
      id={listId}
      popover="manual"
      class="select-menu-list selpop"
      role={kebab ? "menu" : "listbox"}
      tabindex="-1"
      style={menuStyle}
      onkeydown={onListKeydown}
      onfocusout={onFocusOut}
    >
      {#each options as option, i (i)}
        <button
          bind:this={optionEls[i]}
          type="button"
          role={kebab ? "menuitem" : "option"}
          tabindex="-1"
          class="select-menu-option"
          class:active={!kebab && i === selectedIndex}
          class:danger={option.danger}
          aria-selected={kebab ? undefined : i === selectedIndex}
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

  /* The list is portalled out of its field, which resets its controls (Field.svelte). Reset the options here. */
  .select-menu-list :global(button) {
    margin: 0 !important;
  }
  .select-menu-list {
    position: fixed;
    inset: auto;
    margin: 0;
    box-sizing: border-box;
    overflow-y: auto;
    z-index: 100;
    padding: 4px 0;
    border: 1px solid var(--hair);
    border-radius: var(--pico-border-radius);
    box-shadow: var(--shadow-float);
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

  .select-menu-option.danger {
    color: var(--crit);
  }

  .select-menu-kebab {
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    margin: 0;
    padding: 0;
    background: none;
    border: 0;
    border-radius: 14px;
    color: var(--text);
    cursor: pointer;
  }

  .select-menu-kebab:focus-visible {
    outline: 3px solid var(--focus, currentColor);
    outline-offset: 2px;
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
