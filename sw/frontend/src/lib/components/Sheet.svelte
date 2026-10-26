<script lang="ts">
  import type { Snippet } from "svelte";
  import Icon from "$lib/Icon.svelte";
  import { overlay } from "$lib/overlay";
  import "./overlay.css";

  interface Props {
    /** One-way: the parent owns it. A dismiss calls onClose, and the parent decides whether to close. */
    open?: boolean;
    title: string;
    onClose?: () => void;
    dismissible?: boolean;
    children: Snippet;
    footer?: Snippet;
  }

  let { open = false, title, onClose, dismissible = true, children, footer }: Props = $props();
  const uid = $props.id();
  const titleId = `${uid}-title`;
</script>

<dialog use:overlay={{ open, dismissible, onClose }} class="overlay sheet" aria-labelledby={titleId}>
  <div class="grab" aria-hidden="true"></div>
  <header>
    <h2 id={titleId}>{title}</h2>
    {#if dismissible}
      <button type="button" class="close" aria-label="Close" onclick={() => onClose?.()}>
        <Icon name="x" size={20} />
      </button>
    {/if}
  </header>
  {#if open}
    <div class="body">{@render children()}</div>
    {#if footer}
      <footer>{@render footer()}</footer>
    {/if}
  {/if}
</dialog>
