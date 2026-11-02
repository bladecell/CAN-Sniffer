<script lang="ts">
  import type { Snippet } from "svelte";
  import { overlay } from "$lib/overlay";
  import "./overlay.css";

  interface Props {
    /** One-way: the parent owns it. Cancel, Esc and a backdrop click call onCancel. */
    open?: boolean;
    title: string;
    message?: string;
    confirmLabel: string;
    cancelLabel?: string;
    danger?: boolean;
    /** Extra body content (a field), shown under the message. */
    children?: Snippet;
    onConfirm: () => void;
    onCancel?: () => void;
  }

  let {
    open = false,
    title,
    message,
    confirmLabel,
    cancelLabel = "Cancel",
    danger = false,
    children,
    onConfirm,
    onCancel,
  }: Props = $props();
  const uid = $props.id();
  const titleId = `${uid}-title`;
</script>

<dialog use:overlay={{ open, dismissible: true, onClose: onCancel }} class="overlay dialog" aria-labelledby={titleId}>
  <div class="content">
    <h2 id={titleId}>{title}</h2>
    {#if message}
      <p>{message}</p>
    {/if}
    {#if children}{@render children()}{/if}
    <div class="actions">
      <!-- svelte-ignore a11y_autofocus -->
      <button type="button" class="btn" autofocus onclick={() => onCancel?.()}>{cancelLabel}</button>
      <button type="button" class="btn" class:pri={!danger} class:danger onclick={onConfirm}>{confirmLabel}</button>
    </div>
  </div>
</dialog>

<style>
  .content {
    padding: 22px;
  }
  p {
    margin: 8px 0 0;
    color: var(--muted);
  }
  .actions {
    display: flex;
    flex-wrap: wrap;
    justify-content: flex-end;
    gap: 10px;
    margin-top: 20px;
  }
</style>
