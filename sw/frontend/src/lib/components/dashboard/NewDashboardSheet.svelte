<script lang="ts">
  import Sheet from "$lib/components/Sheet.svelte";
  import { TEMPLATES, type DashboardTemplate, type TemplateId } from "$lib/dashboardTemplates";
  import TemplatePicker from "./TemplatePicker.svelte";

  interface Props {
    /** Returns an error message, or null when the dashboard was added. */
    onSave: (name: string, template: DashboardTemplate) => string | null;
    onClose: () => void;
  }

  let { onSave, onClose }: Props = $props();
  let name = $state("");
  let templateId = $state<TemplateId>(TEMPLATES[0].id);
  let message = $state("");

  function save(e: SubmitEvent) {
    e.preventDefault();
    const template = TEMPLATES.find((t) => t.id === templateId) ?? TEMPLATES[0];
    const err = onSave(name, template);
    if (err) message = err;
    else onClose();
  }
</script>

<Sheet open title="New dashboard" subtitle="Pick a starting layout and a name. You can change both later." {onClose}>
  <form class="db-form" onsubmit={save}>
    <label class="db-field">
      <span class="db-label">Name</span>
      <input class="db-input" type="text" maxlength="40" autocomplete="off" bind:value={name} />
    </label>
    <div class="db-field">
      <span class="db-label">Starting layout</span>
      <TemplatePicker value={templateId} onChange={(id) => (templateId = id)} />
    </div>
    {#if message}
      <p class="db-error" role="alert">{message}</p>
    {/if}
    <div class="db-actions">
      <button type="button" class="rv-btn" onclick={onClose}>Cancel</button>
      <button type="submit" class="rv-btn pri">Add dashboard</button>
    </div>
  </form>
</Sheet>

<style>
  .db-form {
    display: grid;
    gap: 16px;
    padding: 4px 0 8px;
  }
  .db-actions {
    display: flex;
    justify-content: flex-end;
    gap: 10px;
  }
</style>
