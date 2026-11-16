<script lang="ts">
  import { auth } from "$lib/auth.svelte";
</script>

{#if auth.view.devBypass}
  <aside class="dev-bypass" role="alert">
    <strong>Development build: authentication is disabled.</strong> Gated routes accept any request. Do not use this firmware in a car.
  </aside>
{/if}

<style>
  .dev-bypass {
    margin: 0 0 1rem;
    padding: 0.75rem 1rem;
    border-radius: var(--r-card);
    background: var(--crit-f);
    color: var(--crit);
  }
</style>
