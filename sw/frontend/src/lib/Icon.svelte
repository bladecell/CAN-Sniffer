<script lang="ts">
  // Force vite update
  // Load all SVG files as raw strings
  const svgFiles = import.meta.glob("/src/lib/icons_svg/*.svg", {
    query: "?raw",
    import: "default",
    eager: true,
  });

  let {
    name,
    size = 24,
    class: className = "",
  }: { name: string; size?: number; class?: string } = $props();

  // Dynamically grab the raw SVG string based on the provided name
  let rawSvg = $derived(
    (svgFiles[`/src/lib/icons_svg/${name}.svg`] as string) || ""
  );

  // Inject the requested size, class, and necessary aria roles directly into the root <svg> tag
  let svgContent = $derived(
    rawSvg.replace(
      "<svg",
      `<svg width="${size}" height="${size}" class="${className}" aria-hidden="true" role="img"`
    )
  );
</script>

{#if rawSvg}
  {@html svgContent}
{:else}
  <!-- Fallback invisible square if icon not found -->
  <svg
    width={size}
    height={size}
    class={className}
    viewBox="0 0 24 24"
    aria-hidden="true"
    role="img"
  ></svg>
{/if}
