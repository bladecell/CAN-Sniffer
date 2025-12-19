<script>
    import PIDCard from "./PIDCard.svelte";

    let cards = $state([
        {
            label: "Coolant Temp",
            value: 95,
            unit: "°C",
            icon: "thermometer",
            status: "warning",
            color: "#ef4444",
        },
        {
            label: "Engine RPM",
            value: 2450,
            unit: "rpm",
            icon: "gauge",
            status: "normal",
            color: "#f59e0b",
        },
        {
            label: "Fuel Level",
            value: 67,
            unit: "%",
            icon: "droplet",
            status: "normal",
            color: "#3b82f6",
        },
    ]);
</script>

<div class="cards-grid">
    {#each cards as card}
        <PIDCard {...card} />
    {/each}
</div>

<style>
    .cards-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 1rem;
    }
</style>
