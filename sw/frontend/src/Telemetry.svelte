<script>
    import PIDCard from "./PIDCard.svelte";
    import { createSwapy } from "swapy";
    import { onDestroy, onMount } from "svelte";
    import { scale } from "svelte/transition";
    import { flip } from "svelte/animate";
    import MultiSelect from "svelte-multiselect";

    // let cards = $state([
    //     {
    //         label: "Coolant Temp",
    //         value: 50,
    //         unit: "°C",
    //         icon: "thermometer",
    //         status: "warning",
    //         color: "#ef4444",
    //         min: 0,
    //         max: 100,
    //         supported: true,
    //         valid: true,
    //         visible: true,
    //     },
    //     {
    //         label: "Engine RPM",
    //         value: 2450,
    //         unit: "rpm",
    //         icon: "gauge",
    //         status: "normal",
    //         color: "#f59e0b",
    //         min: 0,
    //         max: 100,
    //         supported: true,
    //         valid: true,
    //         visible: true,
    //     },
    //     {
    //         label: "Fuel Level",
    //         value: 67,
    //         unit: "%",
    //         icon: "droplet",
    //         status: "normal",
    //         color: "#3b82f6",
    //         min: 0,
    //         max: 100,
    //         supported: false,
    //         valid: true,
    //         visible: true,
    //     },
    // ]);

    function createCardFromPID(pidData) {
        return {
            label: pidData.name,
            value: 0, // Initial value, will be updated by API
            unit: pidData.unit,
            icon: pidData.icon,
            color: `#${pidData.color.toString(16).padStart(6, "0")}`,
            min: pidData.min || 0,
            max: pidData.max || 100,
            supported: pidData.supported ?? true,
            valid: true,
            visible: true,
            pid: pidData.pid,
            mode: pidData.mode,
        };
    }

    async function fetchCards() {
        const response = await fetch("/api/v1/pid_def");
        const result = await response.json();
        cards = result.data.map((pidData) => createCardFromPID(pidData));
    }

    let container;
    let swapy = null;
    let cards = $state([]);

    onMount(async () => {
        try {
            await fetchCards();
        } catch (error) {
            console.error("Error Fetching cards:", error);
        }

        if (container) {
            swapy = createSwapy(container);

            swapy.onSwap((event) => {
                console.log("swap", event);
            });
        }
    });

    onDestroy(() => {
        swapy?.destroy();
    });

    function toggleCard(index) {
        cards[index].visible = !cards[index].visible;
        swapy.update();
    }

    // Filter cards based on search term
    let searchTerm = $state("");

    // Filter cards based on search term
    let filteredCards = $derived(
        cards
            .map((card, index) => ({
                ...card,
                index,
                matches: card.label
                    .toLowerCase()
                    .includes(searchTerm.toLowerCase()),
            }))
            .filter((card) => card.matches),
    );
</script>

<div class="controls-container">
    <details class="dropdown">
        <summary> Parameters to show </summary>
        <ul>
            <li class="search-container">
                <input
                    type="search"
                    placeholder="Search..."
                    bind:value={searchTerm}
                    onclick={(e) => e.stopPropagation()}
                />
            </li>
            {#each filteredCards as card}
                <li>
                    <label>
                        <input
                            type="checkbox"
                            checked={card.visible}
                            onchange={() => toggleCard(card.index)}
                        />
                        {card.label}
                    </label>
                </li>
            {/each}
            {#if filteredCards.length === 0}
                <li class="no-results">No matches found</li>
            {/if}
        </ul>
    </details>
</div>

<div bind:this={container} class="cards-grid">
    {#each cards.filter((c) => c.visible) as card, index (card.label)}
        <div
            data-swapy-slot={index.toString()}
            transition:scale={{ duration: 300 }}
            animate:flip={{ duration: 300 }}
        >
            <div data-swapy-item={index.toString()}>
                <PIDCard {...card} />
            </div>
        </div>
    {/each}
</div>

<style>
    .controls-container {
        display: flex;
        justify-content: flex-end;
        margin-bottom: 1rem;
    }

    .dropdown {
        position: relative;
        display: inline-block;
        width: 15vw;
    }
    .cards-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 1rem;
    }

    .search-container {
        position: sticky;
        top: 0;
        padding: 0.5rem;
    }

    .search-container input {
        margin: 0;
    }

    .no-results {
        text-align: center;
        color: var(--pico-muted-color);
        padding: 1rem;
    }
</style>
