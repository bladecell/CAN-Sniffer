<script>
    import PIDCard from "$lib/components/PIDCard.svelte";
    import { createSwapy } from "swapy";
    import { onDestroy, onMount } from "svelte";
    import { scale } from "svelte/transition";
    import { flip } from "svelte/animate";
    import { canStore } from "$lib/canStore.svelte.js";
    import CANConnectionCard from "$lib/components/CANConnectionCard.svelte";
    import Switch from "$lib/components/Switch.svelte";
    import { alertStore } from "./lib/alertStore.svelte";

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

    let container;
    let swapy = null;
    let visibilityState = $state({});
    let cards = $derived(
        canStore.pidDefinitions.map((pidData) => {
            const card = createCardFromPID(pidData);
            card.visible = visibilityState[pidData.pid] ?? true;
            return card;
        }),
    );

    onMount(() => {
        if (container) {
            swapy = createSwapy(container);

            swapy.onSwap((event) => {
                console.log("swap", event);
            });

            canStore.startLogging();
        }
    });

    onDestroy(() => {
        swapy?.destroy();
        canStore.stopLogging();
    });

    function toggleCard(index) {
        const card = cards[index];
        visibilityState[card.pid] = !card.visible;

        setTimeout(() => swapy?.update(), 0);
    }

    let searchTerm = $state("");

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
    <div class="status-container">
        <CANConnectionCard />
        <Switch
            label="Data Polling"
            checked={canStore.obd2Status?.continuous_running}
            statusText={canStore.obd2Status?.continuous_running
                ? "Running"
                : "Stopped"}
            onchange={(val) => canStore.setContinuousPolling(val)}
        />
    </div>

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
    .status-container {
        display: flex;
        flex-direction: row;
        align-items: center;
        gap: 1rem;
    }

    .controls-container {
        display: flex;
        justify-content: space-between;
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
