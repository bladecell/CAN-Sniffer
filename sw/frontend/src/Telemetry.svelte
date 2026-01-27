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
            min: pidData.minValue || 0,
            max: pidData.maxValue || 100,
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
        }
    });

    $effect(() => {
        if (canStore.connected) {
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
    <div class="status-group">
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
    .status-group {
        display: flex;
        gap: 1rem;
        align-items: stretch;
    }

    .controls-container {
        display: flex;
        justify-content: space-between;
        align-items: stretch;
        gap: 1rem;
        margin-bottom: 1rem;
        flex-wrap: wrap;
    }

    .dropdown {
        margin-bottom: 0;
        height: auto;
        min-width: 250px;
    }

    .dropdown summary {
        height: 100%;
        display: flex;
        align-items: center;
        margin-bottom: 0;
        white-space: nowrap;
    }

    .cards-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
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

    @media (max-width: 768px) {
        .controls-container {
            flex-direction: column; /* Stack Group on top, Dropdown below */
        }

        .status-group {
            width: 100%;
        }

        /* Make the two cards share width equally 50/50 */
        .status-group > :global(*) {
            flex: 1;
        }

        .dropdown {
            width: 100%; /* Dropdown takes full width below */
        }
    }
</style>
