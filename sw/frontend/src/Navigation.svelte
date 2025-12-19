<script>
    import Icon from "./Icon.svelte";

    let { activeTab = $bindable() } = $props();

    const tabs = [
        { id: "overview", icon: "home", label: "Overview" },
        { id: "liveview", icon: "chart", label: "Live View" },
        { id: "settings", icon: "gear", label: "Settings" },
    ];
</script>

<div class="nav-wrapper">
    <nav class="pill-nav">
        <ul>
            {#each tabs as tab}
                <li>
                    <button
                        class:active={activeTab === tab.id}
                        onclick={() => (activeTab = tab.id)}
                    >
                        <div class="icon">
                            <Icon class="icon" name={tab.icon} size={26} />
                        </div>
                        <span class="label">{tab.label}</span>
                    </button>
                </li>
            {/each}
        </ul>
    </nav>
</div>

<style>
    .nav-wrapper {
        position: fixed;
        left: 0;
        right: 0;
        display: flex;
        justify-content: center;
        z-index: 1000;
        pointer-events: none;
    }

    .pill-nav {
        pointer-events: auto;
        /* background: var(--pico-card-background-color); */
        border: 1px solid var(--pico-muted-border-color);
        border-radius: 26px;
        padding: 0px 20px;
        box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
        backdrop-filter: blur(10px);
    }

    ul {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        gap: 8px;
    }

    li {
        margin: 0;
    }

    button {
        display: flex;
        flex-direction: column;
        align-items: center;
        justify-items: center;
        gap: 4px;
        padding: 8px 16px;
        border: none;
        border-radius: 26px;
        color: var(--pico-secondary);
        background: transparent;
        cursor: pointer;
        transition: all 0.2s ease;
    }

    button:hover {
        background: rgba(255, 255, 255, 0.05);
    }

    button.active {
        color: var(--pico-primary);
        border: none;
    }

    .icon {
        display: flex;
        align-items: center;
        justify-content: center;
        height: 48px;
        width: 48px;
    }

    @media (min-width: 992px) {
        .nav-wrapper {
            top: 20px;
        }

        .icon {
            display: none;
        }

        button {
            display: block;
        }
    }

    @media (max-width: 991px) {
        .nav-wrapper {
            bottom: 20px;
        }
        .label {
            display: none;
        }
        button {
            height: 56px;
            width: 64px;
            min-height: auto;
        }

        ul {
            height: 80px;
        }

        .pill-nav {
            padding: 0px 10px;
        }
    }
</style>
