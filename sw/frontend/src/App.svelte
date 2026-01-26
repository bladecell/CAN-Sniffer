<script>
	import Navigation from "./Navigation.svelte";
	import Overview from "./Overview.svelte";
	import Telemetry from "./Telemetry.svelte";
	import Settings from "./Settings.svelte";
	import Diagnostics from "./Diagnostics.svelte";
	import CanLogging from "./CanLogging.svelte";
	import { canStore } from "$lib/canStore.svelte.js";
	import { onMount } from "svelte";
	import ToastContainer from "./lib/components/ToastContainer.svelte";

	onMount(() => {
		canStore.connect();
		canStore.loadDefinitions();
		canStore.startCanPolling();
		canStore.getObd2Status();

		const cleanup = () => {
			canStore.disconnect();
			canStore.stopCanPolling();
		};

		window.addEventListener("beforeunload", cleanup);

		return () => {
			cleanup();
			window.removeEventListener("beforeunload", cleanup);
		};
	});

	let activeTab = $state("dashboard");
	let count = $state(0);
</script>

<Navigation bind:activeTab />

<main class="main-content">
	{#if activeTab === "overview"}
		<Overview />
	{:else if activeTab === "telemetry"}
		<Telemetry />
	{:else if activeTab === "settings"}
		<Settings />
	{:else if activeTab === "diagnostics"}
		<Diagnostics />
	{:else if activeTab === "can-logging"}
		<CanLogging />
	{/if}
</main>

<ToastContainer />

<style>
	.main-content {
		margin-top: 100px;
		padding: 1rem;
		width: calc(100vw - 400px);
		max-width: calc(100vw - 400px);
		margin-left: 0;
		margin-right: 0;
		box-sizing: border-box;
	}

	@media (max-width: 991px) {
		.main-content {
			margin-top: 0;
			margin-bottom: 120px;
		}
	}

	:global(:root) {
		--shadow-s: "inset 0 1px 2px #ffffff30, 0 1px 2px #00000030, 0 1px 2px #00000015";
		--shadow-m: "inset 0 1px 2px #ffffff50, 0 2px 4px #00000030, 0 2px 3px #00000015";
		--shadow-l: "inset 0 1px 2px #ffffff70, 0 4px 6px #00000030, 0 4px 6px #00000015";
	}
</style>
