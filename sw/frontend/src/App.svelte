<script>
	let activeTab = $state("dashboard");
	let count = $state(0);
</script>

<nav class="floating-nav">
	<div class="container-fluid">
		<ul>
			<li><strong>🚗 CAN Sniffer</strong></li>
		</ul>
		<ul>
			<li>
				<a
					href="#"
					class={activeTab === "dashboard" ? "" : "secondary"}
					onclick={() => (activeTab = "dashboard")}>Dashboard</a
				>
			</li>
			<li>
				<a
					href="#"
					class={activeTab === "settings" ? "" : "secondary"}
					onclick={() => (activeTab = "settings")}>Settings</a
				>
			</li>
			<li>
				<button class="outline" onclick={() => count++}
					>Packets: {count}</button
				>
			</li>
		</ul>
	</div>
</nav>

<main class="container content-area">
	{#if activeTab === "dashboard"}
		<section>
			<div class="grid">
				<article>
					<header>Bus Status</header>
					<ins>Active</ins> • 500kbps
				</article>
				<article>
					<header>Real-time Load</header>
					{Math.min(count, 100)}%
				</article>
			</div>

			<article>
				<table role="grid">
					<thead>
						<tr>
							<th>ID</th>
							<th>Data (HEX)</th>
						</tr>
					</thead>
					<tbody>
						<tr>
							<td><code>0x123</code></td>
							<td><code>AA BB CC DD</code></td>
						</tr>
					</tbody>
				</table>
			</article>
		</section>
	{:else}
		<section>
			<h2>Settings</h2>
			<fieldset>
				<label for="filter"
					>ID Filter (Hex)
					<input type="text" id="filter" placeholder="0x7FF" />
				</label>
			</fieldset>
		</section>
	{/if}
</main>

<style>
	/* Floating & Full Width Logic */
	.floating-nav {
		position: fixed;
		top: 0;
		left: 0;
		right: 0;
		width: 100%;
		z-index: 1000;
		background-color: var(--pico-background-color);
		border-bottom: 1px solid var(--pico-muted-border-color);
		/* Glass effect (optional) */
		backdrop-filter: blur(10px);
		opacity: 0.98;
		padding: 0 1rem; /* Padding for the inner elements */
	}

	/* Offset the main content so it doesn't hide under the nav */
	.content-area {
		margin-top: 5rem;
	}

	/* Style tweaks for CAN data */
	code {
		background: var(--pico-code-background-color);
		padding: 0.2rem 0.4rem;
		border-radius: 4px;
	}
</style>
