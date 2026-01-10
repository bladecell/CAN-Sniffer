<script>
    let selectedMode = $state("3");

    async function requestDTC() {
        try {
            const response = await fetch(
                "/api/v1/req/dtc?mode=" + selectedMode,
                {
                    method: "POST",
                },
            );

            if (response.ok) {
                console.log("DTC Request:", selectedMode);
                // Handle the response data here
            } else {
                console.error("Request failed:", response.status);
            }
        } catch (error) {
            console.error("Error requesting DTC:", error);
        }
    }
</script>

<div class="container">
    <!-- DTC Request Section -->
    <section>
        <h3>Diagnostic Trouble Codes</h3>
        <div class="grid">
            <select bind:value={selectedMode}>
                <option value="3">Current DTCs</option>
                <option value="7">Pending DTCs</option>
                <option value="10">Permanent DTCs</option>
            </select>
            <button onclick={requestDTC}>Request DTC</button>
        </div>
    </section>
</div>

<div class="container">
    <!-- Additional diagnostic features can be added here -->
    <section>
        <h3>Confirmed DTCs</h3>
        <table role="grid">
            <thead>
                <tr><th>DTC Code</th><th>Fault Description</th></tr>
            </thead>
            <tbody>
                <tr>
                    <td><code>0x123</code></td>
                    <td><code>DEADBEEF</code></td>
                </tr>
            </tbody>
        </table>
    </section>
</div>

<style>
    section {
        margin-bottom: 2rem;
    }
</style>
