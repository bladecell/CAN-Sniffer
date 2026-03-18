<script>
    let count = $state(0);
</script>

<header>
    <h2>CAN Sniffer</h2>
    <p>Monitoring real-time bus traffic</p>
</header>

<div class="grid">
    <article>
        <header>Packets</header>
        <h3 class="count-display">{count}</h3>
    </article>
    <article>
        <header>Bus Status</header>
        <ins>Active</ins>
    </article>
</div>

<figure>
    <table role="grid">
        <thead>
            <tr><th>ID</th><th>Data</th></tr>
        </thead>
        <tbody>
            <tr>
                <td><code>0x123</code></td>
                <td><code>DEADBEEF</code></td>
            </tr>
        </tbody>
    </table>
</figure>

<style>
    .count-display {
        margin: 0;
        color: var(--pico-primary);
        font-size: 2.5rem;
        font-weight: 700;
    }
</style>
