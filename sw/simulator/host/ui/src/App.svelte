<script lang="ts">
  import { onMount } from 'svelte';
  import { Button } from '$lib/components/ui/button';
  import { Card, CardContent, CardHeader, CardTitle } from '$lib/components/ui/card';
  import { apiHeaders } from '$lib/api';

  type Frame = { ev: 'frame'; dir?: string; id?: number; data?: number[]; ts?: number; unmatched?: boolean };
  type Status = { connected?: boolean; port?: string; stats?: Record<string, unknown> };
  type Profile = { name: string };
  let status = $state<Status>({});
  let profiles = $state<Profile[]>([]);
  let frames = $state<Frame[]>([]);
  let selected = $state('example');
  let ecu = $state('engine');
  let pid = $state('0C');
  let value = $state('');
  let faultOn = $state(false);
  let running = $state(false);
  let loading = $state(true);
  let error = $state('');
  let wsState = $state('Connecting');

  async function request<T>(path: string, method = 'GET', body?: unknown): Promise<T> {
    const response = await fetch(path, {
      method,
      headers: apiHeaders(method, body !== undefined),
      ...(body === undefined ? {} : { body: JSON.stringify(body) })
    });
    const result: unknown = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(typeof result === 'object' && result !== null && 'error' in result ? String(result.error) : `Request failed (${response.status})`);
    return result as T;
  }
  async function load() {
    loading = true; error = '';
    try {
      [status, profiles] = await Promise.all([request<Status>('/status'), request<{ profiles: string[] }>('/profiles').then(r => r.profiles.map(name => ({ name })))]);
      if (!profiles.some(p => p.name === selected)) selected = profiles[0]?.name ?? '';
    } catch (e) { error = e instanceof Error ? e.message : 'Could not reach the simulator host'; }
    finally { loading = false; }
  }
  async function act(path: string, body?: unknown) {
    error = '';
    try { await request(path, 'POST', body); await load(); }
    catch (e) { error = e instanceof Error ? e.message : 'Action failed'; }
  }
  function connect() {
    const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`);
    socket.onopen = () => wsState = 'Live';
    socket.onclose = () => { wsState = 'Reconnecting'; setTimeout(connect, 1500); };
    socket.onerror = () => wsState = 'Unavailable';
    socket.onmessage = event => {
      try {
        const data: unknown = JSON.parse(String(event.data));
        if (typeof data !== 'object' || data === null) return;
        if ('ev' in data && data.ev === 'frame') frames = [...frames.slice(-199), data as Frame];
        if ('ev' in data && data.ev === 'connection') void load();
      } catch { /* Ignore malformed event packets. */ }
    };
  }
  onMount(() => {
    if (matchMedia('(prefers-color-scheme: dark)').matches) document.documentElement.classList.add('dark');
    void load(); connect(); const timer = setInterval(() => void load(), 3000); return () => clearInterval(timer);
  });
  const hex = (n: number | undefined) => `0x${(n ?? 0).toString(16).toUpperCase().padStart(3, '0')}`;
</script>

<svelte:head><title>Bench simulator | CAN Sniffer</title><meta name="viewport" content="width=device-width, initial-scale=1" /></svelte:head>
<main>
  <header><div><p class="overline">CAN SNIFFER / BENCH</p><h1>Simulator workbench</h1></div><div class="connection" aria-live="polite"><span class:online={status.connected}></span><span>{status.connected ? 'USB connected' : loading ? 'Checking USB…' : 'USB disconnected'}</span><small>{status.port || 'No serial port'}</small></div></header>
  <p class="safety"><strong>Bench only.</strong> Keep this simulator disconnected from any vehicle.</p>
  {#if error}<div class="error" role="alert"><span>{error}</span><Button variant="outline" onclick={() => void load()}>Retry</Button></div>{/if}
  {#if loading}<p class="notice" aria-live="polite">Checking simulator host…</p>{/if}
  <div class="layout">
    <section class="controls">
      <Card>
        <CardHeader><CardTitle>Profile</CardTitle></CardHeader><CardContent>
          <label for="profile">Configuration profile</label><select id="profile" bind:value={selected} disabled={!profiles.length}><option value="" disabled>Select a profile</option>{#each profiles as item (item.name)}<option value={item.name}>{item.name}</option>{/each}</select>
          {#if !loading && !profiles.length}<p class="hint">No profiles found. Add a profile JSON to the host profiles folder.</p>{/if}
          <Button class="primary" disabled={!status.connected || !selected} onclick={() => void act(`/profiles/${selected}/push`)}>Push profile to simulator</Button>
          <p class="hint">Pushing replaces the simulator’s staged model.</p>
        </CardContent>
      </Card>
      <Card>
        <CardHeader><CardTitle>Live settings</CardTitle></CardHeader><CardContent>
          <label for="ecu">ECU</label><select id="ecu" bind:value={ecu}><option value="engine">Engine</option></select>
          <label for="pid">PID (hex)</label><input id="pid" bind:value={pid} maxlength="4" autocomplete="off" />
          <label for="value">Engineering value</label><input id="value" bind:value={value} inputmode="decimal" placeholder="Enter a value" />
          <Button disabled={!status.connected || !value || !Number.isFinite(Number(value))} onclick={() => void act('/pid', { profile: selected, ecu, pid, value: Number(value) })}>Set PID value</Button>
          <div class="fault-row"><span>ECU response fault</span><Button variant="outline" aria-pressed={faultOn} disabled={!status.connected} onclick={() => { faultOn = !faultOn; void act('/fault', { name: ecu, silent: faultOn }); }}>{faultOn ? 'Disable fault' : 'Enable silence'}</Button></div>
          <div class="scenario"><Button variant="outline" disabled={!status.connected || !selected} onclick={() => void act('/scenario/start', { profile: selected })}>Start scenario</Button><Button variant="outline" disabled={!status.connected} onclick={() => void act('/scenario/stop')}>Stop scenario</Button></div>
        </CardContent>
      </Card>
    </section>
    <section class="monitor">
      <Card>
        <CardHeader class="frame-head"><CardTitle>CAN frames</CardTitle><div><span class="ws-state">{wsState}</span><Button variant="outline" onclick={() => frames = []}>Clear</Button></div></CardHeader>
        <CardContent>
          {#if frames.length}
            <div class="table-wrap"><table><thead><tr><th>Direction</th><th>ID</th><th>Data</th><th>Match</th></tr></thead><tbody>{#each frames as frame, i (frame.ts ?? i)}<tr><td>{frame.dir ?? '—'}</td><td class="mono">{hex(frame.id)}</td><td class="mono bytes">{(frame.data ?? []).map(byte => byte.toString(16).padStart(2, '0').toUpperCase()).join(' ')}</td><td>{frame.unmatched ? 'No rule' : 'Matched'}</td></tr>{/each}</tbody></table></div>
          {:else if loading}<p class="empty">Waiting for the host connection…</p>
          {:else if error}<p class="empty">Frames are unavailable. Check the host connection, then retry.</p>
          {:else}<p class="empty">No frames received yet. Push a profile, then send an OBD request from the sniffer.</p>{/if}
          <p class="hint">Showing the latest 200 events. Stream: {wsState}.</p>
        </CardContent>
      </Card>
      <Card class="stats"><CardHeader><CardTitle>Device status</CardTitle></CardHeader><CardContent><dl><div><dt>USB</dt><dd>{status.connected ? 'Connected' : 'Disconnected'}</dd></div><div><dt>Serial port</dt><dd class="mono">{status.port || 'Not set'}</dd></div><div><dt>Frame feed</dt><dd>{wsState}</dd></div></dl></CardContent></Card>
    </section>
  </div>
</main>
