<script lang="ts">
  import Icon from "$lib/Icon.svelte";
  import Switch from "$lib/components/Switch.svelte";
  import SelectMenu from "$lib/components/SelectMenu.svelte";
  import { settingsStore } from "$lib/settingsStore.svelte";
  import { wifiScanStore } from "$lib/wifiScanStore.svelte";
  import {
    CHANNELS,
    GROUP,
    ROW,
    anyShown,
    buildWifiPatch,
    rowShown,
    scanAge,
    wifiDraft,
    type RowMeta,
    type ScanNetwork,
    type SettingsDoc,
    type WifiDraft,
  } from "$lib/settingsModel";
  import ApplyBar from "./ApplyBar.svelte";
  import NetworkPasswordDialog from "./NetworkPasswordDialog.svelte";
  import SettingRow from "./SettingRow.svelte";

  let { q, doc }: { q: string; doc: SettingsDoc } = $props();
  const show = (row: RowMeta) => rowShown(q, GROUP.wifi, row);
  const channelOptions = CHANNELS.map((n) => ({ value: n, label: String(n) }));

  // Edits sit over the saved values, so a save or reload shows the new values without copying them.
  let edits = $state<Partial<WifiDraft>>({});
  const draft = $derived({ ...wifiDraft(doc.wifi), ...edits });
  const built = $derived(buildWifiPatch(doc.wifi, draft));
  const count = $derived(Object.keys(built.patch).length);
  const blocked = $derived(Object.keys(built.errors).length > 0);

  // Clock for the scan age. Ticks once a second while this group is open.
  let now = $state(Date.now());
  $effect(() => {
    const id = setInterval(() => (now = Date.now()), 1000);
    return () => clearInterval(id);
  });
  const age = $derived(wifiScanStore.scannedAt === null ? null : scanAge(wifiScanStore.scannedAt, now));
  const scanHint = $derived.by(() => {
    if (wifiScanStore.scanning) return "Scanning. This takes a few seconds.";
    if (age === null) return "Scan to list the networks in range.";
    const found = `${wifiScanStore.networks.length} found`;
    return age.stale ? `Scanned ${age.seconds} s ago, so the list may be out of date. ${found}.` : `Scanned ${age.seconds} s ago, ${found}.`;
  });

  /** The password dialog is open for this network name, or closed when null. */
  let joining = $state<string | null>(null);

  async function apply() {
    if (blocked || count === 0) return;
    if (await settingsStore.save("wifi", built.patch)) edits = {};
  }

  function pickNetwork(n: ScanNetwork) {
    if (n.open) edits.staSsid = n.ssid;
    else joining = n.ssid;
  }

  function stageJoin(password: string) {
    if (joining !== null) {
      edits.staSsid = joining;
      edits.staPassword = password;
    }
    joining = null;
  }
</script>

<div class="st-sec">
  <h2>Hotspot</h2>
  {#if anyShown(q, GROUP.wifi, [ROW.hotspotName, ROW.hotspotPassword, ROW.channel])}
  <div class="st-list">
    {#if show(ROW.hotspotName)}
      <SettingRow title={ROW.hotspotName.title} hint="Visible to phones and laptops near the car." error={built.errors.ssid}>
        <input
          class="st-in"
          type="text"
          aria-label={ROW.hotspotName.title}
          value={draft.ssid}
          aria-invalid={built.errors.ssid ? "true" : undefined}
          oninput={(e) => (edits.ssid = e.currentTarget.value)}
        />
      </SettingRow>
    {/if}
    {#if show(ROW.hotspotPassword)}
      <SettingRow title={ROW.hotspotPassword.title} hint="8 to 63 characters. Blank keeps the saved one." error={built.errors.password}>
        <input
          class="st-in"
          type="password"
          autocomplete="new-password"
          placeholder="Leave blank to keep"
          aria-label={ROW.hotspotPassword.title}
          value={draft.password}
          aria-invalid={built.errors.password ? "true" : undefined}
          oninput={(e) => (edits.password = e.currentTarget.value)}
        />
      </SettingRow>
    {/if}
    {#if show(ROW.channel)}
      <SettingRow title={ROW.channel.title} hint="The 2.4 GHz channel the hotspot uses.">
        <SelectMenu options={channelOptions} value={draft.channel} ariaLabel={ROW.channel.title} onChange={(v) => (edits.channel = v)} />
      </SettingRow>
    {/if}
  </div>
  {/if}
</div>

<div class="st-sec">
  <h2>Join a network</h2>
  {#if anyShown(q, GROUP.wifi, [ROW.join, ROW.networks, ROW.staSsid])}
  <div class="st-list">
    {#if show(ROW.join)}
      <SettingRow title={ROW.join.title} hint="Lets the device reach the internet for time and updates." inline>
        <Switch checked={draft.join} aria-label={ROW.join.title} onchange={(on) => (edits.join = on)} />
      </SettingRow>
    {/if}
    {#if show(ROW.networks)}
      <SettingRow title={ROW.networks.title} hint={scanHint} stack error={wifiScanStore.error ?? undefined}>
        <button type="button" class="st-btn sm" onclick={() => wifiScanStore.scan()} disabled={wifiScanStore.scanning}>
          <Icon name="reload" size={16} />{wifiScanStore.scanning ? "Scanning" : age?.stale ? "Scan again" : "Scan"}
        </button>
        {#if wifiScanStore.networks.length > 0}
          <div class="st-nets" role="radiogroup" aria-label="Networks found">
            {#each wifiScanStore.networks as n (n.ssid)}
              <button type="button" class="st-net" role="radio" aria-checked={draft.staSsid === n.ssid} onclick={() => pickNetwork(n)}>
                <span class="st-net-name">{n.ssid}</span>
                <span class="st-net-dbm">{n.rssi} dBm</span>
                {#if !n.open}<Icon name="lock" size={14} />{:else}<span></span>{/if}
              </button>
            {/each}
          </div>
        {/if}
      </SettingRow>
    {/if}
    {#if show(ROW.staSsid)}
      <SettingRow title={ROW.staSsid.title} hint="For a hidden network: type its name, then set its password." error={built.errors.staSsid}>
        <div class="st-pair">
          <input
            class="st-in"
            type="text"
            aria-label={ROW.staSsid.title}
            value={draft.staSsid}
            aria-invalid={built.errors.staSsid ? "true" : undefined}
            oninput={(e) => (edits.staSsid = e.currentTarget.value)}
          />
          <button type="button" class="st-btn sm" disabled={draft.staSsid === ""} onclick={() => (joining = draft.staSsid)}>Password…</button>
        </div>
      </SettingRow>
    {/if}
  </div>
  {/if}
</div>

<ApplyBar {count} busy={settingsStore.saving} {blocked} onRevert={() => (edits = {})} onApply={apply} />

{#if joining !== null}
  <NetworkPasswordDialog ssid={joining} onJoin={stageJoin} onCancel={() => (joining = null)} />
{/if}
