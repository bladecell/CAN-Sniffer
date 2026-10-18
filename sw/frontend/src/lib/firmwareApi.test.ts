import { describe, expect, test } from "bun:test";
import { createFirmwareApi } from "./firmwareApi";

describe("firmware API client", () => {
  test("exposes every non-auth REST operation from the firmware contract", async () => {
    const calls: string[] = [];
    const api = createFirmwareApi({
      fetch: async (url) => { calls.push(`open ${url}`); return new Response("{}"); },
      gatedFetch: async (url, init) => { calls.push(`${init?.method ?? "GET"} ${url}`); return new Response("{}"); },
    });
    await api.index(); await api.system(); await api.reboot(); await api.canBus(); await api.obd2();
    await api.setAutoPoll(true); await api.pidDefinitions(); await api.getPidDefinition(7);
    await api.addPidDefinitions([]); await api.replacePidDefinitions([]); await api.deletePidDefinition(7);
    await api.savePidDefinitions(); await api.loadPidDefinitions(); await api.pidData(); await api.getPidData(7);
    await api.requestStaticPid(); await api.vin(); await api.requestVin(); await api.dtc(); await api.dtc(3);
    await api.dtcDescriptions(["P0301"]); await api.requestDtc(); await api.requestDtc(10); await api.clearDtc();
    await api.sdInfo(); await api.sdTree(); await api.sdTree("logs", { depth: 2, limit: 20, offset: 4 });
    await api.downloadSdFile("logs/trip.csv"); await api.uploadSdFile("logs/trip.csv", new Blob());
    await api.createSdDirectory("logs/new"); await api.deleteSdFile("logs/trip.csv");
    await api.deleteSdFile("logs/old", true); await api.copySdFile("a", "b"); await api.formatSdCard();
    await api.getSettings(); await api.patchSettings({ can: {} }); await api.startWifiScan(); await api.wifiScan();
    expect(calls).toHaveLength(38);
    expect(calls.filter((call) => call === "GET /api/v1/settings" || call === "GET /api/v1/wifi/scan")).toHaveLength(2);
    expect(calls.some((call) => call.includes("/api/v1/req/clear_dtc"))).toBe(true);
  });

  test("uses documented methods, routes, auth and binary response handling", async () => {
    const calls: Array<{ url: string; init?: RequestInit; gated: boolean }> = [];
    const response = () => new Response(JSON.stringify({ status: "success" }), { status: 200 });
    const api = createFirmwareApi({
      fetch: async (url, init) => { calls.push({ url, init, gated: false }); return response(); },
      gatedFetch: async (url, init) => { calls.push({ url, init, gated: true }); return response(); },
    });

    await api.pidData();
    await api.getPidDefinition(12);
    await api.addPidDefinitions([]);
    await api.loadPidDefinitions();
    await api.reboot();
    await api.copySdFile("a", "b");
    await api.formatSdCard();
    await api.wifiScan();
    await api.getSettings();
    await api.patchSettings({ can: { bitrate: 500000 } });
    await api.startWifiScan();
    await api.downloadSdFile("a/b.csv");
    await api.uploadSdFile("a/b.csv", new Blob(["x"]));

    expect(calls.map(({ url, init, gated }) => [init?.method ?? "GET", url, gated])).toEqual([
      ["GET", "/api/v1/pid_data", false],
      ["GET", "/api/v1/pid_def/12", false],
      ["POST", "/api/v1/pid_def", true],
      ["POST", "/api/v1/pid_def/load", true],
      ["POST", "/api/v1/system/reboot", true],
      ["POST", "/api/v1/sd_card/copy", true],
      ["POST", "/api/v1/sd_card/format", true],
      ["GET", "/api/v1/wifi/scan", true],
      ["GET", "/api/v1/settings", true],
      ["PATCH", "/api/v1/settings", true],
      ["POST", "/api/v1/wifi/scan", true],
      ["GET", "/api/v1/sd_card/file/a/b.csv?download=true", true],
      ["PUT", "/api/v1/sd_card/file/a/b.csv", true],
    ]);
    expect(calls[11].init?.method).toBeUndefined();
    expect(calls[12].init?.body).toBeInstanceOf(Blob);
    expect(new Headers(calls[12].init?.headers).get("Content-Type")).toBe("application/octet-stream");
  });
});
