import { describe, expect, test } from "bun:test";
import {
  GROUP,
  GROUP_IDS,
  buildCanPatch,
  buildHwFilterPatch,
  buildSleepPatch,
  sleepDraft,
  parseVolts,
  supervisorStateText,
  batteryText,
  hwFilterDraft,
  hwFilterPreview,
  buildPidPathPatch,
  buildWifiPatch,
  checkPassword,
  checkPidPath,
  checkSsid,
  formatUptime,
  groupShown,
  matches,
  parseIntField,
  checkJoinPassword,
  parseScanReply,
  parseSdInfo,
  scanAge,
  sdView,
  parseSettingsDoc,
  rowShown,
  wifiDraft,
  type SettingsDoc,
} from "./settingsModel";

const doc: SettingsDoc = {
  wifi: { ssid: "CAN-Sniffer", channel: 6, mode: 2, sta_ssid: "" },
  can: { bitrate: 500000, rx_queue_size: 20, tx_queue_depth: 20, hwFilter: null, debug: null },
  system: { pid_def_path: "/sdcard/pids/pid.json", power: null },
};

describe("search", () => {
  test("empty query matches everything, every word must match", () => {
    expect(matches("", "anything")).toBe(true);
    expect(matches("  ", "anything")).toBe(true);
    expect(matches("can bus", "Vehicle & CAN bus")).toBe(true);
    expect(matches("can gnss", "Vehicle & CAN bus")).toBe(false);
  });

  test("a group title match shows every row of that group", () => {
    expect(rowShown("wi-fi", GROUP.wifi, GROUP.wifi.rows[0]!)).toBe(true);
    expect(rowShown("access", GROUP.wifi, GROUP.wifi.rows[0]!)).toBe(true);
    expect(rowShown("access", GROUP.wifi, GROUP.wifi.rows[4]!)).toBe(false);
    expect(rowShown("hidden", GROUP.wifi, GROUP.wifi.rows[5]!)).toBe(true);
  });

  test("groups hide when neither title, keywords nor a row matches", () => {
    expect(groupShown("format", GROUP.storage)).toBe(true);
    expect(groupShown("format", GROUP.wifi)).toBe(false);
    expect(GROUP_IDS.filter((id) => groupShown("restart", GROUP[id]))).toEqual(["device"]);
  });
});

describe("field checks", () => {
  test("ssid is 1 to 32 bytes, not characters", () => {
    expect(checkSsid("")).not.toBeNull();
    expect(checkSsid("a".repeat(32))).toBeNull();
    expect(checkSsid("a".repeat(33))).not.toBeNull();
    expect(checkSsid("ä".repeat(17))).not.toBeNull(); // 34 bytes
  });

  test("password is blank (keep) or 8 to 63 bytes", () => {
    expect(checkPassword("")).toBeNull();
    expect(checkPassword("1234567")).not.toBeNull();
    expect(checkPassword("12345678")).toBeNull();
    expect(checkPassword("x".repeat(64))).not.toBeNull();
  });

  test("pid path must start under /sdcard/ and avoid .. parts", () => {
    expect(checkPidPath("/sdcard/pids/a.json")).toBeNull();
    expect(checkPidPath("/pids")).not.toBeNull();
    expect(checkPidPath("/sdcard/../etc")).not.toBeNull();
    expect(checkPidPath("/sdcard/" + "a".repeat(250))).not.toBeNull();
  });

  test("integer fields reject signs, decimals, junk and out-of-range values", () => {
    expect(parseIntField("20", 1, 256)).toBe(20);
    expect(parseIntField(" 256 ", 1, 256)).toBe(256);
    expect(parseIntField("257", 1, 256)).toBeNull();
    expect(parseIntField("0", 1, 256)).toBeNull();
    expect(parseIntField("-1", 1, 256)).toBeNull();
    expect(parseIntField("2.5", 1, 256)).toBeNull();
    expect(parseIntField("1e3", 1, 256)).toBeNull();
    expect(parseIntField("", 1, 256)).toBeNull();
  });
});

describe("PATCH builders", () => {
  test("wifi: only changed fields go out, blank passwords are not sent", () => {
    const d = wifiDraft(doc.wifi);
    expect(buildWifiPatch(doc.wifi, d)).toEqual({ patch: {}, errors: {} });
    const out = buildWifiPatch(doc.wifi, { ...d, ssid: "Garage", channel: 11, password: "", join: true });
    expect(out.errors).toEqual({});
    expect(out.patch).toEqual({ ssid: "Garage", channel: 11, mode: 3 });
  });

  test("wifi: a typed password is sent, a bad one is an error and blocks nothing else in the check", () => {
    const d = wifiDraft(doc.wifi);
    expect(buildWifiPatch(doc.wifi, { ...d, password: "longenough" }).patch).toEqual({ password: "longenough" });
    const bad = buildWifiPatch(doc.wifi, { ...d, password: "short", ssid: "" });
    expect(Object.keys(bad.errors).sort()).toEqual(["password", "ssid"]);
    expect(bad.patch).toEqual({});
  });

  test("wifi: switching join off on an AP+STA device sends mode 2", () => {
    const saved = { ...doc.wifi, mode: 3 };
    expect(buildWifiPatch(saved, { ...wifiDraft(saved), join: false }).patch).toEqual({ mode: 2 });
    expect(buildWifiPatch(saved, wifiDraft(saved)).patch).toEqual({});
  });

  test("can: queues and bitrate are checked before they are sent", () => {
    const base = { bitrate: 500000, rx: "20", tx: "20" };
    expect(buildCanPatch(doc.can, base)).toEqual({ patch: {}, errors: {} });
    expect(buildCanPatch(doc.can, { ...base, rx: "128", tx: "64" }).patch).toEqual({ rx_queue_size: 128, tx_queue_depth: 64 });
    const bad = buildCanPatch(doc.can, { ...base, rx: "300", tx: "0", bitrate: 333333 });
    expect(Object.keys(bad.errors).sort()).toEqual(["bitrate", "rx_queue_size", "tx_queue_depth"]);
    expect(bad.patch).toEqual({});
  });

  test("system: pid path is sent only when it changes and is valid", () => {
    expect(buildPidPathPatch(doc.system, doc.system.pid_def_path)).toEqual({ patch: {}, errors: {} });
    expect(buildPidPathPatch(doc.system, "/sdcard/other.json").patch).toEqual({ pid_def_path: "/sdcard/other.json" });
    expect(buildPidPathPatch(doc.system, "/tmp/x").errors.pid_def_path).toBeDefined();
  });
});

describe("wire guards", () => {
  const raw = {
    wifi: { ssid: "CAN-Sniffer", channel: 6, mode: 2, sta_ssid: "", password: "never returned" },
    can: { bitrate: 500000, rx_queue_size: 20, tx_queue_depth: 20, filter: [], debug: true },
    system: { pid_def_path: "/sdcard/pid.json", dtc_desc_path: "/sdcard/dtc.bin" },
  };

  test("parseSettingsDoc keeps the fields the screen uses", () => {
    expect(parseSettingsDoc(raw)).toEqual({
      wifi: { ssid: "CAN-Sniffer", channel: 6, mode: 2, sta_ssid: "" },
      can: { bitrate: 500000, rx_queue_size: 20, tx_queue_depth: 20, hwFilter: null, debug: true },
      system: { pid_def_path: "/sdcard/pid.json", power: null },
    });
  });

  test("power fields are read when present and the sleep patch sends only changes", () => {
    const power = { sleep_enabled: true, stay_awake_voltage_mv: 13200, low_battery_voltage_mv: 11400 };
    const d = parseSettingsDoc({ ...raw, system: { ...raw.system, ...power } });
    expect(d?.system.power).toEqual(power);
    expect(buildSleepPatch(power, sleepDraft(power))).toEqual({ patch: {}, errors: {} });
    expect(buildSleepPatch(power, { enabled: false, awake: "13,5", low: "11.4" }).patch).toEqual({ sleep_enabled: false, stay_awake_voltage_mv: 13500 });
    expect(buildSleepPatch(power, { enabled: true, awake: "12", low: "12" }).errors.low_battery_voltage_mv).toBeDefined();
    expect(buildSleepPatch(power, { enabled: true, awake: "17", low: "abc" }).errors).toHaveProperty("stay_awake_voltage_mv");
    expect(parseVolts("9")).toBe(9000);
    expect(parseVolts("8.9")).toBeNull();
    expect(supervisorStateText(2)).toBe("Waiting for the CAN bus");
    expect(supervisorStateText(99)).toBe("Unknown");
    expect(batteryText(12.64)).toBe("12.6 V");
    expect(batteryText(Number.NaN)).toBe("-");
    expect(batteryText(null)).toBe("-");
  });

  test("hardware filter is read when the firmware sends it, and the patch sends only changes", () => {
    const withFilter = { ...raw, can: { ...raw.can, filter: true, mfilter_cfg: { id: 0x700, mask: 0x700, is_ext: false } } };
    const doc = parseSettingsDoc(withFilter);
    expect(doc?.can.hwFilter).toEqual({ on: true, id: 0x700, mask: 0x700, is_ext: false });
    const saved = doc!.can.hwFilter!;
    expect(buildHwFilterPatch(saved, hwFilterDraft(saved))).toEqual({ patch: {}, errors: {} });
    expect(buildHwFilterPatch(saved, { ...hwFilterDraft(saved), on: false }).patch).toEqual({ filter: false });
    expect(buildHwFilterPatch(saved, { ...hwFilterDraft(saved), id: "7E8" }).patch).toEqual({ mfilter_cfg: { id: 0x7e8, mask: 0x700, is_ext: false } });
    expect(buildHwFilterPatch(saved, { ...hwFilterDraft(saved), mask: "zz" }).errors.mfilter_cfg).toBeDefined();
  });

  test("parseSettingsDoc rejects missing sections and wrong types", () => {
    expect(parseSettingsDoc(null)).toBeNull();
    expect(parseSettingsDoc({ ...raw, can: undefined })).toBeNull();
    expect(parseSettingsDoc({ ...raw, wifi: { ...raw.wifi, channel: "6" } })).toBeNull();
    expect(parseSettingsDoc({ ...raw, system: { pid_def_path: 7 } })).toBeNull();
  });

  test("parseScanReply drops malformed networks and caps the list", () => {
    const many = Array.from({ length: 30 }, (_, i) => ({ ssid: `n${i}`, rssi: -50, auth_mode: 3 }));
    const reply = parseScanReply({ state: "complete", networks: [...many, { ssid: "", rssi: -40 }, null, { ssid: "x" }] });
    expect(reply?.state).toBe("complete");
    expect(reply?.networks).toHaveLength(20);
    expect(reply?.networks[0]).toEqual({ ssid: "n0", rssi: -50, open: false });
    expect(parseScanReply({ state: "weird" })).toBeNull();
    expect(parseScanReply({ state: "idle", networks: [{ ssid: "open", rssi: -70, auth_mode: 0 }] })?.networks[0]?.open).toBe(true);
  });

  test("formatUptime", () => {
    expect(formatUptime(3 * 3600 + 12 * 60 + 5)).toBe("3 h 12 min");
    expect(formatUptime(42 * 60)).toBe("42 min");
    expect(formatUptime(17)).toBe("17 s");
    expect(formatUptime(-4)).toBe("0 s");
  });
});

describe("SD card mapping (GET /sd_card/info)", () => {
  const info = { capacity: 30464, used_space_mb: 14540, is_present: true, is_mounted: true, name: "SD" };

  test("parseSdInfo keeps the documented fields and rejects bad types or negatives", () => {
    expect(parseSdInfo(info)).toEqual({ capacity: 30464, used_space_mb: 14540, is_present: true, is_mounted: true, unavailable: false });
    expect(parseSdInfo({ ...info, unavailable: true })?.unavailable).toBe(true);
    expect(parseSdInfo({ ...info, capacity: "30464" })).toBeNull();
    expect(parseSdInfo({ ...info, used_space_mb: -1 })).toBeNull();
    expect(parseSdInfo({ ...info, is_mounted: "yes" })).toBeNull();
    expect(parseSdInfo(null)).toBeNull();
  });

  test("no card, then not mounted, then the meter", () => {
    expect(sdView({ ...parseSdInfo(info)!, is_present: false })).toEqual({ kind: "no-card" });
    expect(sdView({ ...parseSdInfo(info)!, is_mounted: false })).toEqual({ kind: "not-mounted" });
    expect(sdView({ ...parseSdInfo(info)!, is_mounted: false, unavailable: true })).toEqual({ kind: "unavailable" });
    expect(sdView({ ...parseSdInfo(info)!, capacity: 0 })).toEqual({ kind: "not-mounted" });
    expect(sdView(parseSdInfo(info)!)).toEqual({ kind: "ok", usedGb: "14.2", totalGb: "29.8", percent: 48 });
  });

  test("the meter never passes 100 percent", () => {
    const over = sdView({ capacity: 1024, used_space_mb: 2048, is_present: true, is_mounted: true });
    expect(over).toMatchObject({ kind: "ok", percent: 100 });
  });
});

describe("join password and scan age", () => {
  test("a joined network needs a password of 8 to 63 bytes (blank is not a join)", () => {
    expect(checkJoinPassword("")).not.toBeNull();
    expect(checkJoinPassword("1234567")).not.toBeNull();
    expect(checkJoinPassword("12345678")).toBeNull();
    expect(checkJoinPassword("x".repeat(64))).not.toBeNull();
  });

  test("scan results are stale after 30 s, age never goes negative", () => {
    expect(scanAge(1000, 13000)).toEqual({ seconds: 12, stale: false });
    expect(scanAge(1000, 31000)).toEqual({ seconds: 30, stale: false });
    expect(scanAge(1000, 31001)).toEqual({ seconds: 30, stale: true });
    expect(scanAge(5000, 1000)).toEqual({ seconds: 0, stale: false });
  });
});

describe("hardware filter preview", () => {
  test("a full mask accepts exactly one id", () => {
    expect(hwFilterPreview("7E8", "7FF", false)).toEqual({ pattern: "111 1110 1000", count: 1 });
  });
  test("cleared mask bits are wildcards", () => {
    expect(hwFilterPreview("7E8", "7F8", false)).toEqual({ pattern: "111 1110 1xxx", count: 8 });
  });
  test("a zero mask accepts every id", () => {
    expect(hwFilterPreview("000", "000", false)).toEqual({ pattern: "xxx xxxx xxxx", count: 2048 });
  });
  test("extended ids use 29 bits", () => {
    const p = hwFilterPreview("18DAF100", "1FFFFF00", true);
    expect(p?.pattern.replaceAll(" ", "")).toHaveLength(29);
    expect(p?.count).toBe(256);
  });
  test("bad hex or out of range gives null", () => {
    expect(hwFilterPreview("7E8G", "7FF", false)).toBeNull();
    expect(hwFilterPreview("800", "7FF", false)).toBeNull();
    expect(hwFilterPreview("", "7FF", false)).toBeNull();
  });
});
