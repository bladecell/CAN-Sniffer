import { lstat, mkdir, open, readdir, rename, unlink } from "node:fs/promises";
import { constants } from "node:fs";
import { join, resolve } from "node:path";

const HOST = "127.0.0.1";
const PORT = 8787;
const MAX_BODY = 256 * 1024;
const MAX_LINE = 64 * 1024;
const MAX_ITEMS = 256;
const MAX_PENDING = 64;
const MAX_WS_MESSAGE = 32 * 1024;
const PROFILE_DIR = resolve(import.meta.dir, "../profiles");
const UI_DIST = resolve(import.meta.dir, "../ui/dist");
const UI_ORIGIN = `http://${HOST}:${PORT}`;
type Json = null | boolean | number | string | Json[] | { [key: string]: Json };
type Profile = {
  name: string; bitrate: number; ack: boolean;
  ecus: Array<{ name: string; reqId: number; respId: number; onUnknown?: string; clearable?: boolean; fault?: object; pids?: Array<Record<string, Json>>; dtcs?: Record<string, string[]>; vin?: string; dids?: Array<{ did: number; bytes: number[] }> }>;
  rules?: Array<Record<string, Json>>; cyclic?: Array<Record<string, Json>>;
  scenario?: Array<Record<string, Json>>;
};
type Pending = { resolve: (value: Record<string, Json>) => void; timer: ReturnType<typeof setTimeout> };
type Linear = { offset: number; coefficients: number[] };
const pending = new Map<number, Pending>();
const sockets = new Set<WebSocket>();
let portPath = "";
let connected = false;
let deviceReady = false;
let serialReader: ReadableStreamDefaultReader<Uint8Array> | undefined;
let seq = 1;
let status: Record<string, Json> = { port: "", connected: false, stats: {} };
let scenarioTimer: ReturnType<typeof setTimeout> | undefined;
let scenarioGeneration = 0;

function object(value: unknown, label: string): Record<string, unknown> {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error(`${label} must be an object`);
  return value as Record<string, unknown>;
}
function integer(value: unknown, min: number, max: number, label: string): number {
  if (typeof value !== "number" || !Number.isSafeInteger(value) || value < min || value > max) throw new Error(`${label} out of range`);
  return value;
}
function boundedString(value: unknown, max: number, label: string): string {
  if (typeof value !== "string" || value.length > max) throw new Error(`${label} invalid`);
  return value;
}
function bytes(value: unknown, label: string, max = 4096): number[] {
  if (!Array.isArray(value) || value.length > max) throw new Error(`${label} must be a bounded byte array`);
  return value.map((v, i) => integer(v, 0, 255, `${label}[${i}]`));
}
function linearFormula(source: unknown): Linear | undefined {
  if (typeof source !== "string" || source.length === 0 || source.length > 1024) return undefined;
  const tokens = source.match(/\s*(?:0[xX][0-9a-fA-F]+|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|[A-Da-d]|[()+*/-])/g);
  if (!tokens || tokens.join("").replace(/\s/g, "") !== source.replace(/\s/g, "")) return undefined;
  const values = tokens.map((token) => token.trim());
  let pos = 0;
  const constant = (): Linear => ({ offset: 0, coefficients: [0, 0, 0, 0] });
  const combine = (a: Linear, b: Linear, sign: number): Linear => ({ offset: a.offset + sign * b.offset, coefficients: a.coefficients.map((v, i) => v + sign * b.coefficients[i]!) });
  const scale = (a: Linear, n: number): Linear => ({ offset: a.offset * n, coefficients: a.coefficients.map((v) => v * n) });
  const primary = (): Linear | undefined => {
    const token = values[pos++];
    if (token === undefined) return undefined;
    if (token === "+" || token === "-") { const item = primary(); return item && (token === "-" ? scale(item, -1) : item); }
    if (token === "(") { const item = expression(); if (values[pos++] !== ")") return undefined; return item; }
    if (/^[A-D]$/i.test(token)) { const item = constant(); item.coefficients[token.toUpperCase().charCodeAt(0) - 65] = 1; return item; }
    if (/^(?:0[xX][0-9a-fA-F]+|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)$/.test(token)) {
      const n = Number(token); if (!Number.isFinite(n)) return undefined;
      const item = constant(); item.offset = n; return item;
    }
    return undefined;
  };
  const term = (): Linear | undefined => {
    let left = primary(); if (!left) return undefined;
    while (values[pos] === "*" || values[pos] === "/") {
      const op = values[pos++]!; const right = primary(); if (!right) return undefined;
      const leftHasVars = left.coefficients.some((x) => x !== 0), rightHasVars = right.coefficients.some((x) => x !== 0);
      if (op === "*" && leftHasVars && rightHasVars) return undefined;
      if (op === "/" && (rightHasVars || right.offset === 0)) return undefined;
      left = op === "*" ? (rightHasVars ? scale(right, left.offset) : scale(left, right.offset)) : scale(left, 1 / right.offset);
      if (!Number.isFinite(left.offset) || left.coefficients.some((x) => !Number.isFinite(x))) return undefined;
    }
    return left;
  };
  const expression = (): Linear | undefined => {
    let left = term(); if (!left) return undefined;
    while (values[pos] === "+" || values[pos] === "-") { const op = values[pos++]!; const right = term(); if (!right) return undefined; left = combine(left, right, op === "+" ? 1 : -1); }
    return left;
  };
  const result = expression();
  return result && pos === values.length ? result : undefined;
}
function rawEncoding(entry: Record<string, Json>): { scale: number; offset: number; len: number } | undefined {
  const mode = integer(entry.mode, 0, 255, "mode");
  const formula = entry.formula;
  const rawLen = entry.rawLen ?? (typeof formula === "string" ? formulaByteCount(formula) : undefined);
  const len = integer(rawLen, 1, 4, "raw byte count");
  const linear = linearFormula(entry.formula);
  if (!linear) return undefined;
  for (let i = len; i < 4; i++) if (linear.coefficients[i] !== 0) return undefined;
  const scale = linear.coefficients[len - 1]!;
  if (!Number.isFinite(scale) || scale === 0) return undefined;
  for (let i = 0; i < len; i++) if (Math.abs(linear.coefficients[i]! - scale * 256 ** (len - i - 1)) > Math.max(1, Math.abs(scale)) * 1e-10) return undefined;
  return { scale, offset: linear.offset, len };
}
function formulaByteCount(formula: string): number {
  const variables = [...formula.matchAll(/\b([A-D])\b/gi)].map((match) => match[1]!.toUpperCase().charCodeAt(0) - 65);
  return variables.length === 0 ? 1 : Math.max(...variables) + 1;
}
async function readHandleBounded(handle: Awaited<ReturnType<typeof open>>, label: string): Promise<string> {
  const buffer = Buffer.alloc(MAX_BODY + 1);
  let total = 0;
  while (total < buffer.length) {
    const { bytesRead } = await handle.read(buffer, total, buffer.length - total, total);
    if (bytesRead === 0) break;
    total += bytesRead;
  }
  if (total > MAX_BODY) throw new Error(`${label} too large`);
  return buffer.subarray(0, total).toString("utf8");
}
async function verifyProfileDir(): Promise<void> {
  const stat = await lstat(PROFILE_DIR);
  if (!stat.isDirectory() || stat.isSymbolicLink() || (typeof process.getuid === "function" && stat.uid !== process.getuid()) || (stat.mode & 0o022) !== 0 || (stat.mode & 0o200) === 0) {
    throw new Error("profile directory must be an owner-controlled, non-symlink writable directory");
  }
}
function validateProfile(value: unknown): Profile {
  const p = object(value, "profile");
  const ecus = p.ecus;
  if (!Array.isArray(ecus) || ecus.length > 16) throw new Error("ecus must contain at most 16 entries");
  if (p.rules !== undefined && (!Array.isArray(p.rules) || p.rules.length > MAX_ITEMS)) throw new Error("too many rules");
  if (p.cyclic !== undefined && (!Array.isArray(p.cyclic) || p.cyclic.length > MAX_ITEMS)) throw new Error("too many cyclic frames");
  if (p.scenario !== undefined && (!Array.isArray(p.scenario) || p.scenario.length > 1024)) throw new Error("too many scenario steps");
  const bitrate = integer(p.bitrate, 125000, 1000000, "bitrate");
  if (![125000, 250000, 500000, 1000000].includes(bitrate)) throw new Error("unsupported bitrate");
  if (typeof p.ack !== "boolean") throw new Error("ack must be boolean");
  for (const raw of ecus) {
    const e = object(raw, "ECU");
    boundedString(e.name, 64, "ECU name");
    integer(e.reqId, 0, 0x7ff, "reqId"); integer(e.respId, 0, 0x7ff, "respId");
    if (e.pids !== undefined && (!Array.isArray(e.pids) || e.pids.length > MAX_ITEMS)) throw new Error("too many PIDs");
    if (e.vin !== undefined) boundedString(e.vin, 17, "VIN");
    if (e.dids !== undefined) {
      if (!Array.isArray(e.dids) || e.dids.length > MAX_ITEMS) throw new Error("too many DIDs");
      for (const did of e.dids) { const d = object(did, "DID"); integer(d.did, 0, 0xffff, "DID"); bytes(d.bytes, "DID bytes"); }
    }
  }
  return p as unknown as Profile;
}
function broadcast(value: Record<string, Json>): void {
  const text = JSON.stringify(value);
  for (const ws of sockets) if (ws.readyState === WebSocket.OPEN) ws.send(text);
}
function serialLine(line: string): void {
  if (line.length > MAX_LINE) return;
  let message: Record<string, unknown>;
  try { message = object(JSON.parse(line), "serial message"); } catch { return; }
  if (message.ev === "frame" || message.ev === "drop" || message.ev === "boot") {
    broadcast(message as Record<string, Json>);
    return;
  }
  if (typeof message.seq === "number") {
    const slot = pending.get(message.seq);
    if (slot) { clearTimeout(slot.timer); pending.delete(message.seq); slot.resolve(message as Record<string, Json>); }
  }
}
async function readSerial(path: string): Promise<void> {
  while (portPath === path) {
    try {
      const stream = Bun.file(path).stream();
      serialReader = stream.getReader(); connected = true; status = { ...status, connected: true }; broadcast({ ev: "connection", connected: true });
      void initializeDevice();
      let buffer = "";
      while (portPath === path) {
        const { value, done } = await serialReader.read();
        if (done) break;
        buffer += new TextDecoder().decode(value, { stream: true });
        if (buffer.length > MAX_LINE) { buffer = ""; continue; }
        let end = buffer.indexOf("\n");
        while (end >= 0) { serialLine(buffer.slice(0, end).trim()); buffer = buffer.slice(end + 1); end = buffer.indexOf("\n"); }
      }
    } catch { /* unplugged or unavailable; reconnect below */ }
    connected = false; deviceReady = false; serialReader = undefined; status = { ...status, connected: false }; broadcast({ ev: "connection", connected: false });
    await new Promise((resolveDelay) => setTimeout(resolveDelay, 1000));
  }
}
async function initializeDevice(): Promise<void> {
  try {
    const hello = await command("hello");
    if (hello.ok !== true) throw new Error("simulator did not confirm hello");
    deviceReady = true;
    if (process.env.SIM_DEFAULT_PROFILE) await pushProfile(await loadProfile(process.env.SIM_DEFAULT_PROFILE));
    await command("stream", { on: true, maxPerSec: integer(Number(process.env.SIM_STREAM_MAX_PER_SEC ?? 50), 1, 10000, "SIM_STREAM_MAX_PER_SEC") });
    broadcast({ ev: "ready" });
  } catch (error) { broadcast({ ev: "device-error", error: error instanceof Error ? error.message : "handshake failed" }); }
}
async function command(cmd: string, data: Record<string, Json> = {}): Promise<Record<string, Json>> {
  if (!connected || !portPath) throw new Error("simulator serial port is disconnected");
  if (!deviceReady && cmd !== "hello") throw new Error("simulator handshake is not complete");
  if (pending.size >= MAX_PENDING) throw new Error("too many pending serial commands");
  const id = seq++;
  if (seq > 0x7fffffff) seq = 1;
  const reply = new Promise<Record<string, Json>>((resolveReply, reject) => {
    const timer = setTimeout(() => { pending.delete(id); reject(new Error(`timeout waiting for ${cmd}`)); }, 5000);
    pending.set(id, { resolve: resolveReply, timer });
  });
  try {
    await Bun.write(portPath, `${JSON.stringify({ ...data, cmd, seq: id })}\n`);
    const result = await reply;
    if (result.ok === false) throw new Error(String(result.err ?? `${cmd} failed`));
    return result;
  } catch (error) { const item = pending.get(id); if (item) clearTimeout(item.timer); pending.delete(id); throw error; }
}
const builtin: Record<string, { pid: number; len: number; scale: number; offset: number }> = {
  "04": { pid: 4, len: 1, scale: 100 / 255, offset: 0 }, "05": { pid: 5, len: 1, scale: 1, offset: -40 },
  "0C": { pid: 12, len: 2, scale: 0.25, offset: 0 }, "0D": { pid: 13, len: 1, scale: 1, offset: 0 },
  "0F": { pid: 15, len: 1, scale: 1, offset: -40 }, "11": { pid: 17, len: 1, scale: 100 / 255, offset: 0 },
};
function pidCommands(entry: Record<string, Json>, ecu: string): Array<[string, Record<string, Json>]> {
  let mode = integer(entry.mode ?? 1, 0, 255, "PID mode");
  let pid: number;
  let raw = entry.raw;
  let imported: ReturnType<typeof rawEncoding>;
  if (typeof entry.pid === "string") {
    const key = entry.pid.replace(/^0x/i, "").toUpperCase();
    if (!/^[0-9A-F]{1,4}$/.test(key)) throw new Error("PID must be hexadecimal");
    pid = integer(parseInt(key, 16), 0, 0xffff, "PID");
    imported = entry.formula !== undefined ? rawEncoding(entry) : undefined;
    mode = 1;
  } else {
    pid = integer(entry.pid, 0, 0xffff, "PID");
    imported = entry.formula !== undefined ? rawEncoding(entry) : undefined;
  }
  const builtinDef = mode === 1 ? builtin[pid.toString(16).toUpperCase().padStart(2, "0")] : undefined;
  const data: Record<string, Json> = { ecu, mode, pid };
  const len = integer(entry.len ?? entry.length ?? imported?.len ?? builtinDef?.len ?? (mode === 0x22 ? 3 : 1), 1, 32, "PID len");
  if (entry.rawOnly === true && raw === undefined) raw = Array.from({ length: len }, () => 0) as Json;
  if (raw !== undefined) data.raw = bytes(raw, "PID raw", 32) as Json;
  else if (entry.value !== undefined) {
    if (typeof entry.value !== "number" || !Number.isFinite(entry.value)) throw new Error("PID value must be finite");
    if (len > 8) throw new Error("numeric PID sources support at most 8 bytes");
    data.value = entry.value;
  } else if (entry.gen !== undefined) {
    if (len > 8) throw new Error("generated PID sources support at most 8 bytes");
    data.gen = object(entry.gen, "PID generator") as Json;
  } else if (imported) {
    if (len > 8) throw new Error("numeric PID sources support at most 8 bytes");
    data.value = imported.offset;
  }
  else throw new Error("PID requires value, raw, or gen");
  const scale = typeof entry.scale === "number" ? entry.scale : imported?.scale ?? builtinDef?.scale ?? 1;
  const offset = typeof entry.offset === "number" ? entry.offset : imported?.offset ?? builtinDef?.offset ?? 0;
  if (!Number.isFinite(scale) || scale === 0 || Math.abs(scale) < 1e-30 || !Number.isFinite(offset)) throw new Error("invalid PID scale or offset");
  return [["pid.add", { ...data, len, scale, offset }]];
}
function valueBytes(value: number, scale: number, offset: number, len: number): number[] {
  if (!Number.isFinite(value) || !Number.isFinite(scale) || scale === 0 || !Number.isFinite(offset)) throw new Error("value has no finite raw encoding");
  const raw = Math.round((value - offset) / scale);
  if (!Number.isSafeInteger(raw) || raw < 0 || raw >= 256 ** len) throw new Error("value is outside the raw PID range");
  return Array.from({ length: len }, (_, i) => Math.floor(raw / 256 ** (len - i - 1)) % 256);
}
async function importPidDefinitions(profileName: string, ecuName: string, source?: string): Promise<{ imported: number; rawOnly: string[] }> {
  const profile = await loadProfile(profileName);
  const ecu = profile.ecus.find((item) => item.name === ecuName);
  if (!ecu) throw new Error("ECU not found in profile");
  const file = source ?? process.env.PID_DEF_PATH ?? join(import.meta.dir, "../../../firmware/pid_def.json");
  const fileHandle = await open(file, constants.O_RDONLY | constants.O_NOFOLLOW);
  let text: string;
  try {
    const stat = await fileHandle.stat();
    if (!stat.isFile() || stat.size > MAX_BODY) throw new Error("PID definition file must be regular and within size limit");
    text = await readHandleBounded(fileHandle, "PID definition file");
  } finally { await fileHandle.close(); }
  const definitions: unknown = JSON.parse(text);
  if (!Array.isArray(definitions) || definitions.length > MAX_ITEMS) throw new Error("PID definition root must be an array of at most 256 entries");
  const rawOnly: string[] = [];
  const imported = definitions.map((value, index) => {
    const d = object(value, `PID definition ${index}`);
    const mode = integer(d.mode, 0, 255, "mode");
    const pid = integer(d.pid, 0, 0xffff, "pid");
    if (mode !== 1 && mode !== 0x22) return undefined;
    const requestLength = integer(d.length ?? d.len, mode === 0x22 ? 3 : 2, mode === 0x22 ? 3 : 2, "request length");
    const formula = boundedString(d.formula, 1024, "formula");
    const name = boundedString(d.name, 64, "name");
    const rawLen = formulaByteCount(formula);
    const encoding = rawEncoding({ mode, rawLen, formula });
    if (!encoding) rawOnly.push(`${name} (mode ${mode}, PID 0x${pid.toString(16).toUpperCase()})`);
    return { mode, pid, len: encoding?.len ?? rawLen, requestLength, name, unit: boundedString(d.unit ?? "", 32, "unit"), formula, rawOnly: !encoding } as unknown as Record<string, Json>;
  }).filter((entry): entry is Record<string, Json> => entry !== undefined);
  if (imported.length + (ecu.pids?.length ?? 0) > MAX_ITEMS) throw new Error("import exceeds ECU PID limit");
  ecu.pids = [...(ecu.pids ?? []), ...imported];
  await saveProfile(profileName, profile);
  for (const warning of rawOnly) console.warn(`PID formula is not linear in big-endian raw bytes; engineering writes disabled: ${warning}`);
  return { imported: imported.length, rawOnly };
}
async function engineeringPidWrite(data: Record<string, Json>): Promise<Record<string, Json>> {
  if (typeof data.value !== "number") return data;
  const ecuName = boundedString(data.ecu, 64, "ecu");
  const mode = integer(data.mode ?? 1, 0, 255, "mode");
  const pidValue = typeof data.pid === "string" ? parseInt(data.pid.replace(/^0x/i, ""), 16) : integer(data.pid, 0, 0xffff, "pid");
  const profileName = typeof data.profile === "string" ? boundedString(data.profile, 64, "profile") : process.env.SIM_DEFAULT_PROFILE;
  const profile = profileName ? await loadProfile(profileName) : undefined;
  const ecu = profile?.ecus.find((item) => item.name === ecuName);
  const entry = ecu?.pids?.find((item) => item.mode === mode && (typeof item.pid === "string" ? parseInt(item.pid, 16) : item.pid) === pidValue);
  const builtinDef = mode === 1 && typeof data.pid === "string" ? builtin[data.pid.replace(/^0x/i, "").toUpperCase().padStart(2, "0")] : undefined;
  const encoding = entry?.formula !== undefined ? rawEncoding(entry) : builtinDef;
  if (!encoding) throw new Error("PID has no supported linear formula; send raw bytes instead");
  return { ecu: ecuName, mode, pid: pidValue, raw: valueBytes(data.value, encoding.scale, encoding.offset, encoding.len) as unknown as Json };
}
async function pushProfile(profile: Profile): Promise<void> {
  const commands: Array<[string, Record<string, Json>]> = [["can.config", { bitrate: profile.bitrate, ack: profile.ack }]];
  for (const e of profile.ecus) {
    commands.push(["ecu.add", { name: e.name, reqId: e.reqId, respId: e.respId, onUnknown: e.onUnknown ?? "silent", clearable: e.clearable ?? true }]);
    if (e.fault) commands.push(["ecu.fault", { name: e.name, ...object(e.fault, "ECU fault") as Record<string, Json> }]);
    for (const pid of e.pids ?? []) commands.push(...pidCommands(pid, e.name));
    for (const [kind, codes] of Object.entries(e.dtcs ?? {})) {
      if (!["stored", "pending", "permanent"].includes(kind) || !Array.isArray(codes) || codes.length > 256) throw new Error("invalid DTC list");
      if (codes.some((code) => typeof code !== "string" || !/^[PCBU][0-3][0-9A-F]{3}$/i.test(code))) throw new Error("invalid DTC code");
      commands.push(["dtc.set", { ecu: e.name, kind, codes: codes as unknown as Json }]);
    }
    if (e.vin) commands.push(["vin.set", { ecu: e.name, vin: boundedString(e.vin, 17, "VIN") }]);
    for (const d of e.dids ?? []) commands.push(["did.set", { ecu: e.name, did: integer(d.did, 0, 0xffff, "DID"), bytes: bytes(d.bytes, "DID bytes") as Json }]);
  }
  for (const rule of profile.rules ?? []) commands.push(["rule.add", object(rule, "rule") as Record<string, Json>]);
  for (const frame of profile.cyclic ?? []) commands.push(["cyclic.add", object(frame, "cyclic frame") as Record<string, Json>]);
  await command("reset");
  for (const [cmd, data] of commands) await command(cmd, data);
  await command("save");
}
async function loadProfile(name: string): Promise<Profile> {
  if (!/^[A-Za-z0-9_-]{1,64}$/.test(name)) throw new Error("invalid profile name");
  const handle = await open(join(PROFILE_DIR, `${name}.json`), constants.O_RDONLY | constants.O_NOFOLLOW);
  let data: string;
  try {
    const stat = await handle.stat();
    if (!stat.isFile() || stat.size > MAX_BODY) throw new Error("profile must be a regular file within size limit");
    data = await readHandleBounded(handle, "profile");
  } finally { await handle.close(); }
  return validateProfile(JSON.parse(data));
}
async function bodyJson(request: Request): Promise<unknown> {
  const len = Number(request.headers.get("content-length") ?? 0);
  if (len > MAX_BODY) throw new Error("request body too large");
  if (!request.body) return {};
  const reader = request.body.getReader();
  const chunks: Uint8Array[] = [];
  let total = 0;
  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      total += value.byteLength;
      if (total > MAX_BODY) { await reader.cancel(); throw new Error("request body too large"); }
      chunks.push(value);
    }
  } finally { reader.releaseLock(); }
  const buffer = new Uint8Array(total);
  let offset = 0;
  for (const chunk of chunks) { buffer.set(chunk, offset); offset += chunk.byteLength; }
  return JSON.parse(new TextDecoder().decode(buffer));
}
async function saveProfile(name: string, profile: Profile): Promise<void> {
  if (!/^[A-Za-z0-9_-]{1,64}$/.test(name)) throw new Error("invalid profile name");
  const target = join(PROFILE_DIR, `${name}.json`);
  const temp = join(PROFILE_DIR, `.${name}.${crypto.randomUUID()}.tmp`);
  const contents = `${JSON.stringify(profile, null, 2)}\n`;
  if (Buffer.byteLength(contents) > MAX_BODY) throw new Error("serialized profile too large");
  const handle = await open(temp, "wx", 0o600);
  try { await handle.writeFile(contents); await handle.sync(); }
  catch (error) { await handle.close(); await unlink(temp).catch(() => undefined); throw error; }
  await handle.close();
  try { await rename(temp, target); }
  catch (error) { await unlink(temp).catch(() => undefined); throw error; }
}
function response(value: unknown, code = 200): Response { return Response.json(value, { status: code }); }
function hasFetchMetadata(request: Request): boolean {
  return ["sec-fetch-site", "sec-fetch-mode", "sec-fetch-dest"].some((header) => request.headers.has(header));
}
function originAllowed(request: Request, websocket = false): boolean {
  const origin = request.headers.get("origin");
  const site = request.headers.get("sec-fetch-site");
  const isTopLevelNavigation = !websocket && request.method === "GET" && new URL(request.url).pathname === "/" && request.headers.get("sec-fetch-mode") === "navigate" && request.headers.get("sec-fetch-dest") === "document";
  if (site === "cross-site") return false;
  if (origin !== null && origin !== UI_ORIGIN) return false;
  if (websocket && hasFetchMetadata(request) && origin !== UI_ORIGIN) return false;
  if (origin !== null && !websocket && !["GET", "HEAD"].includes(request.method) && request.headers.get("x-simulator-intent") !== "ui") return false;
  if (origin === null && hasFetchMetadata(request) && !(site === "same-origin" && ["GET", "HEAD"].includes(request.method)) && !(site === "none" && isTopLevelNavigation)) return false;
  return true;
}
async function serveUi(pathname: string): Promise<Response> {
  const asset = pathname === "/" ? "index.html" : /^\/assets\/[A-Za-z0-9_-]+\.(?:js|css|svg|woff2?)$/.test(pathname) ? pathname.slice(1) : undefined;
  if (!asset) return response({ error: "not found" }, 404);
  try {
    const file = Bun.file(join(UI_DIST, asset));
    if (!(await file.exists())) return response({ error: "not found" }, 404);
    const type = asset.endsWith(".html") ? "text/html; charset=utf-8" : asset.endsWith(".js") ? "text/javascript; charset=utf-8" : asset.endsWith(".css") ? "text/css; charset=utf-8" : asset.endsWith(".svg") ? "image/svg+xml" : "font/woff2";
    return new Response(file, { headers: { "Content-Type": type, "Content-Security-Policy": "default-src 'self'; connect-src 'self' ws://127.0.0.1:8787; img-src 'self' data:; style-src 'self' 'unsafe-inline'; font-src 'self'; object-src 'none'; base-uri 'self'; frame-ancestors 'none'", "X-Content-Type-Options": "nosniff", "X-Frame-Options": "DENY" } });
  } catch { return response({ error: "UI unavailable" }, 404); }
}
function restAction(url: string, data: Record<string, Json>): { cmd: string; payload: Record<string, Json> } {
  const fields = url.split("/").filter(Boolean);
  if (url === "/pid") {
    const payload: Record<string, Json> = { ecu: boundedString(data.ecu, 64, "ecu") };
    if (data.profile !== undefined) payload.profile = boundedString(data.profile, 64, "profile");
    if (data.mode !== undefined) payload.mode = integer(data.mode, 0, 255, "mode");
    if (data.pid !== undefined) {
      if (typeof data.pid === "string") {
        if (!/^(?:0x)?[0-9a-fA-F]{1,4}$/i.test(data.pid)) throw new Error("invalid PID");
        payload.pid = data.pid;
      } else payload.pid = integer(data.pid, 0, 0xffff, "pid");
    } else throw new Error("pid is required");
    if (data.value !== undefined) {
      if (typeof data.value !== "number" || !Number.isFinite(data.value)) throw new Error("value must be finite");
      payload.value = data.value;
    }
    if (data.raw !== undefined) payload.raw = bytes(data.raw, "raw", 8) as Json;
    if (data.gen !== undefined) payload.gen = object(data.gen, "generator") as Json;
    if (payload.value === undefined && payload.raw === undefined && payload.gen === undefined) throw new Error("value, raw, or gen is required");
    return { cmd: "pid.set", payload };
  }
  if (url === "/fault") {
    const payload: Record<string, Json> = { name: boundedString(data.name, 64, "name") };
    for (const key of ["silent"] as const) if (data[key] !== undefined) { if (typeof data[key] !== "boolean") throw new Error(`${key} must be boolean`); payload[key] = data[key]; }
    for (const key of ["delayMs", "nrc", "pendingMs"] as const) if (data[key] !== undefined) payload[key] = integer(data[key], 0, key === "nrc" ? 255 : 60000, key);
    return { cmd: "ecu.fault", payload };
  }
  if (url === "/dtc") {
    if (!["stored", "pending", "permanent"].includes(String(data.kind)) || !Array.isArray(data.codes) || data.codes.length > 256 || data.codes.some((code) => typeof code !== "string" || !/^[PCBU][0-3][0-9A-F]{3}$/i.test(code))) throw new Error("invalid DTC list");
    return { cmd: "dtc.set", payload: { ecu: boundedString(data.ecu, 64, "ecu"), kind: data.kind, codes: data.codes as Json } };
  }
  if (url === "/vin") return { cmd: "vin.set", payload: { ecu: boundedString(data.ecu, 64, "ecu"), vin: boundedString(data.vin, 17, "VIN") } };
  if (url === "/ack") {
    if (typeof data.ack !== "boolean") throw new Error("ack must be boolean");
    return { cmd: "can.config", payload: { ack: data.ack } };
  }
  if (fields[0] === "scenario" && fields[1] === "start") return { cmd: "scenario.start", payload: {} };
  throw new Error("unknown action");
}
async function scenarioStart(profileName: string): Promise<void> {
  const profile = await loadProfile(profileName);
  const steps = profile.scenario ?? [];
  if (steps.length === 0) throw new Error("scenario must contain at least one step");
  scenarioStop();
  const generation = scenarioGeneration;
  let at = 0;
  let previousAt = 0;
  const run = (): void => {
    if (at >= steps.length) { broadcast({ ev: "scenario", state: "complete" }); return; }
    const step = steps[at++]!;
    const absoluteAt = integer(step.atMs, 0, 86_400_000, "scenario atMs");
    if (absoluteAt < previousAt) throw new Error("scenario steps must be sorted by atMs");
    const delay = absoluteAt - previousAt;
    previousAt = absoluteAt;
    scenarioTimer = setTimeout(async () => {
      try {
        if (generation !== scenarioGeneration) return;
        const action = object(step, "scenario step");
        const kind = typeof action.set === "object" ? "pid" : typeof action.fault === "object" ? "fault" : typeof action.dtc === "object" ? "dtc" : "";
        const payload = object(action[kind], "scenario action") as Record<string, Json>;
        const mapping: Record<string, string> = { pid: "pid.set", fault: "ecu.fault", dtc: "dtc.set" };
        const commandPayload = kind === "pid" ? await engineeringPidWrite({ ...payload, profile: profileName }) : payload;
        await command(mapping[kind]!, commandPayload);
        if (generation !== scenarioGeneration) return;
        broadcast({ ev: "scenario", index: at - 1, state: "step" }); run();
      } catch (error) { broadcast({ ev: "scenario", state: "error", error: String(error) }); }
    }, delay);
  };
  broadcast({ ev: "scenario", state: "started", count: steps.length }); run();
}
function scenarioStop(): void { scenarioGeneration++; if (scenarioTimer) clearTimeout(scenarioTimer); scenarioTimer = undefined; broadcast({ ev: "scenario", state: "stopped" }); }

await mkdir(PROFILE_DIR, { recursive: true, mode: 0o700 });
await verifyProfileDir();
const server = Bun.serve({
  hostname: HOST, port: PORT,
  maxRequestBodySize: MAX_BODY,
  fetch: async (request, serverInstance) => {
    const url = new URL(request.url);
    if (request.headers.get("host") !== `${HOST}:${PORT}`) return response({ error: "invalid Host header" }, 403);
    if (!originAllowed(request, url.pathname === "/ws")) return response({ error: "request origin or browser metadata denied" }, 403);
    try { await verifyProfileDir(); }
    catch (error) { return response({ error: error instanceof Error ? error.message : "profile directory unavailable" }, 500); }
    if (url.pathname === "/ws" && serverInstance.upgrade(request, { data: { browser: request.headers.get("origin") === UI_ORIGIN } })) return undefined;
    try {
      if (["GET", "HEAD"].includes(request.method) && (url.pathname === "/" || url.pathname.startsWith("/assets/"))) return await serveUi(url.pathname);
      if (url.pathname === "/status" && request.method === "GET") {
        if (connected) {
          try { const stats = await command("stats"); status = { ...status, stats }; }
          catch { /* retain the last known statistics while a device is unavailable */ }
        }
        return response({ ...status, port: portPath, connected });
      }
      if (url.pathname === "/profiles" && request.method === "GET") return response({ profiles: (await readdir(PROFILE_DIR)).filter((f) => f.endsWith(".json")).map((f) => f.slice(0, -5)) });
      const match = /^\/profiles\/([A-Za-z0-9_-]{1,64})(?:\/(push))?$/.exec(url.pathname);
      if (match) {
        const [, name, push] = match;
        if (push === "push") {
          if (request.method === "POST") { await pushProfile(await loadProfile(name!)); return response({ ok: true }); }
          return response({ error: "method not allowed" }, 405);
        }
        if (request.method === "GET") return response(await loadProfile(name!));
        if (request.method === "DELETE") { await unlink(join(PROFILE_DIR, `${name}.json`)); return response({ ok: true }); }
        if (request.method === "PUT") { const profile = validateProfile(await bodyJson(request)); await saveProfile(name!, profile); return response(profile, 201); }
        if (request.method === "POST") {
          const data = object(await bodyJson(request), "request");
          if (data.action !== "import-pids") return response({ error: "unknown profile action" }, 400);
          return response(await importPidDefinitions(name!, boundedString(data.ecu, 64, "ecu")));
        }
        return response({ error: "method not allowed" }, 405);
      }
      if (request.method === "POST" && url.pathname === "/scenario/start") { const data = object(await bodyJson(request), "request"); await scenarioStart(boundedString(data.profile, 64, "profile")); return response({ ok: true }); }
      if (request.method === "POST" && url.pathname === "/scenario/stop") { scenarioStop(); return response({ ok: true }); }
      if (request.method === "POST" && ["/pid", "/fault", "/dtc", "/vin", "/ack"].includes(url.pathname)) {
        const data = object(await bodyJson(request), "request") as Record<string, Json>;
        const action = restAction(url.pathname, data);
        if (action.cmd === "can.config") {
          // Firmware retains fields omitted from can.config; sending bitrate here could overwrite a live setting.
        }
        const payload = url.pathname === "/pid" ? await engineeringPidWrite(action.payload) : action.payload;
        const reply = await command(action.cmd, payload); broadcast({ ev: "action", cmd: action.cmd, data }); return response(reply);
      }
      return response({ error: "not found" }, 404);
    } catch (error) { return response({ error: error instanceof Error ? error.message : "request failed" }, 400); }
  },
  websocket: {
    data: {} as { browser: boolean },
    maxPayloadLength: MAX_WS_MESSAGE,
    maxBackpressure: MAX_WS_MESSAGE,
    open(ws) { sockets.add(ws); ws.send(JSON.stringify({ ev: "status", ...status, connected })); },
    close(ws) { sockets.delete(ws); },
    async message(ws, message) {
      try {
        if (ws.data.browser) throw new Error("browser WebSocket is read-only");
        if (typeof message !== "string" || Buffer.byteLength(message) > MAX_WS_MESSAGE) throw new Error("message too large");
        const data = object(JSON.parse(message), "message") as Record<string, Json>;
        if (data.action === "scenario.start") { await scenarioStart(boundedString(data.profile, 64, "profile")); return; }
        if (data.action === "scenario.stop") { scenarioStop(); return; }
        const url = `/${String(data.action ?? "")}`;
        const action = restAction(url, data);
        if (action.cmd === "can.config") {
          // Omitted can.config fields must be preserved by the firmware.
        }
        const payload = url === "/pid" ? await engineeringPidWrite(action.payload) : action.payload;
        ws.send(JSON.stringify(await command(action.cmd, payload)));
      } catch (error) { ws.send(JSON.stringify({ error: error instanceof Error ? error.message : "action failed" })); }
    },
  },
});
if (process.env.SIM_PORT) {
  portPath = resolve(process.env.SIM_PORT);
  status = { ...status, port: portPath };
  void readSerial(portPath);
}
console.log(`simulator host listening on http://${HOST}:${server.port}`);
console.log("Set SIM_PORT to the USB-Serial-JTAG device path to connect.");
