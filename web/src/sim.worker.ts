// Runs the Rust simulator (WASM) off the main thread at a fixed step and
// streams snapshots to the page.

import wasmUrl from './wasm/swarm.wasm?url';
import type { FromWorker, ToWorker } from './protocol';
import { C, CHATTER_STRIDE, ChatterKind, TL } from './chatter';
import { CHANNEL_LEN, LEGACY_CHANNEL_LEN, packageSeq } from './telemetry';
import { LEGACY_VERSION, VERSION } from './snapshot';

const STEP_MS = 50;

interface SwarmExports {
  memory: WebAssembly.Memory;
  sim_new(seed: number, drones: number): number;
  sim_new_preset(preset: number, seed: number, drones: number): number;
  sim_preset(sim: number): number;
  sim_chatter_ptr(sim: number): number;
  sim_chatter_len(sim: number): number;
  sim_step(sim: number, dtMs: number): void;
  sim_snapshot_ptr(sim: number): number;
  sim_snapshot_len(sim: number): number;
  sim_terrain_ptr(sim: number): number;
  sim_terrain_size(sim: number): number;
  sim_terrain_extent(sim: number): number;
  sim_kill(sim: number, id: number): number;
  sim_command(sim: number, intent: number, x: number, z: number): number;
  sim_set_loss(sim: number, loss: number): void;
  sim_ignite(sim: number, x: number, z: number): void;
  sim_fire_ptr(sim: number): number;
  sim_fire_size(sim: number): number;
  sim_set_channel(sim: number, enabled: number, externalBusy: number): void;
  sim_set_telemetry(sim: number, enabled: number, variant: number, rateMode: number, hz: number, routing: number): number;
  sim_telemetry_ptr(sim: number): number;
  sim_telemetry_len(sim: number): number;
  sim_channel_ptr(sim: number): number;
  sim_channel_len(sim: number): number;
  sim_tlm_sample_ptr(sim: number, id: number): number;
  sim_tlm_sample_len(sim: number, id: number): number;
}

/** The fire changes slowly; stream it at most this often. */
const FIRE_INTERVAL_MS = 250;
/** Radio chatter is batched: one message per this interval at most. */
const CHATTER_INTERVAL_MS = 100;
/** Telemetry stats at most 5 Hz. */
const TLMSTATS_INTERVAL_MS = 200;

let wasm: SwarmExports | null = null;
let sim = 0;
let lastFire: Uint8Array | null = null;
let fireAt = -Infinity;
/** Chatter floats since the last post; grown by doubling, reused across posts. */
let chatterBuf = new Float32Array(4096);
let chatterLen = 0;
let chatterAt = -Infinity;
/** Sim steps per tick; see the protocol's `speed` message. */
let speed = 1;
const MAX_SPEED = 8;
/** Snapshot header offsets of the per-step counters summed across substeps (see snapshot.ts). */
const COUNTERS = [6, 7, 8] as const;
const counterSums = new Float64Array(COUNTERS.length);
/** Channel / telemetry models on (stats are only streamed while one is). */
let channelOn = false;
let telemetryOn = false;
let tlmStatsAt = -Infinity;
let tlmSampleSub = false;
const utf8 = new TextDecoder();
/**
 * Layout checks, once per session: a WASM built before snapshot v3 / the 28-float
 * channel summary still runs (links read with quality 1, the page falls back to
 * brain-only byte counts), so only an unknown layout is an error.
 */
let layoutChecked = false;
/** Floats in the channel summary the WASM exports (CHANNEL_LEN, or LEGACY_CHANNEL_LEN before v3). */
let channelLen = 0;

function post(msg: FromWorker, transfer: Transferable[] = []): void {
  self.postMessage(msg, { transfer });
}

/** Copies out of WASM memory: the view is invalidated if memory grows. */
function copyF32(exports: SwarmExports, ptr: number, len: number): Float32Array {
  return new Float32Array(exports.memory.buffer, ptr, len).slice();
}

async function init(preset: number, seed: number, drones: number): Promise<void> {
  const bytes = await (await fetch(wasmUrl)).arrayBuffer();
  const { instance } = await WebAssembly.instantiate(bytes, {});
  const exports = instance.exports as unknown as SwarmExports;
  sim = exports.sim_new_preset(preset, seed, drones);
  if (sim === 0) throw new Error(`unknown map preset ${preset}`);
  wasm = exports;

  const size = exports.sim_terrain_size(sim);
  const heights = copyF32(exports, exports.sim_terrain_ptr(sim), size * size);
  const extent = exports.sim_terrain_extent(sim);
  post({ type: 'terrain', size, extent, heights, preset: exports.sim_preset(sim) }, [heights.buffer]);

  setInterval(step, STEP_MS);
}

function step(): void {
  if (!wasm) return;
  counterSums.fill(0);
  for (let k = 0; k < speed; k++) {
    wasm.sim_step(sim, STEP_MS);
    if (speed > 1) {
      const header = new Float32Array(wasm.memory.buffer, wasm.sim_snapshot_ptr(sim), COUNTERS[2] + 1);
      for (let c = 0; c < COUNTERS.length; c++) counterSums[c] += header[COUNTERS[c]];
    }
    // Chatter is per step, so collect it after every substep.
    collectChatter(wasm, k === speed - 1);
  }
  const data = copyF32(wasm, wasm.sim_snapshot_ptr(sim), wasm.sim_snapshot_len(sim));
  if (!layoutChecked && !checkLayout(wasm, data[0])) return;
  if (speed > 1) for (let c = 0; c < COUNTERS.length; c++) data[COUNTERS[c]] = counterSums[c];
  post({ type: 'snapshot', data, stepMs: STEP_MS }, [data.buffer]);
  postFireIfChanged(wasm);
  postTelemetry(wasm);
}

/** Warns about a legacy WASM layout; stops the sim (false) on one this page cannot read. */
function checkLayout(exports: SwarmExports, version: number): boolean {
  layoutChecked = true;
  channelLen = typeof exports.sim_channel_len === 'function' ? exports.sim_channel_len(sim) : 0;
  if (version !== VERSION && version !== LEGACY_VERSION) {
    wasm = null;
    post({ type: 'error', message: `snapshot version ${version}, expected ${VERSION}: rebuild the WASM (npm run wasm)` });
    return false;
  }
  const legacy: string[] = [];
  if (version === LEGACY_VERSION) legacy.push(`snapshot v${version} (no link quality)`);
  if (channelLen < CHANNEL_LEN) legacy.push(`channel summary ${channelLen} floats (no byte breakdown)`);
  if (legacy.length > 0) console.warn(`swarm.wasm predates the current layout: ${legacy.join(', ')}. Run npm run wasm.`);
  if (channelLen !== CHANNEL_LEN && channelLen !== LEGACY_CHANNEL_LEN) {
    console.warn(`unexpected channel summary length ${channelLen} (expected ${CHANNEL_LEN})`);
  }
  return true;
}

/** Streams telemetry stats (≤ 5 Hz) while either model is on. */
function postTelemetry(exports: SwarmExports): void {
  if (!channelOn && !telemetryOn) return;
  const now = performance.now();
  if (now - tlmStatsAt >= TLMSTATS_INTERVAL_MS) {
    tlmStatsAt = now;
    const data = copyF32(exports, exports.sim_telemetry_ptr(sim), exports.sim_telemetry_len(sim));
    // Legacy (16-float) summaries are posted as they are; the page checks `hasAirBytes`.
    const channel = copyF32(exports, exports.sim_channel_ptr(sim), channelLen || exports.sim_channel_len(sim));
    post({ type: 'tlmstats', data, channel }, [data.buffer, channel.buffer]);
  }
}

/**
 * For each TLM chatter record logged in the step just run, posts the package text
 * the drone last sent if it is that record's package (same "s"). A drone may send a
 * second package later in the same step; then the text belongs to another frame and
 * the console line keeps just "s=<seq>". Chatter logs ≤ 1 TLM per drone per second.
 */
function postTlmSamples(exports: SwarmExports, chatter: Float32Array): void {
  if (!tlmSampleSub || !telemetryOn) return;
  for (let o = 0; o + CHATTER_STRIDE <= chatter.length; o += CHATTER_STRIDE) {
    if (chatter[o + C.kind] !== ChatterKind.Tlm) continue;
    const id = chatter[o + C.src];
    const seq = chatter[o + TL.seq];
    const len = exports.sim_tlm_sample_len(sim, id);
    if (len === 0) continue;
    const bytes = new Uint8Array(exports.memory.buffer, exports.sim_tlm_sample_ptr(sim, id), len);
    const text = utf8.decode(bytes);
    // The record's seq is an f32; round the text's the same way.
    if (Math.fround(packageSeq(text)) === seq) post({ type: 'tlmsample', id, seq, text });
  }
}

/** Appends this step's chatter; posts the batch on the last substep if it is due. */
function collectChatter(exports: SwarmExports, mayPost: boolean): void {
  const len = exports.sim_chatter_len(sim);
  if (len > 0) {
    if (chatterLen + len > chatterBuf.length) {
      const grown = new Float32Array(Math.max(chatterBuf.length * 2, chatterLen + len));
      grown.set(chatterBuf.subarray(0, chatterLen));
      chatterBuf = grown;
    }
    chatterBuf.set(new Float32Array(exports.memory.buffer, exports.sim_chatter_ptr(sim), len), chatterLen);
    // Package texts are read now, while the sample is still this step's package.
    postTlmSamples(exports, chatterBuf.subarray(chatterLen, chatterLen + len));
    chatterLen += len;
  }
  const now = performance.now();
  if (!mayPost || chatterLen === 0 || now - chatterAt < CHATTER_INTERVAL_MS) return;
  chatterAt = now;
  const data = chatterBuf.slice(0, chatterLen);
  chatterLen = 0;
  post({ type: 'chatter', data }, [data.buffer]);
}

function postFireIfChanged(exports: SwarmExports): void {
  const now = performance.now();
  if (now - fireAt < FIRE_INTERVAL_MS) return;
  fireAt = now;
  const size = exports.sim_fire_size(sim);
  const view = new Uint8Array(exports.memory.buffer, exports.sim_fire_ptr(sim), size * size);
  if (lastFire && lastFire.length === view.length && lastFire.every((v, i) => v === view[i])) return;
  lastFire = view.slice();
  const cells = view.slice();
  post({ type: 'fire', size, cells }, [cells.buffer]);
}

/** Controls that arrive while the WASM module is still loading, applied once it is up. */
const pending: ToWorker[] = [];

function apply(exports: SwarmExports, msg: ToWorker): void {
  switch (msg.type) {
    case 'init':
      break;
    case 'kill':
      exports.sim_kill(sim, msg.id);
      break;
    case 'command':
      exports.sim_command(sim, msg.intent, msg.x, msg.z);
      break;
    case 'loss':
      exports.sim_set_loss(sim, msg.value);
      break;
    case 'ignite':
      exports.sim_ignite(sim, msg.x, msg.z);
      break;
    case 'speed':
      speed = Math.max(1, Math.min(MAX_SPEED, Math.round(msg.value) || 1));
      break;
    case 'channel':
      channelOn = msg.enabled;
      exports.sim_set_channel(sim, msg.enabled ? 1 : 0, msg.external);
      break;
    case 'telemetry':
      if (exports.sim_set_telemetry(sim, msg.enabled ? 1 : 0, msg.variant, msg.rateMode, msg.hz, msg.routing) === 1) {
        telemetryOn = msg.enabled;
      }
      break;
    case 'tlmsample-sub':
      tlmSampleSub = msg.on;
      break;
  }
}

self.onmessage = (e: MessageEvent<ToWorker>) => {
  const msg = e.data;
  if (msg.type === 'init') {
    init(msg.preset, msg.seed, msg.drones)
      .then(() => {
        for (const queued of pending.splice(0)) if (wasm) apply(wasm, queued);
      })
      .catch((err: unknown) => {
        post({ type: 'error', message: err instanceof Error ? err.message : String(err) });
      });
  } else if (wasm) {
    apply(wasm, msg);
  } else {
    pending.push(msg);
  }
};
