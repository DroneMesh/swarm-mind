// Reader for the telemetry / shared-channel stats the simulator builds every step.
// Mirrors crates/swarm-sim/src/telemetry.rs (the "Stats layout" section); keep in sync.
//
// tlmstats.data: TLM_STRIDE floats per drone, in id order (rates over 2 s):
//   [id, requested_hz, delivered_hz, loss_pct (10 s, from "s" gaps), latency_ms,
//    tx_bytes_per_s (own packages incl. mesh JSON + TCP/IP + MAC), relay_bytes_per_s
//    (root: uplink datagrams), airtime_frac, busy_frac (heard + external),
//    queue_drops_per_s, tree_parent (-1 none), tree_hops (-1 none), last_frame_bytes,
//    last_package_bytes (incl. '\0'), brain_bytes_per_s, variant (0 S3 / 1 C6),
//    operator_hz, tx_all_bytes_per_s (everything the node transmits),
//    ack_bytes_per_s (802.11 + TCP ACKs it sends), reserved]
// tlmstats.channel: CHANNEL_LEN floats, fleet-wide (offsets mirror Rust `telemetry::chan`).
// Byte counters 16..23 are per second, MPDU bytes on air including retries.
// A WASM built before the v3 layout sends LEGACY_CHANNEL_LEN floats and zeros in
// tx_all / ack; `hasAirBytes` tells the two apart.

export const TLM_STRIDE = 20;

/** Field offsets within a per-drone record. */
export const TLM = {
  id: 0,
  requestedHz: 1,
  deliveredHz: 2,
  lossPct: 3,
  latencyMs: 4,
  txBytesPerS: 5,
  relayBytesPerS: 6,
  airtimeFrac: 7,
  busyFrac: 8,
  queueDropsPerS: 9,
  treeParent: 10,
  treeHops: 11,
  lastFrameBytes: 12,
  lastPackageBytes: 13,
  brainBytesPerS: 14,
  variant: 15,
  operatorHz: 16,
  /** Everything the node transmits (own + relay + ACKs + brain + sync + beacons), B/s. */
  txAllBytesPerS: 17,
  /** 802.11 + TCP ACKs the node sends, B/s. */
  ackBytesPerS: 18,
} as const;

export const CHANNEL_LEN = 28;
/** Channel summary length of a WASM built before the per-class byte counters. */
export const LEGACY_CHANNEL_LEN = 16;

/** Field offsets within the fleet channel summary. */
export const CH = {
  ppsOnAir: 0,
  busyMean: 1,
  busyMax: 2,
  externalBusy: 3,
  gcsPps: 4,
  gcsBytesPerS: 5,
  budgetPps: 6,
  mode: 7,
  channelOn: 8,
  telemetryOn: 9,
  medianLossPct: 10,
  bridgeUp: 11,
  airBrain: 12,
  airMesh: 13,
  airUplink: 14,
  online: 15,
  /** Total MPDU bytes/s on air, retries included; the sum of the classes below. */
  bytesOnAir: 16,
  bytesBrain: 17,
  bytesTlmData: 18,
  /** 802.11 + TCP ACKs. */
  bytesAck: 19,
  /** painlessMesh NodeSync + TimeSync. */
  bytesMeshControl: 20,
  bytesUplink: 21,
  bytesBeacon: 22,
  /** The part of `bytesOnAir` that is retransmissions. */
  bytesRetry: 23,
  /** Every frame on air per second. */
  framesOnAir: 24,
  linkQualityMean: 25,
  /** Brain links with quality < 0.5. */
  linksDegraded: 26,
  /** `bytesMeshControl` averaged over 60 s (sync runs every 60 s, so 2 s is mostly 0). */
  bytesMeshControl60s: 27,
} as const;

/** Whether `channel` carries the per-class byte counters (a v3 WASM). */
export function hasAirBytes(channel: Float32Array | null): channel is Float32Array {
  return channel !== null && channel.length >= CHANNEL_LEN;
}

/** Everything on air, by class, per second (from the channel summary). */
export interface AirBytes {
  total: number;
  frames: number;
  brain: number;
  tlm: number;
  ack: number;
  sync: number;
  uplink: number;
  beacon: number;
  retry: number;
  linkQualityMean: number;
  linksDegraded: number;
}

export function newAirBytes(): AirBytes {
  return { total: 0, frames: 0, brain: 0, tlm: 0, ack: 0, sync: 0, uplink: 0, beacon: 0, retry: 0, linkQualityMean: 1, linksDegraded: 0 };
}

/** Reads the byte counters into `out`; false (out untouched) on a legacy summary. */
export function airBytes(channel: Float32Array | null, out: AirBytes): boolean {
  if (!hasAirBytes(channel)) return false;
  const v = (k: number) => (channel[k] > 0 ? channel[k] : 0);
  out.total = v(CH.bytesOnAir);
  out.frames = v(CH.framesOnAir);
  out.brain = v(CH.bytesBrain);
  out.tlm = v(CH.bytesTlmData);
  out.ack = v(CH.bytesAck);
  // The 60 s average: NodeSync / TimeSync fire every 60 s, so the 2 s figure is mostly 0.
  out.sync = v(CH.bytesMeshControl60s);
  out.uplink = v(CH.bytesUplink);
  out.beacon = v(CH.bytesBeacon);
  out.retry = v(CH.bytesRetry);
  out.linkQualityMean = channel[CH.linkQualityMean] >= 0 ? channel[CH.linkQualityMean] : 0;
  out.linksDegraded = v(CH.linksDegraded);
  return true;
}

/** Compact byte count: "0", "40", "0.8k", "52k". */
export function fmtK(v: number): string {
  if (!(v > 0)) return '0';
  if (v < 100) return v.toFixed(0);
  if (v < 10_000) return `${(v / 1000).toFixed(1)}k`;
  return `${(v / 1000).toFixed(0)}k`;
}

/** Bytes per second with a unit: "640 B/s", "52.3 kB/s". */
export function fmtByteRate(v: number): string {
  if (!(v > 0)) return '0 B/s';
  return v >= 1000 ? `${(v / 1000).toFixed(1)} kB/s` : `${v.toFixed(0)} B/s`;
}

/** The per-class breakdown: "brain 0.8k · tlm 52k · ack 9k · sync 0.3k · uplink 6k · beacon 4k · retry 3k" (B/s). */
export function airMixText(a: AirBytes): string {
  return (
    `brain ${fmtK(a.brain)} · tlm ${fmtK(a.tlm)} · ack ${fmtK(a.ack)} · sync ${fmtK(a.sync)} · ` +
    `uplink ${fmtK(a.uplink)} · beacon ${fmtK(a.beacon)} · retry ${fmtK(a.retry)}`
  );
}

/** Link health: "q 0.93 · 2 degraded" (quality: the measured brain delivery ratio, mean over links). */
export function linkHealthText(mean: number, degraded: number): string {
  return `q ${mean.toFixed(2)} · ${degraded > 0 ? `${degraded.toFixed(0)} degraded` : 'none degraded'}`;
}

/** Node firmware (`variant` in the telemetry message; the stats field is 0 / 1 only). */
export const Variant = { XiaoS3: 0, C6Rtk: 1, Mixed: 2 } as const;
export type Variant = (typeof Variant)[keyof typeof Variant];

/** Telemetry rate modes (`rateMode`; also the summary's `mode`). */
export const RateMode = { Fixed: 0, BackendBudget: 1, Brain: 2 } as const;
export type RateMode = (typeof RateMode)[keyof typeof RateMode];

/** How packages are addressed. */
export const Routing = { Single: 0, Broadcast: 1 } as const;
export type Routing = (typeof Routing)[keyof typeof Routing];

/** Most external busy fraction the slider may add (swarm-sim MAX_EXTERNAL_BUSY). */
export const MAX_EXTERNAL_BUSY = 0.8;

/** Pseudo node id of the WiFi station in chatter (ROOT_UPLINK dst). */
export const STATION_ID = 0xfffd;

/** Number of drone records in `data`. */
export function tlmCount(data: Float32Array): number {
  return Math.floor(data.length / TLM_STRIDE);
}

/** Field `field` (a `TLM` offset) of drone record `i`. Allocation-free. */
export function tlmField(data: Float32Array, i: number, field: number): number {
  return data[i * TLM_STRIDE + field];
}

/** Root of the painlessMesh tree (the base station bridges to the WiFi station). */
export const MESH_ROOT = 0;

/** Display names for the rate modes, indexed by `RateMode`. */
export const RATE_MODE_NAMES: readonly string[] = ['FIXED', 'BUDGET', 'BRAIN'];

/** Fleet-wide figures for captions and the HUD (medians over sending drones). */
export interface FleetTlm {
  /** Drones currently sending telemetry (requested Hz > 0). */
  senders: number;
  /** Median rate the drones send at. */
  reqHz: number;
  /** Median rate delivered at the ground station. */
  gcsHz: number;
  /**
   * Median loss from "s" gaps, percent, over the senders the ground station hears
   * (delivered Hz > 0), as the backend and the sim's median do. A sender whose
   * packages never arrive has no gaps to count, so it is reported in `silent`
   * instead of as a 0 % loss. 0 when no sender is heard.
   */
  lossPct: number;
  /** Senders the ground station hears (delivered Hz > 0). */
  heard: number;
  /** Senders whose packages do not reach the ground station at all. */
  silent: number;
  /**
   * Telemetry packages per second offered to the WiFi station: the sum of the
   * senders' requested rates. This is the figure the field calibration (one AP:
   * clean ≤ ~60–70 pps, collapse ~150 pps) is stated in; `ppsOnAir` counts every
   * frame on the channel (mesh hops, ACKs, sync, brain frames, uplink).
   */
  offeredPps: number;
  /** Mean telemetry bytes per second each sender puts on the air. */
  txBytesPerS: number;
  /** Mean / max busy fraction (heard + external), 0..1. */
  busyMean: number;
  busyMax: number;
  ppsOnAir: number;
  gcsPps: number;
  budgetPps: number;
  bridgeUp: boolean;
}

export function newFleetTlm(): FleetTlm {
  return {
    senders: 0,
    reqHz: 0,
    gcsHz: 0,
    lossPct: 0,
    heard: 0,
    silent: 0,
    offeredPps: 0,
    txBytesPerS: 0,
    busyMean: 0,
    busyMax: 0,
    ppsOnAir: 0,
    gcsPps: 0,
    budgetPps: 0,
    bridgeUp: false,
  };
}

let scratchA = new Float32Array(64);
let scratchB = new Float32Array(64);
let scratchC = new Float32Array(64);

/**
 * Window of `loss_pct` (and the sim's fleet median), seconds: the sim's `Long`
 * window, 40 buckets of 250 ms (crates/swarm-sim/src/telemetry.rs).
 */
export const LOSS_WINDOW_S = 10;

/** Window of `delivered_hz`, seconds (`BySent`: 2 s, read 0.75 s late). */
export const RATE_WINDOW_S = 2;

/** Delivered rate at or below which the ground station is taken to hear nothing. */
export const HEARD_HZ = 0.05;

function median(v: Float32Array, n: number): number {
  if (n === 0) return 0;
  const s = v.subarray(0, n).sort();
  const m = n >> 1;
  return n % 2 === 1 ? s[m] : 0.5 * (s[m - 1] + s[m]);
}

/**
 * Summarises the latest stats into `out`. `alive(id)` excludes downed drones
 * (their last rates would otherwise linger); the mesh root is never a sender.
 */
export function fleetTlm(
  data: Float32Array | null,
  channel: Float32Array | null,
  alive: (id: number) => boolean,
  out: FleetTlm = newFleetTlm(),
): FleetTlm {
  let n = 0;
  let heard = 0;
  let tx = 0;
  let offered = 0;
  if (data) {
    const count = tlmCount(data);
    if (scratchA.length < count) {
      scratchA = new Float32Array(count);
      scratchB = new Float32Array(count);
      scratchC = new Float32Array(count);
    }
    for (let i = 0; i < count; i++) {
      if (i === MESH_ROOT || !alive(i)) continue;
      const req = tlmField(data, i, TLM.requestedHz);
      if (!(req > 0)) continue;
      const gcs = tlmField(data, i, TLM.deliveredHz);
      scratchA[n] = req;
      scratchB[n] = gcs;
      // Loss is only measurable from packages that arrive (gaps in "s").
      if (gcs > HEARD_HZ) scratchC[heard++] = tlmField(data, i, TLM.lossPct);
      tx += tlmField(data, i, TLM.txBytesPerS);
      offered += req;
      n++;
    }
  }
  out.senders = n;
  out.heard = heard;
  out.silent = n - heard;
  out.offeredPps = offered;
  out.reqHz = median(scratchA, n);
  out.gcsHz = median(scratchB, n);
  out.lossPct = median(scratchC, heard);
  out.txBytesPerS = n > 0 ? tx / n : 0;
  out.busyMean = channel ? channel[CH.busyMean] : 0;
  out.busyMax = channel ? channel[CH.busyMax] : 0;
  out.ppsOnAir = channel ? channel[CH.ppsOnAir] : 0;
  out.gcsPps = channel ? channel[CH.gcsPps] : 0;
  out.budgetPps = channel ? channel[CH.budgetPps] : 0;
  out.bridgeUp = channel ? channel[CH.bridgeUp] > 0 : false;
  return out;
}

/**
 * Whether the per-drone loss (10 s) and the delivered-rate shortfall
 * `1 − delivered / requested` (2 s) tell the same story: in a steady state every
 * frame is either delivered or a gap in "s", so they agree; right after a rate
 * change or during stalls the two windows differ. Median over heard senders, within `tolerancePct` points.
 */
export function lossAgreesWithRate(data: Float32Array | null, alive: (id: number) => boolean, tolerancePct: number): boolean {
  if (!data) return false;
  const diffs: number[] = [];
  for (let i = 0; i < tlmCount(data); i++) {
    if (i === MESH_ROOT || !alive(i)) continue;
    const req = tlmField(data, i, TLM.requestedHz);
    const gcs = tlmField(data, i, TLM.deliveredHz);
    if (!(req > 0) || !(gcs > HEARD_HZ)) continue;
    const shortfall = 100 * Math.max(0, 1 - gcs / req);
    diffs.push(Math.abs(tlmField(data, i, TLM.lossPct) - shortfall));
  }
  if (diffs.length === 0) return true;
  diffs.sort((a, b) => a - b);
  return diffs[diffs.length >> 1] <= tolerancePct;
}

/** Display tone: '' none, g good, a warn, r bad. */
export type Tone = '' | 'g' | 'a' | 'r';

const TONE_RANK: Record<Tone, number> = { '': 0, g: 1, a: 2, r: 3 };

/** The worse of two tones. */
export function worst(a: Tone, b: Tone): Tone {
  return TONE_RANK[b] > TONE_RANK[a] ? b : a;
}

/** Loss percent: < 3 good (the field's clean AP is ~1 %), < 10 warn, else bad. */
export function toneLoss(p: number): Tone {
  return p < 3 ? 'g' : p < 10 ? 'a' : 'r';
}

/**
 * Fleet loss for the header: the median over heard senders, plus how many are
 * not heard at all. Labelled "med 10 s": per-drone loss is pooled over the sim's
 * 10 s window, while the delivered Hz beside it is a 2 s rate.
 */
export function fleetLossText(f: FleetTlm): string {
  if (f.senders === 0) return 'loss —';
  if (f.heard === 0) return 'loss — · none reach the GCS';
  return `loss ${f.lossPct.toFixed(0)}% (med ${LOSS_WINDOW_S} s)${f.silent > 0 ? ` · ${f.silent} not heard` : ''}`;
}

/** Red when nothing (or the median sender) reaches the GCS; amber when some senders are not heard. */
export function fleetLossTone(f: FleetTlm): Tone {
  if (f.senders === 0) return '';
  if (f.heard === 0 || f.gcsHz <= HEARD_HZ) return 'r';
  return worst(toneLoss(f.lossPct), f.silent > 0 ? 'a' : '');
}

/** Number of live drones whose painlessMesh path to the root passes through `id`. */
export function treeDescendants(data: Float32Array, id: number, alive: (id: number) => boolean): number {
  const count = tlmCount(data);
  let n = 0;
  for (let i = 0; i < count; i++) {
    if (i === id || !alive(i)) continue;
    let at = tlmField(data, i, TLM.treeParent);
    for (let k = 0; k < count && at >= 0; k++) {
      if (at === id) {
        n++;
        break;
      }
      at = tlmField(data, at, TLM.treeParent);
    }
  }
  return n;
}

const SEQ_RE = /\\?"s\\?":(\d+)/;

/**
 * The "s" (sequence) value inside a painlessMesh telemetry package or a bare
 * telemetry JSON (inner quotes may be escaped), or -1 if it has none.
 */
export function packageSeq(text: string): number {
  const m = SEQ_RE.exec(text);
  return m ? Number(m[1]) : -1;
}

/** Radio console header: the on-air figure, the compact breakdown and the tooltip. */
export interface AirHeader {
  /** "air 830 fr/s · 52.3 kB/s", or "brain 41 fr/s · 1.2 kB/s" when only brain frames are counted. */
  text: string;
  /** The breakdown (air only, else ''). */
  mix: string;
  title: string;
}

/**
 * Header text for the radio console. `air` is the channel summary's byte counters
 * (null when both models are off, or the WASM predates them): then only the brain
 * frames the snapshots count are known, and the figure says "brain". `legacy`
 * notes that the models are on but the counters are missing.
 */
export function airHeader(air: AirBytes | null, brainFramesPerS: number, brainBytesPerS: number, legacy = false): AirHeader {
  if (air) {
    const mix = airMixText(air);
    return {
      text: `air ${air.frames.toFixed(0)} fr/s · ${fmtByteRate(air.total)}`,
      mix,
      title:
        'All communication on air, per second: every frame (brain, telemetry data, 802.11 + TCP ACKs, ' +
        'painlessMesh NodeSync + TimeSync, root uplink, beacons) as MPDU bytes, retries included.\n' +
        `${mix} (B/s; retry is the retransmitted part of the total)`,
    };
  }
  return {
    text: `brain ${brainFramesPerS.toFixed(0)} fr/s · ${fmtByteRate(brainBytesPerS)}`,
    mix: '',
    title: legacy
      ? 'Brain frames only: this swarm.wasm predates the per-class byte counters (run npm run wasm).'
      : 'Brain frames only (heartbeats, intents). Turn on the telemetry or channel model to count all traffic on air.',
  };
}
