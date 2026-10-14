// Reader for the decoded radio traffic the simulator logs every step.
// Mirrors crates/swarm-sim/src/chatter.rs; keep the two in sync.
//
// Record = CHATTER_STRIDE floats: [t_ms, src, dst (-1 broadcast), kind, f0..f7].
// Heartbeat f: [hops (255 = no route), parent (-1 none), role, sink, slot (-1 none),
//               flags (FLAG_* bits), need (-1 none), intent_seq].
// Intent / mission f: [seq, intent, target_x, target_z, origin_x, origin_z, 0, 0];
//               an intent has no origin, so origin_x / origin_z are NaN.
// Undecodable: f all 0; src / dst from the raw frame header.
// Telemetry-model kinds (only while telemetry is on; TLM / ROOT_UPLINK sampled to
// at most one record per drone per second):
//   TLM f: [bytes (JSON payload), package_bytes (mesh JSON incl. '\0'), hops_to_root
//           (-1 none), seq, has_slow (0/1), 0, 0, 0]; dst = root, or -1 when flooded.
//   PM_SYNC / PM_TIME f: [bytes, package_type (5/6 or 3/4), 0, ...].
//   ROOT_UPLINK f: [bytes (UDP payload), origin drone, seq, 0, ...]; dst = STATION_ID (65533).

export const CHATTER_STRIDE = 12;

/** Record kinds (field `kind`). */
export const ChatterKind = {
  Heartbeat: 0,
  Intent: 1,
  Mission: 2,
  Tlm: 3,
  PmSync: 4,
  PmTime: 5,
  RootUplink: 6,
  Undecodable: 255,
} as const;
export type ChatterKind = (typeof ChatterKind)[keyof typeof ChatterKind];

/** Sender id of commands typed at the operator console (swarm-brain's OPERATOR). */
export const OPERATOR_ID = 0xfffe;

/** Field offsets within a record. */
export const C = { tMs: 0, src: 1, dst: 2, kind: 3, f0: 4 } as const;

/** Heartbeat field offsets within a record. */
export const HB = { hops: 4, parent: 5, role: 6, sink: 7, slot: 8, flags: 9, need: 10, intentSeq: 11 } as const;

/** Telemetry-kind field offsets within a record (TLM; ROOT_UPLINK uses bytes / origin / seq). */
export const TL = { bytes: 4, packageBytes: 5, hops: 6, seq: 7, hasSlow: 8, origin: 5, uplinkSeq: 6, pkgType: 5 } as const;

/** Intent / mission field offsets within a record. */
export const MI = { seq: 4, intent: 5, targetX: 6, targetZ: 7, originX: 8, originZ: 9 } as const;

/** Heartbeat flag bits, matching swarm-brain's wire module. */
export const Flag = { Settled: 1, Landed: 2, Filling: 4, Carrying: 8, Orphan: 16 } as const;

export interface Heartbeat {
  hops: number;
  /** -1 for none. */
  parent: number;
  role: number;
  sink: number;
  /** -1 for none. */
  slot: number;
  flags: number;
  /** -1 for none. */
  need: number;
  intentSeq: number;
}

export interface MissionRecord {
  seq: number;
  intent: number;
  targetX: number;
  targetZ: number;
  /** NaN for an operator intent. */
  originX: number;
  originZ: number;
}

/** Number of whole records in `data`. */
export function chatterCount(data: Float32Array): number {
  return Math.floor(data.length / CHATTER_STRIDE);
}

/** Field `field` of record `i`. Allocation-free; prefer it in hot loops. */
export function chatterField(data: Float32Array, i: number, field: number): number {
  return data[i * CHATTER_STRIDE + field];
}

export function chatterKind(data: Float32Array, i: number): number {
  return data[i * CHATTER_STRIDE + C.kind];
}

/** Decodes record `i`'s heartbeat fields into `out` (reused to avoid allocation). */
export function readHeartbeat(data: Float32Array, i: number, out: Heartbeat): Heartbeat {
  const o = i * CHATTER_STRIDE;
  out.hops = data[o + HB.hops];
  out.parent = data[o + HB.parent];
  out.role = data[o + HB.role];
  out.sink = data[o + HB.sink];
  out.slot = data[o + HB.slot];
  out.flags = data[o + HB.flags];
  out.need = data[o + HB.need];
  out.intentSeq = data[o + HB.intentSeq];
  return out;
}

/** Decodes record `i`'s intent / mission fields into `out`. */
export function readMission(data: Float32Array, i: number, out: MissionRecord): MissionRecord {
  const o = i * CHATTER_STRIDE;
  out.seq = data[o + MI.seq];
  out.intent = data[o + MI.intent];
  out.targetX = data[o + MI.targetX];
  out.targetZ = data[o + MI.targetZ];
  out.originX = data[o + MI.originX];
  out.originZ = data[o + MI.originZ];
  return out;
}

export function newHeartbeat(): Heartbeat {
  return { hops: 0, parent: -1, role: 0, sink: 0, slot: -1, flags: 0, need: -1, intentSeq: 0 };
}

export function newMission(): MissionRecord {
  return { seq: 0, intent: 0, targetX: 0, targetZ: 0, originX: NaN, originZ: NaN };
}

/** Slot id ranges, matching swarm-brain's plan module (RELAY_BASE, SCOUT_BASE, PARK_BASE). */
const RELAY_BASE = 1;
const SCOUT_BASE = 32;
const PARK_BASE = 40;
const MAX_ROWS = 12;
const SCOUTS = 4;
const MAX_PARKS = 2 * (MAX_ROWS + 1);

/**
 * Short operator name for a formation slot: relay row `r` on rail 0/1 is
 * `A<r>` / `B<r>`, scout sector `k` is `S<k+1>`, parking spot `j` is
 * `P<j+1>`; '' for none (-1 / 0) and `#<id>` for an id the plan never uses.
 */
export function slotName(slot: number): string {
  if (slot <= 0 || !Number.isInteger(slot)) return '';
  if (slot < RELAY_BASE + 2 * MAX_ROWS) {
    const i = slot - RELAY_BASE;
    return `${i % 2 === 0 ? 'A' : 'B'}${(i >> 1) + 1}`;
  }
  if (slot >= SCOUT_BASE && slot < SCOUT_BASE + SCOUTS) return `S${slot - SCOUT_BASE + 1}`;
  if (slot >= PARK_BASE && slot < PARK_BASE + MAX_PARKS) return `P${slot - PARK_BASE + 1}`;
  return `#${slot}`;
}

/** Names of the set heartbeat flags, in bit order, for the console. */
export function flagNames(flags: number): string[] {
  const out: string[] = [];
  if (flags & Flag.Settled) out.push('settled');
  if (flags & Flag.Landed) out.push('landed');
  if (flags & Flag.Filling) out.push('filling');
  if (flags & Flag.Carrying) out.push('carrying');
  if (flags & Flag.Orphan) out.push('ORPHAN');
  return out;
}
