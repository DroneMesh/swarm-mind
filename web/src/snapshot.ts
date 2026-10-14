// Reader for the flat snapshot the simulator emits every step.
// Mirrors crates/swarm-sim/src/snapshot.rs; keep the two in sync.
// v3 links are `[a, b, kind, quality]`; v2 (`[a, b, kind]`) is still read so the
// page keeps working against a WASM built before the bump.

export const VERSION = 3;
/** The previous layout (links `[a, b, kind]`), still read until the WASM is rebuilt. */
export const LEGACY_VERSION = 2;
export const HEADER_LEN = 12;
export const DRONE_STRIDE = 8;
export const LINK_STRIDE = 4;
const LEGACY_LINK_STRIDE = 3;
export const PULSE_STRIDE = 3;
export const NO_ROUTE = 255;

/** Field offsets within a drone record. */
export const D = { id: 0, x: 1, y: 2, z: 3, alive: 4, role: 5, hops: 6, parent: 7 } as const;

export const Role = { Reserve: 0, Scout: 1, Relay: 2, Sink: 3 } as const;
export type Role = (typeof Role)[keyof typeof Role];

/** Display names indexed by role value. */
export const ROLE_NAMES: readonly string[] = ['Reserve', 'Scout', 'Relay', 'Base'];

/** Field offsets within a link record. */
export const L = { a: 0, b: 1, kind: 2, quality: 3 } as const;

/** Link `kind`: route to the parent; anything else is a neighbour link. */
export const LINK_ROUTE = 1;

/** A link below this delivery ratio counts as degraded (swarm-sim LINKS_DEGRADED). */
export const DEGRADED_QUALITY = 0.5;

/** Pulse kinds, matching the frame kinds in swarm-brain's wire module. */
export const PulseKind = { Heartbeat: 0, Intent: 1 } as const;

export interface LinkHealth {
  links: number;
  /** Mean quality, 0..1. */
  mean: number;
  /** Links with quality < DEGRADED_QUALITY. */
  degraded: number;
  /** False on a legacy snapshot: there is no measured quality. */
  measured: boolean;
}

export function newLinkHealth(): LinkHealth {
  return { links: 0, mean: 1, degraded: 0, measured: false };
}

export class Snapshot {
  readonly droneCount: number;
  readonly linkCount: number;
  readonly pulseCount: number;
  /** Layout version: VERSION, or LEGACY_VERSION from a WASM not yet rebuilt. */
  readonly version: number;
  /** Whether links carry a measured `quality` (v3); legacy links read as 1. */
  readonly hasQuality: boolean;
  private readonly linkStride: number;
  readonly data: Float32Array;

  constructor(data: Float32Array) {
    this.data = data;
    const v = data[0];
    if (v !== VERSION && v !== LEGACY_VERSION) {
      throw new Error(`snapshot version ${v}, expected ${VERSION}`);
    }
    this.version = v;
    this.hasQuality = v === VERSION;
    this.linkStride = this.hasQuality ? LINK_STRIDE : LEGACY_LINK_STRIDE;
    this.droneCount = data[3];
    this.linkCount = data[4];
    this.pulseCount = data[5];
  }

  get time(): number {
    return this.data[2];
  }

  /** Frames put on the air during this step. */
  get frames(): number {
    return this.data[6];
  }

  get bytes(): number {
    return this.data[7];
  }

  /** Deliveries dropped because a receiver's inbox was full. */
  get dropped(): number {
    return this.data[8];
  }

  get burningCells(): number {
    return this.data[9];
  }

  /** The intent the base station currently holds (see protocol `Intent`). */
  get baseIntent(): number {
    return this.data[10];
  }

  /** Offset of drone `i`'s record. */
  drone(i: number): number {
    return HEADER_LEN + i * DRONE_STRIDE;
  }

  /** Offset of link `i`'s record: `[a, b, kind, quality]` (see `L`; v2 has no quality). */
  link(i: number): number {
    return HEADER_LEN + this.droneCount * DRONE_STRIDE + i * this.linkStride;
  }

  /**
   * Measured delivery ratio of link `i`, 0..1: brain frames delivered between its
   * ends over ~4 s, the worse direction. 1 on a legacy (v2) snapshot.
   */
  linkQuality(i: number): number {
    if (!this.hasQuality) return 1;
    const q = this.data[this.link(i) + L.quality];
    return q >= 0 ? (q <= 1 ? q : 1) : 0; // also maps NaN to 0
  }

  /**
   * Mean link quality and the number of degraded links (< DEGRADED_QUALITY), into
   * `out` (allocation-free). With no links the mean is 1.
   */
  linkHealth(out: LinkHealth): LinkHealth {
    let sum = 0;
    let degraded = 0;
    for (let l = 0; l < this.linkCount; l++) {
      const q = this.linkQuality(l);
      sum += q;
      if (q < DEGRADED_QUALITY) degraded++;
    }
    out.links = this.linkCount;
    out.mean = this.linkCount > 0 ? sum / this.linkCount : 1;
    out.degraded = degraded;
    out.measured = this.hasQuality;
    return out;
  }

  /** Offset of pulse `i`'s record: `[src, dst, kind]`. */
  pulse(i: number): number {
    return this.link(this.linkCount) + i * PULSE_STRIDE;
  }

  alive(i: number): boolean {
    return this.data[this.drone(i) + D.alive] === 1;
  }

  countAlive(): number {
    let n = 0;
    for (let i = 0; i < this.droneCount; i++) if (this.alive(i)) n++;
    return n;
  }

  /** Field `f` (see `D`) of drone `i`. */
  get(i: number, f: number): number {
    return this.data[this.drone(i) + f];
  }

  countRouted(): number {
    let n = 0;
    for (let i = 0; i < this.droneCount; i++) {
      if (this.alive(i) && this.data[this.drone(i) + D.hops] !== NO_ROUTE) n++;
    }
    return n;
  }

  /**
   * Whether drone `i`'s parent chain reaches a live sink through live drones.
   * Stricter than `hops != NO_ROUTE`: right after a kill, orphans still
   * advertise a route through the dead drone until they notice the silence.
   */
  routeIntact(i: number): boolean {
    for (let step = 0, at = i; step <= this.droneCount; step++) {
      if (!this.alive(at)) return false;
      if (this.get(at, D.role) === Role.Sink) return true;
      if (this.get(at, D.hops) === NO_ROUTE) return false;
      const parent = this.get(at, D.parent);
      if (parent < 0 || parent >= this.droneCount) return false;
      at = parent;
    }
    return false; // a parent loop
  }
}
