// Messages between the page and the simulation worker.

export const Intent = { Hold: 0, MonitorFire: 1, MapPerimeter: 2, ReturnToBase: 3 } as const;
export type Intent = (typeof Intent)[keyof typeof Intent];

/** Display names indexed by intent value. */
export const INTENT_NAMES: readonly string[] = ['Hold', 'Monitor fire', 'Map perimeter', 'Return to base'];

/** Fire cell states in the `fire` message, matching swarm-sim's fire module. */
export const FireCell = { Unburnt: 0, Burning: 1, Burnt: 2 } as const;

export type ToWorker =
  /** `preset`: swarm-sim `Preset` value, an index into `PRESETS` (web/src/presets.ts). */
  | { type: 'init'; seed: number; drones: number; preset: number }
  | { type: 'kill'; id: number }
  /** `x`, `z`: area of interest in world units, e.g. the reported fire. */
  | { type: 'command'; intent: Intent; x: number; z: number }
  /** Probability in [0, 1] that any single radio delivery is lost. */
  | { type: 'loss'; value: number }
  /** Starts a spot fire at world `(x, z)`, e.g. embers landing past the front. */
  | { type: 'ignite'; x: number; z: number }
  /**
   * Sim steps per worker tick (1..8), for fast-forwarding demos. Snapshots stay
   * one per tick; their frame / byte / drop counters sum the tick's steps.
   */
  | { type: 'speed'; value: number }
  /**
   * Shared-airtime model on / off, and airtime used by other networks
   * (`external`, fraction 0..0.8; see MAX_EXTERNAL_BUSY in telemetry.ts).
   */
  | { type: 'channel'; enabled: boolean; external: number }
  /**
   * Real-system telemetry over painlessMesh. `variant`, `rateMode`, `routing`: the
   * Variant / RateMode / Routing values in telemetry.ts; `hz`: the operator's
   * requested rate (the fixed rate in RateMode.Fixed).
   */
  | { type: 'telemetry'; enabled: boolean; variant: number; rateMode: number; hz: number; routing: number }
  /** Start / stop the per-drone `tlmsample` stream. */
  | { type: 'tlmsample-sub'; on: boolean };

export type FromWorker =
  | { type: 'terrain'; size: number; extent: number; heights: Float32Array; preset: number }
  | { type: 'snapshot'; data: Float32Array; stepMs: number }
  /** Row-major cell states covering the whole map; sent only when changed, at most 4 Hz. */
  | { type: 'fire'; size: number; cells: Uint8Array }
  /**
   * Decoded radio frames (see web/src/chatter.ts for the layout) accumulated since the
   * previous chatter message; posted at most every 100 ms, only when non-empty.
   */
  | { type: 'chatter'; data: Float32Array }
  /**
   * Telemetry / airtime stats (layouts in web/src/telemetry.ts): `data` TLM_STRIDE floats
   * per drone, `channel` the CHANNEL_LEN fleet summary. At most 5 Hz, only while the
   * channel or telemetry model is on.
   */
  | { type: 'tlmstats'; data: Float32Array; channel: Float32Array }
  /**
   * The exact painlessMesh package text of the TLM chatter record from drone `id` with
   * sequence `seq` (its "s"). Read in the same step the record was logged and sent only
   * when the text's "s" equals the record's, so it is that record's package (chatter
   * logs at most one TLM per drone per second). Only while subscribed with
   * `tlmsample-sub` (never the base: it is the bridge).
   */
  | { type: 'tlmsample'; id: number; seq: number; text: string }
  | { type: 'error'; message: string };
