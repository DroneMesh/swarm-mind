// One-click showcase demos: small timelines of operator actions, captions
// and camera shots, played against the live simulation.
//
// Timing is in sim seconds (so a slow machine does not rush the story).
// A demo is a list of steps `{ at, action }`; `at` counts from the start of
// the current segment, and a segment starts at the demo's start and again
// whenever a gate (wait for deployment / recovery / a fresh session)
// resolves. So "3 s after the swarm healed" is simply `{ at: 3 }` after a
// `waitRecovered` gate, however long the healing took.

import { callsign } from '../callsigns';
import type { SwarmMonitor } from '../monitor';
import { Intent } from '../protocol';
import { D, Role, type Snapshot } from '../snapshot';
import {
  CH,
  LOSS_WINDOW_S,
  MESH_ROOT,
  RATE_WINDOW_S,
  RateMode,
  Routing,
  TLM,
  Variant,
  fleetTlm,
  lossAgreesWithRate,
  newFleetTlm,
  tlmCount,
  tlmField,
  treeDescendants,
  type FleetTlm,
} from '../telemetry';
import type { CameraDirector } from './camera';
import type { AirtimeSettings } from './linkbudget';

/** Who an action targets, resolved against the live snapshot when it runs. */
export type Selector = 'busiest-relay' | 'busiest-mesh-relay' | 'base' | 'half' | 'acting-sink' | 'last-killed';

export type Shot =
  | 'overview'
  | 'fire'
  | 'target'
  | 'base'
  | 'killsite'
  | 'swarm'
  | { follow: Selector }
  | { frame: Selector };

/** Caption text, or a function of the demo's measurements so far. */
export type Text = string | ((c: DemoContext) => string);

export type Action =
  | { do: 'caption'; title?: Text; text: Text }
  | { do: 'camera'; shot: Shot }
  | { do: 'kill'; who: Selector }
  | { do: 'loss'; value: number }
  /** Lights a small spot fire at the jump point (see `jumpPoint`). */
  | { do: 'spotFire' }
  | { do: 'command'; intent: Intent; target: 'fire' | 'jump' }
  | { do: 'speed'; value: number }
  /**
   * Gate: wait until the swarm is deployed on the current command target.
   * `onFire` first makes sure that target is the fire under MonitorFire
   * (the prelude's job); without it the current command is kept (e.g. after
   * a retarget). `title` replaces the caption title while waiting.
   */
  | { do: 'waitDeployed'; label?: string; onFire?: boolean; title?: string }
  /** Gate: wait until every live drone has an intact route again after the last kill. */
  | { do: 'waitRecovered'; timeout: number }
  /** Gate: start a fresh session and wait for its first snapshot. */
  | { do: 'reset'; title?: string }
  /** Changes the telemetry / channel settings (the link-budget panel follows). */
  | { do: 'airtime'; set: Partial<AirtimeSettings> }
  /** Stores the current fleet telemetry figures under `key` for later captions. */
  | { do: 'measure'; key: string }
  /**
   * Gate: wait until the telemetry stats describe only the current airtime
   * settings: at least `min` sim seconds (default: the loss window plus
   * SETTLE_MARGIN_S) since the last `airtime` step, a stats sample taken after
   * that, and per-drone loss agreeing with the delivered-rate shortfall (or
   * SETTLE_TIMEOUT_S more). Without it a caption quotes a loss still counting
   * the previous mode's gaps.
   */
  | { do: 'waitSettled'; min?: number }
  /**
   * Gate: wait until every drone whose painlessMesh path ran through the last
   * kill has a new, live mesh parent again (or `timeout` sim seconds pass).
   */
  | { do: 'waitMeshRerouted'; timeout: number }
  /** Starts a new segment now: later `at`s count from here. */
  | { do: 'segment' }
  | { do: 'end' };

export interface Step {
  at: number;
  action: Action;
}

export interface Demo {
  key: string;
  name: string;
  blurb: string;
  steps: Step[];
}

/** What the runner measured; captions can quote it. */
export interface DemoContext {
  killed: number[];
  /** Sim time of the first kill in the current recovery. */
  killAt: number;
  /** Seconds the last `waitRecovered` took, or NaN if it timed out. */
  recovery: number;
  /** Seconds the last `waitDeployed` took. */
  deployedIn: number;
  /** Routed-through count of the last relay picked, when it was picked. */
  carried: number;
  snapshot: Snapshot | null;
  target: { x: number; z: number };
  /** Latest telemetry stats / channel summary (null while the models are off). */
  tlm: Float32Array | null;
  channel: Float32Array | null;
  /** Live drones whose mesh path ran through the last kill, when it happened. */
  meshCarried: number;
  /** Seconds from the last kill until those drones had a new mesh parent; NaN while pending / timed out. */
  meshReroute: number;
  /** Fleet figures stored by `measure` steps. */
  measured: Record<string, FleetTlm>;
  /** Sim time of the last `airtime` step (NaN before any). */
  airtimeAt: number;
  /** False when the last `waitSettled` timed out with loss and delivered rate still disagreeing. */
  settled: boolean;
}

export interface DemoHost {
  snapshot(): Snapshot | null;
  /** Bumped by every session reset. */
  session(): number;
  readonly monitor: SwarmMonitor;
  readonly camera: CameraDirector;
  /** Kills `id`, the base included; logs it as `who`. */
  kill(id: number, who: string): boolean;
  setLoss(v: number): void;
  /** Starts a spot fire at world `(x, z)`. */
  ignite(x: number, z: number): void;
  command(intent: Intent, x: number, z: number): void;
  reset(): void;
  setSpeed(n: number): void;
  /** Burning centroid, or the ignition estimate before the fire is seen. */
  fireCentre(): { x: number; z: number };
  /** Last commanded target, or null before any command. */
  lastTarget(): { x: number; z: number } | null;
  groundAt(x: number, z: number): number;
  /** The map's opening camera. */
  overview(): void;
  extent(): number;
  positions(): Float32Array;
  /** Current telemetry / channel settings, and a change to them. */
  airtime(): Readonly<AirtimeSettings>;
  setAirtime(p: Partial<AirtimeSettings>): void;
  /** Latest 'tlmstats' arrays, or null. */
  tlm(): Float32Array | null;
  channel(): Float32Array | null;
}

// --- demo scripts --------------------------------------------------------------

/** Cell offsets (world units) lit around the jump point for a spot fire. */
const SPOT_FIRE_OFFSETS: readonly (readonly [number, number])[] = [
  [0, 0],
  [12, 0],
  [-12, 0],
  [0, 12],
  [0, -12],
];

const s = (n: number) => (Number.isFinite(n) ? `${n.toFixed(1)} s` : 'no recovery');
const routed = (c: DemoContext) => {
  const snap = c.snapshot;
  if (!snap) return 0;
  let n = 0;
  for (let i = 0; i < snap.droneCount; i++) if (snap.alive(i) && snap.get(i, D.role) !== Role.Sink && snap.routeIntact(i)) n++;
  return n;
};
const alive = (c: DemoContext) => c.snapshot?.countAlive() ?? 0;
const victim = (c: DemoContext) => (c.killed.length ? callsign(c.killed[c.killed.length - 1]) : 'the relay');

const RELAY_DOWN: Step[] = [
  { at: 0, action: { do: 'camera', shot: { frame: 'busiest-relay' } } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: 'Relay down',
      text: 'The scouts on the fire report home through a ladder of relays strung over the ridge.',
    },
  },
  {
    at: 4,
    action: {
      do: 'caption',
      text: 'Now kill the relay carrying the most traffic. No one will tell the swarm.',
    },
  },
  { at: 6, action: { do: 'kill', who: 'busiest-relay' } },
  {
    at: 6,
    action: {
      do: 'caption',
      text: (c) =>
        `${victim(c)} is down with ${c.carried} drone${c.carried === 1 ? '' : 's'} routed through it. Its neighbours only notice the silence: missed heartbeats.`,
    },
  },
  { at: 6.5, action: { do: 'camera', shot: 'killsite' } },
  { at: 6.5, action: { do: 'waitRecovered', timeout: 90 } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => `Healed in ${s(c.recovery)}`,
      text: (c) =>
        `Orphans picked new parents on their own; reserves move up to refill the ladder. ${routed(c)} drones routed to base again, no operator input.`,
    },
  },
  { at: 0.5, action: { do: 'camera', shot: 'overview' } },
  { at: 9, action: { do: 'end' } },
];

const BASE_DESTROYED: Step[] = [
  { at: 0, action: { do: 'camera', shot: 'base' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: 'Base destroyed',
      text: 'OVERWATCH is the base station, the sink every route leads to. Take it away entirely.',
    },
  },
  { at: 5, action: { do: 'kill', who: 'base' } },
  {
    at: 5,
    action: {
      do: 'caption',
      text: 'Base lost. Its witnesses wait out the silence, then the lowest-id survivor declares itself acting sink.',
    },
  },
  { at: 5, action: { do: 'waitRecovered', timeout: 120 } },
  { at: 0, action: { do: 'camera', shot: { follow: 'acting-sink' } } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => `New sink in ${s(c.recovery)}`,
      text: (c) => {
        const sink = select(c.snapshot, 'acting-sink', c);
        const name = sink.length ? callsign(sink[0]) : 'A survivor';
        return `${name} now acts as the sink; ${routed(c)} drones route to it and the mission carries on without a base.`;
      },
    },
  },
  { at: 10, action: { do: 'end' } },
];

const RADIO_STORM: Step[] = [
  { at: 0, action: { do: 'camera', shot: 'swarm' } },
  {
    at: 0,
    action: { do: 'caption', title: 'Radio storm', text: 'Interference: 45% of every radio delivery is about to be lost.' },
  },
  { at: 3, action: { do: 'loss', value: 0.45 } },
  {
    at: 3,
    action: {
      do: 'caption',
      text: 'Heartbeats go missing at random. Each drone estimates link loss and waits longer before calling a neighbour dead, so good links survive.',
    },
  },
  {
    at: 13,
    action: { do: 'caption', text: (c) => `Mid-storm: ${routed(c)} of ${Math.max(0, alive(c) - 1)} drones still routed to base.` },
  },
  { at: 20, action: { do: 'loss', value: 0 } },
  {
    at: 20,
    action: {
      do: 'caption',
      title: 'Storm over',
      text: (c) => `${routed(c)} drones routed. The mesh bent but did not tear down its routes.`,
    },
  },
  { at: 28, action: { do: 'end' } },
];

const FIRE_JUMPS: Step[] = [
  { at: 0, action: { do: 'camera', shot: 'fire' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: 'Fire jumps',
      text: 'Wind carries embers past the main front. Watch for a spot fire: the operator will send the swarm to it with one command.',
    },
  },
  { at: 3, action: { do: 'spotFire' } },
  { at: 3, action: { do: 'camera', shot: 'target' } },
  { at: 3, action: { do: 'caption', text: 'A spot fire takes hold beyond the front.' } },
  { at: 6, action: { do: 'command', intent: Intent.MonitorFire, target: 'jump' } },
  {
    at: 4,
    action: {
      do: 'caption',
      text: 'A new mission floods the mesh. Every drone re-plans its own slot: the relay ladder bends, the scouts re-sector.',
    },
  },
  { at: 7, action: { do: 'waitDeployed', label: 'Re-deploying to the spot fire' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => `Re-deployed in ${s(c.deployedIn)}`,
      text: 'Scouts are over the spot fire. Now back to the main front: map its perimeter.',
    },
  },
  { at: 4, action: { do: 'command', intent: Intent.MapPerimeter, target: 'fire' } },
  { at: 4, action: { do: 'camera', shot: 'fire' } },
  { at: 14, action: { do: 'end' } },
];

const MASS_CASUALTY: Step[] = [
  { at: 0, action: { do: 'camera', shot: 'swarm' } },
  {
    at: 0,
    action: { do: 'caption', title: 'Mass casualty', text: 'Half of the swarm goes down at once. The base is spared.' },
  },
  { at: 4, action: { do: 'kill', who: 'half' } },
  {
    at: 4,
    action: {
      do: 'caption',
      text: (c) => `${c.killed.length} drones lost. The survivors rebuild routes and re-plan the formation with what is left.`,
    },
  },
  { at: 4.5, action: { do: 'waitRecovered', timeout: 150 } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => (Number.isFinite(c.recovery) ? `Recovered in ${s(c.recovery)}` : 'Partial recovery'),
      text: (c) => `${alive(c)} drones left, ${routed(c)} routed to base. A thinner ladder, but still connected.`,
    },
  },
  { at: 0.5, action: { do: 'camera', shot: 'overview' } },
  { at: 10, action: { do: 'end' } },
];

/** Fleet telemetry figures from the latest stats (live drones only). */
const ft = (c: DemoContext): FleetTlm => {
  const snap = c.snapshot;
  return fleetTlm(c.tlm, c.channel, (id) => snap !== null && id < snap.droneCount && snap.alive(id));
};
const hz = (v: number) => `${v.toFixed(1)} Hz`;
const pc = (v: number) => `${Math.round(v)} %`;
const busyPc = (v: number) => `${Math.round(v * 100)} %`;
const online = (c: DemoContext) => (c.channel ? c.channel[CH.online] : 0);
/** Window labels, so a 2 s rate and a 10 s loss beside it can be reconciled. */
const gcsText = (f: FleetTlm) => `${hz(f.gcsHz)} at the GCS (median, last ${RATE_WINDOW_S} s)`;
const lossText = (c: DemoContext, f: FleetTlm) =>
  `loss ${pc(f.lossPct)} (median, last ${LOSS_WINDOW_S} s${c.settled ? '' : ', still settling'})`;
const meshText = (c: DemoContext) =>
  c.meshCarried === 0
    ? `no drone's telemetry was routed through ${victim(c)}, so painlessMesh had nothing to reroute`
    : Number.isFinite(c.meshReroute)
      ? `telemetry from ${c.meshCarried} drone${c.meshCarried === 1 ? '' : 's'} found a new mesh parent after ${s(c.meshReroute)}`
      : `telemetry from ${c.meshCarried} drone${c.meshCarried === 1 ? '' : 's'} still has no new mesh parent`;

const CHANNEL_SATURATION: Step[] = [
  {
    at: 0,
    action: {
      do: 'airtime',
      set: {
        telemetry: true,
        channel: true,
        external: 0,
        variant: Variant.XiaoS3,
        routing: Routing.Single,
        rateMode: RateMode.BackendBudget,
        hz: 25,
        showTree: true,
        showHeat: true,
      },
    },
  },
  { at: 0, action: { do: 'camera', shot: 'overview' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: 'Channel saturation',
      text: 'Besides the brains’ small frames, every real drone streams its firmware telemetry JSON over painlessMesh to OVERWATCH, which bridges it to a WiFi station. All of it shares one 2.4 GHz channel.',
    },
  },
  // Let the 10 s loss window fill at the budget rate before quoting it: the
  // deploy ran at the default fixed 25 Hz, whose gaps would otherwise linger.
  { at: 1, action: { do: 'waitSettled' } },
  { at: 0, action: { do: 'measure', key: 'budget' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: 'Backend budget',
      text: (c) => {
        const f = c.measured.budget ?? ft(c);
        return `The backend shares 60 packages/s across ${online(c).toFixed(0)} nodes: ${hz(f.reqHz)} each. ${f.offeredPps.toFixed(0)} packages/s offered (${f.ppsOnAir.toFixed(0)} frames/s on air with hops and ACKs) · ${gcsText(f)} · ${lossText(c, f)} · busy ${busyPc(f.busyMean)}.`;
      },
    },
  },
  { at: 8, action: { do: 'airtime', set: { rateMode: RateMode.Fixed, hz: 25 } } },
  {
    at: 8,
    action: {
      do: 'caption',
      title: 'Everyone at 25 Hz',
      text: (c) =>
        `The operator overrides the budget: all ${ft(c).senders} drones send at 25 Hz. That is ~${ft(c).senders * 25} packages/s, each forwarded hop by hop and acknowledged, on the same air the brains use.`,
    },
  },
  { at: 9, action: { do: 'waitSettled' } },
  { at: 0, action: { do: 'measure', key: 'collapse' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => ((c.measured.collapse ?? ft(c)).lossPct >= 10 ? 'Collapse' : 'Contention'),
      text: (c) => {
        const f = c.measured.collapse ?? ft(c);
        return `${f.offeredPps.toFixed(0)} packages/s offered to the station (field: clean ≤ 60, collapse ~150), ${f.ppsOnAir.toFixed(0)} frames/s on air with hops and ACKs. ${gcsText(f)} of ${hz(f.reqHz)} sent · ${lossText(c, f)}${f.silent > 0 ? ` · ${f.silent} not heard at all` : ''} · busy ${busyPc(f.busyMean)} mean, ${busyPc(f.busyMax)} max.`;
      },
    },
  },
  { at: 10, action: { do: 'camera', shot: { frame: 'busiest-mesh-relay' } } },
  { at: 10, action: { do: 'caption', text: 'Now kill the drone that forwards the most telemetry in the painlessMesh tree.' } },
  { at: 14, action: { do: 'kill', who: 'busiest-mesh-relay' } },
  {
    at: 14,
    action: {
      do: 'caption',
      text: (c) =>
        `${victim(c)} is down. The brains notice missed heartbeats within seconds; painlessMesh only drops a parent after NODE_TIMEOUT (10 s), then rescans and reconnects.`,
    },
  },
  { at: 14.5, action: { do: 'camera', shot: 'killsite' } },
  { at: 14.5, action: { do: 'waitRecovered', timeout: 90 } },
  { at: 0, action: { do: 'waitMeshRerouted', timeout: 60 } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => `Brain ${s(c.recovery)} · mesh ${c.meshCarried === 0 ? 'n/a' : s(c.meshReroute)}`,
      text: (c) => `The brains rerouted around ${victim(c)} in ${s(c.recovery)}; ${meshText(c)}.`,
    },
  },
  { at: 9, action: { do: 'airtime', set: { rateMode: RateMode.Brain, external: 0.3 } } },
  { at: 9, action: { do: 'camera', shot: 'overview' } },
  {
    at: 9,
    action: {
      do: 'caption',
      title: 'Brain-managed rate, 30 % less air',
      text: 'Phase 5: each brain now sets its own telemetry interval from what it hears on the channel. At the same time a neighbouring network takes 30 % of the airtime.',
    },
  },
  // The brains adapt their intervals over several seconds: give them longer than one loss window.
  { at: 10, action: { do: 'waitSettled', min: 22 } },
  { at: 0, action: { do: 'measure', key: 'brain' } },
  {
    at: 0,
    action: {
      do: 'caption',
      title: (c) => {
        const f = c.measured.brain ?? ft(c);
        const before = c.measured.collapse;
        if (f.lossPct >= 10) return 'Still saturated';
        return before && before.lossPct >= 10 ? 'Recovered' : 'Holding up';
      },
      text: (c) => {
        const f = c.measured.brain ?? ft(c);
        const before = c.measured.collapse;
        const was = before ? ` (at 25 Hz: ${pc(before.lossPct)} loss, ${hz(before.gcsHz)} at the GCS)` : '';
        return `With 30 % of the air gone: ${hz(f.reqHz)} sent · ${gcsText(f)} · ${lossText(c, f)}${was} · busy ${busyPc(f.busyMean)} · ${f.offeredPps.toFixed(0)} packages/s offered, ${f.ppsOnAir.toFixed(0)} frames/s on air.`;
      },
    },
  },
  { at: 13, action: { do: 'end' } },
];

function demo(key: string, name: string, blurb: string, body: Step[]): Demo {
  return { key, name, blurb, steps: body };
}

const SINGLE: Demo[] = [
  demo('relay-down', 'Relay down', 'Kill the busiest relay; watch the mesh heal.', RELAY_DOWN),
  demo('base-destroyed', 'Base destroyed', 'Lose the sink; a survivor takes over.', BASE_DESTROYED),
  demo('radio-storm', 'Radio storm', '45% packet loss for 17 seconds.', RADIO_STORM),
  demo('fire-jumps', 'Fire jumps', 'A spot fire flares up past the front; the swarm retargets mid-mission.', FIRE_JUMPS),
  demo('mass-casualty', 'Mass casualty', 'Half the swarm down at once.', MASS_CASUALTY),
];

/**
 * Every single demo in turn, each on a fresh, deployed swarm. Each demo's
 * `end` becomes a segment mark, so its closing caption keeps its full time
 * before the next reset.
 */
const TOUR: Step[] = SINGLE.flatMap((d, i) => {
  // Name the next demo while resetting / deploying, so the previous one's
  // closing title ("Healed in …") does not linger over the prelude.
  const title = `${i + 1}/${SINGLE.length} · ${d.name}`;
  return [
    ...(i > 0 ? [{ at: 0, action: { do: 'reset', title } } as Step] : []),
    { at: 0, action: { do: 'waitDeployed', onFire: true, title } } as Step,
    ...d.steps.map((st): Step => (st.action.do === 'end' ? { at: st.at, action: { do: 'segment' } } : st)),
  ];
}).concat([
  {
    at: 4,
    action: { do: 'caption', title: 'Tour complete', text: 'Every decision you saw was made on board, by each drone’s own brain.' },
  },
  { at: 10, action: { do: 'end' } },
]);

export const DEMOS: Demo[] = [
  ...SINGLE,
  demo(
    'channel-saturation',
    'Channel saturation',
    'Real telemetry over painlessMesh: budget → 25 Hz collapse → relay kill → brain-managed rate.',
    CHANNEL_SATURATION,
  ),
  demo('tour', 'Full tour', 'All five, back to back.', TOUR),
];

// --- selectors -------------------------------------------------------------------

/** Scratch for throughput counts. */
let through = new Int32Array(0);

function throughput(snap: Snapshot): Int32Array {
  const n = snap.droneCount;
  if (through.length < n) through = new Int32Array(n);
  through.fill(0);
  for (let i = 0; i < n; i++) {
    if (!snap.alive(i)) continue;
    let at = snap.get(i, D.parent);
    for (let k = 0; k < n && at >= 0 && at < n; k++) {
      through[at]++;
      at = snap.get(at, D.parent);
    }
  }
  return through;
}

/** Drone ids `who` names in `snap` (may be empty). */
export function select(snap: Snapshot | null, who: Selector, c: DemoContext): number[] {
  if (!snap) return [];
  const n = snap.droneCount;
  switch (who) {
    case 'busiest-relay': {
      const t = throughput(snap);
      let best = -1;
      let bestScore = -1;
      for (let i = 0; i < n; i++) {
        if (!snap.alive(i) || snap.get(i, D.role) === Role.Sink) continue;
        // Relays beat non-relays; within a class, more traffic wins.
        const score = (snap.get(i, D.role) === Role.Relay ? 10000 : 0) + t[i];
        if (score > bestScore) {
          bestScore = score;
          best = i;
        }
      }
      return best >= 0 ? [best] : [];
    }
    case 'busiest-mesh-relay': {
      // The node the most live drones' telemetry passes through; without a
      // mesh tree (telemetry off), the brains' busiest relay.
      const d = c.tlm;
      const alive = (id: number) => id < n && snap.alive(id);
      let best = -1;
      let bestCount = 0;
      if (d) {
        for (let i = 0; i < Math.min(n, tlmCount(d)); i++) {
          if (i === MESH_ROOT || !snap.alive(i) || snap.get(i, D.role) === Role.Sink) continue;
          const k = treeDescendants(d, i, alive);
          if (k > bestCount) {
            bestCount = k;
            best = i;
          }
        }
      }
      return best >= 0 ? [best] : select(snap, 'busiest-relay', c);
    }
    case 'base':
      return snap.alive(0) ? [0] : [];
    case 'acting-sink': {
      for (let i = 1; i < n; i++) if (snap.alive(i) && snap.get(i, D.role) === Role.Sink) return [i];
      return snap.alive(0) ? [0] : [];
    }
    case 'half': {
      const live: number[] = [];
      for (let i = 0; i < n; i++) if (snap.alive(i) && snap.get(i, D.role) !== Role.Sink) live.push(i);
      // Every other drone by id: spreads the losses over every role.
      return live.filter((_, k) => k % 2 === 1);
    }
    case 'last-killed':
      return c.killed.length ? [c.killed[c.killed.length - 1]] : [];
  }
}

// --- runner ------------------------------------------------------------------------

type Gate =
  | { kind: 'none' }
  | { kind: 'deployed'; since: number; heldFrom: number; label: string }
  | { kind: 'recovered'; since: number; timeout: number }
  | { kind: 'mesh'; since: number; timeout: number }
  | { kind: 'settled'; from: number; timeout: number }
  | { kind: 'reset'; session: number }
  /** A gate resolved, but the current caption has not been up long enough to read. */
  | { kind: 'hold'; until: number };

const DEPLOY_SPEED = 4;
const NEAR_TARGET = 140;
const MIN_SCOUTS = 2;
const DEPLOY_HOLD_S = 3;
const DEPLOY_TIMEOUT_S = 180;
/** Cross-fade between two captions (wall ms). */
/** Sim seconds past one loss window before a mode change is measured (stats lag, in-flight frames). */
const SETTLE_MARGIN_S = 3;
/** Most sim seconds `waitSettled` waits past its minimum for loss and delivered rate to agree. */
const SETTLE_TIMEOUT_S = 15;
/** Per-drone |loss − delivered shortfall| (percentage points, median) taken as agreeing. */
const SETTLE_TOLERANCE_PCT = 12;
const CAPTION_FADE_MS = 280;
/** Minimum time a caption stays readable (wall ms), and reading speed (chars/s). */
const CAPTION_MIN_READ_MS = 3000;
const CAPTION_READ_CPS = 15;

/** Wall time a caption needs on screen to be read. */
export function readMs(text: string): number {
  return Math.max(CAPTION_MIN_READ_MS, (text.length / CAPTION_READ_CPS) * 1000);
}

export class DemoRunner {
  /** Fires when a demo starts or stops (for the menu's active state). */
  onChange: ((active: Demo | null) => void) | null = null;

  private demo: Demo | null = null;
  private pc = 0;
  private segT0 = 0;
  private gate: Gate = { kind: 'none' };
  /** Sim time the current telemetry sample was first seen (NaN: none). */
  private tlmAt = NaN;
  private ctx: DemoContext = freshContext();
  private changedLoss = false;
  /** Settings to restore if the user cancels a demo that changed them. */
  private airtimeBefore: AirtimeSettings | null = null;
  /** Drones whose mesh path ran through the last kill, watched until rerouted. */
  private meshWatch: number[] = [];
  private readonly root: HTMLElement;
  private readonly kicker: HTMLElement;
  private readonly title: HTMLElement;
  private readonly text: HTMLElement;
  private readonly bar: HTMLElement;
  private hideTimer: ReturnType<typeof setTimeout> | undefined;
  private swapTimer: ReturnType<typeof setTimeout> | undefined;
  /** Wall ms until which the current caption must stay (its read time). */
  private holdUntil = 0;
  /** A status caption that arrived during a hold; shown when the hold ends. */
  private deferred: { title: string | null; text: string } | null = null;
  private deferTimer: ReturnType<typeof setTimeout> | undefined;

  constructor(
    root: HTMLElement,
    private readonly host: DemoHost,
  ) {
    this.root = root;
    root.classList.add('caption');
    root.setAttribute('aria-live', 'polite');
    const top = document.createElement('div');
    top.className = 'top';
    this.kicker = document.createElement('span');
    this.kicker.className = 'kicker';
    const cancel = document.createElement('button');
    cancel.className = 'cancel';
    cancel.textContent = 'Cancel  Esc';
    cancel.addEventListener('click', () => this.cancel());
    top.append(this.kicker, cancel);
    this.title = document.createElement('div');
    this.title.className = 'ctitle';
    this.text = document.createElement('div');
    this.text.className = 'ctext';
    const track = document.createElement('div');
    track.className = 'track';
    this.bar = document.createElement('div');
    this.bar.className = 'bar';
    track.append(this.bar);
    root.append(top, this.title, this.text, track);
  }

  get active(): Demo | null {
    return this.demo;
  }

  /** For automated checks: the measurements so far. */
  get context(): DemoContext {
    return this.ctx;
  }

  start(key: string): void {
    const d = DEMOS.find((x) => x.key === key);
    if (!d) return;
    this.stop(false);
    this.demo = d;
    this.ctx = freshContext();
    this.pc = 0;
    clearTimeout(this.hideTimer);
    this.root.classList.add('show');
    this.kicker.textContent = `SHOWCASE · ${d.name.toUpperCase()}`;
    // Start every demo from a clean, deployed swarm, unless it already is one.
    // A live swarm on another intent (Fire jumps leaves it mapping the
    // perimeter) needs no reset: the `onFire` deploy gate re-aims it.
    const snap = this.host.snapshot();
    const pristine = snap !== null && snap.countAlive() === snap.droneCount && !this.changedLoss;
    const prelude: Step[] = [];
    if (!pristine) prelude.push({ at: 0, action: { do: 'reset' } });
    prelude.push({ at: 0, action: { do: 'waitDeployed', onFire: true } });
    this.steps = d.key === 'tour' && !pristine ? [{ at: 0, action: { do: 'reset' } }, ...d.steps] : [...prelude, ...d.steps];
    this.segT0 = snap?.time ?? 0;
    this.gate = { kind: 'none' };
    this.onChange?.(d);
    // Title and blurb go up at once and are held for their read time; the
    // prelude's status lines wait behind them.
    this.clearCaptionTimers();
    this.holdUntil = 0;
    this.setCaption(d.name, d.blurb, true, true);
  }

  /** Steps of the running demo, with the reset / deploy prelude. */
  private steps: Step[] = [];

  cancel(): void {
    if (!this.demo) return;
    this.stop(true);
    this.host.monitor.log('Demo cancelled');
  }

  /** Advances the timeline; call on every snapshot. */
  tick(snap: Snapshot): void {
    if (!this.demo) return;
    this.ctx.snapshot = snap;
    const tlm = this.host.tlm();
    // A new array is a new sample from the worker; note when it was first seen.
    if (tlm !== this.ctx.tlm) this.tlmAt = tlm ? snap.time : NaN;
    this.ctx.tlm = tlm;
    this.ctx.channel = this.host.channel();
    this.watchMesh(snap);
    if (!this.pollGate(snap)) {
      this.progress();
      return;
    }
    // Run every step that is due; stop at the first gate.
    while (this.demo && this.pc < this.steps.length) {
      const step = this.steps[this.pc];
      if (snap.time - this.segT0 < step.at) break;
      this.pc++;
      this.run(step.action, snap);
      if (this.gate.kind !== 'none') break;
    }
    this.progress();
  }

  // --- internals ---

  private stop(byUser: boolean): void {
    if (!this.demo) return;
    this.demo = null;
    this.gate = { kind: 'none' };
    this.host.setSpeed(1);
    if (this.changedLoss && byUser) {
      this.host.setLoss(0);
      this.changedLoss = false;
    }
    if (this.airtimeBefore && byUser) this.host.setAirtime(this.airtimeBefore);
    this.airtimeBefore = null;
    this.meshWatch = [];
    if (byUser) {
      this.host.camera.cancel();
      this.clearCaptionTimers();
    }
    this.onChange?.(null);
    clearTimeout(this.hideTimer);
    this.hideTimer = setTimeout(() => this.root.classList.remove('show'), byUser ? 0 : 6000);
  }

  private progress(): void {
    const total = this.steps.length;
    this.bar.style.width = `${total ? Math.round((this.pc / total) * 100) : 0}%`;
  }

  /** True when no gate is pending (or it just resolved, restarting the segment). */
  private pollGate(snap: Snapshot): boolean {
    const g = this.gate;
    switch (g.kind) {
      case 'none':
        return true;
      case 'reset':
        if (this.host.session() !== g.session || snap.time <= 0) return false;
        break;
      case 'deployed': {
        if (snap.time - g.since > DEPLOY_TIMEOUT_S) {
          this.host.monitor.log('Demo: deployment is taking too long; carrying on', 'warn');
          break;
        }
        if (!this.deployed(snap)) {
          g.heldFrom = NaN;
          this.setCaption(null, `${g.label}… ${this.scoutsNear(snap)}/${MIN_SCOUTS} scouts on target · fast-forward ×${DEPLOY_SPEED}`, true);
          return false;
        }
        if (Number.isNaN(g.heldFrom)) g.heldFrom = snap.time;
        if (snap.time - g.heldFrom < DEPLOY_HOLD_S) return false;
        this.ctx.deployedIn = g.heldFrom - g.since;
        this.host.setSpeed(1);
        break;
      }
      case 'recovered': {
        const r = this.host.monitor.recovery;
        const done = r !== null && r.end !== null;
        const failed = (r !== null && r.timedOut) || snap.time - g.since > g.timeout;
        if (!done && !failed) return false;
        this.ctx.recovery = done && r && r.end !== null ? r.end - this.ctx.killAt : NaN;
        break;
      }
      case 'mesh':
        if (Number.isNaN(this.ctx.meshReroute) && snap.time - g.since <= g.timeout) return false;
        break;
      case 'settled': {
        // Only a stats sample taken after the minimum describes the new mode alone.
        if (snap.time < g.from || !(this.tlmAt >= g.from)) {
          const left = Math.max(0, g.from - snap.time);
          this.setCaption(null, `Measuring the new rate: letting the ${LOSS_WINDOW_S} s loss window refill… ${left.toFixed(0)} s`, true);
          return false;
        }
        const agree = lossAgreesWithRate(this.ctx.tlm, (id) => id < snap.droneCount && snap.alive(id), SETTLE_TOLERANCE_PCT);
        if (!agree && snap.time - g.from < g.timeout) {
          this.setCaption(null, 'Measuring the new rate: waiting for loss and delivered rate to agree…', true);
          return false;
        }
        this.ctx.settled = agree;
        break;
      }
      case 'hold':
        if (performance.now() < g.until) return false;
        break;
    }
    // A fast gate (a 1 s heal) must not cut the caption explaining it short:
    // hold the segment start until that caption has been readable long enough.
    if (g.kind !== 'hold' && performance.now() < this.holdUntil) {
      this.gate = { kind: 'hold', until: this.holdUntil };
      return false;
    }
    this.gate = { kind: 'none' };
    this.segT0 = snap.time;
    return true;
  }

  private run(a: Action, snap: Snapshot): void {
    const host = this.host;
    switch (a.do) {
      case 'caption':
        this.setCaption(a.title === undefined ? null : resolve(a.title, this.ctx), resolve(a.text, this.ctx));
        break;
      case 'camera':
        this.shoot(a.shot, snap);
        break;
      case 'kill': {
        const ids = select(snap, a.who, this.ctx);
        if (a.who === 'busiest-relay' && ids.length) this.ctx.carried = this.host.monitor.descendants(snap, ids[0]);
        this.ctx.killed = [];
        this.ctx.killAt = snap.time;
        this.armMeshWatch(snap, ids);
        for (const id of ids) if (host.kill(id, 'Demo')) this.ctx.killed.push(id);
        break;
      }
      case 'airtime':
        if (!this.airtimeBefore) this.airtimeBefore = { ...host.airtime() };
        host.setAirtime(a.set);
        this.ctx.airtimeAt = snap.time;
        break;
      case 'measure':
        this.ctx.measured[a.key] = fleetTlm(this.ctx.tlm, this.ctx.channel, (id) => id < snap.droneCount && snap.alive(id), newFleetTlm());
        break;
      case 'waitMeshRerouted':
        this.gate = { kind: 'mesh', since: snap.time, timeout: a.timeout };
        break;
      case 'waitSettled': {
        const base = Number.isNaN(this.ctx.airtimeAt) ? snap.time : this.ctx.airtimeAt;
        this.gate = { kind: 'settled', from: base + (a.min ?? LOSS_WINDOW_S + SETTLE_MARGIN_S), timeout: SETTLE_TIMEOUT_S };
        break;
      }
      case 'loss':
        host.setLoss(a.value);
        this.changedLoss = a.value !== 0;
        break;
      case 'spotFire': {
        const j = this.jumpPoint();
        // A few cells, so the flare-up is visible at once and catches reliably.
        for (const [dx, dz] of SPOT_FIRE_OFFSETS) host.ignite(j.x + dx, j.z + dz);
        break;
      }
      case 'command': {
        const t = a.target === 'jump' ? this.jumpPoint() : host.fireCentre();
        this.ctx.target = t;
        host.command(a.intent, t.x, t.z);
        break;
      }
      case 'speed':
        host.setSpeed(a.value);
        break;
      case 'waitDeployed': {
        const t = host.lastTarget();
        const f = host.fireCentre();
        // The gate counts scouts within NEAR_TARGET of the target, which only
        // MonitorFire guarantees (MapPerimeter pushes scouts out to ~150).
        const onTarget = a.onFire
          ? t !== null && snap.baseIntent === Intent.MonitorFire && Math.hypot(t.x - f.x, t.z - f.z) <= NEAR_TARGET
          : t !== null;
        if (!onTarget) host.command(Intent.MonitorFire, f.x, f.z);
        this.ctx.target = host.lastTarget() ?? f;
        host.setSpeed(DEPLOY_SPEED);
        const label = a.label ?? 'Deploying to the fire';
        this.gate = { kind: 'deployed', since: snap.time, heldFrom: NaN, label };
        if (host.camera.kind === 'idle') host.overview();
        this.setCaption(a.title ?? null, `${label}…`, true);
        break;
      }
      case 'waitRecovered':
        this.gate = { kind: 'recovered', since: snap.time, timeout: a.timeout };
        break;
      case 'reset':
        host.reset();
        this.changedLoss = false;
        this.gate = { kind: 'reset', session: host.session() };
        this.setCaption(a.title ?? null, 'Fresh swarm, fresh fire: resetting…', true);
        break;
      case 'segment':
        this.segT0 = snap.time;
        break;
      case 'end':
        this.pc = this.steps.length;
        this.kicker.textContent = `SHOWCASE · ${this.demo?.name.toUpperCase() ?? ''} · DONE`;
        this.stop(false);
        break;
    }
  }

  /** Records which live drones' painlessMesh paths run through the drones about to die. */
  private armMeshWatch(snap: Snapshot, victims: number[]): void {
    this.meshWatch = [];
    this.ctx.meshReroute = NaN;
    const d = this.ctx.tlm;
    if (d) {
      const n = Math.min(snap.droneCount, tlmCount(d));
      for (let i = 0; i < n; i++) {
        if (!snap.alive(i) || victims.includes(i)) continue;
        let at = tlmField(d, i, TLM.treeParent);
        for (let k = 0; k < n && at >= 0; k++) {
          if (victims.includes(at)) {
            this.meshWatch.push(i);
            break;
          }
          at = tlmField(d, at, TLM.treeParent);
        }
      }
    }
    this.ctx.meshCarried = this.meshWatch.length;
    if (this.meshWatch.length === 0) this.ctx.meshReroute = 0;
  }

  /** Marks the mesh rerouted once every watched (still live) drone has a live parent path again. */
  private watchMesh(snap: Snapshot): void {
    const d = this.ctx.tlm;
    if (!d || this.meshWatch.length === 0 || !Number.isNaN(this.ctx.meshReroute)) return;
    const n = Math.min(snap.droneCount, tlmCount(d));
    for (const i of this.meshWatch) {
      if (i >= n || !snap.alive(i)) continue;
      if (tlmField(d, i, TLM.treeHops) < 0) return;
      // The whole path must be live: a parent that is down but not yet timed out still counts as broken.
      let at = tlmField(d, i, TLM.treeParent);
      for (let k = 0; k < n && at >= 0; k++) {
        if (at >= n || !snap.alive(at)) return;
        at = tlmField(d, at, TLM.treeParent);
      }
    }
    this.ctx.meshReroute = snap.time - this.ctx.killAt;
  }

  private scoutsNear(snap: Snapshot): number {
    const t = this.ctx.target;
    const p = this.host.positions();
    let near = 0;
    for (let i = 0; i < snap.droneCount; i++) {
      if (!snap.alive(i) || snap.get(i, D.role) !== Role.Scout) continue;
      const x = i * 3 < p.length ? p[i * 3] : snap.get(i, D.x);
      const z = i * 3 + 2 < p.length ? p[i * 3 + 2] : snap.get(i, D.z);
      if (Math.hypot(x - t.x, z - t.z) <= NEAR_TARGET) near++;
    }
    return near;
  }

  /** Mirrors the sim's scenario check: scouts on target and every scout routed. */
  private deployed(snap: Snapshot): boolean {
    if (this.scoutsNear(snap) < MIN_SCOUTS) return false;
    for (let i = 0; i < snap.droneCount; i++) {
      if (snap.alive(i) && snap.get(i, D.role) === Role.Scout && !snap.routeIntact(i)) return false;
    }
    return true;
  }

  /**
   * The spot-fire point, ~280 units from the main fire, on whichever side stays
   * on the map. The Fire jumps demo lights a spot fire here before sending
   * the swarm, so it must be stable within one demo run.
   */
  private jumpPoint(): { x: number; z: number } {
    const f = this.host.fireCentre();
    const half = this.host.extent() / 2 - 80;
    const candidates = [
      { x: f.x, z: f.z + 280 },
      { x: f.x, z: f.z - 280 },
      { x: f.x + 200, z: f.z + 200 },
      { x: f.x - 200, z: f.z - 200 },
    ];
    for (const c of candidates) if (Math.abs(c.x) <= half && Math.abs(c.z) <= half) return c;
    return { x: Math.max(-half, Math.min(half, f.x)), z: Math.max(-half, Math.min(half, f.z + 200)) };
  }

  private shoot(shot: Shot, snap: Snapshot): void {
    const cam = this.host.camera;
    const at = (x: number, z: number, dist: number, elev = 0.62) => cam.frame(x, this.host.groundAt(x, z) + 20, z, dist, elev);
    const p = this.host.positions();
    if (shot === 'overview') this.host.overview();
    else if (shot === 'fire') {
      const f = this.host.fireCentre();
      at(f.x, f.z, 560);
    } else if (shot === 'target') {
      const t = this.ctx.target;
      at(t.x, t.z, 640, 0.7);
    } else if (shot === 'base') {
      at(snap.get(0, D.x), snap.get(0, D.z), 300, 0.45);
    } else if (shot === 'killsite') {
      const id = this.ctx.killed[0];
      if (id !== undefined) at(p[id * 3], p[id * 3 + 2], 360, 0.5);
    } else if (shot === 'swarm') {
      let sx = 0;
      let sz = 0;
      let n = 0;
      for (let i = 0; i < snap.droneCount; i++) {
        if (!snap.alive(i)) continue;
        sx += snap.get(i, D.x);
        sz += snap.get(i, D.z);
        n++;
      }
      if (n) at(sx / n, sz / n, 900, 0.75);
    } else if ('follow' in shot) {
      const id = select(snap, shot.follow, this.ctx)[0];
      if (id !== undefined) cam.follow(id, 260);
    } else {
      const id = select(snap, shot.frame, this.ctx)[0];
      if (id !== undefined) at(snap.get(id, D.x), snap.get(id, D.z), 380, 0.55);
    }
  }

  /**
   * `title` null keeps the current title. A story caption (not sticky)
   * cross-fades in and is then held for its read time. A sticky caption
   * (gate status) updates in place, but waits while a story caption is still
   * being read, so it can neither cut that short nor drop its pending title.
   * `hold` makes a sticky caption hold like a story one (the demo's opening).
   */
  private setCaption(title: string | null, text: string, sticky = false, hold = !sticky): void {
    const now = performance.now();
    if (sticky) {
      if (now < this.holdUntil) {
        this.deferred = { title: title ?? this.deferred?.title ?? null, text };
        if (this.deferTimer === undefined) {
          this.deferTimer = setTimeout(() => {
            this.deferTimer = undefined;
            const d = this.deferred;
            this.deferred = null;
            if (d) this.setCaption(d.title, d.text, true, false);
          }, this.holdUntil - now);
        }
        return;
      }
      this.root.classList.remove('swap');
      this.text.textContent = text;
      if (title !== null) this.title.textContent = title;
      if (hold) this.holdUntil = now + readMs(text);
      return;
    }
    // A newer story caption supersedes any status line still waiting.
    this.deferred = null;
    clearTimeout(this.deferTimer);
    this.deferTimer = undefined;
    clearTimeout(this.swapTimer);
    this.root.classList.add('swap');
    if (hold) this.holdUntil = now + CAPTION_FADE_MS + readMs(text);
    this.swapTimer = setTimeout(() => {
      if (title !== null) this.title.textContent = title;
      this.text.textContent = text;
      this.root.classList.remove('swap');
    }, CAPTION_FADE_MS);
  }

  private clearCaptionTimers(): void {
    clearTimeout(this.swapTimer);
    clearTimeout(this.deferTimer);
    this.deferTimer = undefined;
    this.deferred = null;
    this.root.classList.remove('swap');
  }
}

function resolve(t: Text, c: DemoContext): string {
  return typeof t === 'function' ? t(c) : t;
}

function freshContext(): DemoContext {
  return {
    killed: [],
    killAt: 0,
    recovery: NaN,
    deployedIn: NaN,
    carried: 0,
    snapshot: null,
    target: { x: 0, z: 0 },
    tlm: null,
    channel: null,
    meshCarried: 0,
    meshReroute: NaN,
    measured: {},
    airtimeAt: NaN,
    settled: true,
  };
}

