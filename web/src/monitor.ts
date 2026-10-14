// Derives what the operator should notice from successive snapshots: an event
// log, per-role counts and a recovery timer after a kill. No DOM here.
// Every event is also handed to subscribers (the radio console prints them
// as SYSTEM lines), so the log panel and the console never disagree.

import { callsign } from './callsigns';
import { INTENT_NAMES } from './protocol';
import { D, NO_ROUTE, ROLE_NAMES, Role, Snapshot } from './snapshot';

const MAX_EVENTS = 8;
/** Give up timing a recovery after this much sim time. */
const RECOVERY_TIMEOUT_S = 120;

export type Tone = 'info' | 'good' | 'warn' | 'bad';

export interface LogEvent {
  time: number;
  text: string;
  tone: Tone;
  /** The drone the event is about, or -1; lets the console filter by drone. */
  subject: number;
}

export interface Recovery {
  victim: number;
  start: number;
  /** Sim time at which every live drone had an intact route again. */
  end: number | null;
  timedOut: boolean;
}

export class SwarmMonitor {
  /** Newest first. */
  readonly events: LogEvent[] = [];
  /** Bumped whenever `events` changes, so the DOM only redraws then. */
  version = 0;
  readonly roleCounts = [0, 0, 0, 0];
  noRoute = 0;
  recovery: Recovery | null = null;
  /** Sim time of the latest snapshot. */
  time = 0;
  /** Drones that went down in the latest observed snapshot. */
  readonly downed: number[] = [];
  /** Called with every event as it is logged. */
  readonly listeners: ((e: LogEvent) => void)[] = [];

  /** Drones that lost their route and have not been logged as rerouted yet. */
  private readonly orphaned = new Set<number>();
  /** Scratch for counting the drones routed through a casualty. */
  private through = new Int32Array(0);

  observe(prev: Snapshot | null, curr: Snapshot): void {
    this.time = curr.time;
    this.downed.length = 0;
    this.roleCounts.fill(0);
    this.noRoute = 0;
    for (let i = 0; i < curr.droneCount; i++) {
      if (!curr.alive(i)) continue;
      this.roleCounts[curr.get(i, D.role)]++;
      if (curr.get(i, D.hops) === NO_ROUTE) this.noRoute++;
    }
    if (prev && prev.droneCount === curr.droneCount) this.diff(prev, curr);
    this.trackRecovery(curr);
  }

  /** Starts (or restarts) the recovery timer for a kill sent just now. */
  noteKill(victim: number, who = 'Operator'): void {
    this.recovery = { victim, start: this.time, end: null, timedOut: false };
    this.log(`${who} killed ${callsign(victim)}`, 'bad', victim);
  }

  log(text: string, tone: Tone = 'info', subject = -1): void {
    const event: LogEvent = { time: this.time, text, tone, subject };
    this.events.unshift(event);
    if (this.events.length > MAX_EVENTS) this.events.length = MAX_EVENTS;
    this.version++;
    for (const fn of this.listeners) fn(event);
  }

  reset(): void {
    this.events.length = 0;
    this.version++;
    this.roleCounts.fill(0);
    this.noRoute = 0;
    this.recovery = null;
    this.time = 0;
    this.downed.length = 0;
    this.orphaned.clear();
  }

  /** How many live drones' parent chains in `s` pass through `id`. */
  descendants(s: Snapshot, id: number): number {
    const n = s.droneCount;
    let count = 0;
    for (let i = 0; i < n; i++) {
      if (i === id || !s.alive(i)) continue;
      let at = s.get(i, D.parent);
      for (let step = 0; step < n && at >= 0 && at < n; step++) {
        if (at === id) {
          count++;
          break;
        }
        at = s.get(at, D.parent);
      }
    }
    return count;
  }

  /** Per-drone count of other drones routed through it, into a reused array. */
  throughput(s: Snapshot): Int32Array {
    const n = s.droneCount;
    if (this.through.length < n) this.through = new Int32Array(n);
    const t = this.through;
    t.fill(0);
    for (let i = 0; i < n; i++) {
      if (!s.alive(i)) continue;
      let at = s.get(i, D.parent);
      for (let step = 0; step < n && at >= 0 && at < n; step++) {
        t[at]++;
        at = s.get(at, D.parent);
      }
    }
    return t;
  }

  private diff(prev: Snapshot, curr: Snapshot): void {
    if (prev.baseIntent !== curr.baseIntent) {
      this.log(`Swarm intent → ${INTENT_NAMES[curr.baseIntent] ?? `#${curr.baseIntent}`}`, 'info');
    }
    for (let i = 0; i < curr.droneCount; i++) {
      const name = callsign(i);
      if (prev.alive(i) && !curr.alive(i)) {
        this.downed.push(i);
        this.orphaned.delete(i);
        const wasSink = prev.get(i, D.role) === Role.Sink;
        const riders = this.descendants(prev, i);
        const tail = riders > 0 ? ` — ${riders} drone${riders === 1 ? '' : 's'} rerouting` : '';
        this.log(wasSink ? `${name} down — sink lost${tail}` : `${name} lost${tail}`, 'bad', i);
        continue;
      }
      if (!curr.alive(i)) continue;
      const role = curr.get(i, D.role);
      const oldRole = prev.get(i, D.role);
      if (role !== oldRole) {
        if (role === Role.Sink) this.log(`${name} elected acting sink`, 'warn', i);
        else if (oldRole === Role.Sink) this.log(`${name} stands down as sink → ${ROLE_NAMES[role] ?? role}`, 'good', i);
        else this.log(`${name} → ${ROLE_NAMES[role] ?? `role ${role}`}`, 'info', i);
      }

      const hops = curr.get(i, D.hops);
      const wasRouted = prev.get(i, D.hops) !== NO_ROUTE;
      const parent = curr.get(i, D.parent);
      const oldParent = prev.get(i, D.parent);
      if (wasRouted && hops === NO_ROUTE) {
        this.orphaned.add(i);
        this.log(`${name} lost route`, 'warn', i);
      } else if (hops !== NO_ROUTE && (this.orphaned.has(i) || (parent !== oldParent && oldParent >= 0 && !curr.alive(oldParent)))) {
        // Only reroutes after a failure are news; routine parent swaps are not.
        this.orphaned.delete(i);
        const via = parent >= 0 ? `via ${callsign(parent)}` : 'direct';
        this.log(`${name} rerouted ${via} (${hops} hop${hops === 1 ? '' : 's'})`, 'good', i);
      }
    }
  }

  private trackRecovery(curr: Snapshot): void {
    const r = this.recovery;
    if (!r || r.end !== null || r.timedOut) return;
    // Wait until the kill has reached the sim.
    if (r.victim < curr.droneCount && curr.alive(r.victim)) return;
    if (curr.time - r.start > RECOVERY_TIMEOUT_S) {
      r.timedOut = true;
      this.log(`No recovery within ${RECOVERY_TIMEOUT_S} s`, 'bad');
      return;
    }
    for (let i = 0; i < curr.droneCount; i++) {
      if (!curr.alive(i) || curr.get(i, D.role) === Role.Sink) continue;
      if (!curr.routeIntact(i)) return;
    }
    r.end = curr.time;
    this.log(`Recovered in ${(r.end - r.start).toFixed(1)} s — every drone routed again`, 'good');
  }
}
