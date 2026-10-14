// Radio console: a terminal docked at the bottom of the page that prints the
// decoded radio traffic (see chatter.ts) and the monitor's SYSTEM events with
// callsigns, plus filters along its bottom edge.
//
// Cost model: records land in a fixed ring of typed arrays (no per-message
// objects); the DOM holds at most MAX_LINES pooled line elements, and new
// lines are written at most RENDER_HZ times a second by recycling the oldest
// element, so steady-state work is proportional to new lines, not to the
// buffer. Line text is built only when a line is actually shown.

import { callsign, parseCallsign } from '../callsigns';
import {
  C,
  CHATTER_STRIDE,
  ChatterKind,
  Flag,
  HB,
  MI,
  OPERATOR_ID,
  TL,
  chatterCount,
  flagNames,
  slotName,
} from '../chatter';
import type { LogEvent, Tone } from '../monitor';
import { INTENT_NAMES } from '../protocol';
import { type AirBytes, STATION_ID, TLM, TLM_STRIDE, airHeader } from '../telemetry';

/** Entries remembered (all kinds, before filtering). */
const CAP = 4096;
/** Line elements in the DOM at most. */
const MAX_LINES = 400;
const RENDER_INTERVAL_MS = 100;
const RATE_WINDOW_MS = 1000;
/** Pixels from the bottom within which the view keeps following new lines. */
const STICK_SLOP_PX = 24;
const MIN_HEIGHT_PX = 120;
const COLLAPSED_PX = 32;
const STORAGE_KEY = 'swarm-mind.console';

/** Entry kinds; also the filter toggles' keys. */
const K = { Hb: 0, Intent: 1, Mission: 2, System: 3, Raw: 4, Tlm: 5, PmSync: 6, PmTime: 7, Uplink: 8 } as const;
type EntryKind = (typeof K)[keyof typeof K];
const KIND_KEYS = ['hb', 'intent', 'mission', 'system', 'tlm', 'pmsync', 'pmtime', 'uplink'] as const;
type KindKey = (typeof KIND_KEYS)[number];
/** Line label and CSS class per entry kind. */
const KIND_LABEL: readonly string[] = ['HB', 'INTENT', 'MISSION', '', 'RAW', 'TLM', 'PM-SYNC', 'PM-TIME', 'UPLINK'];
const KIND_CLASS: readonly string[] = ['hb', 'intent', 'mission', 'sys', 'raw', 'tlm', 'pm', 'pm', 'uplink'];
/** painlessMesh package types (protocol.hpp) for PM-SYNC / PM-TIME lines. */
const PM_TYPE_NAMES: Record<number, string> = {
  3: 'TIME_DELAY',
  4: 'TIME_SYNC',
  5: 'NODE_SYNC_REQUEST',
  6: 'NODE_SYNC_REPLY',
  8: 'BROADCAST',
  9: 'SINGLE',
};

/** Sender role of an operator intent, so the BASE toggle covers it. */
const ROLE_OPERATOR = 4;
const ROLE_KEYS = ['reserve', 'scout', 'relay', 'base'] as const;
type RoleKey = (typeof ROLE_KEYS)[number];
const ROLE_WORD = ['reserve', 'scout', 'relay', 'base'];
const TONES: readonly Tone[] = ['info', 'good', 'warn', 'bad'];
/** Heartbeat fields compared by "changes only" (every field the beacon carries). */
const HB_FIELDS = [HB.hops, HB.parent, HB.role, HB.sink, HB.slot, HB.flags, HB.need, HB.intentSeq];
const NF = HB_FIELDS.length;
const MAX_SENDERS = 4096;

export interface ConsoleHost {
  /** A callsign was clicked: select / follow that drone (-1 = stop). */
  select(id: number): void;
  /** The TLM filter was switched: subscribe to / drop the package-text samples. */
  tlmFilter?(on: boolean): void;
}

interface Row {
  el: HTMLDivElement;
  t: HTMLSpanElement;
  src: HTMLSpanElement;
  arr: HTMLSpanElement;
  dst: HTMLSpanElement;
  k: HTMLSpanElement;
  b: HTMLSpanElement;
}

interface Settings {
  kinds: Record<KindKey, boolean>;
  roles: Record<RoleKey, boolean>;
  changesOnly: boolean;
  heightFrac: number;
  collapsed: boolean;
}

function defaults(): Settings {
  return {
    kinds: { hb: true, intent: true, mission: true, system: true, tlm: true, pmsync: true, pmtime: true, uplink: false },
    roles: { reserve: true, scout: true, relay: true, base: true },
    changesOnly: true,
    heightFrac: 0.28,
    collapsed: false,
  };
}

function loadSettings(): Settings {
  const s = defaults();
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return s;
    const v = JSON.parse(raw) as Partial<Settings>;
    Object.assign(s.kinds, v.kinds);
    Object.assign(s.roles, v.roles);
    if (typeof v.changesOnly === 'boolean') s.changesOnly = v.changesOnly;
    if (typeof v.heightFrac === 'number' && v.heightFrac > 0.05 && v.heightFrac < 0.9) s.heightFrac = v.heightFrac;
    if (typeof v.collapsed === 'boolean') s.collapsed = v.collapsed;
  } catch {
    // Storage blocked or corrupt: defaults are fine.
  }
  return s;
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls = '', text = ''): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

function fmtTime(s: number): string {
  return `[${s.toFixed(2).padStart(6, '0')}]`;
}

function fmtXZ(x: number, z: number): string {
  return `(${Math.round(x)}, ${Math.round(z)})`;
}

export class RadioConsole {
  /** Fires with the console's on-screen height whenever it changes. */
  onResize: ((heightPx: number) => void) | null = null;

  // --- entry ring (struct of arrays) ---
  private readonly kind = new Uint8Array(CAP);
  private readonly time = new Float64Array(CAP);
  private readonly src = new Int32Array(CAP);
  private readonly dst = new Int32Array(CAP);
  private readonly role = new Int8Array(CAP);
  private readonly subject = new Int32Array(CAP);
  private readonly changed = new Uint8Array(CAP);
  private readonly tone = new Uint8Array(CAP);
  private readonly fields = new Float32Array(CAP * 8);
  /** SYSTEM text, or a shown line's cached body. */
  private readonly text: (string | null)[] = new Array<string | null>(CAP).fill(null);
  /** Entries ever pushed; entry `seq` lives at `seq % CAP`. */
  private total = 0;
  /** Entries before this are cleared. */
  private floor = 0;

  // --- per-sender memory for "changes only" and role filtering ---
  private readonly lastHb = new Float32Array(MAX_SENDERS * NF);
  private readonly hasHb = new Uint8Array(MAX_SENDERS);
  private readonly lastMissionSeq = new Float32Array(MAX_SENDERS).fill(-1);
  private readonly roleOf = new Int8Array(MAX_SENDERS).fill(-1);
  /** Latest exact package text per drone ('tlmsample') and its "s" (-1: none). */
  private readonly sample: string[] = [];
  private readonly sampleSeq = new Float64Array(MAX_SENDERS).fill(-1);
  private tlmStats: Float32Array | null = null;

  // --- DOM ---
  private readonly root: HTMLElement;
  private readonly list: HTMLDivElement;
  private readonly pool: Row[] = [];
  /** Shown rows in DOM order, as a circular buffer starting at `head`. */
  private readonly order: Row[] = [];
  private head = 0;
  private shown = 0;
  /** Seq of the newest entry considered for display. */
  private renderedTo = 0;
  private needsRebuild = true;
  private stick = true;
  private paused = false;
  private renderAt = 0;
  private readonly settings: Settings;
  private droneFilter = -1;
  private followId = -1;
  private droneCount = 0;

  private readonly rateEl: HTMLElement;
  private readonly statusEl: HTMLElement;
  private readonly jumpEl: HTMLButtonElement;
  private readonly pauseEl: HTMLButtonElement;
  private readonly searchEl: HTMLInputElement;
  private readonly datalist: HTMLDataListElement;
  private readonly followEl: HTMLElement;
  private readonly collapsedLast: HTMLElement;
  private readonly kindButtons = new Map<KindKey, HTMLButtonElement>();
  private readonly roleButtons = new Map<RoleKey, HTMLButtonElement>();
  private readonly changesEl: HTMLButtonElement;

  // --- rates ---
  private rateCount = 0;
  private rateAt = 0;
  private msgsPerSec = 0;
  private brainFramesPerSec = 0;
  private brainBytesPerSec = 0;
  /** Channel byte counters, or null: then the header counts brain frames only. */
  private air: AirBytes | null = null;
  private airLegacy = false;
  private readonly airEl: HTMLElement;
  private readonly mixEl: HTMLElement;
  private airTitle = '';
  /** Scratch: seqs selected for display, newest last. */
  private readonly pick = new Int32Array(MAX_LINES);

  constructor(root: HTMLElement, private readonly host: ConsoleHost) {
    this.root = root;
    this.settings = loadSettings();
    root.classList.add('radio');
    root.setAttribute('role', 'log');
    root.setAttribute('aria-label', 'Radio console');

    // Header: title, live rates, follow chip, collapse.
    const grip = el('div', 'grip');
    grip.title = 'Drag to resize';
    const head = el('div', 'head');
    const collapse = el('button', 'icon', '▾');
    collapse.title = 'Collapse / expand (`)';
    collapse.addEventListener('click', () => this.toggle());
    head.addEventListener('dblclick', (e) => {
      if (e.target === head) this.toggle();
    });
    const title = el('span', 'title', 'RADIO NET');
    const sub = el('span', 'sub', 'decoded mesh traffic');
    this.rateEl = el('span', 'rate', '— msg/s');
    this.rateEl.title = 'Records decoded into this console per second (one line each); not frames on air';
    this.mixEl = el('span', 'mix');
    this.airEl = el('span', 'air', 'brain —');
    this.followEl = el('span', 'chip');
    this.followEl.hidden = true;
    this.followEl.addEventListener('click', () => this.host.select(-1));
    this.collapsedLast = el('span', 'lastline');
    this.statusEl = el('span', 'status');
    head.append(collapse, title, sub, this.collapsedLast, this.statusEl, this.followEl, this.mixEl, this.rateEl, this.airEl);

    // Scrollback.
    this.list = el('div', 'lines');
    this.list.addEventListener('scroll', () => {
      const l = this.list;
      const atBottom = l.scrollTop + l.clientHeight >= l.scrollHeight - STICK_SLOP_PX;
      if (atBottom !== this.stick) {
        this.stick = atBottom;
        this.updateStatus();
      }
    });
    this.list.addEventListener('click', (e) => {
      const target = (e.target as HTMLElement).closest<HTMLElement>('[data-id]');
      if (!target) return;
      const id = Number(target.dataset.id);
      if (id >= 0 && id !== OPERATOR_ID) this.host.select(id);
    });
    this.jumpEl = el('button', 'jump', '▼ live');
    this.jumpEl.hidden = true;
    this.jumpEl.addEventListener('click', () => {
      this.stick = true;
      this.list.scrollTop = this.list.scrollHeight;
      this.updateStatus();
    });
    for (let i = 0; i < MAX_LINES; i++) this.pool.push(this.makeRow());

    // Filters along the bottom.
    const bar = el('div', 'filters');
    const kinds = el('div', 'group');
    kinds.append(el('span', 'label', 'MSG'));
    const kindLabels: Record<'hb' | 'mission' | 'intent' | 'system', string> = { hb: 'HB', mission: 'MISSION', intent: 'INTENT', system: 'SYSTEM' };
    for (const key of ['hb', 'mission', 'intent', 'system'] as const) {
      const b = this.toggleButton(kindLabels[key], `k-${key}`, () => {
        this.settings.kinds[key] = !this.settings.kinds[key];
        this.filtersChanged();
      });
      this.kindButtons.set(key, b);
      kinds.append(b);
    }
    const air = el('div', 'group');
    air.append(el('span', 'label', 'AIR'));
    const airLabels: Record<KindKey, string> = {
      hb: '',
      mission: '',
      intent: '',
      system: '',
      tlm: 'TLM',
      pmsync: 'PM-SYNC',
      pmtime: 'PM-TIME',
      uplink: 'UPLINK',
    };
    const airTitles: Record<string, string> = {
      tlm: 'Telemetry packages over painlessMesh, with the exact package text (one line per drone per second)',
      pmsync: 'painlessMesh NODE_SYNC request / reply (subtree exchange)',
      pmtime: 'painlessMesh TIME_SYNC / TIME_DELAY',
      uplink: 'Base → WiFi station UDP datagrams (one line per drone per second)',
    };
    for (const key of ['tlm', 'pmsync', 'pmtime', 'uplink'] as const) {
      const b = this.toggleButton(airLabels[key], `k-${key}`, () => {
        this.settings.kinds[key] = !this.settings.kinds[key];
        this.filtersChanged();
        if (key === 'tlm') this.host.tlmFilter?.(this.settings.kinds.tlm);
      });
      b.title = airTitles[key];
      this.kindButtons.set(key, b);
      air.append(b);
    }
    const roles = el('div', 'group');
    roles.append(el('span', 'label', 'ROLE'));
    for (const key of ROLE_KEYS) {
      const b = this.toggleButton(key.toUpperCase(), `r-${key}`, () => {
        this.settings.roles[key] = !this.settings.roles[key];
        this.filtersChanged();
      });
      this.roleButtons.set(key, b);
      roles.append(b);
    }
    const drone = el('div', 'group search');
    drone.append(el('span', 'label', 'DRONE'));
    this.searchEl = el('input');
    this.searchEl.type = 'search';
    this.searchEl.id = 'console-drone-search';
    this.searchEl.name = 'console-drone-search';
    this.searchEl.placeholder = 'callsign or id';
    this.searchEl.spellcheck = false;
    this.searchEl.autocomplete = 'off';
    this.datalist = el('datalist');
    this.datalist.id = 'callsign-options';
    this.searchEl.setAttribute('list', this.datalist.id);
    this.searchEl.addEventListener('input', () => {
      const v = this.searchEl.value;
      const id = parseCallsign(v);
      const valid = v.trim() === '' || (id >= 0 && (this.droneCount === 0 || id < this.droneCount));
      this.searchEl.classList.toggle('bad', !valid);
      if (valid) this.setDroneFilter(v.trim() === '' ? -1 : id, false);
    });
    this.searchEl.addEventListener('keydown', (e) => {
      if (e.key === 'Escape') {
        this.searchEl.value = '';
        this.setDroneFilter(-1, false);
        this.searchEl.blur();
        e.stopPropagation();
      } else if (e.key === 'Enter' && this.droneFilter >= 0) {
        this.host.select(this.droneFilter);
      }
    });
    drone.append(this.searchEl, this.datalist);
    const view = el('div', 'group');
    this.changesEl = this.toggleButton('CHANGES ONLY', 'changes', () => {
      this.settings.changesOnly = !this.settings.changesOnly;
      this.filtersChanged();
    });
    this.changesEl.title = 'Hide heartbeats that repeat the sender’s previous one, and mission re-floods';
    this.pauseEl = el('button', 'btn', '❚❚ PAUSE');
    this.pauseEl.addEventListener('click', () => this.setPaused(!this.paused));
    const clear = el('button', 'btn', 'CLEAR');
    clear.addEventListener('click', () => this.clear());
    view.append(this.changesEl, this.pauseEl, clear);
    bar.append(kinds, air, roles, drone, view);

    const body = el('div', 'body');
    body.append(this.list, this.jumpEl);
    root.append(grip, head, body, bar);
    this.installResize(grip);
    this.syncButtons();
    this.applyHeight();
    window.addEventListener('resize', () => this.applyHeight());
  }

  // --- input -----------------------------------------------------------------

  /** Appends a batch of chatter records (the worker's 'chatter' message). */
  ingest(data: Float32Array): void {
    const n = chatterCount(data);
    this.rateCount += n;
    for (let r = 0; r < n; r++) {
      const o = r * CHATTER_STRIDE;
      const rawKind = data[o + C.kind];
      const srcId = data[o + C.src];
      const i = this.push(entryKind(rawKind), data[o + C.tMs] / 1000, srcId, data[o + C.dst]);
      const f = i * 8;
      for (let k = 0; k < 8; k++) this.fields[f + k] = data[o + C.f0 + k];
      const s = srcId >= 0 && srcId < MAX_SENDERS ? srcId : -1;
      const kind = this.kind[i];
      if (kind === K.Hb && s >= 0) {
        const role = data[o + HB.role];
        this.roleOf[s] = role;
        this.role[i] = role;
        let diff = this.hasHb[s] === 0;
        for (let k = 0; k < NF; k++) {
          const v = data[o + HB_FIELDS[k]];
          if (this.lastHb[s * NF + k] !== v) diff = true;
          this.lastHb[s * NF + k] = v;
        }
        this.hasHb[s] = 1;
        this.changed[i] = diff ? 1 : 0;
      } else if (kind === K.Intent) {
        this.role[i] = srcId === OPERATOR_ID ? ROLE_OPERATOR : s >= 0 ? this.roleOf[s] : -1;
        this.changed[i] = 1;
      } else if (kind === K.Mission && s >= 0) {
        this.role[i] = this.roleOf[s];
        const seq = data[o + MI.seq];
        this.changed[i] = this.lastMissionSeq[s] !== seq ? 1 : 0;
        this.lastMissionSeq[s] = seq;
      } else if (kind === K.Tlm || kind === K.PmSync || kind === K.PmTime || kind === K.Uplink) {
        this.role[i] = s >= 0 ? this.roleOf[s] : -1;
        this.changed[i] = 1;
        // Bind the package text now, and only the text of this very package (same "s").
        if (kind === K.Tlm) this.text[i] = this.describeTlm(i, s);
      } else {
        this.role[i] = -1;
        this.changed[i] = 1;
      }
    }
  }

  /**
   * The exact package text of drone `id`'s package with sequence `seq` ('tlmsample').
   * The worker posts it before the chatter batch carrying that package's TLM record.
   */
  setTlmSample(id: number, seq: number, text: string): void {
    if (id < 0 || id >= MAX_SENDERS) return;
    this.sample[id] = text;
    this.sampleSeq[id] = seq;
  }

  /** Latest per-drone telemetry stats, for the rate on TLM lines. */
  setTlmStats(data: Float32Array | null): void {
    this.tlmStats = data;
  }

  /** Whether TLM lines are shown (the page subscribes to package samples only then). */
  get tlmWanted(): boolean {
    return this.settings.kinds.tlm;
  }

  /** Prints a monitor event as a SYSTEM line. */
  system(e: LogEvent): void {
    const i = this.push(K.System, e.time, -1, -1);
    this.subject[i] = e.subject;
    this.tone[i] = Math.max(0, TONES.indexOf(e.tone));
    this.text[i] = e.text;
    this.changed[i] = 1;
    this.role[i] = -1;
  }

  /** Brain frames and bytes per sim second, from the snapshots (chatter carries no sizes). */
  setBrainRate(framesPerSec: number, bytesPerSec: number): void {
    this.brainFramesPerSec = framesPerSec;
    this.brainBytesPerSec = bytesPerSec;
  }

  /**
   * Everything on air from the channel summary (while the telemetry or channel model
   * is on), or null to count brain frames only. `legacy`: the models are on but the
   * WASM predates the byte counters.
   */
  setAir(air: AirBytes | null, legacy = false): void {
    this.air = air;
    this.airLegacy = legacy;
  }

  /** Number of drones, for the search box's autocomplete and validation. */
  setDroneCount(n: number): void {
    if (n === this.droneCount) return;
    this.droneCount = n;
    this.datalist.replaceChildren(
      ...Array.from({ length: n }, (_, id) => {
        const o = el('option');
        o.value = callsign(id);
        o.label = `#${id}`;
        return o;
      }),
    );
  }

  /** Restricts the view to lines involving `id` (-1 = everyone). */
  setDroneFilter(id: number, updateInput = true): void {
    if (updateInput) {
      this.searchEl.value = id >= 0 ? callsign(id) : '';
      this.searchEl.classList.remove('bad');
    }
    if (id === this.droneFilter) return;
    this.droneFilter = id;
    this.needsRebuild = true;
  }

  /**
   * The selected drone (-1 none), shown as a chip and highlighted in the
   * lines; `following` says whether the camera is still tracking it.
   */
  setFollow(id: number, following: boolean): void {
    this.followEl.hidden = id < 0;
    this.followEl.textContent = id >= 0 ? `${following ? '◉ FOLLOW' : '◎'} ${callsign(id)}  ×` : '';
    this.followEl.title = 'Clear selection (Esc)';
    this.followEl.classList.toggle('live', following);
    if (id === this.followId) return;
    this.followId = id;
    this.needsRebuild = true;
  }

  get droneFilterId(): number {
    return this.droneFilter;
  }

  get collapsed(): boolean {
    return this.settings.collapsed;
  }

  get isPaused(): boolean {
    return this.paused;
  }

  /** Lines currently in the DOM (for automated checks). */
  get lineCount(): number {
    return this.shown;
  }

  setPaused(paused: boolean): void {
    this.paused = paused;
    this.pauseEl.textContent = paused ? '▶ RESUME' : '❚❚ PAUSE';
    this.pauseEl.classList.toggle('on', paused);
    this.updateStatus();
  }

  clear(): void {
    this.floor = this.total;
    this.needsRebuild = true;
  }

  /** Forgets everything, for a new simulation session. */
  reset(): void {
    this.clear();
    this.hasHb.fill(0);
    this.lastMissionSeq.fill(-1);
    this.roleOf.fill(-1);
    this.sample.length = 0;
    this.sampleSeq.fill(-1);
    this.tlmStats = null;
    this.air = null;
  }

  toggle(): void {
    this.settings.collapsed = !this.settings.collapsed;
    this.save();
    this.applyHeight();
    if (!this.settings.collapsed) this.needsRebuild = true;
  }

  /** Call every animation frame; does DOM work at most RENDER_HZ times a second. */
  tick(now: number): void {
    if (now - this.rateAt >= RATE_WINDOW_MS) {
      const dt = now - this.rateAt;
      this.msgsPerSec = this.rateAt === 0 ? 0 : (this.rateCount * 1000) / dt;
      this.rateAt = now;
      this.rateCount = 0;
      this.rateEl.textContent = `${this.msgsPerSec.toFixed(0)} msg/s`;
      const h = airHeader(this.air, this.brainFramesPerSec, this.brainBytesPerSec, this.airLegacy);
      this.airEl.textContent = h.text;
      this.mixEl.textContent = h.mix;
      if (h.title !== this.airTitle) {
        this.airTitle = h.title;
        this.airEl.title = h.title;
        this.mixEl.title = h.title;
      }
    }
    if (now - this.renderAt < RENDER_INTERVAL_MS) return;
    this.renderAt = now;
    if (this.settings.collapsed) {
      this.updateCollapsedLine();
      return;
    }
    if (this.paused || !this.stick) {
      // Frozen: no new lines, but filter / follow / clear changes still
      // re-render the lines the reader is looking at (up to the watermark).
      if (this.needsRebuild) this.rebuildFrozen();
      else this.updateStatus();
      return;
    }
    if (this.needsRebuild || this.total - this.renderedTo > CAP) this.rebuild(this.total);
    else this.appendNew();
    if (this.stick) this.list.scrollTop = this.list.scrollHeight;
  }

  /**
   * Rebuilds against the frozen watermark (`renderedTo`, the newest entry
   * already on screen) so pausing or scrolling up keeps holding back newer
   * traffic. If the ring has overrun the watermark, it moves up to the oldest
   * entry still held. Keeps the scroll position when the reader scrolled up.
   */
  private rebuildFrozen(): void {
    const upTo = Math.min(this.total, Math.max(this.renderedTo, this.total - CAP + MAX_LINES));
    if (this.stick) {
      this.rebuild(upTo);
      this.list.scrollTop = this.list.scrollHeight;
      return;
    }
    const l = this.list;
    const fromBottom = l.scrollHeight - l.scrollTop;
    this.rebuild(upTo);
    l.scrollTop = Math.max(0, l.scrollHeight - fromBottom);
  }

  // --- filtering & rendering --------------------------------------------------

  private push(kind: EntryKind, t: number, src: number, dst: number): number {
    const i = this.total % CAP;
    this.total++;
    this.kind[i] = kind;
    this.time[i] = t;
    this.src[i] = src;
    this.dst[i] = dst;
    this.subject[i] = -1;
    this.tone[i] = 0;
    this.text[i] = null;
    return i;
  }

  private passes(seq: number): boolean {
    const i = seq % CAP;
    const kind = this.kind[i];
    const s = this.settings;
    switch (kind) {
      case K.Hb:
      case K.Raw:
        if (!s.kinds.hb) return false;
        break;
      case K.Intent:
        if (!s.kinds.intent) return false;
        break;
      case K.Mission:
        if (!s.kinds.mission) return false;
        break;
      case K.System:
        if (!s.kinds.system) return false;
        break;
      case K.Tlm:
        if (!s.kinds.tlm) return false;
        break;
      case K.PmSync:
        if (!s.kinds.pmsync) return false;
        break;
      case K.PmTime:
        if (!s.kinds.pmtime) return false;
        break;
      case K.Uplink:
        if (!s.kinds.uplink) return false;
        break;
    }
    if (s.changesOnly && this.changed[i] === 0) return false;
    if (kind !== K.System) {
      const role = this.role[i];
      if (role >= 0) {
        const key = ROLE_KEYS[role === ROLE_OPERATOR ? 3 : role];
        if (key && !s.roles[key]) return false;
      }
    }
    const d = this.droneFilter;
    if (d >= 0) {
      if (kind === K.System) return this.subject[i] === d;
      if (this.src[i] === d || this.dst[i] === d) return true;
      // Heartbeats naming `d` as their parent: who routes through it.
      return kind === K.Hb && this.fields[i * 8 + (HB.parent - C.f0)] === d;
    }
    return true;
  }

  /** Re-renders from scratch the newest passing entries older than `upTo`. */
  private rebuild(upTo: number): void {
    this.needsRebuild = false;
    const lo = Math.max(this.floor, this.total - CAP);
    let n = 0;
    for (let seq = upTo - 1; seq >= lo && n < MAX_LINES; seq--) {
      if (this.passes(seq)) this.pick[n++] = seq;
    }
    this.list.replaceChildren();
    for (let k = 0; k < n; k++) {
      const row = this.pool[k];
      this.fill(row, this.pick[n - 1 - k]);
      this.order[k] = row;
      this.list.appendChild(row.el);
    }
    this.head = 0;
    this.shown = n;
    this.renderedTo = upTo;
    this.updateStatus();
  }

  private appendNew(): void {
    // Of the new entries, only the last MAX_LINES that pass can ever show.
    let n = 0;
    const lo = Math.max(this.renderedTo, this.floor);
    for (let seq = this.total - 1; seq >= lo && n < MAX_LINES; seq--) {
      if (this.passes(seq)) this.pick[n++] = seq;
    }
    this.renderedTo = this.total;
    for (let k = n - 1; k >= 0; k--) {
      let row: Row;
      if (this.shown < MAX_LINES) {
        row = this.pool[this.shown];
        this.order[this.shown] = row;
        this.shown++;
      } else {
        // Recycle the oldest line: it becomes the newest.
        row = this.order[this.head];
        this.head = (this.head + 1) % MAX_LINES;
      }
      this.fill(row, this.pick[k]);
      this.list.appendChild(row.el);
    }
    if (n > 0) this.updateStatus();
  }

  private pendingCount(): number {
    let n = 0;
    const lo = Math.max(this.renderedTo, this.floor, this.total - CAP);
    for (let seq = lo; seq < this.total; seq++) if (this.passes(seq)) n++;
    return n;
  }

  private makeRow(): Row {
    const row: Row = {
      el: el('div', 'l'),
      t: el('span', 't'),
      src: el('span', 'who'),
      arr: el('span', 'arr'),
      dst: el('span', 'who dst'),
      k: el('span', 'k'),
      b: el('span', 'b'),
    };
    row.el.append(row.t, row.src, row.arr, row.dst, row.k, row.b);
    return row;
  }

  private fill(row: Row, seq: number): void {
    const i = seq % CAP;
    const kind = this.kind[i];
    row.t.textContent = fmtTime(this.time[i]);
    let cls = 'l';
    if (kind === K.System) {
      const tone = TONES[this.tone[i]];
      cls += ` sys ${tone}`;
      row.src.textContent = 'SYSTEM';
      delete row.src.dataset.id;
      row.arr.textContent = '';
      row.dst.textContent = '';
      delete row.dst.dataset.id;
      row.k.textContent = '';
      row.b.textContent = this.text[i] ?? '';
      if (row.el.title) row.el.title = '';
      if (this.subject[i] >= 0) row.b.dataset.id = String(this.subject[i]);
      else delete row.b.dataset.id;
    } else {
      const src = this.src[i];
      const dst = this.dst[i];
      cls += ` ${KIND_CLASS[kind] ?? 'raw'}`;
      if (src === this.followId || (dst >= 0 && dst === this.followId)) cls += ' me';
      row.src.textContent = callsign(src);
      row.src.dataset.id = String(src);
      row.arr.textContent = '→';
      row.dst.textContent = dst < 0 ? 'ALL' : callsign(dst);
      if (dst >= 0 && dst !== STATION_ID && dst !== OPERATOR_ID) row.dst.dataset.id = String(dst);
      else delete row.dst.dataset.id;
      row.k.textContent = KIND_LABEL[kind] ?? 'RAW';
      delete row.b.dataset.id;
      let body = this.text[i];
      if (body === null) {
        body = this.describe(i, kind);
        this.text[i] = body;
      }
      row.b.textContent = body;
      if (kind === K.Tlm) row.el.title = body;
      else if (row.el.title) row.el.title = '';
      if (kind === K.Hb && (this.fields[i * 8 + (HB.flags - C.f0)] & Flag.Orphan) !== 0) cls += ' orphan';
    }
    row.el.className = cls;
  }

  /** Human-readable body of a radio entry. */
  private describe(i: number, kind: number): string {
    const f = (field: number) => this.fields[i * 8 + (field - C.f0)];
    if (kind === K.Hb) {
      const src = this.src[i];
      const role = f(HB.role);
      const hops = f(HB.hops);
      const parent = f(HB.parent);
      const sink = f(HB.sink);
      const parts: string[] = [];
      parts.push(role === 3 && src !== 0 ? 'ACTING SINK' : (ROLE_WORD[role] ?? `role ${role}`));
      const slot = slotName(f(HB.slot));
      if (slot) parts.push(`slot ${slot}`);
      if (hops === 255) parts.push('NO ROUTE');
      else if (hops === 0) parts.push('sink');
      else {
        parts.push(`hops ${hops}${parent >= 0 ? ` via ${callsign(parent)}` : ''}`);
        parts.push(`sink ${callsign(sink)}`);
      }
      for (const name of flagNames(f(HB.flags))) parts.push(name);
      const need = slotName(f(HB.need));
      if (need) parts.push(`needs ${need}`);
      parts.push(`m#${f(HB.intentSeq)}`);
      return parts.join(' · ');
    }
    if (kind === K.Intent || kind === K.Mission) {
      const intent = (INTENT_NAMES[f(MI.intent)] ?? `intent ${f(MI.intent)}`).toUpperCase();
      let s = `#${f(MI.seq)} ${intent} @ ${fmtXZ(f(MI.targetX), f(MI.targetZ))}`;
      const ox = f(MI.originX);
      if (kind === K.Mission && Number.isFinite(ox)) s += ` · origin ${fmtXZ(ox, f(MI.originZ))}`;
      return s;
    }
    if (kind === K.Tlm) return this.describeTlm(i, -1);
    if (kind === K.PmSync || kind === K.PmTime) {
      const type = f(TL.pkgType);
      return `${PM_TYPE_NAMES[type] ?? 'type'} (type ${type}) · ${f(TL.bytes)} B`;
    }
    if (kind === K.Uplink) {
      const origin = f(TL.origin);
      return `UDP "drone/drone-${origin}/data" (${callsign(origin)}) · ${f(TL.bytes)} B · s=${f(TL.uplinkSeq)}`;
    }
    return 'undecodable frame';
  }

  /**
   * TLM body: send rate, package size, and the exact package text sampled from
   * drone `s` (-1: none bound, e.g. the line arrived before any sample).
   */
  private describeTlm(i: number, s: number): string {
    const f = (field: number) => this.fields[i * 8 + (field - C.f0)];
    const st = this.tlmStats;
    const src = this.src[i];
    const hz = st && src >= 0 && (src + 1) * TLM_STRIDE <= st.length ? st[src * TLM_STRIDE + TLM.requestedHz] : NaN;
    const hops = f(TL.hops);
    let head = `${Number.isFinite(hz) ? `${hz.toFixed(1)} Hz · ` : ''}${f(TL.packageBytes)} B pkg`;
    head += hops < 0 ? ' · no route to root' : ` · ${hops} hop${hops === 1 ? '' : 's'}`;
    if (f(TL.hasSlow) > 0) head += ' · +slow';
    // Never another frame's text: the header (size, +slow, hops) describes this package.
    const text = s >= 0 && this.sampleSeq[s] === f(TL.seq) ? this.sample[s] : undefined;
    return text ? `${head} · ${text}` : `${head} · s=${f(TL.seq)}`;
  }

  private updateStatus(): void {
    const behind = this.paused || !this.stick;
    const pending = behind ? this.pendingCount() : 0;
    this.statusEl.textContent = this.paused ? `PAUSED · ${pending} buffered` : '';
    this.statusEl.classList.toggle('on', this.paused);
    this.jumpEl.hidden = this.paused || this.stick;
    if (!this.stick && !this.paused) this.jumpEl.textContent = pending > 0 ? `▼ ${pending} new` : '▼ live';
  }

  private updateCollapsedLine(): void {
    // Newest passing entry, as one plain line.
    const lo = Math.max(this.floor, this.total - CAP, this.total - 200);
    for (let seq = this.total - 1; seq >= lo; seq--) {
      if (!this.passes(seq)) continue;
      const i = seq % CAP;
      const kind = this.kind[i];
      const t = fmtTime(this.time[i]);
      if (kind === K.System) this.collapsedLast.textContent = `${t} SYSTEM ${this.text[i] ?? ''}`;
      else {
        const body = this.text[i] ?? this.describe(i, kind);
        this.text[i] = body;
        const dst = this.dst[i] < 0 ? 'ALL' : callsign(this.dst[i]);
        this.collapsedLast.textContent = `${t} ${callsign(this.src[i])} → ${dst}  ${body}`;
      }
      return;
    }
  }

  // --- controls & layout --------------------------------------------------------

  private toggleButton(label: string, cls: string, onClick: () => void): HTMLButtonElement {
    const b = el('button', `tog ${cls}`, label);
    b.setAttribute('aria-pressed', 'true');
    b.addEventListener('click', onClick);
    return b;
  }

  private filtersChanged(): void {
    this.syncButtons();
    this.save();
    this.needsRebuild = true;
  }

  private syncButtons(): void {
    const s = this.settings;
    for (const [key, b] of this.kindButtons) b.setAttribute('aria-pressed', String(s.kinds[key]));
    for (const [key, b] of this.roleButtons) b.setAttribute('aria-pressed', String(s.roles[key]));
    this.changesEl.setAttribute('aria-pressed', String(s.changesOnly));
  }

  private save(): void {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(this.settings));
    } catch {
      // Not persisting is fine.
    }
  }

  private applyHeight(): void {
    const collapsed = this.settings.collapsed;
    const max = window.innerHeight * 0.7;
    const h = collapsed ? COLLAPSED_PX : Math.round(Math.min(max, Math.max(MIN_HEIGHT_PX, this.settings.heightFrac * window.innerHeight)));
    this.root.style.height = `${h}px`;
    this.root.classList.toggle('collapsed', collapsed);
    document.documentElement.style.setProperty('--console-h', `${h}px`);
    const icon = this.root.querySelector('.head .icon');
    if (icon) icon.textContent = collapsed ? '▴' : '▾';
    this.onResize?.(h);
  }

  private installResize(grip: HTMLElement): void {
    let dragging = false;
    grip.addEventListener('pointerdown', (e) => {
      dragging = true;
      grip.setPointerCapture(e.pointerId);
      e.preventDefault();
    });
    grip.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      const h = window.innerHeight - e.clientY;
      if (this.settings.collapsed && h > COLLAPSED_PX + 40) {
        this.settings.collapsed = false;
        this.needsRebuild = true;
      }
      this.settings.heightFrac = Math.min(0.7, Math.max(MIN_HEIGHT_PX / window.innerHeight, h / window.innerHeight));
      this.applyHeight();
    });
    const end = () => {
      if (!dragging) return;
      dragging = false;
      this.save();
    };
    grip.addEventListener('pointerup', end);
    grip.addEventListener('pointercancel', end);
  }
}

function entryKind(raw: number): EntryKind {
  switch (raw) {
    case ChatterKind.Heartbeat:
      return K.Hb;
    case ChatterKind.Intent:
      return K.Intent;
    case ChatterKind.Mission:
      return K.Mission;
    case ChatterKind.Tlm:
      return K.Tlm;
    case ChatterKind.PmSync:
      return K.PmSync;
    case ChatterKind.PmTime:
      return K.PmTime;
    case ChatterKind.RootUplink:
      return K.Uplink;
    default:
      return K.Raw;
  }
}
