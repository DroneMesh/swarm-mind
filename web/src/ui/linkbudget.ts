// Link-budget panel: the real system's telemetry (painlessMesh to the base,
// bridged to a WiFi station) on the shared 2.4 GHz channel. Fleet header,
// controls for the telemetry / channel models and a sortable per-drone table.
//
// The panel owns the airtime settings; main.ts forwards every change to the
// worker (`onChange`). Stats arrive at most 5 Hz; the DOM is written at most
// TABLE_HZ times a second, into pooled rows, and only where text changed.

import { callsign } from '../callsigns';
import {
  CH,
  HEARD_HZ,
  MAX_EXTERNAL_BUSY,
  MESH_ROOT,
  RATE_MODE_NAMES,
  RateMode,
  Routing,
  TLM,
  Variant,
  type Tone,
  airBytes,
  airMixText,
  fleetLossText,
  fleetLossTone,
  fleetTlm,
  fmtByteRate,
  hasAirBytes,
  linkHealthText,
  newAirBytes,
  newFleetTlm,
  tlmCount,
  tlmField,
  toneLoss,
} from '../telemetry';

export interface AirtimeSettings {
  telemetry: boolean;
  variant: Variant;
  rateMode: RateMode;
  /** Operator's requested Hz: the fixed rate, the budget cap, or the brain's request. */
  hz: number;
  routing: Routing;
  channel: boolean;
  /** Airtime used by other networks on the channel, 0..MAX_EXTERNAL_BUSY. */
  external: number;
  /** Map overlays. */
  showTree: boolean;
  showBrainLinks: boolean;
  showHeat: boolean;
}

export function defaultAirtime(): AirtimeSettings {
  return {
    telemetry: true,
    variant: Variant.XiaoS3,
    rateMode: RateMode.Fixed,
    hz: 25,
    routing: Routing.Single,
    channel: true,
    external: 0,
    showTree: true,
    showBrainLinks: true,
    showHeat: true,
  };
}

export interface LinkBudgetHost {
  /** Whether drone `id` is up (from the latest snapshot). */
  alive(id: number): boolean;
  /** A row was clicked. */
  select(id: number): void;
}

/**
 * Telemetry packages per second offered to one AP (nodes × Hz), as the field
 * measured them (UDP_VS_MQTT.md): clean up to ~60–70 pps, collapse at ~150 pps.
 * The bar compares `FleetTlm.offeredPps` with these, not every frame on air.
 */
export const CLEAN_PPS = 60;
export const COLLAPSE_PPS = 150;
const PPS_SCALE = 200;
const TABLE_INTERVAL_MS = 400;
const STORAGE_KEY = 'swarm-mind.linkbudget';
const MIN_HZ = 1;
const MAX_HZ = 50;


/** Table columns: header, sort field (TLM offset, or -1 = callsign / id), tooltip. */
const COLUMNS: readonly { label: string; field: number; title: string }[] = [
  { label: 'DRONE', field: -1, title: 'Call sign (click a row to follow)' },
  { label: 'REQ', field: TLM.requestedHz, title: 'Rate the drone sends at (Hz)' },
  {
    label: '@GCS 2s',
    field: TLM.deliveredHz,
    title:
      'Rate delivered at the ground station (Hz): frames by send time over the last 2 s, read 0.75 s late. Short window: it follows bursts and stalls, so it need not match LOSS (10 s)',
  },
  {
    label: 'LOSS 10s',
    field: TLM.lossPct,
    title:
      'Loss at the ground station from gaps in "s" (%), pooled over the last 10 s: a stall that ended a few seconds ago still counts here while @GCS (2 s) has recovered; "none": no package arrives, so there is no loss to measure',
  },
  { label: 'LAT', field: TLM.latencyMs, title: 'Send to ground-station latency (ms)' },
  {
    label: 'TX',
    field: TLM.txAllBytesPerS,
    title:
      'Everything this node transmits on air (B/s, MPDU bytes incl. retries): own telemetry, relayed packages, 802.11 + TCP ACKs, brain frames, mesh sync, beacons. Hover a cell for own / relay / ack. (Older swarm.wasm: own telemetry only)',
  },
  { label: 'RELAY', field: TLM.relayBytesPerS, title: 'Others’ packages forwarded (B/s); the base: uplink to the WiFi station' },
  { label: 'AIR', field: TLM.airtimeFrac, title: 'Share of airtime this node transmits (%)' },
  { label: 'BUSY', field: TLM.busyFrac, title: 'Channel busy as heard here: neighbours + external (%)' },
  { label: 'HOP', field: TLM.treeHops, title: 'painlessMesh tree hops to the root' },
];

interface Row {
  el: HTMLTableRowElement;
  cells: HTMLTableCellElement[];
  text: string[];
  tone: Tone[];
  id: number;
}

function el<K extends keyof HTMLElementTagNameMap>(tag: K, cls = '', text = ''): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text) e.textContent = text;
  return e;
}

export function fmtBytes(v: number): string {
  if (!(v > 0)) return '0';
  if (v >= 10_000) return `${(v / 1000).toFixed(0)}k`;
  if (v >= 1000) return `${(v / 1000).toFixed(1)}k`;
  return v.toFixed(0);
}

function pct(frac: number): string {
  return `${Math.round(frac * 100)}`;
}

function toneBusy(f: number): Tone {
  return f < 0.3 ? 'g' : f < 0.6 ? 'a' : 'r';
}


function tonePps(pps: number): Tone {
  return pps <= CLEAN_PPS * 1.2 ? 'g' : pps < COLLAPSE_PPS * 0.85 ? 'a' : 'r';
}

export class LinkBudgetPanel {
  /** Fires on every settings change made here or through `set`. */
  onChange: ((s: AirtimeSettings, changed: Partial<AirtimeSettings>) => void) | null = null;

  private readonly s: AirtimeSettings = defaultAirtime();
  private readonly root: HTMLElement;
  private readonly summary: HTMLElement;
  private readonly ppsFill: HTMLElement;
  private readonly fleetLines: HTMLElement[];
  private readonly tbody: HTMLTableSectionElement;
  private readonly heads: HTMLTableCellElement[] = [];
  private readonly rows: Row[] = [];
  private readonly order: number[] = [];
  private readonly buttons = new Map<string, HTMLButtonElement>();
  private readonly hzInput: HTMLInputElement;
  private readonly hzLabel: HTMLElement;
  private readonly hzValue: HTMLElement;
  private readonly extInput: HTMLInputElement;
  private readonly extValue: HTMLElement;
  private data: Float32Array | null = null;
  private channel: Float32Array | null = null;
  private dirty = false;
  private drawnAt = 0;
  private sortField = -1;
  private sortDesc = false;
  private collapsed = false;
  private showControls = true;
  private readonly fleet = newFleetTlm();
  private readonly air = newAirBytes();
  private readonly compare = (a: number, b: number): number => {
    const d = this.data;
    if (!d || this.sortField < 0) return this.sortDesc ? b - a : a - b;
    // Without the v3 counters the TX column shows (and sorts by) own telemetry.
    const field = this.sortField === TLM.txAllBytesPerS && !hasAirBytes(this.channel) ? TLM.txBytesPerS : this.sortField;
    const va = tlmField(d, a, field);
    const vb = tlmField(d, b, field);
    const diff = this.sortDesc ? vb - va : va - vb;
    return diff !== 0 && Number.isFinite(diff) ? diff : a - b;
  };

  constructor(
    root: HTMLElement,
    private readonly host: LinkBudgetHost,
  ) {
    this.root = root;
    root.classList.add('panel', 'linkbudget');
    root.setAttribute('aria-label', 'Telemetry link budget');
    try {
      const v = localStorage.getItem(STORAGE_KEY) ?? '';
      this.collapsed = v.startsWith('collapsed');
      this.showControls = !v.endsWith('no-ctl');
    } catch {
      // Storage blocked: start expanded.
    }

    // Header: collapse toggle, title, one-line fleet summary.
    const head = el('div', 'lb-head');
    const toggle = el('button', 'lb-toggle', '▾');
    toggle.type = 'button';
    toggle.id = 'lb-toggle';
    toggle.name = 'lb-toggle';
    toggle.title = 'Collapse / expand the link budget';
    head.append(toggle, el('span', 'lb-title', 'LINK BUDGET'));
    this.summary = el('span', 'lb-sum', 'models off');
    head.append(this.summary);
    const ctlToggle = this.button('ctl', 'CTL', 'Show / hide the controls (more table rows)', () => {
      this.showControls = !this.showControls;
      this.save();
      this.applyCollapsed();
    });
    ctlToggle.classList.add('lb-ctltog');
    head.append(ctlToggle);
    head.addEventListener('click', () => this.setCollapsed(!this.collapsed));

    const body = el('div', 'lb-body');

    // Fleet: pps on air against the clean budget and the collapse point.
    const fleet = el('div', 'lb-fleet');
    const bar = el('div', 'lb-pps');
    bar.title = `Telemetry packages/s offered to the WiFi station (senders × Hz). Field: one 2.4 GHz AP is clean up to ~${CLEAN_PPS} pps (~1 % loss); ~${COLLAPSE_PPS} pps of ~400 B frames gives 26–32 % loss. Frames on air (mesh hops, ACKs, sync, brain, uplink) are counted separately.`;
    this.ppsFill = el('div', 'fill');
    const m60 = el('i', 'mark clean');
    m60.style.left = `${(CLEAN_PPS / PPS_SCALE) * 100}%`;
    const m150 = el('i', 'mark collapse');
    m150.style.left = `${(COLLAPSE_PPS / PPS_SCALE) * 100}%`;
    bar.append(this.ppsFill, m60, m150, el('span', 'lbl clean', `${CLEAN_PPS} clean`), el('span', 'lbl collapse', `~${COLLAPSE_PPS} collapse`));
    bar.querySelector<HTMLElement>('.lbl.clean')!.style.left = `${(CLEAN_PPS / PPS_SCALE) * 100}%`;
    bar.querySelector<HTMLElement>('.lbl.collapse')!.style.left = `${(COLLAPSE_PPS / PPS_SCALE) * 100}%`;
    this.fleetLines = [el('div', 'lb-line'), el('div', 'lb-line'), el('div', 'lb-line'), el('div', 'lb-line')];
    fleet.append(this.fleetLines[0], bar, this.fleetLines[3], this.fleetLines[1], this.fleetLines[2]);

    // Controls.
    const ctl = el('div', 'lb-ctl');
    const row1 = el('div', 'lb-row');
    row1.append(
      el('span', 'k', 'TLM'),
      this.button('tlm', 'ON', 'Telemetry model on / off', () => this.set({ telemetry: !this.s.telemetry })),
      el('span', 'sep'),
      this.button('v0', 'XIAO S3', 'XIAO ESP32-S3: no GNSS keys', () => this.set({ variant: Variant.XiaoS3 })),
      this.button('v1', 'C6 RTK', 'ESP32-C6 with RTK GNSS: adds the g* keys', () => this.set({ variant: Variant.C6Rtk })),
      this.button('v2', 'MIXED', 'Odd ids C6 RTK, even ids XIAO S3', () => this.set({ variant: Variant.Mixed })),
      el('span', 'sep'),
      this.button('r0', 'SINGLE', 'mesh.sendSingle to the root (type 9)', () => this.set({ routing: Routing.Single })),
      this.button('r1', 'BCAST', 'mesh.sendBroadcast (type 8): flooded by every node', () => this.set({ routing: Routing.Broadcast })),
    );
    const row2 = el('div', 'lb-row');
    this.hzLabel = el('span', 'k', 'RATE');
    this.hzInput = el('input');
    this.hzInput.type = 'range';
    this.hzInput.id = 'lb-hz';
    this.hzInput.name = 'lb-hz';
    this.hzInput.min = String(MIN_HZ);
    this.hzInput.max = String(MAX_HZ);
    this.hzInput.step = '1';
    this.hzInput.setAttribute('aria-label', 'Requested telemetry rate (Hz)');
    this.hzInput.addEventListener('input', () => this.set({ hz: Number(this.hzInput.value) }));
    this.hzValue = el('span', 'v', '25 Hz');
    row2.append(
      this.hzLabel,
      this.button('m0', 'FIXED', 'Every drone sends at the slider’s rate', () => this.set({ rateMode: RateMode.Fixed })),
      this.button('m1', 'BUDGET', 'Backend rule: clamp(budget / online, 5, 25) Hz, AIMD on the fleet-median loss', () =>
        this.set({ rateMode: RateMode.BackendBudget }),
      ),
      this.button('m2', 'BRAIN', 'Phase 5: each drone’s brain sets its own telemetry interval', () => this.set({ rateMode: RateMode.Brain })),
    );
    row2.append(this.hzInput, this.hzValue);
    const row4 = el('div', 'lb-row');
    this.extInput = el('input');
    this.extInput.type = 'range';
    this.extInput.id = 'lb-external';
    this.extInput.name = 'lb-external';
    this.extInput.min = '0';
    this.extInput.max = String(Math.round(MAX_EXTERNAL_BUSY * 100));
    this.extInput.step = '5';
    this.extInput.title = 'Airtime used by others: neighbouring networks on the same channel';
    this.extInput.setAttribute('aria-label', 'External airtime (%)');
    this.extInput.addEventListener('input', () => this.set({ external: Number(this.extInput.value) / 100 }));
    this.extValue = el('span', 'v', '0 %');
    row4.append(
      el('span', 'k', 'CHAN'),
      this.button('ch', 'ON', 'Shared-airtime model on / off', () => this.set({ channel: !this.s.channel })),
      el('span', 'k ext', 'EXTERNAL'),
      this.extInput,
      this.extValue,
    );
    const row5 = el('div', 'lb-row');
    row5.append(
      el('span', 'k', 'MAP'),
      this.button('tree', 'MESH TREE', 'painlessMesh tree links (dashed orange)', () => this.set({ showTree: !this.s.showTree })),
      this.button('brain', 'BRAIN LINKS', 'The brains’ own routing links', () => this.set({ showBrainLinks: !this.s.showBrainLinks })),
      this.button('heat', 'BUSY HEAT', 'Ground disc per drone: channel busy as heard there', () => this.set({ showHeat: !this.s.showHeat })),
    );
    ctl.append(row1, row2, row4, row5);

    // Table.
    const wrap = el('div', 'lb-table');
    const table = el('table');
    const thead = el('thead');
    const hr = el('tr');
    COLUMNS.forEach((c, k) => {
      const th = el('th', '', c.label);
      th.title = `${c.title}. Click to sort.`;
      th.addEventListener('click', () => {
        if (this.sortField === c.field) this.sortDesc = !this.sortDesc;
        else {
          this.sortField = c.field;
          this.sortDesc = c.field >= 0;
        }
        this.syncSortHeads();
        this.dirty = true;
        this.drawnAt = 0;
      });
      this.heads[k] = th;
      hr.append(th);
    });
    thead.append(hr);
    this.tbody = el('tbody');
    this.tbody.addEventListener('click', (e) => {
      const tr = (e.target as HTMLElement).closest('tr');
      const id = tr ? Number(tr.dataset.id) : -1;
      if (id >= 0) this.host.select(id);
    });
    table.append(thead, this.tbody);
    wrap.append(table);

    body.append(fleet, ctl, wrap);
    root.append(head, body);
    this.syncControls();
    this.syncSortHeads();
    this.applyCollapsed();
  }

  get settings(): Readonly<AirtimeSettings> {
    return this.s;
  }

  get isCollapsed(): boolean {
    return this.collapsed;
  }

  /** Changes settings (from the controls or a demo) and reports them. */
  set(p: Partial<AirtimeSettings>): void {
    const changed: Partial<AirtimeSettings> = {};
    let any = false;
    for (const key of Object.keys(p) as (keyof AirtimeSettings)[]) {
      let v = p[key];
      if (v === undefined) continue;
      if (key === 'external') v = Math.min(MAX_EXTERNAL_BUSY, Math.max(0, Number(v) || 0));
      if (key === 'hz') v = Math.min(MAX_HZ, Math.max(MIN_HZ, Math.round(Number(v)) || 25));
      if (this.s[key] === v) continue;
      (this.s as unknown as Record<string, unknown>)[key] = v;
      (changed as Record<string, unknown>)[key] = v;
      any = true;
    }
    if (!any) return;
    this.syncControls();
    this.onChange?.(this.s, changed);
  }

  /** Latest stats from the worker ('tlmstats'). */
  ingest(data: Float32Array, channel: Float32Array): void {
    this.data = data;
    this.channel = channel;
    this.dirty = true;
  }

  /** Forgets the stats, for a new simulation session. */
  reset(): void {
    this.data = null;
    this.channel = null;
    this.dirty = true;
    this.drawnAt = 0;
  }

  /** Call every animation frame; writes the DOM at most every TABLE_INTERVAL_MS. */
  tick(now: number): void {
    if (!this.dirty || now - this.drawnAt < TABLE_INTERVAL_MS) return;
    this.dirty = false;
    this.drawnAt = now;
    this.drawFleet();
    if (!this.collapsed) this.drawTable();
  }

  setCollapsed(c: boolean): void {
    this.collapsed = c;
    this.save();
    this.applyCollapsed();
    this.dirty = true;
    this.drawnAt = 0;
  }

  // --- internals -------------------------------------------------------------

  private save(): void {
    try {
      localStorage.setItem(STORAGE_KEY, `${this.collapsed ? 'collapsed' : 'open'}${this.showControls ? '' : ' no-ctl'}`);
    } catch {
      // Not persisting is fine.
    }
  }

  private applyCollapsed(): void {
    this.root.classList.toggle('collapsed', this.collapsed);
    this.root.classList.toggle('no-ctl', !this.showControls);
    this.press('ctl', this.showControls);
    const t = this.root.querySelector('.lb-toggle');
    if (t) t.textContent = this.collapsed ? '▸' : '▾';
  }

  private button(key: string, label: string, title: string, onClick: () => void): HTMLButtonElement {
    const b = el('button', 'lb-btn', label);
    b.type = 'button';
    b.id = `lb-${key}`;
    b.name = `lb-${key}`;
    b.title = title;
    b.addEventListener('click', (e) => {
      e.stopPropagation();
      onClick();
    });
    this.buttons.set(key, b);
    return b;
  }

  private press(key: string, on: boolean): void {
    this.buttons.get(key)?.setAttribute('aria-pressed', String(on));
  }

  private syncControls(): void {
    const s = this.s;
    this.press('tlm', s.telemetry);
    this.buttons.get('tlm')!.textContent = s.telemetry ? 'ON' : 'OFF';
    for (let v = 0; v < 3; v++) this.press(`v${v}`, s.variant === v);
    for (let r = 0; r < 2; r++) this.press(`r${r}`, s.routing === r);
    for (let m = 0; m < 3; m++) this.press(`m${m}`, s.rateMode === m);
    this.press('ch', s.channel);
    this.buttons.get('ch')!.textContent = s.channel ? 'ON' : 'OFF';
    this.press('tree', s.showTree);
    this.press('brain', s.showBrainLinks);
    this.press('heat', s.showHeat);
    this.hzInput.title =
      s.rateMode === RateMode.Fixed
        ? 'Rate every drone sends at'
        : s.rateMode === RateMode.BackendBudget
          ? 'Cap on the backend’s per-node rate'
          : 'Rate the operator asks the brains for; each brain may send less';
    this.hzValue.title = this.hzInput.title;
    if (this.hzInput.value !== String(s.hz)) this.hzInput.value = String(s.hz);
    this.hzValue.textContent = `${s.rateMode === RateMode.Fixed ? '' : s.rateMode === RateMode.BackendBudget ? '≤' : 'ask '}${s.hz} Hz`;
    const ext = Math.round(s.external * 100);
    if (this.extInput.value !== String(ext)) this.extInput.value = String(ext);
    this.extValue.textContent = `${ext} %`;
    this.extInput.disabled = !s.channel;
    this.root.classList.toggle('tlm-off', !s.telemetry);
  }

  private syncSortHeads(): void {
    COLUMNS.forEach((c, k) => {
      const th = this.heads[k];
      const on = c.field === this.sortField;
      th.className = on ? 'sorted' : '';
      th.textContent = on ? `${c.label}${this.sortDesc ? '▾' : '▴'}` : c.label;
    });
  }

  private drawFleet(): void {
    const ch = this.channel;
    const f = fleetTlm(this.data, ch, this.host.alive, this.fleet);
    const s = this.s;
    if (!ch) {
      this.summary.textContent = s.telemetry || s.channel ? 'waiting for stats…' : 'models off';
      this.summary.className = 'lb-sum';
      this.summary.title = '';
      this.ppsFill.style.width = '0';
      this.fleetLines[0].textContent = 'Telemetry and channel models are off.';
      this.fleetLines[1].textContent = '';
      this.fleetLines[2].textContent = '';
      this.fleetLines[3].textContent = '';
      return;
    }
    const telemetryOn = ch[CH.telemetryOn] > 0;
    const channelOn = ch[CH.channelOn] > 0;
    const mode = RATE_MODE_NAMES[ch[CH.mode]] ?? '?';
    // The bar and its field thresholds are in packages offered to the station.
    const offered = telemetryOn ? f.offeredPps : 0;
    const tone = telemetryOn ? tonePps(offered) : '';
    // Byte / frame counters for every class on air (v3 WASM); older ones count airtime
    // only with the channel model, so without it they are not live.
    const ab = airBytes(ch, this.air) ? this.air : null;
    const onAir = ab ? `${ab.frames.toFixed(0)}` : channelOn ? `${f.ppsOnAir.toFixed(0)}` : '—';
    const busy = channelOn ? ` · busy ${pct(f.busyMean)}%` : ' · chan off';
    if (telemetryOn) {
      const loss = fleetLossText(f);
      // Both are medians over the senders, but over different windows: say so, so they can be reconciled.
      this.summary.textContent = `${offered.toFixed(0)} pps · ${f.gcsHz.toFixed(1)} Hz@GCS (med 2 s) · ${loss}${busy}`;
      this.summary.title =
        'Median over the sending drones. Hz@GCS: delivered rate over the last 2 s. Loss: from gaps in "s", over the last 10 s, so it lags a change of rate or a stall by up to 10 s.';
      this.summary.className = `lb-sum ${fleetLossTone(f)}`;
    } else {
      this.summary.textContent = `${onAir} frames/s on air${busy} · telemetry off`;
      this.summary.title = '';
      this.summary.className = 'lb-sum';
    }
    this.ppsFill.style.width = `${Math.min(100, (offered / PPS_SCALE) * 100)}%`;
    this.ppsFill.className = `fill ${tone}`;
    this.fleetLines[0].innerHTML = '';
    this.fleetLines[0].append(
      'offered ',
      telemetryOn ? tag(`${offered.toFixed(0)} pkg/s`, tone) : 'telemetry off',
      channelOn || ab
        ? ` · on air ${onAir} frames/s${ab ? ` ${fmtByteRate(ab.total)}` : ''}`
        : ' · on air — (channel model OFF: airtime not counted)',
      channelOn
        ? ` · airtime brain ${pct(ch[CH.airBrain])} % · mesh ${pct(ch[CH.airMesh])} % · uplink ${pct(ch[CH.airUplink])} %`
        : '',
    );
    this.fleetLines[3].innerHTML = '';
    if (ab) {
      const q = ab.linkQualityMean;
      this.fleetLines[3].title = 'Bytes on air per second by class (MPDU, retries included; retry is the retransmitted part). Link q: measured brain delivery ratio, mean over links; degraded < 0.5.';
      this.fleetLines[3].append(
        `${airMixText(ab)} · links `,
        tag(linkHealthText(q, ab.linksDegraded), q >= 0.9 && ab.linksDegraded === 0 ? 'g' : q >= 0.7 ? 'a' : 'r'),
      );
    } else {
      this.fleetLines[3].title = '';
      this.fleetLines[3].textContent = 'bytes by class — (this swarm.wasm predates the counters: npm run wasm)';
    }
    this.fleetLines[1].innerHTML = '';
    if (channelOn) {
      this.fleetLines[1].append(
        'busy ',
        tag(`${pct(f.busyMean)} %`, toneBusy(f.busyMean)),
        ' mean · ',
        tag(`${pct(f.busyMax)} %`, toneBusy(f.busyMax)),
        ` max · ext ${pct(ch[CH.externalBusy])} %`,
      );
    } else {
      this.fleetLines[1].append('busy — · channel model OFF (no airtime loss)');
    }
    this.fleetLines[2].innerHTML = '';
    const modeText =
      ch[CH.mode] === RateMode.BackendBudget
        ? `${mode} ${ch[CH.budgetPps].toFixed(0)} pps / ${ch[CH.online].toFixed(0)} online`
        : ch[CH.mode] === RateMode.Fixed
          ? `${mode} ${s.hz} Hz`
          : mode;
    this.fleetLines[2].append(
      `GCS ${fmtBytes(ch[CH.gcsBytesPerS])}B/s · ${ch[CH.gcsPps].toFixed(0)} pps (2 s) · median loss (10 s) `,
      // The sim's median (heard drones only; what the backend's AIMD acts on), and who is not heard.
      telemetryOn && f.heard > 0 ? tag(`${ch[CH.medianLossPct].toFixed(0)} %`, toneLoss(ch[CH.medianLossPct])) : '—',
      telemetryOn && f.silent > 0 ? ` (${f.silent} not heard)` : '',
      ` · ${modeText} · `,
      f.bridgeUp ? tag('bridge up', 'g') : tag('NO BRIDGE', 'r'),
    );
  }

  private drawTable(): void {
    const d = this.data;
    const n = d ? tlmCount(d) : 0;
    while (this.rows.length < n) this.rows.push(this.makeRow());
    for (let k = 0; k < this.rows.length; k++) {
      const r = this.rows[k];
      const want = k < n;
      if (want && r.el.parentNode !== this.tbody) this.tbody.append(r.el);
      else if (!want && r.el.parentNode) r.el.remove();
    }
    if (!d || n === 0) return;
    this.order.length = n;
    for (let i = 0; i < n; i++) this.order[i] = i;
    this.order.sort(this.compare);
    const telemetryOn = this.channel ? this.channel[CH.telemetryOn] > 0 : false;
    const channelOn = this.channel ? this.channel[CH.channelOn] > 0 : false;
    const v3 = hasAirBytes(this.channel);
    for (let k = 0; k < n; k++) {
      const i = this.order[k];
      const row = this.rows[k];
      if (row.id !== i) {
        row.id = i;
        row.el.dataset.id = String(i);
      }
      const alive = this.host.alive(i);
      const root = i === MESH_ROOT;
      row.el.className = `${alive ? '' : 'down'}${root ? ' root' : ''}`;
      const f = (field: number) => tlmField(d, i, field);
      this.cell(row, 0, root ? `${callsign(i)}` : callsign(i), '');
      if (!alive) {
        this.cell(row, 1, 'down', 'r');
        for (let c = 2; c < COLUMNS.length; c++) this.cell(row, c, '—', '');
        this.cellTitle(row, 5, '');
        continue;
      }
      const req = f(TLM.requestedHz);
      const gcs = f(TLM.deliveredHz);
      const loss = f(TLM.lossPct);
      const lat = f(TLM.latencyMs);
      const sending = telemetryOn && !root && req > 0;
      if (root) {
        this.cell(row, 1, 'root', '');
        this.cell(row, 2, '', '');
        this.cell(row, 3, '', '');
        this.cell(row, 4, '', '');
      } else {
        this.cell(row, 1, sending ? req.toFixed(1) : '—', '');
        const ratio = req > 0 ? gcs / req : 0;
        this.cell(row, 2, sending ? gcs.toFixed(1) : '—', sending ? (ratio > 0.9 ? 'g' : ratio > 0.6 ? 'a' : 'r') : '');
        // No package arrives: there are no "s" gaps to count, so the measured 0 % means "no data".
        const heard = gcs > HEARD_HZ;
        this.cell(row, 3, sending ? (heard ? loss.toFixed(0) : 'none') : '—', sending ? (heard ? toneLoss(loss) : 'r') : '');
        this.cell(row, 4, sending && lat > 0 ? lat.toFixed(0) : '—', sending ? (lat < 100 ? 'g' : lat < 500 ? 'a' : 'r') : '');
      }
      if (v3) {
        this.cell(row, 5, fmtBytes(f(TLM.txAllBytesPerS)), '');
        this.cellTitle(
          row,
          5,
          `all ${fmtBytes(f(TLM.txAllBytesPerS))} B/s · own tlm ${fmtBytes(f(TLM.txBytesPerS))} · relay ${fmtBytes(f(TLM.relayBytesPerS))} · ack ${fmtBytes(f(TLM.ackBytesPerS))} · brain ${fmtBytes(f(TLM.brainBytesPerS))}`,
        );
      } else {
        this.cell(row, 5, fmtBytes(f(TLM.txBytesPerS)), '');
        this.cellTitle(row, 5, 'own telemetry only (older swarm.wasm)');
      }
      this.cell(row, 6, fmtBytes(f(TLM.relayBytesPerS)), '');
      if (channelOn) {
        const air = f(TLM.airtimeFrac);
        this.cell(row, 7, pct(air), air < 0.05 ? 'g' : air < 0.15 ? 'a' : 'r');
        const busy = f(TLM.busyFrac);
        this.cell(row, 8, pct(busy), toneBusy(busy));
      } else {
        // Airtime is not counted without the channel model.
        this.cell(row, 7, '—', '');
        this.cell(row, 8, '—', '');
      }
      const hops = f(TLM.treeHops);
      this.cell(row, 9, hops < 0 ? (telemetryOn ? '✕' : '—') : hops.toFixed(0), hops < 0 && telemetryOn && !root ? 'r' : '');
    }
  }

  private cell(row: Row, c: number, text: string, tone: Tone): void {
    if (row.text[c] !== text) {
      row.text[c] = text;
      row.cells[c].textContent = text;
    }
    if (row.tone[c] !== tone) {
      row.tone[c] = tone;
      row.cells[c].className = tone;
    }
  }

  private cellTitle(row: Row, c: number, title: string): void {
    const td = row.cells[c];
    if (td.title !== title) td.title = title;
  }

  private makeRow(): Row {
    const tr = el('tr');
    const cells: HTMLTableCellElement[] = [];
    for (let c = 0; c < COLUMNS.length; c++) {
      const td = el('td');
      cells.push(td);
      tr.append(td);
    }
    return { el: tr, cells, text: new Array<string>(COLUMNS.length).fill(''), tone: new Array<Tone>(COLUMNS.length).fill(''), id: -1 };
  }
}

function tag(text: string, tone: Tone): HTMLElement {
  return el('b', tone, text);
}
