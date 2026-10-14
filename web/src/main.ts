import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { callsign } from './callsigns';
import { SwarmMonitor } from './monitor';
import { PRESETS } from './presets';
import { INTENT_NAMES, Intent, type FromWorker, type ToWorker } from './protocol';
import { AirtimeView } from './render/airtime';
import { FireLayer } from './render/fire';
import { createFx } from './render/fx';
import { TargetMarker } from './render/marker';
import { PulseView } from './render/pulses';
import { SwarmView } from './render/swarm';
import { Terrain } from './render/terrain';
import { D, NO_ROUTE, ROLE_NAMES, Role, Snapshot, newLinkHealth } from './snapshot';
import { CameraDirector } from './ui/camera';
import { RadioConsole } from './ui/console';
import { DEMOS, DemoRunner, type DemoHost } from './ui/demos';
import { ScenePicker, presetFromHash, writeHash } from './ui/picker';
import { LinkBudgetPanel, fmtBytes, type AirtimeSettings } from './ui/linkbudget';
import { SelectionMarker } from './ui/selection';
import {
  CH,
  HEARD_HZ,
  MESH_ROOT,
  RATE_MODE_NAMES,
  TLM,
  airBytes,
  fleetTlm,
  hasAirBytes,
  linkHealthText,
  newAirBytes,
  newFleetTlm,
  tlmCount,
  tlmField,
} from './telemetry';

const SEED = 1;
const DRONES = 20;
const HUD_INTERVAL_MS = 250;
/** Smoothing for the traffic readout; roughly a one-second average at 20 Hz. */
const TRAFFIC_SMOOTHING = 0.05;
const FOV = 50;
/** Where the Alpine sim lights the fire; the command target until a fire grid arrives. */
const IGNITION = new THREE.Vector2(330, 40);
/** The demo starts itself this long after the first fire report. */
const AUTO_MONITOR_MS = 2000;
/** Pointer travel (px) below which a press counts as a click, not an orbit. */
const CLICK_SLOP_PX = 5;
const PICK_INTERVAL_MS = 100;

/** swarm-sim `Preset` value, an index into PRESETS; persisted in the URL hash. */
let preset = presetFromHash();

// --- Scene -------------------------------------------------------------------

const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const sky = new THREE.Color(PRESETS[preset].sky);
scene.background = sky;
const fog = new THREE.Fog(sky.getHex(), PRESETS[preset].fogNear, PRESETS[preset].fogFar);
scene.fog = fog;
scene.add(new THREE.HemisphereLight(0xb9c8ff, 0x3a2e26, 1.1));
const sun = new THREE.DirectionalLight(0xffd2a1, 2.2);
sun.position.set(-600, 800, 300);
scene.add(sun);

const camera = new THREE.PerspectiveCamera(FOV, window.innerWidth / window.innerHeight, 1, 6000);
camera.position.fromArray(PRESETS[preset].camera.pos);
const fx = createFx(renderer, scene, camera);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.fromArray(PRESETS[preset].camera.target);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.zoomToCursor = true;
controls.screenSpacePanning = false;
controls.maxPolarAngle = Math.PI * 0.46;
controls.minDistance = 80;
controls.maxDistance = 2600;

const fire = new FireLayer();
const swarm = new SwarmView(DRONES);
const pulses = new PulseView(DRONES);
const marker = new TargetMarker();
const selection = new SelectionMarker();
const airtime = new AirtimeView(DRONES);
scene.add(fire.object, swarm.object, pulses.object, marker.object, selection.object, airtime.object);
const monitor = new SwarmMonitor();
const director = new CameraDirector(camera, controls, () => swarm.positions);

function fitViewport(): void {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  fx.resize(window.innerWidth, window.innerHeight);
  const heightPx = window.innerHeight * renderer.getPixelRatio();
  fire.setViewport(heightPx, FOV);
  pulses.setViewport(heightPx, FOV);
}
fitViewport();
window.addEventListener('resize', fitViewport);

/** Applies a preset's sky and fog (terrain colours come with the terrain). */
function applyLook(p: number): void {
  const info = PRESETS[p];
  sky.setHex(info.sky);
  fog.color.setHex(info.sky);
  fog.near = info.fogNear;
  fog.far = info.fogFar;
  document.body.style.background = `#${info.sky.toString(16).padStart(6, '0')}`;
}
applyLook(preset);

// --- DOM ---------------------------------------------------------------------

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const hud = $<HTMLElement>('hud');
const tooltip = $<HTMLElement>('tooltip');
const targetLabel = $<HTMLElement>('target');
const lossInput = $<HTMLInputElement>('loss');
const lossValue = $<HTMLElement>('loss-value');
const intentButtons = [...document.querySelectorAll<HTMLButtonElement>('#operator [data-intent]')];
const log = $<HTMLElement>('log');
const logRows = Array.from({ length: 8 }, () => {
  const row = document.createElement('div');
  const time = document.createElement('span');
  const text = document.createElement('b');
  text.style.fontWeight = 'normal';
  row.append(time, text);
  log.append(row);
  return { row, time, text };
});

const radio = new RadioConsole($<HTMLElement>('console'), {
  select: (id) => selectDrone(id),
  tlmFilter: () => sendTlmSub(),
});
monitor.listeners.push((e) => radio.system(e));

// --- Telemetry & airtime -------------------------------------------------------

const isAlive = (id: number): boolean => curr !== null && id >= 0 && id < curr.droneCount && curr.alive(id);
const linkBudget = new LinkBudgetPanel($<HTMLElement>('linkbudget'), { alive: isAlive, select: (id) => selectDrone(id) });
/** Latest 'tlmstats' (null until the worker sends one, and after a reset). */
let tlmData: Float32Array | null = null;
let chanData: Float32Array | null = null;
const fleet = newFleetTlm();
/** On-air byte counters from the latest 'tlmstats' (valid while `airValid`). */
const air = newAirBytes();
let airValid = false;
const linkHealth = newLinkHealth();

/** Sends every airtime setting to the worker (on a change and on each new session). */
function sendAirtime(s: Readonly<AirtimeSettings> = linkBudget.settings): void {
  send({ type: 'channel', enabled: s.channel, external: s.external });
  send({ type: 'telemetry', enabled: s.telemetry, variant: s.variant, rateMode: s.rateMode, hz: s.hz, routing: s.routing });
  sendTlmSub(true);
}

let tlmSubOn: boolean | null = null;
/** Package-text samples only while the console shows TLM lines and telemetry is on. */
function sendTlmSub(force = false): void {
  const on = radio.tlmWanted && linkBudget.settings.telemetry;
  if (!force && on === tlmSubOn) return;
  tlmSubOn = on;
  send({ type: 'tlmsample-sub', on });
}

function applyOverlays(s: Readonly<AirtimeSettings>): void {
  airtime.showTree = s.showTree;
  airtime.showHeat = s.showHeat;
  swarm.showLinks = s.showBrainLinks;
}

/** Both models off: the worker stops streaming stats, so drop the last snapshot rather than show it as live. */
function clearTlmStats(): void {
  linkBudget.reset();
  tlmData = chanData = null;
  airValid = false;
  airtime.reset();
  radio.setTlmStats(null);
  radio.setAir(null);
}

linkBudget.onChange = (s, changed) => {
  applyOverlays(s);
  if (!s.telemetry && !s.channel && ('telemetry' in changed || 'channel' in changed)) clearTlmStats();
  if ('channel' in changed || 'external' in changed) send({ type: 'channel', enabled: s.channel, external: s.external });
  if ('telemetry' in changed || 'variant' in changed || 'rateMode' in changed || 'hz' in changed || 'routing' in changed) {
    send({ type: 'telemetry', enabled: s.telemetry, variant: s.variant, rateMode: s.rateMode, hz: s.hz, routing: s.routing });
    sendTlmSub();
  }
};
applyOverlays(linkBudget.settings);

// The link-budget panel sits between the HUD and the legend; both change height.
const legend = $<HTMLElement>('legend');
const legendTitle = legend.querySelector<HTMLElement>('.title');
function syncLegendCollapsed(c: boolean): void {
  legend.classList.toggle('collapsed', c);
  if (legendTitle) legendTitle.textContent = c ? 'LEGEND ▸' : 'LEGEND ▾';
}
// Small screens start with the legend folded so the link budget has room, and
// shrinking the window below that folds it again (the panel also scrolls inside
// its max-height, so nothing spills onto the legend either way).
const LEGEND_FOLD_BELOW = 1000;
let wasTall = window.innerHeight >= LEGEND_FOLD_BELOW;
syncLegendCollapsed(!wasTall);
window.addEventListener('resize', () => {
  const tall = window.innerHeight >= LEGEND_FOLD_BELOW;
  if (wasTall && !tall) syncLegendCollapsed(true);
  wasTall = tall;
});
legendTitle?.addEventListener('click', () => syncLegendCollapsed(!legend.classList.contains('collapsed')));
function layoutSidePanels(): void {
  const root = document.documentElement.style;
  root.setProperty('--lb-top', `${Math.round(hud.getBoundingClientRect().bottom + 12)}px`);
  const lg = getComputedStyle(legend).display === 'none' ? 0 : legend.offsetHeight + 12;
  root.setProperty('--legend-h', `${lg}px`);
}
new ResizeObserver(layoutSidePanels).observe(hud);
new ResizeObserver(layoutSidePanels).observe(legend);
window.addEventListener('resize', layoutSidePanels);

const picker = new ScenePicker($<HTMLElement>('picker'), (p) => switchPreset(p));
picker.set(preset);

const qualityButton = $<HTMLButtonElement>('quality');
function syncQuality(): void {
  qualityButton.textContent = `FX ${fx.quality.toUpperCase()}`;
  qualityButton.classList.toggle('active', fx.quality === 'high');
  qualityButton.title = fx.quality === 'high' ? 'Bloom and searchlights on (click for low)' : 'Low quality: no bloom (click for high)';
}
qualityButton.addEventListener('click', () => {
  fx.setQuality(fx.quality === 'high' ? 'low' : 'high');
  syncQuality();
});
syncQuality();

// --- Session state (cleared by reset) ----------------------------------------

let worker: Worker;
let session = 0;
let terrain: Terrain | null = null;
let prev: Snapshot | null = null;
let curr: Snapshot | null = null;
let currAt = 0;
let stepMs = 50;
let framesPerSec = 0;
let bytesPerSec = 0;
let droppedPerSec = 0;
let droppedTotal = 0;
let autoTimer: ReturnType<typeof setTimeout> | undefined;
let sawFire = false;
let commanded = false;
let customTarget: THREE.Vector2 | null = null;
/** The intent last sent while a custom target was set, so clearing that target can re-aim it. */
let customIntent: Intent | null = null;
let lastTarget: { x: number; z: number } | null = null;
let speed = 1;
let logVersion = -1;
let failed = false;
/** Drone picked from the console (highlighted, filtered, maybe followed), or -1. */
let selected = -1;

const send = (msg: ToWorker) => worker.postMessage(msg);

function startSession(): void {
  const w = new Worker(new URL('./sim.worker.ts', import.meta.url), { type: 'module' });
  worker = w;
  session++;
  // Ignore anything a terminated worker still had in flight.
  w.onmessage = (e: MessageEvent<FromWorker>) => {
    if (w === worker) onWorkerMessage(e.data);
  };
  w.onerror = (e) => {
    if (w === worker) fail(e.message);
  };
  send({ type: 'init', seed: SEED, drones: DRONES, preset });
  // A fresh worker starts with both models off: re-apply the panel's settings.
  sendAirtime();
  monitor.log(`Swarm launched · ${PRESETS[preset].name}`);
}

/** Tears the simulation down and starts a fresh one, leaving nothing behind. */
function resetSession(): void {
  worker.terminate();
  clearTimeout(autoTimer);
  autoTimer = undefined;
  terrain?.dispose();
  terrain = null;
  fire.setTerrain(null);
  fire.reset();
  swarm.reset();
  pulses.reset();
  monitor.reset();
  radio.reset();
  airtime.reset();
  linkBudget.reset();
  tlmData = chanData = null;
  marker.hide();
  selectDrone(-1);
  prev = curr = null;
  framesPerSec = bytesPerSec = droppedPerSec = droppedTotal = 0;
  sawFire = commanded = failed = false;
  customTarget = null;
  customIntent = null;
  lastTarget = null;
  speed = 1;
  hovered(-1);
  lossInput.value = '0';
  lossValue.textContent = '0%';
  updateTargetLabel();
  hud.textContent = 'Starting swarm…';
  startSession();
}

function switchPreset(p: number): void {
  if (!PRESETS[p]) return;
  demos.cancel();
  preset = p;
  writeHash(p);
  picker.set(p);
  applyLook(p);
  resetSession();
  overview();
}

function overview(durationMs = 2400): void {
  const c = PRESETS[preset].camera;
  director.shot({ x: c.pos[0], y: c.pos[1], z: c.pos[2] }, { x: c.target[0], y: c.target[1], z: c.target[2] }, durationMs);
}

function fail(message: string): void {
  failed = true;
  hud.textContent = `Simulation failed: ${message}`;
}

function onWorkerMessage(msg: FromWorker): void {
  switch (msg.type) {
    case 'terrain':
      terrain?.dispose();
      terrain = new Terrain(msg.size, msg.extent, msg.heights, fire.patch, msg.preset);
      fire.setTerrain(terrain);
      swarm.setGround(groundAt);
      scene.add(terrain.mesh);
      break;
    case 'snapshot':
      onSnapshot(new Snapshot(msg.data), msg.stepMs);
      break;
    case 'fire':
      fire.setCells(msg.size, msg.cells);
      if (!sawFire) {
        sawFire = true;
        monitor.log('Fire reported', 'warn');
        autoTimer = setTimeout(() => {
          if (!commanded) issue(Intent.MonitorFire, 'Auto');
        }, AUTO_MONITOR_MS);
      }
      updateTargetLabel();
      break;
    case 'chatter':
      radio.ingest(msg.data);
      break;
    case 'tlmstats':
      // A frame already in flight when both models were switched off is stale.
      if (!linkBudget.settings.telemetry && !linkBudget.settings.channel) break;
      tlmData = msg.data;
      chanData = msg.channel;
      linkBudget.ingest(msg.data, msg.channel);
      airtime.setStats(msg.data, msg.channel[CH.channelOn] > 0, msg.channel[CH.telemetryOn] > 0);
      radio.setTlmStats(msg.data);
      // A WASM built before the byte counters sends the 16-float summary: brain-only header then.
      airValid = airBytes(msg.channel, air);
      radio.setAir(airValid ? air : null, !airValid);
      break;
    case 'tlmsample':
      radio.setTlmSample(msg.id, msg.seq, msg.text);
      break;
    case 'error':
      fail(msg.message);
      break;
  }
}

function onSnapshot(s: Snapshot, ms: number): void {
  prev = curr;
  curr = s;
  currAt = performance.now();
  stepMs = ms;
  // Rates per sim second: when fast-forwarding, one snapshot spans several steps.
  const simDt = prev ? s.time - prev.time : ms / 1000;
  if (simDt > 0) {
    const perSec = 1 / simDt;
    framesPerSec += (s.frames * perSec - framesPerSec) * TRAFFIC_SMOOTHING;
    bytesPerSec += (s.bytes * perSec - bytesPerSec) * TRAFFIC_SMOOTHING;
    droppedPerSec += (s.dropped * perSec - droppedPerSec) * TRAFFIC_SMOOTHING;
  }
  droppedTotal += s.dropped;
  radio.setBrainRate(framesPerSec, bytesPerSec);
  radio.setDroneCount(s.droneCount);

  monitor.observe(prev, s);
  if (swarm.count === 0) swarm.interpolate(null, s, 1);
  pulses.ingest(s, swarm.positions);
  for (const id of monitor.downed) pulses.down(swarm.positions, id);
  demos.tick(s);
  if (hoverId >= 0) updateTooltip();
}

// --- Operator commands -------------------------------------------------------

function commandTarget(): THREE.Vector2 {
  if (customTarget) return customTarget;
  return fire.burning > 0 ? fire.centroid : IGNITION;
}

function command(intent: Intent, x: number, z: number, who = 'Operator'): void {
  commanded = true;
  lastTarget = { x, z };
  send({ type: 'command', intent, x, z });
  monitor.log(`${who}: ${INTENT_NAMES[intent]} @ ${x.toFixed(0)}, ${z.toFixed(0)}`);
}

function issue(intent: Intent, who = 'Operator'): void {
  const t = commandTarget();
  customIntent = customTarget ? intent : null;
  command(intent, t.x, t.y, who);
}

/** Kills drone `id`; sinks (the base, acting sinks) only with `force`. */
function kill(id: number, who = 'Operator', force = false): boolean {
  if (!curr || id < 0 || id >= curr.droneCount || !curr.alive(id)) return false;
  if (!force && curr.get(id, D.role) === Role.Sink) {
    monitor.log(`${callsign(id)} is a sink: protected (try the Base destroyed demo)`, 'warn', id);
    return false;
  }
  send({ type: 'kill', id });
  monitor.noteKill(id, who);
  return true;
}

function setLoss(v: number): void {
  const pct = Math.round(v * 100);
  lossInput.value = String(pct);
  lossValue.textContent = `${pct}%`;
  send({ type: 'loss', value: v });
  monitor.log(`Packet loss set to ${pct}%`, pct > 0 ? 'warn' : 'info');
}

function setSpeed(n: number): void {
  if (n === speed) return;
  speed = n;
  send({ type: 'speed', value: n });
}

/** A live relay, else whichever drone carries the most others' routes. */
function pickRelay(s: Snapshot): number {
  const throughput = monitor.throughput(s);
  let best = -1;
  let bestScore = 0;
  for (let i = 0; i < s.droneCount; i++) {
    if (!s.alive(i) || s.get(i, D.role) === Role.Sink) continue;
    // Relays always win over non-relays; ties within a class go to throughput.
    const score = (s.get(i, D.role) === Role.Relay ? 1000 : 0) + throughput[i];
    if (score > bestScore) {
      bestScore = score;
      best = i;
    }
  }
  return best;
}

for (const button of intentButtons) {
  button.addEventListener('click', () => issue(Number(button.dataset.intent) as Intent));
}
$<HTMLButtonElement>('kill-relay').addEventListener('click', () => {
  if (!curr) return;
  const id = pickRelay(curr);
  if (id < 0) monitor.log('No drone is relaying yet', 'warn');
  else kill(id);
});
$<HTMLButtonElement>('reset').addEventListener('click', () => {
  demos.cancel();
  resetSession();
});
lossInput.addEventListener('input', () => {
  const pct = Number(lossInput.value);
  lossValue.textContent = `${pct}%`;
  send({ type: 'loss', value: pct / 100 });
});
lossInput.addEventListener('change', () => monitor.log(`Packet loss set to ${lossInput.value}%`, Number(lossInput.value) > 0 ? 'warn' : 'info'));

function updateTargetLabel(): void {
  if (customTarget) {
    targetLabel.textContent = `Target: custom ${customTarget.x.toFixed(0)}, ${customTarget.y.toFixed(0)}`;
  } else {
    const t = commandTarget();
    const what = fire.burning > 0 ? 'fire centroid' : 'ignition point';
    targetLabel.textContent = `Target: ${what} ${t.x.toFixed(0)}, ${t.y.toFixed(0)}`;
  }
}
updateTargetLabel();

// --- Selection: console callsigns, follow camera ---------------------------------

/** Selects `id` from the console: highlight, filter its lines and follow it (-1 clears). */
function selectDrone(id: number): void {
  if (id >= 0 && curr && id >= curr.droneCount) return;
  selected = id;
  selection.set(id);
  radio.setDroneFilter(id);
  if (id >= 0) {
    director.follow(id);
    radio.setFollow(id, true);
  } else {
    if (director.followId >= 0) director.cancel();
    radio.setFollow(-1, false);
  }
}

director.onCancel = () => {
  // The user took the camera: stop following but keep the selection.
  if (selected >= 0) radio.setFollow(selected, false);
};

// --- Demos -----------------------------------------------------------------------

const demoHost: DemoHost = {
  snapshot: () => curr,
  session: () => session,
  monitor,
  camera: director,
  kill: (id, who) => kill(id, who, true),
  setLoss,
  ignite: (x, z) => send({ type: 'ignite', x, z }),
  command: (intent, x, z) => command(intent, x, z, 'Demo'),
  reset: resetSession,
  setSpeed,
  fireCentre: () => {
    const t = fire.burning > 0 ? fire.centroid : IGNITION;
    return { x: t.x, z: t.y };
  },
  lastTarget: () => lastTarget,
  groundAt: (x, z) => terrain?.heightAt(x, z) ?? 0,
  overview: () => overview(),
  extent: () => terrain?.extent ?? 1200,
  positions: () => swarm.positions,
  airtime: () => linkBudget.settings,
  setAirtime: (p) => linkBudget.set(p),
  tlm: () => tlmData,
  channel: () => chanData,
};
const demos = new DemoRunner($<HTMLElement>('caption'), demoHost);
const demoMenu = $<HTMLElement>('demos');
const demoButtons = DEMOS.map((d) => {
  const b = document.createElement('button');
  b.textContent = d.name;
  b.title = d.blurb;
  b.dataset.demo = d.key;
  b.addEventListener('click', () => {
    if (demos.active?.key === d.key) demos.cancel();
    else demos.start(d.key);
  });
  demoMenu.append(b);
  return b;
});
demos.onChange = (active) => {
  for (const b of demoButtons) b.classList.toggle('active', b.dataset.demo === active?.key);
  document.body.classList.toggle('demo', active !== null);
};

// --- Pointer: hover, click to kill, shift+click to target ---------------------

const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let pointerInside = false;
let pointerMoved = false;
let pickAt = 0;
let hoverId = -1;
let downX = 0;
let downY = 0;
let pointerX = 0;
let pointerY = 0;
const canvas = renderer.domElement;

function hovered(id: number): void {
  hoverId = id;
  canvas.style.cursor = id >= 0 ? 'pointer' : '';
  if (id < 0) tooltip.style.display = 'none';
  else updateTooltip();
}

function placeTooltip(): void {
  tooltip.style.left = `${Math.min(pointerX + 14, window.innerWidth - tooltip.offsetWidth - 8)}px`;
  tooltip.style.top = `${Math.min(pointerY + 14, window.innerHeight - tooltip.offsetHeight - 8)}px`;
}

function updateTooltip(): void {
  if (!curr || hoverId < 0 || hoverId >= curr.droneCount) return;
  const alive = curr.alive(hoverId);
  const role = curr.get(hoverId, D.role);
  const hops = curr.get(hoverId, D.hops);
  const parent = curr.get(hoverId, D.parent);
  const sink = role === Role.Sink;
  const roleName = sink && hoverId !== 0 ? 'Acting sink' : (ROLE_NAMES[role] ?? `role ${role}`);
  tooltip.textContent = [
    `${callsign(hoverId)}  #${hoverId}`,
    `role    ${alive ? roleName : `down (was ${roleName})`}`,
    `hops    ${hops === NO_ROUTE ? 'no route' : hops}`,
    `parent  ${parent < 0 ? '—' : callsign(parent)}`,
    ...tooltipTelemetry(hoverId, alive),
    alive ? (sink ? 'sink (protected)' : 'click to kill') : '',
  ]
    .filter(Boolean)
    .join('\n');
  tooltip.style.display = 'block';
  placeTooltip();
}

/** Telemetry / airtime lines for the tooltip (none while the models are off). */
function tooltipTelemetry(id: number, alive: boolean): string[] {
  const d = tlmData;
  const ch = chanData;
  if (!d || !ch || id >= tlmCount(d) || !alive) return [];
  const f = (field: number) => tlmField(d, id, field);
  const out: string[] = [];
  const telemetryOn = ch[CH.telemetryOn] > 0;
  if (telemetryOn) {
    if (id === MESH_ROOT) {
      out.push(`mesh    root · bridge to WiFi station`);
      out.push(`uplink  ${fmtBytes(f(TLM.relayBytesPerS))} B/s`);
    } else {
      const hops = f(TLM.treeHops);
      const parent = f(TLM.treeParent);
      out.push(`mesh    ${hops < 0 ? 'not connected' : `${hops} hop${hops === 1 ? '' : 's'} via ${callsign(parent)}`}`);
      const heard = f(TLM.deliveredHz) > HEARD_HZ;
      out.push(
        `tlm     ${f(TLM.requestedHz).toFixed(1)} Hz → ${f(TLM.deliveredHz).toFixed(1)} Hz @GCS · ${heard ? `loss ${f(TLM.lossPct).toFixed(0)} %` : 'not heard'}`,
      );
      out.push(`pkg     ${f(TLM.lastPackageBytes).toFixed(0)} B (${f(TLM.variant) > 0 ? 'C6 RTK' : 'XIAO S3'}) · ${f(TLM.latencyMs).toFixed(0)} ms`);
      out.push(`tx      ${fmtBytes(f(TLM.txBytesPerS))} B/s tlm · ${fmtBytes(f(TLM.relayBytesPerS))} B/s relay`);
    }
  }
  if (hasAirBytes(ch)) out.push(`tx all  ${fmtBytes(f(TLM.txAllBytesPerS))} B/s on air · ack ${fmtBytes(f(TLM.ackBytesPerS))} B/s`);
  out.push(`brain   ${fmtBytes(f(TLM.brainBytesPerS))} B/s on air`);
  if (ch[CH.channelOn] > 0) {
    const drops = f(TLM.queueDropsPerS);
    out.push(`air     ${Math.round(f(TLM.airtimeFrac) * 100)} % own · busy ${Math.round(f(TLM.busyFrac) * 100)} %${drops > 0 ? ` · ${drops.toFixed(1)} drops/s` : ''}`);
  }
  return out;
}

canvas.addEventListener('pointermove', (e) => {
  pointerInside = true;
  pointerMoved = true;
  pointerX = e.clientX;
  pointerY = e.clientY;
  ndc.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  if (hoverId >= 0) placeTooltip();
});
canvas.addEventListener('pointerleave', () => {
  pointerInside = false;
  hovered(-1);
});
canvas.addEventListener('pointerdown', (e) => {
  downX = e.clientX;
  downY = e.clientY;
});
canvas.addEventListener('pointerup', (e) => {
  if (e.button !== 0 || Math.hypot(e.clientX - downX, e.clientY - downY) > CLICK_SLOP_PX) return;
  ndc.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  if (e.shiftKey) setTargetAt(ndc);
  else {
    pickNow();
    if (hoverId >= 0) kill(hoverId);
  }
});

function typing(e: KeyboardEvent): boolean {
  const t = e.target as HTMLElement | null;
  return t !== null && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable);
}

window.addEventListener('keydown', (e) => {
  if (e.key === '`' && !typing(e)) {
    radio.toggle();
    e.preventDefault();
    return;
  }
  if (e.key !== 'Escape') return;
  // One thing per press, most transient first.
  if (demos.active) demos.cancel();
  else if (selected >= 0 || director.kind !== 'idle') {
    director.cancel();
    selectDrone(-1);
  } else if (customTarget) {
    customTarget = null;
    marker.hide();
    updateTargetLabel();
    monitor.log('Custom target cleared');
    // The swarm is still flying to the cleared point; re-aim it at what the label now shows.
    if (customIntent !== null) issue(customIntent);
  }
});

function setTargetAt(at: THREE.Vector2): void {
  if (!terrain) return;
  raycaster.setFromCamera(at, camera);
  const hit = raycaster.intersectObject(terrain.mesh, false)[0];
  if (!hit) return;
  customTarget = new THREE.Vector2(hit.point.x, hit.point.z);
  marker.show(hit.point.x, hit.point.y, hit.point.z);
  updateTargetLabel();
  monitor.log(`Target set ${hit.point.x.toFixed(0)}, ${hit.point.z.toFixed(0)}`);
}

function pickNow(): void {
  if (!curr) return;
  raycaster.setFromCamera(ndc, camera);
  const id = swarm.pick(raycaster, camera, ndc.x, ndc.y, window.innerWidth, window.innerHeight);
  if (id !== hoverId) hovered(id);
}

// --- HUD ---------------------------------------------------------------------

function updateHud(s: Snapshot): void {
  if (failed) return;
  const alive = s.countAlive();
  const perDrone = (v: number) => (alive > 0 ? v / alive : 0);
  const [reserve, scout, relay] = monitor.roleCounts;
  const intent = INTENT_NAMES[s.baseIntent] ?? `#${s.baseIntent}`;
  const sinkName = s.alive(0) ? callsign(0) : actingSinkName(s);
  const lines = [
    `SWARM MIND · ${PRESETS[preset].name.toUpperCase()}`,
    `t ${s.time.toFixed(1)} s · intent ${intent}${speed > 1 ? ` · ×${speed}` : ''}`,
    `drones  ${alive}/${s.droneCount} up · ${s.countRouted()} routed · sink ${sinkName}`,
    `roles   ${scout} scout · ${relay} relay · ${reserve} reserve · ${monitor.noRoute} no route`,
    `fire    ${s.burningCells} cells burning`,
    radioLine(perDrone),
    airLine(s),
    linksLine(s),
    `drops   ${droppedPerSec.toFixed(1)}/s · ${droppedTotal} total (inbox full)`,
  ];
  hud.textContent = lines.join('\n');
  const r = monitor.recovery;
  if (r) {
    const line = document.createElement('span');
    if (r.end !== null) {
      line.className = 'ok';
      line.textContent = `recovered in ${(r.end - r.start).toFixed(1)} s (${callsign(r.victim)} lost)`;
    } else if (r.timedOut) {
      line.textContent = `not recovered after ${callsign(r.victim)} was lost`;
    } else {
      line.className = 'busy';
      line.textContent = `recovering… ${(s.time - r.start).toFixed(1)} s`;
    }
    hud.append('\nheal    ', line);
  }
  for (const b of intentButtons) b.classList.toggle('active', Number(b.dataset.intent) === s.baseIntent);
  // Fx may downgrade itself on a slow machine.
  if (qualityButton.textContent !== `FX ${fx.quality.toUpperCase()}`) syncQuality();
}

/**
 * HUD line: everything on air per live drone (the console's "air" total / live
 * drones) while a model is on and the WASM counts it, else brain frames only.
 */
function radioLine(perDrone: (v: number) => number): string {
  if (airValid && chanData) {
    return `radio   ${perDrone(air.frames).toFixed(1)} frames/s · ${fmtBytes(perDrone(air.total))} B/s on air per drone (all traffic)`;
  }
  return `radio   ${perDrone(framesPerSec).toFixed(2)} frames/s · ${perDrone(bytesPerSec).toFixed(0)} B/s brain per drone`;
}

/** HUD line: measured brain-link health from the snapshot (v3), or why there is none. */
function linksLine(s: Snapshot): string {
  const h = s.linkHealth(linkHealth);
  if (!h.measured) return `links   ${h.links} · quality not measured (old swarm.wasm)`;
  return `links   ${linkHealthText(h.mean, h.degraded)} (<0.5) of ${h.links}`;
}

/** HUD line: own telemetry per sender, delivery and channel busy, or why there is none. */
function airLine(_s: Snapshot): string {
  const ch = chanData;
  if (!ch) return `tlm     ${linkBudget.settings.telemetry ? 'starting…' : 'off'}`;
  const f = fleetTlm(tlmData, ch, isAlive, fleet);
  const silent = f.silent > 0 ? ` · ${f.heard === 0 ? 'none' : f.silent} not heard` : '';
  // Own packages only (the radio line above carries the total on air).
  const tlm = ch[CH.telemetryOn] > 0 ? `${fmtBytes(f.txBytesPerS)} B/s own per sender · ${f.gcsHz.toFixed(1)} Hz@GCS${silent}` : 'off';
  const busy = ch[CH.channelOn] > 0 ? ` · busy ${Math.round(f.busyMean * 100)}%` : '';
  return `tlm     ${tlm}${busy}`;
}

function actingSinkName(s: Snapshot): string {
  for (let i = 1; i < s.droneCount; i++) if (s.alive(i) && s.get(i, D.role) === Role.Sink) return `${callsign(i)} (acting)`;
  return 'none';
}

function updateLog(): void {
  if (logVersion === monitor.version) return;
  logVersion = monitor.version;
  for (let i = 0; i < logRows.length; i++) {
    const ev = monitor.events[i];
    const { row, time, text } = logRows[i];
    row.style.display = ev ? '' : 'none';
    if (!ev) continue;
    time.textContent = ev.time.toFixed(1).padStart(5);
    text.textContent = ev.text;
    row.className = ev.tone;
  }
}

// --- Frame loop --------------------------------------------------------------

const groundAt = (x: number, z: number) => terrain?.heightAt(x, z) ?? 0;
let hudAt = 0;
renderer.setAnimationLoop((now) => {
  director.update(now);
  controls.update();
  const timeS = now / 1000;
  if (curr) {
    swarm.interpolate(prev, curr, Math.min((performance.now() - currAt) / stepMs, 1));
    if (pointerInside && (pointerMoved || now - pickAt > PICK_INTERVAL_MS)) {
      pointerMoved = false;
      pickAt = now;
      pickNow();
    }
    swarm.hovered = hoverId >= 0 ? hoverId : selected;
    swarm.update(curr, timeS);
    if (now - hudAt > HUD_INTERVAL_MS) {
      hudAt = now;
      updateHud(curr);
    }
  }
  selection.update(swarm.positions, swarm.count, camera, groundAt, timeS);
  airtime.update(swarm.positions, swarm.count, isAlive, groundAt);
  linkBudget.tick(now);
  updateLog();
  radio.tick(now);
  pulses.update(swarm.positions);
  fire.update();
  marker.update(timeS);
  fx.render();
});

startSession();

/** Hooks for automated checks from the browser console. */
Object.assign(window, {
  swarmDebug: {
    snapshot: () => curr,
    monitor,
    fire,
    issue,
    kill,
    pickRelay: () => (curr ? pickRelay(curr) : -1),
    reset: resetSession,
    renderer,
    fx,
    camera,
    controls,
    radio,
    demos,
    director,
    selectDrone,
    switchPreset,
    preset: () => preset,
    setSpeed,
    linkBudget,
    airtime,
    tlm: () => ({ data: tlmData, channel: chanData, fleet: fleetTlm(tlmData, chanData, isAlive), air: airValid ? { ...air } : null }),
    rateModes: RATE_MODE_NAMES,
  },
});
