import * as THREE from 'three';
import { D, L, LINK_ROUTE, NO_ROUTE, PulseKind, Role, Snapshot } from '../snapshot';
import { BaseStation } from './base-station';
import {
  DRONE_SPAN,
  LIGHT,
  LIGHT_OFFSETS,
  LIGHT_SLOTS,
  SKID_DEPTH,
  antennaGeometry,
  bodyGeometry,
  frameGeometry,
  ringGeometry,
  rotorGeometry,
  searchlightGeometry,
  spriteGeometry,
} from './drone-model';
import { renderQuality } from './fx';
import { linkStyle } from './link-style';
import { SmokePool } from './wrecks';

/** Matches MAX_NEIGHBORS in swarm-brain: no drone reports more links. */
const MAX_LINKS_PER_DRONE = 16;
/** Screen-space pick radius for drones too small to hit with a ray. */
const PICK_RADIUS_PX = 18;
/** The base station is always drone 0 (swarm-sim `BASE_ID`); it renders as a ground station. */
const BASE_ID = 0;
/** A drone is never drawn smaller than this many pixels across, so it stays readable and clickable. */
const MIN_SPAN_PX = 16;
const MAX_SCREEN_SCALE = 10;
/**
 * World size of the model relative to drone-model.ts units. At 0.62 a drone
 * spans about 8 units: the sim parks drones only ~9 units apart around the base.
 */
const MODEL_SCALE = 0.62;
/** swarm-brain counts a drone as landed below this height above ground ... */
const LANDED_AGL = 8;
/** ... and the sim parks drones this high (MIN_CLEARANCE). */
const PARKED_AGL = 5;
/** A wreck counts as down once it is this close to the ground (sim WRECK_CLEARANCE is 1). */
const WRECK_AGL = 1.35;
/** Sim positions jumping faster than this are a reset or teleport, not flight. */
const TELEPORT_SPEED = 120;
/** How long a wreck smoulders and its ember glows, in seconds. */
const SMOULDER_S = 16;
/**
 * Link quality smoothing time constants, seconds: falling faster than rising, so
 * a failing link shows promptly but one lucky window does not flash it healthy.
 * The sim measures over ~4 s; this only removes the step between snapshots.
 */
const LINK_TAU_DOWN_S = 0.5;
const LINK_TAU_UP_S = 1.2;
/** Dotted pattern of degraded brain links, world units (the mesh tree's dashes are 7 / 5). */
const LINK_DOT = 2.5;
const LINK_GAP = 3.5;
/** Flicker of degraded links, rad/s. */
const LINK_FLICKER = 7;

/** Drone colours; the legend in index.html mirrors these. */
export const DRONE_COLORS = {
  sink: new THREE.Color(0xffb81f),
  reserve: new THREE.Color(0x4fc3f7),
  scout: new THREE.Color(0x7cff6b),
  relay: new THREE.Color(0xc77dff),
  noRoute: new THREE.Color(0xff9f43),
  dead: new THREE.Color(0xa9b0b8),
};

const GRAPHITE = new THREE.Color(0x3b424c);
const WHITE = new THREE.Color(0xffffff);
const NAV_RED = new THREE.Color(1, 0.08, 0.06);
const NAV_GREEN = new THREE.Color(0.1, 1, 0.3);
const STROBE = new THREE.Color(1, 1, 1);
const EMBER = new THREE.Color(1, 0.38, 0.06);
const ANTENNA = DRONE_COLORS.relay.clone().lerp(new THREE.Color(1, 1, 1), 0.4);
const ZERO = new THREE.Matrix4().makeScale(0, 0, 0);

/** Height sampler for the ground under world `(x, z)`. */
export type GroundFn = (x: number, z: number) => number;

/**
 * Draws the swarm as procedural quadcopters (one instanced mesh per part),
 * the base station, link lines, wreck smoke and a hover bracket.
 *
 * Everything is derived from snapshots: heading and banking come from the
 * interpolated motion, landed/crashed state from height above the ground,
 * relay antenna blinks from the heartbeat pulses in the snapshot. The
 * camera is captured at render time (via `onBeforeRender`) to keep drones
 * at a minimum on-screen size, so main.ts needs no extra wiring.
 */
export class SwarmView {
  readonly object = new THREE.Group();
  /** Interpolated xyz per drone for the current frame, indexed by drone id. */
  readonly positions: Float32Array;
  /** Drones drawn in the latest frame. */
  count = 0;
  /** Drone under the pointer, or -1. */
  hovered = -1;

  private readonly body: THREE.InstancedMesh;
  private readonly frame: THREE.InstancedMesh;
  private readonly ring: THREE.InstancedMesh;
  private readonly antenna: THREE.InstancedMesh;
  private readonly rotors: THREE.InstancedMesh;
  private readonly lights: THREE.InstancedMesh;
  private readonly beams: THREE.InstancedMesh;
  private readonly halo: THREE.InstancedMesh;
  /** Invisible, unparented sphere per drone: a generous ray target. */
  private readonly proxy: THREE.InstancedMesh;
  private readonly station = new BaseStation();
  private readonly smoke = new SmokePool();
  private readonly hoverMarker: THREE.LineSegments;
  /** Meshes with one instance per drone, sharing its transform (or hidden). */
  private readonly perDrone: THREE.InstancedMesh[];

  private readonly rotorData: Float32Array;
  private readonly lightData: Float32Array;
  private readonly beamData: Float32Array;
  private readonly rotorAttr: THREE.InstancedBufferAttribute;
  private readonly lightAttr: THREE.InstancedBufferAttribute;
  private readonly beamAttr: THREE.InstancedBufferAttribute;
  private readonly rotorTime = { value: 0 };

  private readonly linkGeometry = new THREE.BufferGeometry();
  private readonly links: THREE.LineSegments;
  private readonly linkPositions: Float32Array;
  /** RGBA per vertex: alpha follows the link's quality. */
  private readonly linkColors: Float32Array;
  /** Degraded links (quality < 0.5, or an end down): dotted and flickering. */
  private readonly weakGeometry = new THREE.BufferGeometry();
  private readonly weakLinks: THREE.LineSegments;
  private readonly weakPositions: Float32Array;
  private readonly weakColors: Float32Array;
  private readonly weakDistance: Float32Array;
  /** Smoothed quality per unordered drone pair (index min * maxDrones + max). */
  private readonly pairQuality: Float32Array;
  /** Link frame in which a pair was last drawn; a pair absent last frame restarts from its measured quality. */
  private readonly pairSeen: Uint32Array;
  private linkFrame = 1;
  private readonly linkRgba = new Float32Array(4);

  // Per-drone motion and life state, indexed by drone id.
  private readonly seen: Uint8Array;
  private readonly wasAlive: Uint8Array;
  private readonly last: Float32Array;
  private readonly vel: Float32Array;
  private readonly acc: Float32Array;
  private readonly yaw: Float32Array;
  private readonly pitch: Float32Array;
  private readonly roll: Float32Array;
  private readonly spin: Float32Array;
  private readonly rotorAngle: Float32Array;
  private readonly flashAt: Float32Array;
  private readonly diedAt: Float32Array;
  private readonly downAt: Float32Array;
  private readonly puffAt: Float32Array;
  private readonly scales: Float32Array;
  /** Rendered centre of each drone (after landing offset, bob and scale). */
  private readonly visual: Float32Array;
  private readonly fallFrom: THREE.Quaternion[];
  private readonly settleFrom: THREE.Quaternion[];
  private readonly orient: THREE.Quaternion[];

  private ground: GroundFn | null = null;
  private groundExplicit = false;
  private groundMesh: THREE.Mesh | null = null;
  private groundSearchAt = -Infinity;
  private camera: THREE.Camera | null = null;
  /** Viewport height in CSS px: the unit MIN_SPAN_PX is expressed in. */
  private viewHeightPx = 800;
  /** Drawing-buffer px per CSS px; gl_PointSize is in buffer px, so smoke needs it. */
  private pixelRatio = 1;
  /** Smoke viewport (buffer height × fov) last handed to the pool; -1 = never. */
  private smokeKey = -1;
  private lastTime = -1;
  private lastSnapshot: Snapshot | null = null;

  private readonly matrix = new THREE.Matrix4();
  private readonly lightMatrix = new THREE.Matrix4();
  private readonly position = new THREE.Vector3();
  private readonly scale = new THREE.Vector3();
  private readonly rotation = new THREE.Quaternion();
  private readonly tumble = new THREE.Quaternion();
  private readonly euler = new THREE.Euler(0, 0, 0, 'YXZ');
  private readonly axis = new THREE.Vector3();
  private readonly color = new THREE.Color();
  private readonly projected = new THREE.Vector3();
  private readonly hits: THREE.Intersection[] = [];
  private readonly identity = new THREE.Quaternion();

  constructor(private readonly maxDrones: number) {
    const instanced = (geometry: THREE.BufferGeometry, material: THREE.Material, count = maxDrones) => {
      const mesh = new THREE.InstancedMesh(geometry, material, count);
      mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
      // Instances move every frame, so the mesh's bounding sphere is never current.
      mesh.frustumCulled = false;
      mesh.count = 0;
      return mesh;
    };

    this.body = instanced(bodyGeometry(), new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.38, metalness: 0.25 }));
    this.body.setColorAt(0, WHITE); // allocates the colour buffer
    this.frame = instanced(frameGeometry(), new THREE.MeshStandardMaterial({ color: 0x1f232a, roughness: 0.5, metalness: 0.55 }));
    this.ring = instanced(ringGeometry(), new THREE.MeshBasicMaterial({ toneMapped: false }));
    this.ring.setColorAt(0, WHITE);
    this.antenna = instanced(antennaGeometry(), new THREE.MeshStandardMaterial({ color: 0x30353d, roughness: 0.5, metalness: 0.6 }));

    const rotorGeo = rotorGeometry();
    this.rotorData = new Float32Array(maxDrones * 2);
    this.rotorAttr = new THREE.InstancedBufferAttribute(this.rotorData, 2).setUsage(THREE.DynamicDrawUsage);
    rotorGeo.setAttribute('iRotor', this.rotorAttr);
    this.rotors = instanced(
      rotorGeo,
      new THREE.ShaderMaterial({
        uniforms: THREE.UniformsUtils.merge([THREE.UniformsLib.fog, { uTime: { value: 0 } }]),
        vertexShader: ROTOR_VERTEX,
        fragmentShader: ROTOR_FRAGMENT,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        fog: true,
      }),
    );
    (this.rotors.material as THREE.ShaderMaterial).uniforms.uTime = this.rotorTime;
    this.rotors.renderOrder = 1;

    const lightGeo = spriteGeometry();
    this.lightData = new Float32Array(maxDrones * LIGHT_SLOTS * 4);
    this.lightAttr = new THREE.InstancedBufferAttribute(this.lightData, 4).setUsage(THREE.DynamicDrawUsage);
    lightGeo.setAttribute('iColor', this.lightAttr);
    this.lights = instanced(
      lightGeo,
      new THREE.ShaderMaterial({
        vertexShader: LIGHT_VERTEX,
        fragmentShader: LIGHT_FRAGMENT,
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
      }),
      maxDrones * LIGHT_SLOTS,
    );
    this.lights.renderOrder = 3;

    const beamGeo = searchlightGeometry();
    this.beamData = new Float32Array(maxDrones);
    this.beamAttr = new THREE.InstancedBufferAttribute(this.beamData, 1).setUsage(THREE.DynamicDrawUsage);
    beamGeo.setAttribute('iBeam', this.beamAttr);
    this.beams = instanced(
      beamGeo,
      new THREE.ShaderMaterial({
        vertexShader: BEAM_VERTEX,
        fragmentShader: BEAM_FRAGMENT,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
        blending: THREE.AdditiveBlending,
      }),
    );
    this.beams.renderOrder = 2;

    this.halo = instanced(
      new THREE.TorusGeometry(1, 0.06, 6, 48).rotateX(Math.PI / 2),
      new THREE.MeshBasicMaterial({ color: new THREE.Color(1.6, 1.15, 0.3), toneMapped: false, transparent: true, opacity: 0.9, blending: THREE.AdditiveBlending, depthWrite: false }),
    );
    this.proxy = instanced(new THREE.IcosahedronGeometry(DRONE_SPAN * 0.42, 1), new THREE.MeshBasicMaterial());

    // Capture the camera and viewport whenever the swarm is drawn; the next
    // update sizes drones from them (a one-frame lag nobody can see).
    this.body.onBeforeRender = (renderer, _scene, camera) => {
      this.camera = camera;
      const h = renderer.domElement.clientHeight;
      if (h > 0) {
        this.viewHeightPx = h;
        this.pixelRatio = renderer.getPixelRatio();
        this.smokeViewport();
      }
    };

    this.hoverMarker = new THREE.LineSegments(
      bracketGeometry(),
      new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.95, depthTest: false, toneMapped: false }),
    );
    this.hoverMarker.visible = false;
    this.hoverMarker.renderOrder = 5;

    const maxLinks = maxDrones * MAX_LINKS_PER_DRONE;
    this.linkPositions = new Float32Array(maxLinks * 6);
    this.linkColors = new Float32Array(maxLinks * 8);
    const positionAttr = new THREE.BufferAttribute(this.linkPositions, 3).setUsage(THREE.DynamicDrawUsage);
    // Four components: three.js then multiplies vertex alpha into the opacity.
    const colorAttr = new THREE.BufferAttribute(this.linkColors, 4).setUsage(THREE.DynamicDrawUsage);
    this.linkGeometry.setAttribute('position', positionAttr);
    this.linkGeometry.setAttribute('color', colorAttr);
    this.linkGeometry.setDrawRange(0, 0);
    const links = new THREE.LineSegments(
      this.linkGeometry,
      new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 1, depthWrite: false }),
    );
    links.frustumCulled = false;
    this.links = links;

    this.weakPositions = new Float32Array(maxLinks * 6);
    this.weakColors = new Float32Array(maxLinks * 8);
    this.weakDistance = new Float32Array(maxLinks * 2);
    this.weakGeometry.setAttribute('position', new THREE.BufferAttribute(this.weakPositions, 3).setUsage(THREE.DynamicDrawUsage));
    this.weakGeometry.setAttribute('color', new THREE.BufferAttribute(this.weakColors, 4).setUsage(THREE.DynamicDrawUsage));
    this.weakGeometry.setAttribute('lineDistance', new THREE.BufferAttribute(this.weakDistance, 1).setUsage(THREE.DynamicDrawUsage));
    this.weakGeometry.setDrawRange(0, 0);
    this.weakLinks = new THREE.LineSegments(
      this.weakGeometry,
      new THREE.LineDashedMaterial({ vertexColors: true, dashSize: LINK_DOT, gapSize: LINK_GAP, transparent: true, opacity: 1, depthWrite: false }),
    );
    this.weakLinks.frustumCulled = false;
    // Toggled with the links; a child so `showLinks` covers both.
    links.add(this.weakLinks);
    this.pairQuality = new Float32Array(maxDrones * maxDrones);
    this.pairSeen = new Uint32Array(maxDrones * maxDrones);

    this.perDrone = [this.body, this.frame, this.ring, this.antenna, this.rotors, this.beams, this.proxy];
    this.positions = new Float32Array(maxDrones * 3);
    this.seen = new Uint8Array(maxDrones);
    this.wasAlive = new Uint8Array(maxDrones);
    this.last = new Float32Array(maxDrones * 3);
    this.vel = new Float32Array(maxDrones * 3);
    this.acc = new Float32Array(maxDrones * 3);
    this.yaw = new Float32Array(maxDrones);
    this.pitch = new Float32Array(maxDrones);
    this.roll = new Float32Array(maxDrones);
    this.spin = new Float32Array(maxDrones);
    this.rotorAngle = new Float32Array(maxDrones);
    this.flashAt = new Float32Array(maxDrones).fill(-1e9);
    this.diedAt = new Float32Array(maxDrones).fill(-1);
    this.downAt = new Float32Array(maxDrones).fill(-1);
    this.puffAt = new Float32Array(maxDrones);
    this.scales = new Float32Array(maxDrones).fill(MODEL_SCALE);
    this.visual = new Float32Array(maxDrones * 3);
    this.fallFrom = Array.from({ length: maxDrones }, () => new THREE.Quaternion());
    this.settleFrom = Array.from({ length: maxDrones }, () => new THREE.Quaternion());
    this.orient = Array.from({ length: maxDrones }, () => new THREE.Quaternion());

    this.object.add(
      this.station.object,
      this.body,
      this.frame,
      this.ring,
      this.antenna,
      this.rotors,
      this.beams,
      this.halo,
      this.lights,
      links,
      this.smoke.object,
      this.hoverMarker,
    );
  }

  /**
   * Overrides the ground sampler (e.g. `(x, z) => terrain.heightAt(x, z)`).
   * Without one, the view reads heights from the terrain mesh it finds in
   * its parent scene; pass `null` to go back to that.
   */
  setGround(fn: GroundFn | null): void {
    this.ground = fn;
    this.groundExplicit = fn !== null;
    this.groundMesh = null;
  }

  /** Shows / hides the brains' routing and neighbour links. */
  set showLinks(on: boolean) {
    this.links.visible = on;
  }

  get showLinks(): boolean {
    return this.links.visible;
  }

  /** Blinks drone `id`'s antenna and strobe now, as if it had just transmitted. */
  flash(id: number): void {
    if (id >= 0 && id < this.maxDrones) this.flashAt[id] = this.lastTime;
  }

  /** Writes drone positions for `curr`, blending from `prev` by `alpha` (0..1). */
  interpolate(prev: Snapshot | null, curr: Snapshot, alpha: number): void {
    const n = Math.min(curr.droneCount, this.maxDrones);
    const blend = prev !== null && prev.droneCount === curr.droneCount;
    const c = curr.data;
    for (let i = 0; i < n; i++) {
      const o = curr.drone(i);
      for (let k = 0; k < 3; k++) {
        const now = c[o + D.x + k];
        this.positions[i * 3 + k] = blend ? lerp(prev.data[prev.drone(i) + D.x + k], now, alpha) : now;
      }
    }
    this.count = n;
  }

  /** Draws `curr` at the positions from the latest `interpolate`. */
  update(curr: Snapshot, timeS: number): void {
    const n = this.count;
    const c = curr.data;
    const dt = this.lastTime < 0 ? 0 : Math.min(Math.max(timeS - this.lastTime, 0), 0.1);
    this.lastTime = timeS;
    this.rotorTime.value = timeS;
    this.refreshGround(timeS);

    if (curr !== this.lastSnapshot) {
      this.lastSnapshot = curr;
      for (let p = 0; p < curr.pulseCount; p++) {
        const o = curr.pulse(p);
        const src = c[o];
        if (c[o + 2] === PulseKind.Heartbeat && src >= 0 && src < n) this.flashAt[src] = timeS;
      }
    }

    const pxAt1 = this.pixelsPerUnitAt1();
    const camPos = this.camera ? this.cameraPosition() : null;
    const beamsOn = renderQuality() === 'high';
    const kVel = 1 - Math.exp(-dt / 0.25);
    const kAcc = 1 - Math.exp(-dt / 0.35);
    const kAtt = 1 - Math.exp(-dt * 4);
    let halos = 0;
    let beamsLive = 0;

    for (let i = 0; i < n; i++) {
      const o = curr.drone(i);
      const alive = c[o + D.alive] === 1;
      const role = c[o + D.role];
      const noRoute = c[o + D.hops] === NO_ROUTE;
      const i3 = i * 3;
      const x = this.positions[i3];
      const y = this.positions[i3 + 1];
      const z = this.positions[i3 + 2];

      // --- motion estimate from interpolated positions ---
      if (!this.seen[i]) {
        this.seen[i] = 1;
        this.wasAlive[i] = alive ? 1 : 0;
        this.yaw[i] = i * 2.399963;
        this.vel.fill(0, i3, i3 + 3);
        this.acc.fill(0, i3, i3 + 3);
        if (!alive) {
          // Already a wreck when first seen: skip the crash and go straight to the resting pose.
          this.diedAt[i] = timeS - SMOULDER_S;
          this.downAt[i] = timeS - SMOULDER_S;
          this.wreckPose(i, this.orient[i]);
        }
      } else if (dt > 0) {
        for (let k = 0; k < 3; k++) {
          const raw = (this.positions[i3 + k] - this.last[i3 + k]) / dt;
          const v0 = this.vel[i3 + k];
          const v1 = Math.abs(raw) > TELEPORT_SPEED ? 0 : v0 + (raw - v0) * kVel;
          this.vel[i3 + k] = v1;
          this.acc[i3 + k] += ((v1 - v0) / dt - this.acc[i3 + k]) * kAcc;
        }
      }
      this.last[i3] = x;
      this.last[i3 + 1] = y;
      this.last[i3 + 2] = z;

      const groundY = this.ground ? this.ground(x, z) : Number.NaN;
      const agl = Number.isNaN(groundY) ? 100 : y - groundY;
      const s = this.screenScale(x, y, z, camPos, pxAt1) * (i === this.hovered ? 1.15 : 1);
      this.scales[i] = s;

      if (i === BASE_ID) {
        this.station.update(x, y, z, timeS, alive);
        this.visual[i3] = x;
        this.visual[i3 + 1] = y;
        this.visual[i3 + 2] = z;
        this.hide(i);
        this.wasAlive[i] = alive ? 1 : 0;
        if (!alive && timeS - this.puffAt[i] > 0.25) {
          this.puffAt[i] = timeS;
          this.smoke.puff(x + 10.5, y + 22, z - 7, timeS, 1.6);
        }
        continue;
      }

      if (this.wasAlive[i] && !alive) this.onDeath(i, x, y, z, timeS);
      if (!this.wasAlive[i] && alive) this.diedAt[i] = this.downAt[i] = -1; // revived (reset)
      this.wasAlive[i] = alive ? 1 : 0;

      const q = this.orient[i];
      let vy = y;
      if (alive) {
        const vx = this.vel[i3];
        const vz = this.vel[i3 + 2];
        const speed = Math.hypot(vx, vz);
        // 1 when parked on the ground, 0 from LANDED_AGL up.
        const parked = Number.isNaN(groundY) ? 0 : clamp01((LANDED_AGL - agl) / (LANDED_AGL - PARKED_AGL));
        const grounded = parked > 0.5 && speed < 1.5 && Math.abs(this.vel[i3 + 1]) < 1;
        this.spin[i] += ((grounded ? 0 : 1) - this.spin[i]) * (1 - Math.exp(-dt * (grounded ? 1.2 : 3)));
        this.rotorAngle[i] = (this.rotorAngle[i] + dt * 14 * this.spin[i]) % (Math.PI * 2);

        if (speed > 1.2) this.yaw[i] += wrapAngle(Math.atan2(vx, vz) - this.yaw[i]) * (1 - Math.exp(-dt * 2.5));
        const sy = Math.sin(this.yaw[i]);
        const cy = Math.cos(this.yaw[i]);
        const fwdV = vx * sy + vz * cy;
        const aF = this.acc[i3] * sy + this.acc[i3 + 2] * cy;
        const aR = -this.acc[i3] * cy + this.acc[i3 + 2] * sy;
        const air = 1 - parked;
        // Real multirotors lean into their speed (to push against drag) and into turns.
        const pitchT = clamp(0.02 * fwdV + 0.07 * aF, -0.42, 0.42) * air;
        const rollT = clamp(0.08 * aR, -0.5, 0.5) * air;
        this.pitch[i] += (pitchT - this.pitch[i]) * kAtt;
        this.roll[i] += (rollT - this.roll[i]) * kAtt;
        this.euler.set(this.pitch[i], this.yaw[i], this.roll[i]);
        q.setFromEuler(this.euler);

        const bob = air * Math.sin(timeS * 1.9 + i * 1.7) * 0.35 * s;
        vy = lerp(y, groundY + SKID_DEPTH * s, parked) + bob;
      } else {
        vy = this.wreckFrame(i, x, y, z, groundY, agl, s, timeS);
      }

      this.visual[i3] = x;
      this.visual[i3 + 1] = vy;
      this.visual[i3 + 2] = z;
      this.position.set(x, vy, z);
      this.scale.setScalar(s);
      this.matrix.compose(this.position, q, this.scale);
      this.body.setMatrixAt(i, this.matrix);
      this.frame.setMatrixAt(i, this.matrix);
      this.ring.setMatrixAt(i, this.matrix);
      this.rotors.setMatrixAt(i, this.matrix);
      this.proxy.setMatrixAt(i, this.matrix);
      this.antenna.setMatrixAt(i, alive && role === Role.Relay ? this.matrix : ZERO);

      // Acting sink (a drone other than the base took over): gold halo.
      if (alive && role === Role.Sink) {
        this.position.set(x, vy, z);
        this.rotation.setFromAxisAngle(THREE.Object3D.DEFAULT_UP, timeS * 0.8);
        this.scale.setScalar(s * (8.5 + 0.6 * Math.sin(timeS * 3)));
        this.lightMatrix.compose(this.position, this.rotation, this.scale);
        this.halo.setMatrixAt(halos++, this.lightMatrix);
      }

      // Scout searchlight, reaching down toward the ground.
      let beam = 0;
      if (beamsOn && alive && role === Role.Scout && this.spin[i] > 0.5) {
        const len = clamp((Number.isNaN(groundY) ? 40 : vy - groundY) - 1.5 * s, 0, 70);
        if (len > 3) {
          this.position.set(x, vy - 1.3 * s, z);
          this.euler.set(this.pitch[i] * 0.6, this.yaw[i], this.roll[i] * 0.6);
          this.rotation.setFromEuler(this.euler);
          this.scale.set(len * 0.2, len, len * 0.2);
          this.lightMatrix.compose(this.position, this.rotation, this.scale);
          beam = (this.spin[i] - 0.5) * 2;
          beamsLive++;
        }
      }
      this.beams.setMatrixAt(i, beam > 0 ? this.lightMatrix : ZERO);
      this.beamData[i] = beam;

      this.paint(i, c, o, alive, role, noRoute, timeS);
      this.placeLights(i, s, alive, role, noRoute, timeS);
    }
    this.hide(BASE_ID); // also covers n === 0 edge where the loop never ran

    for (const m of this.perDrone) {
      m.count = n;
      m.instanceMatrix.needsUpdate = true;
    }
    this.lights.count = n * LIGHT_SLOTS;
    this.lights.instanceMatrix.needsUpdate = true;
    this.halo.count = halos;
    this.halo.visible = halos > 0;
    this.halo.instanceMatrix.needsUpdate = true;
    this.beams.visible = beamsLive > 0;
    if (this.body.instanceColor) this.body.instanceColor.needsUpdate = true;
    if (this.ring.instanceColor) this.ring.instanceColor.needsUpdate = true;
    this.rotorAttr.needsUpdate = true;
    this.lightAttr.needsUpdate = true;
    this.beamAttr.needsUpdate = true;
    if (n === 0) this.station.hide();
    this.smoke.update(timeS);

    this.updateHover(curr, n, timeS);
    this.updateLinks(curr, timeS, dt);
  }

  /**
   * The drone under normalised device coords `(x, y)`, or -1. Tries an exact
   * ray hit first, then the nearest drone within a few pixels on screen,
   * since a drone far from the camera covers only a pixel or two.
   */
  pick(raycaster: THREE.Raycaster, camera: THREE.Camera, x: number, y: number, widthPx: number, heightPx: number): number {
    if (this.count === 0) return -1;
    this.hits.length = 0;
    this.proxy.computeBoundingSphere();
    raycaster.intersectObject(this.proxy, false, this.hits);
    raycaster.intersectObjects(this.station.pickables, false, this.hits);
    let best = -1;
    let bestDist = Infinity;
    for (const h of this.hits) {
      const id = h.object === this.proxy ? (h.instanceId ?? -1) : BASE_ID;
      if (id >= 0 && id < this.count && h.distance < bestDist) {
        bestDist = h.distance;
        best = id;
      }
    }
    if (best >= 0) return best;

    let bestD2 = PICK_RADIUS_PX * PICK_RADIUS_PX;
    for (let i = 0; i < this.count; i++) {
      this.projected.fromArray(this.visual, i * 3).project(camera);
      if (this.projected.z > 1) continue; // behind the camera
      const dx = ((this.projected.x - x) * widthPx) / 2;
      const dy = ((this.projected.y - y) * heightPx) / 2;
      const d2 = dx * dx + dy * dy;
      if (d2 < bestD2) {
        bestD2 = d2;
        best = i;
      }
    }
    return best;
  }

  /** Forgets every drone, e.g. on a simulation reset. */
  reset(): void {
    this.count = 0;
    this.hovered = -1;
    for (const m of [this.body, this.frame, this.ring, this.antenna, this.rotors, this.beams, this.proxy, this.lights, this.halo]) m.count = 0;
    this.hoverMarker.visible = false;
    this.linkGeometry.setDrawRange(0, 0);
    this.weakGeometry.setDrawRange(0, 0);
    this.pairSeen.fill(0);
    this.linkFrame = 1;
    this.seen.fill(0);
    this.diedAt.fill(-1);
    this.downAt.fill(-1);
    this.flashAt.fill(-1e9);
    this.spin.fill(0);
    this.pitch.fill(0);
    this.roll.fill(0);
    this.lastSnapshot = null;
    this.station.hide();
    this.smoke.reset();
    if (!this.groundExplicit) {
      this.ground = null;
      this.groundMesh = null;
    }
  }

  // --- internals ---------------------------------------------------------------

  private onDeath(i: number, x: number, y: number, z: number, timeS: number): void {
    this.diedAt[i] = timeS;
    this.downAt[i] = -1;
    this.fallFrom[i].copy(this.orient[i]);
    this.spin[i] = 0;
    this.smoke.burst(x, y, z, timeS);
    this.puffAt[i] = timeS;
  }

  /** Orientation and height of a dead drone: tumbling while it falls, then settling as a wreck. */
  private wreckFrame(i: number, x: number, y: number, z: number, groundY: number, agl: number, s: number, timeS: number): number {
    const q = this.orient[i];
    const age = timeS - this.diedAt[i];
    const onGround = Number.isNaN(groundY) ? age > 4 : agl <= WRECK_AGL;
    if (!onGround) {
      // Tumble about a per-drone axis, faster as it falls.
      this.axis.set(Math.sin(i * 1.3), 0.35, Math.cos(i * 1.7)).normalize();
      this.tumble.setFromAxisAngle(this.axis, age * (3 + age * 2.5));
      q.copy(this.fallFrom[i]).multiply(this.tumble);
      if (timeS - this.puffAt[i] > 0.07) {
        this.puffAt[i] = timeS;
        this.smoke.puff(x, y, z, timeS, 1);
      }
      return y;
    }
    if (this.downAt[i] < 0) {
      this.downAt[i] = timeS;
      this.settleFrom[i].copy(q);
      this.smoke.burst(x, y, z, timeS);
    }
    const settled = timeS - this.downAt[i];
    this.wreckPose(i, this.rotation);
    q.copy(this.settleFrom[i]).slerp(this.rotation, clamp01(settled / 0.35));
    if (settled < SMOULDER_S && timeS - this.puffAt[i] > 0.3 + settled * 0.05) {
      this.puffAt[i] = timeS;
      this.smoke.puff(x, (Number.isNaN(groundY) ? y : groundY) + 1.5 * s, z, timeS, 1 - settled / SMOULDER_S);
    }
    // Lie low: the sim rests wrecks 1 unit up; the broken legs put the hull nearer the ground.
    return (Number.isNaN(groundY) ? y : groundY) + 0.9 * s;
  }

  /** Resting pose of a wreck: on its side-ish, yaw kept, deterministic per drone. */
  private wreckPose(i: number, out: THREE.Quaternion): void {
    this.euler.set(0.25 * Math.sin(i * 2.1), this.yaw[i], 0.42 * (i % 2 === 0 ? 1 : -1) + 0.1 * Math.cos(i));
    out.setFromEuler(this.euler);
  }

  private paint(i: number, c: Float32Array, o: number, alive: boolean, role: number, noRoute: boolean, timeS: number): void {
    const hovered = i === this.hovered;
    if (!alive) {
      this.color.copy(DRONE_COLORS.dead).multiplyScalar(0.8);
      if (hovered) this.color.lerp(WHITE, 0.3);
      this.body.setColorAt(i, this.color);
      this.color.setRGB(0.03, 0.03, 0.03);
      this.ring.setColorAt(i, this.color);
      this.rotorData[i * 2] = this.rotorAngle[i];
      this.rotorData[i * 2 + 1] = 0;
      return;
    }
    const rc = droneColor(c, o);
    this.color.copy(GRAPHITE).lerp(rc, 0.78);
    if (hovered) this.color.lerp(WHITE, 0.3);
    this.body.setColorAt(i, this.color);
    let glow = role === Role.Reserve ? 0.7 : 1.1;
    // An orphan blinks its band: the thing to notice, whatever role it held.
    if (noRoute && role !== Role.Sink) glow = Math.sin(timeS * 9) > 0 ? 1.6 : 0.25;
    this.color.copy(rc).multiplyScalar(glow);
    this.ring.setColorAt(i, this.color);
    this.rotorData[i * 2] = this.rotorAngle[i];
    this.rotorData[i * 2 + 1] = this.spin[i];
  }

  private placeLights(i: number, s: number, alive: boolean, role: number, noRoute: boolean, timeS: number): void {
    this.matrix.compose(this.position.fromArray(this.visual, i * 3), this.orient[i], this.scale.setScalar(s));
    const dim = role === Role.Reserve ? 0.4 : 1;
    const flash = Math.exp(-(timeS - this.flashAt[i]) * 7);
    for (let slot = 0; slot < LIGHT_SLOTS; slot++) {
      const li = i * LIGHT_SLOTS + slot;
      let col: THREE.Color = STROBE;
      let level = 0;
      let size = 0.75;
      if (alive) {
        switch (slot) {
          case LIGHT.port:
            col = NAV_RED;
            level = 1.1 * dim;
            break;
          case LIGHT.starboard:
            col = NAV_GREEN;
            level = 1.1 * dim;
            break;
          case LIGHT.strobe: {
            if (noRoute) {
              col = DRONE_COLORS.noRoute;
              level = Math.sin(timeS * 9) > 0 ? 2.2 : 0;
              size = 1.4;
            } else {
              // Anti-collision strobe: a double flash every 1.5 s, phase-shifted per drone.
              const p = (timeS + i * 0.37) % 1.5;
              col = STROBE;
              level = (p < 0.06 || (p > 0.2 && p < 0.26) ? 2.4 : 0) * (this.spin[i] > 0.3 ? 1 : 0.15) * dim;
              size = 1.25;
            }
            break;
          }
          case LIGHT.antenna:
            if (role === Role.Relay) {
              col = ANTENNA;
              level = 0.35 + 2.6 * flash;
              size = 1.0 + 0.9 * flash;
            }
            break;
        }
      } else if (slot === LIGHT.strobe) {
        // Ember glow in the wreck, flickering and fading out.
        const down = this.downAt[i] >= 0 ? timeS - this.downAt[i] : 0;
        const fade = Math.exp(-down / (SMOULDER_S * 0.4));
        const flicker = 0.75 + 0.25 * Math.sin(timeS * 17 + i) * Math.sin(timeS * 7.3 + i * 2);
        col = EMBER;
        level = 2.2 * fade * flicker;
        size = 2.4;
      }
      if (level <= 0.01) {
        this.lights.setMatrixAt(li, ZERO);
        this.lightData[li * 4 + 3] = 0;
        continue;
      }
      this.position.copy(LIGHT_OFFSETS[slot]).applyMatrix4(this.matrix);
      this.scale.setScalar(size * s);
      this.lightMatrix.compose(this.position, this.identity, this.scale);
      this.lights.setMatrixAt(li, this.lightMatrix);
      this.lightData[li * 4] = col.r;
      this.lightData[li * 4 + 1] = col.g;
      this.lightData[li * 4 + 2] = col.b;
      this.lightData[li * 4 + 3] = level;
    }
  }

  private hide(i: number): void {
    if (i >= this.maxDrones) return;
    for (const m of this.perDrone) m.setMatrixAt(i, ZERO);
    for (let slot = 0; slot < LIGHT_SLOTS; slot++) {
      this.lights.setMatrixAt(i * LIGHT_SLOTS + slot, ZERO);
      this.lightData[(i * LIGHT_SLOTS + slot) * 4 + 3] = 0;
    }
    this.beamData[i] = 0;
  }

  private updateHover(curr: Snapshot, n: number, timeS: number): void {
    const id = this.hovered;
    this.hoverMarker.visible = id >= 0 && id < n;
    if (!this.hoverMarker.visible) return;
    const base = id === BASE_ID;
    this.hoverMarker.position.fromArray(this.visual, id * 3);
    if (base) this.hoverMarker.position.y += 4;
    if (this.camera) this.hoverMarker.quaternion.copy(this.camera.quaternion);
    const pulse = 1 + 0.06 * Math.sin(timeS * 6);
    this.hoverMarker.scale.setScalar((base ? 15 : DRONE_SPAN * 0.62 * this.scales[id]) * pulse);
    const mat = this.hoverMarker.material as THREE.LineBasicMaterial;
    mat.color.copy(curr.alive(id) ? WHITE : DRONE_COLORS.dead);
  }

  /**
   * Brain links, faded by measured quality (snapshot v3; legacy snapshots read 1):
   * healthy links keep their colour, below ~0.95 they turn amber, below 0.8 red,
   * and below 0.5 (or with an end down) they move to the dotted, flickering set.
   * A link vanishes when the drones drop the neighbour (the snapshot stops listing it).
   * Allocation-free: runs every frame.
   */
  private updateLinks(curr: Snapshot, timeS: number, dt: number): void {
    const c = curr.data;
    const maxLinks = this.linkPositions.length / 6;
    const links = Math.min(curr.linkCount, maxLinks);
    const lp = this.linkPositions;
    const lc = this.linkColors;
    const wp = this.weakPositions;
    const wc = this.weakColors;
    const wd = this.weakDistance;
    const pos = this.positions;
    const rgba = this.linkRgba;
    const n = this.maxDrones;
    const frame = ++this.linkFrame;
    const kDown = dt > 0 ? 1 - Math.exp(-dt / LINK_TAU_DOWN_S) : 0;
    const kUp = dt > 0 ? 1 - Math.exp(-dt / LINK_TAU_UP_S) : 0;
    let solid = 0;
    let weak = 0;
    for (let l = 0; l < links; l++) {
      const o = curr.link(l);
      const a = c[o + L.a];
      const b = c[o + L.b];
      if (!(a >= 0 && a < n && b >= 0 && b < n)) continue;
      // Smoothed per unordered pair: both directions of a link share it.
      const pair = a < b ? a * n + b : b * n + a;
      const target = curr.linkQuality(l);
      let q = this.pairQuality[pair];
      const seen = this.pairSeen[pair];
      if (seen < frame - 1) q = target; // new (or re-formed) link: no history to blend from
      else if (seen === frame - 1) q += (target - q) * (target < q ? kDown : kUp);
      this.pairQuality[pair] = q;
      this.pairSeen[pair] = frame;
      const stale = !curr.alive(a) || !curr.alive(b);
      const flicker = 0.5 + 0.5 * Math.sin(timeS * LINK_FLICKER + ((a * 7 + b * 13) % 17) * 0.37);
      const degraded = linkStyle(q, c[o + L.kind] === LINK_ROUTE, stale, flicker, rgba);
      // Scalar copies: `subarray` would allocate two views per link every frame.
      const pa = a * 3;
      const pb = b * 3;
      const k = degraded ? weak++ : solid++;
      const p = degraded ? wp : lp;
      const col = degraded ? wc : lc;
      const o6 = k * 6;
      p[o6] = pos[pa];
      p[o6 + 1] = pos[pa + 1];
      p[o6 + 2] = pos[pa + 2];
      p[o6 + 3] = pos[pb];
      p[o6 + 4] = pos[pb + 1];
      p[o6 + 5] = pos[pb + 2];
      const o8 = k * 8;
      for (let m = 0; m < 4; m++) {
        col[o8 + m] = rgba[m];
        col[o8 + 4 + m] = rgba[m];
      }
      if (degraded) {
        const dx = pos[pb] - pos[pa];
        const dy = pos[pb + 1] - pos[pa + 1];
        const dz = pos[pb + 2] - pos[pa + 2];
        wd[k * 2] = 0;
        wd[k * 2 + 1] = Math.sqrt(dx * dx + dy * dy + dz * dz);
      }
    }
    this.linkGeometry.setDrawRange(0, solid * 2);
    this.linkGeometry.getAttribute('position').needsUpdate = true;
    this.linkGeometry.getAttribute('color').needsUpdate = true;
    this.weakGeometry.setDrawRange(0, weak * 2);
    if (weak > 0) {
      this.weakGeometry.getAttribute('position').needsUpdate = true;
      this.weakGeometry.getAttribute('color').needsUpdate = true;
      this.weakGeometry.getAttribute('lineDistance').needsUpdate = true;
    }
  }

  /**
   * Model-to-world scale for a drone at `(x, y, z)`: MODEL_SCALE, grown so the
   * drone is at least MIN_SPAN_PX across on screen. Every model-space offset
   * (skid depth, lamp offset, bob) is multiplied by this.
   */
  private screenScale(x: number, y: number, z: number, cam: THREE.Vector3 | null, pxAt1: number): number {
    if (!cam || pxAt1 <= 0) return MODEL_SCALE;
    const dist = Math.hypot(x - cam.x, y - cam.y, z - cam.z);
    const spanPx = (DRONE_SPAN * MODEL_SCALE * pxAt1) / Math.max(dist, 1);
    return MODEL_SCALE * clamp(MIN_SPAN_PX / spanPx, 1, MAX_SCREEN_SCALE);
  }

  private pixelsPerUnitAt1(): number {
    const cam = this.camera;
    if (!(cam instanceof THREE.PerspectiveCamera)) return 0;
    return (this.viewHeightPx * cam.zoom) / (2 * Math.tan(THREE.MathUtils.degToRad(cam.fov) / 2));
  }

  private readonly camWorld = new THREE.Vector3();
  private cameraPosition(): THREE.Vector3 {
    return this.camWorld.setFromMatrixPosition((this.camera as THREE.Camera).matrixWorld);
  }

  /**
   * Point sprites are sized in drawing-buffer pixels, so the smoke pool gets
   * the buffer height (CSS height × pixel ratio), like fire and pulses do in
   * main.ts; otherwise wreck smoke shrinks by the DPR on HiDPI screens.
   */
  private smokeViewport(): void {
    const cam = this.camera;
    if (!(cam instanceof THREE.PerspectiveCamera)) return;
    const bufferH = this.viewHeightPx * this.pixelRatio;
    const key = bufferH * 1000 + cam.fov;
    if (key === this.smokeKey) return;
    this.smokeKey = key;
    this.smoke.setViewport(bufferH, cam.fov);
  }

  /**
   * Keeps a ground sampler current. Without an explicit one, finds the
   * terrain mesh among the parent scene's children: the PlaneGeometry with
   * the most segments (so a flat water plane never wins), as built by
   * render/terrain.ts.
   */
  private refreshGround(timeS: number): void {
    if (this.groundExplicit) return;
    if (this.groundMesh && this.groundMesh.parent) return;
    this.ground = null;
    this.groundMesh = null;
    if (timeS - this.groundSearchAt < 0.5) return;
    this.groundSearchAt = timeS;
    const parent = this.object.parent;
    if (!parent) return;
    let best: THREE.Mesh | null = null;
    let bestSegs = 1;
    for (const child of parent.children) {
      if (!(child instanceof THREE.Mesh) || !(child.geometry instanceof THREE.PlaneGeometry)) continue;
      const segs = child.geometry.parameters.widthSegments;
      if (segs > bestSegs) {
        bestSegs = segs;
        best = child;
      }
    }
    if (!best) return;
    this.groundMesh = best;
    this.ground = planeSampler(best.geometry as THREE.PlaneGeometry);
  }
}

/** Bilinear height lookup on a rotated PlaneGeometry heightfield (row r along +z, column c along +x). */
function planeSampler(geometry: THREE.PlaneGeometry): GroundFn {
  const { width, widthSegments } = geometry.parameters;
  const size = widthSegments + 1;
  const cell = width / widthSegments;
  const arr = (geometry.getAttribute('position') as THREE.BufferAttribute).array;
  const max = size - 1;
  return (x, z) => {
    const fc = clamp((x + width / 2) / cell, 0, max);
    const fr = clamp((z + width / 2) / cell, 0, max);
    const col = Math.min(Math.floor(fc), max - 1);
    const row = Math.min(Math.floor(fr), max - 1);
    const tc = fc - col;
    const tr = fr - row;
    const i = row * size + col;
    const h00 = arr[i * 3 + 1];
    const h01 = arr[(i + 1) * 3 + 1];
    const h10 = arr[(i + size) * 3 + 1];
    const h11 = arr[(i + size + 1) * 3 + 1];
    const top = h00 + (h01 - h00) * tc;
    const bottom = h10 + (h11 - h10) * tc;
    return top + (bottom - top) * tr;
  };
}

/** Four corner brackets around a unit square, for the hover marker. */
function bracketGeometry(): THREE.BufferGeometry {
  const k = 0.35;
  const v: number[] = [];
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      v.push(sx, sy, 0, sx - sx * k, sy, 0);
      v.push(sx, sy, 0, sx, sy - sy * k, 0);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(v, 3));
  return g;
}

function droneColor(c: Float32Array, o: number): THREE.Color {
  if (c[o + D.alive] !== 1) return DRONE_COLORS.dead;
  const role = c[o + D.role];
  if (role === Role.Sink) return DRONE_COLORS.sink;
  // An orphan is the thing to notice, whatever role it last held.
  if (c[o + D.hops] === NO_ROUTE) return DRONE_COLORS.noRoute;
  if (role === Role.Scout) return DRONE_COLORS.scout;
  if (role === Role.Relay) return DRONE_COLORS.relay;
  return DRONE_COLORS.reserve;
}

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

function clamp01(v: number): number {
  return clamp(v, 0, 1);
}

function wrapAngle(a: number): number {
  return a - Math.PI * 2 * Math.round(a / (Math.PI * 2));
}

// --- shaders ---------------------------------------------------------------------

const ROTOR_VERTEX = /* glsl */ `
  attribute vec2 aLocal;
  attribute float aDir;
  attribute vec2 iRotor;
  varying vec2 vLocal;
  varying float vSpin;
  varying float vAngle;
  #include <common>
  #include <fog_pars_vertex>
  void main() {
    vLocal = aLocal;
    vSpin = iRotor.y;
    vAngle = iRotor.x * aDir;
    vec4 mvPosition = modelViewMatrix * instanceMatrix * vec4(position, 1.0);
    gl_Position = projectionMatrix * mvPosition;
    #include <fog_vertex>
  }
`;

/**
 * Stopped: two sharp carbon blades. Spinning: a translucent blur disc with
 * faint streaks and a brighter tip ring, the way a fast prop reads on camera.
 */
const ROTOR_FRAGMENT = /* glsl */ `
  uniform float uTime;
  varying vec2 vLocal;
  varying float vSpin;
  varying float vAngle;
  #include <common>
  #include <fog_pars_fragment>
  void main() {
    float r = length(vLocal);
    if (r > 1.0) discard;
    float c = cos(vAngle);
    float s = sin(vAngle);
    vec2 p = mat2(c, -s, s, c) * vLocal;
    float w = mix(0.14, 0.06, r);
    float blade = smoothstep(w, w * 0.55, abs(p.y)) * smoothstep(1.0, 0.93, r);
    float hub = smoothstep(0.17, 0.12, r);
    // atan(0, 0) is undefined (NaN on some GPUs), and bloom would smear one NaN pixel over the screen.
    float streak = r < 1e-3 ? 0.0 : pow(clamp(abs(cos(atan(p.y, p.x) + uTime * 3.0)), 0.0, 1.0), 8.0);
    float tip = smoothstep(0.84, 0.95, r) * smoothstep(1.0, 0.96, r);
    float disc = 0.07 + 0.12 * streak * r + 0.42 * tip;
    float blur = smoothstep(0.2, 0.7, vSpin);
    float a = max(mix(blade * 0.95, disc, blur), hub);
    vec3 col = mix(vec3(0.06, 0.065, 0.075), vec3(0.82, 0.86, 0.9), blur * (1.0 - hub));
    gl_FragColor = vec4(col, a);
    #include <fog_fragment>
    #include <colorspace_fragment>
  }
`;

/** Camera-facing glow sprites placed at each instance's translation; size from its scale. */
const LIGHT_VERTEX = /* glsl */ `
  attribute vec4 iColor;
  varying vec4 vColor;
  varying vec2 vUv;
  void main() {
    vColor = iColor;
    vUv = position.xy;
    float size = length(instanceMatrix[0].xyz);
    vec4 center = modelViewMatrix * instanceMatrix * vec4(0.0, 0.0, 0.0, 1.0);
    // Pull toward the eye so the airframe never half-covers its own light.
    center.xyz += normalize(-center.xyz) * size * 0.8;
    gl_Position = projectionMatrix * (center + vec4(position.xy * size, 0.0, 0.0));
    if (iColor.a <= 0.0 || size <= 0.0) gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
  }
`;

const LIGHT_FRAGMENT = /* glsl */ `
  varying vec4 vColor;
  varying vec2 vUv;
  void main() {
    float r = length(vUv);
    if (r > 1.0) discard;
    float core = smoothstep(0.3, 0.0, r);
    float halo = pow(max(1.0 - r, 0.0), 2.4);
    vec3 col = vColor.rgb * halo * 0.9 + mix(vColor.rgb, vec3(1.0), 0.55) * core;
    gl_FragColor = vec4(col * vColor.a, 1.0);
  }
`;

const BEAM_VERTEX = /* glsl */ `
  attribute float aT;
  attribute float aPart;
  attribute float iBeam;
  varying float vT;
  varying float vPart;
  varying float vBeam;
  varying float vEdge;
  void main() {
    vT = aT;
    vPart = aPart;
    vBeam = iBeam;
    vec4 mvPosition = modelViewMatrix * instanceMatrix * vec4(position, 1.0);
    vec3 n = normalize(normalMatrix * (mat3(instanceMatrix) * normal));
    vEdge = abs(dot(n, normalize(-mvPosition.xyz)));
    if (iBeam <= 0.0) vEdge = 0.0;
    gl_Position = projectionMatrix * mvPosition;
  }
`;

const BEAM_FRAGMENT = /* glsl */ `
  varying float vT;
  varying float vPart;
  varying float vBeam;
  varying float vEdge;
  void main() {
    vec3 warm = vec3(1.0, 0.93, 0.74);
    float a;
    if (vPart < 0.5) {
      // Brightest at the lamp, fading toward the ground; soft at the silhouette.
      float t = clamp(vT, 0.0, 1.0);
      a = pow(1.0 - t, 2.2) * pow(clamp(vEdge, 0.0, 1.0), 2.0) * 0.075;
    } else {
      a = (1.0 - smoothstep(0.1, 1.1, vT)) * 0.07;
    }
    gl_FragColor = vec4(warm * a * vBeam, 1.0);
  }
`;
