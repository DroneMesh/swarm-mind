import * as THREE from 'three';
import { L, PulseKind, type Snapshot } from '../snapshot';

/** Pool sizes. When a pool is full the oldest effect is recycled. */
const MAX_RINGS = 128;
const MAX_DOTS = 768;
/** Matches MAX_NEIGHBORS in swarm-brain. */
const MAX_LINKS_PER_DRONE = 16;
/** Links measured below this delivery ratio are treated as gone for heartbeat dots. */
const GONE_QUALITY = 0.1;

interface Style {
  color: THREE.Color;
  /** Ring radius in world units at the end of its life. */
  ring: number;
  ringLife: number;
  ringAlpha: number;
  dotSize: number;
  dotLife: number;
  dotAlpha: number;
}

/** Heartbeats are constant background chatter, so they stay subtle. */
const STYLES: Record<number, Style> = {
  [PulseKind.Heartbeat]: {
    color: new THREE.Color(0x5fd8ea),
    /** Kept small so close-ups of the base are not drowned in rings. */
    ring: 14,
    ringLife: 0.55,
    ringAlpha: 0.35,
    dotSize: 3.2,
    dotLife: 0.45,
    dotAlpha: 0.55,
  },
  [PulseKind.Intent]: {
    color: new THREE.Color(0xffb81f),
    ring: 60,
    ringLife: 0.9,
    ringAlpha: 0.6,
    dotSize: 6,
    dotLife: 0.6,
    dotAlpha: 1,
  },
};
const UNKNOWN: Style = {
  color: new THREE.Color(0xc9ced8),
  ring: 40,
  ringLife: 0.7,
  ringAlpha: 0.6,
  dotSize: 4.5,
  dotLife: 0.5,
  dotAlpha: 0.8,
};
/** A burst for a drone going down; not a radio message. */
const DOWN: Style = { ...UNKNOWN, color: new THREE.Color(0xff4d4d), ring: 90, ringLife: 1.2, ringAlpha: 1 };

/**
 * Message pulses from the snapshot: a broadcast shows an expanding ring at
 * the sender and dots along its links; a unicast shows a dot from src to dst.
 * Rings animate on the GPU from their birth time; dots are re-positioned per
 * frame into preallocated buffers because their endpoints move.
 */
export class PulseView {
  readonly object = new THREE.Group();
  private readonly t0 = performance.now();
  private readonly time = { value: 0 };
  private readonly scale = { value: 800 };

  private readonly ringGeometry = new THREE.BufferGeometry();
  private readonly ringPos = new Float32Array(MAX_RINGS * 3);
  private readonly ringColor = new Float32Array(MAX_RINGS * 4);
  /** [born, life, radius] per ring; born < 0 = free. */
  private readonly ringTiming = new Float32Array(MAX_RINGS * 3).fill(-1);
  private ringNext = 0;

  private readonly dotGeometry = new THREE.BufferGeometry();
  private readonly dotPos = new Float32Array(MAX_DOTS * 3);
  private readonly dotColor = new Float32Array(MAX_DOTS * 4);
  private readonly dotSize = new Float32Array(MAX_DOTS);
  private readonly dotSrc = new Int16Array(MAX_DOTS);
  private readonly dotDst = new Int16Array(MAX_DOTS);
  private readonly dotBorn = new Float32Array(MAX_DOTS).fill(-1);
  private readonly dotLife = new Float32Array(MAX_DOTS);
  private readonly dotAlpha = new Float32Array(MAX_DOTS);
  private dotNext = 0;
  private dotsLive = 0;

  /** Undirected adjacency from the latest snapshot's links. */
  private readonly adjacency: Int16Array;
  private readonly degree: Uint8Array;

  constructor(private readonly maxDrones: number) {
    this.adjacency = new Int16Array(maxDrones * MAX_LINKS_PER_DRONE);
    this.degree = new Uint8Array(maxDrones);

    this.ringGeometry.setAttribute('position', dynamic(this.ringPos, 3));
    this.ringGeometry.setAttribute('aColor', dynamic(this.ringColor, 4));
    this.ringGeometry.setAttribute('aTiming', dynamic(this.ringTiming, 3));
    const rings = new THREE.Points(
      this.ringGeometry,
      new THREE.ShaderMaterial({
        uniforms: { uTime: this.time, uScale: this.scale },
        vertexShader: RING_VERTEX,
        fragmentShader: RING_FRAGMENT,
        transparent: true,
        depthWrite: false,
        // Rings sit at ground level for landed drones; the terrain would clip them in half.
        depthTest: false,
        blending: THREE.AdditiveBlending,
      }),
    );

    this.dotGeometry.setAttribute('position', dynamic(this.dotPos, 3));
    this.dotGeometry.setAttribute('aColor', dynamic(this.dotColor, 4));
    this.dotGeometry.setAttribute('aSize', dynamic(this.dotSize, 1));
    const dots = new THREE.Points(
      this.dotGeometry,
      new THREE.ShaderMaterial({
        uniforms: { uScale: this.scale },
        vertexShader: DOT_VERTEX,
        fragmentShader: DOT_FRAGMENT,
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending,
      }),
    );
    for (const p of [rings, dots]) {
      p.frustumCulled = false;
      p.renderOrder = 3;
    }
    this.object.add(rings, dots);
  }

  setViewport(heightPx: number, fovDeg: number): void {
    this.scale.value = heightPx / (2 * Math.tan(THREE.MathUtils.degToRad(fovDeg) / 2));
  }

  /** Spawns effects for every pulse in a freshly received snapshot. */
  ingest(s: Snapshot, positions: Float32Array): void {
    this.buildAdjacency(s);
    const now = this.now();
    const c = s.data;
    for (let p = 0; p < s.pulseCount; p++) {
      const o = s.pulse(p);
      const src = c[o];
      const dst = c[o + 1];
      if (src < 0 || src >= this.maxDrones) continue;
      const style = STYLES[c[o + 2]] ?? UNKNOWN;
      if (dst < 0) {
        this.ring(positions, src, style, now);
        const base = src * MAX_LINKS_PER_DRONE;
        for (let k = 0; k < this.degree[src]; k++) this.dot(src, this.adjacency[base + k], style, now);
      } else if (dst < this.maxDrones) {
        this.dot(src, dst, style, now);
      }
    }
  }

  /** A red burst where a drone went down. */
  down(positions: Float32Array, id: number): void {
    this.ring(positions, id, DOWN, this.now());
  }

  /** Moves live dots along their (moving) endpoints. Allocation-free. */
  update(positions: Float32Array): void {
    const now = this.now();
    this.time.value = now;
    if (this.dotsLive === 0) return;
    let live = 0;
    for (let i = 0; i < MAX_DOTS; i++) {
      const born = this.dotBorn[i];
      if (born < 0) continue;
      const t = (now - born) / this.dotLife[i];
      if (t >= 1) {
        this.dotBorn[i] = -1;
        this.dotColor[i * 4 + 3] = 0;
        continue;
      }
      live++;
      const a = this.dotSrc[i] * 3;
      const b = this.dotDst[i] * 3;
      const e = t * t * (3 - 2 * t);
      for (let k = 0; k < 3; k++) this.dotPos[i * 3 + k] = positions[a + k] + (positions[b + k] - positions[a + k]) * e;
      this.dotColor[i * 4 + 3] = this.dotAlpha[i] * Math.min(1, t * 8, (1 - t) * 5);
    }
    this.dotsLive = live;
    this.dotGeometry.getAttribute('position').needsUpdate = true;
    this.dotGeometry.getAttribute('aColor').needsUpdate = true;
  }

  /** Clears every effect, e.g. on a simulation reset. */
  reset(): void {
    this.ringTiming.fill(-1);
    this.dotBorn.fill(-1);
    for (let i = 0; i < MAX_DOTS; i++) this.dotColor[i * 4 + 3] = 0;
    this.degree.fill(0);
    this.dotsLive = 0;
    this.ringGeometry.getAttribute('aTiming').needsUpdate = true;
    this.dotGeometry.getAttribute('aColor').needsUpdate = true;
  }

  private buildAdjacency(s: Snapshot): void {
    this.degree.fill(0);
    const c = s.data;
    for (let l = 0; l < s.linkCount; l++) {
      // A link the neighbour table still holds but that delivers (almost) nothing
      // carries no heartbeat: no dot along it.
      if (s.linkQuality(l) < GONE_QUALITY) continue;
      const o = s.link(l);
      this.connect(c[o + L.a], c[o + L.b]);
      this.connect(c[o + L.b], c[o + L.a]);
    }
  }

  private connect(a: number, b: number): void {
    if (a < 0 || a >= this.maxDrones || b < 0 || b >= this.maxDrones) return;
    const base = a * MAX_LINKS_PER_DRONE;
    const n = this.degree[a];
    for (let k = 0; k < n; k++) if (this.adjacency[base + k] === b) return;
    if (n >= MAX_LINKS_PER_DRONE) return;
    this.adjacency[base + n] = b;
    this.degree[a] = n + 1;
  }

  private ring(positions: Float32Array, id: number, style: Style, now: number): void {
    const i = this.ringNext;
    this.ringNext = (i + 1) % MAX_RINGS;
    for (let k = 0; k < 3; k++) this.ringPos[i * 3 + k] = positions[id * 3 + k];
    style.color.toArray(this.ringColor, i * 4);
    this.ringColor[i * 4 + 3] = style.ringAlpha;
    this.ringTiming[i * 3] = now;
    this.ringTiming[i * 3 + 1] = style.ringLife;
    this.ringTiming[i * 3 + 2] = style.ring;
    this.ringGeometry.getAttribute('position').needsUpdate = true;
    this.ringGeometry.getAttribute('aColor').needsUpdate = true;
    this.ringGeometry.getAttribute('aTiming').needsUpdate = true;
  }

  private dot(src: number, dst: number, style: Style, now: number): void {
    const i = this.dotNext;
    this.dotNext = (i + 1) % MAX_DOTS;
    this.dotSrc[i] = src;
    this.dotDst[i] = dst;
    this.dotBorn[i] = now;
    this.dotLife[i] = style.dotLife;
    this.dotAlpha[i] = style.dotAlpha;
    this.dotSize[i] = style.dotSize;
    style.color.toArray(this.dotColor, i * 4);
    this.dotColor[i * 4 + 3] = 0;
    this.dotsLive++;
    this.dotGeometry.getAttribute('aSize').needsUpdate = true;
  }

  private now(): number {
    return (performance.now() - this.t0) / 1000;
  }
}

function dynamic(array: Float32Array, itemSize: number): THREE.BufferAttribute {
  return new THREE.BufferAttribute(array, itemSize).setUsage(THREE.DynamicDrawUsage);
}

const RING_VERTEX = /* glsl */ `
uniform float uTime;
uniform float uScale;
attribute vec4 aColor;
attribute vec3 aTiming;
varying vec4 vColor;
varying float vT;
void main() {
  float t = (uTime - aTiming.x) / aTiming.y;
  if (aTiming.x < 0.0 || t < 0.0 || t > 1.0) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    gl_PointSize = 0.0;
    vColor = vec4(0.0);
    vT = 0.0;
    return;
  }
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mv;
  float r = aTiming.z * (0.15 + 0.85 * (1.0 - (1.0 - t) * (1.0 - t)));
  gl_PointSize = min(2.0 * r * uScale / -mv.z, 512.0);
  vColor = aColor;
  vT = t;
}`;

const RING_FRAGMENT = /* glsl */ `
varying vec4 vColor;
varying float vT;
void main() {
  float d = length(gl_PointCoord - 0.5) * 2.0;
  float ring = smoothstep(0.72, 0.9, d) * (1.0 - smoothstep(0.92, 1.0, d));
  float a = ring * vColor.a * (1.0 - vT) * (1.0 - vT);
  if (a < 0.003) discard;
  gl_FragColor = vec4(vColor.rgb * a, 1.0);
}`;

const DOT_VERTEX = /* glsl */ `
uniform float uScale;
attribute vec4 aColor;
attribute float aSize;
varying vec4 vColor;
void main() {
  vColor = aColor;
  if (aColor.a <= 0.0) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    gl_PointSize = 0.0;
    return;
  }
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mv;
  // Never smaller than a few pixels, so traffic stays legible when zoomed out.
  gl_PointSize = clamp(aSize * uScale / -mv.z, 3.0, 24.0);
}`;

const DOT_FRAGMENT = /* glsl */ `
varying vec4 vColor;
void main() {
  float d = length(gl_PointCoord - 0.5) * 2.0;
  if (d > 1.0) discard;
  float a = vColor.a * (1.0 - d * d);
  gl_FragColor = vec4(vColor.rgb * a, 1.0);
}`;
