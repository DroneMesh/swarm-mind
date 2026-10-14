// Pooled smoke and spark particles for downed drones. One Points draw call;
// particles animate on the GPU from their birth time, so a spawn writes a
// handful of floats and nothing is touched per frame afterwards.

import * as THREE from 'three';

const MAX_PARTICLES = 640;
/** Floats per particle in each attribute. */
const ORIGIN = 3;
const MOTION = 4; // vx, vy, vz, rise
const TIMING = 3; // born, life, size
const LOOK = 4; // r, g, b, alpha

export class SmokePool {
  readonly object: THREE.Points;
  private readonly geometry = new THREE.BufferGeometry();
  private readonly origin = new Float32Array(MAX_PARTICLES * ORIGIN);
  private readonly motion = new Float32Array(MAX_PARTICLES * MOTION);
  private readonly timing = new Float32Array(MAX_PARTICLES * TIMING).fill(-1);
  private readonly look = new Float32Array(MAX_PARTICLES * LOOK);
  private readonly time = { value: 0 };
  private readonly scale = { value: 800 };
  private next = 0;
  /** Time after which no particle is alive: lets the pool skip its draw call. */
  private quietAt = 0;
  private dirtyFrom = MAX_PARTICLES;
  private dirtyTo = -1;
  /** Deterministic jitter: no Math.random so captures are reproducible. */
  private seed = 1;

  constructor() {
    this.geometry.setAttribute('position', dyn(this.origin, ORIGIN));
    this.geometry.setAttribute('aMotion', dyn(this.motion, MOTION));
    this.geometry.setAttribute('aTiming', dyn(this.timing, TIMING));
    this.geometry.setAttribute('aLook', dyn(this.look, LOOK));
    this.object = new THREE.Points(
      this.geometry,
      new THREE.ShaderMaterial({
        uniforms: { uTime: this.time, uScale: this.scale },
        vertexShader: VERTEX,
        fragmentShader: FRAGMENT,
        transparent: true,
        depthWrite: false,
      }),
    );
    this.object.frustumCulled = false;
    this.object.renderOrder = 2;
    this.object.visible = false;
  }

  /** Pixels per world unit at distance 1, for point sizes. */
  setViewport(heightPx: number, fovDeg: number): void {
    this.scale.value = heightPx / (2 * Math.tan(THREE.MathUtils.degToRad(fovDeg) / 2));
  }

  /** A burst at the moment of death: dark smoke plus a few hot sparks. */
  burst(x: number, y: number, z: number, now: number): void {
    for (let i = 0; i < 14; i++) {
      const g = 0.16 + this.rand() * 0.1;
      this.spawn(x, y, z, this.spread(6), this.rand() * 3 + 1, this.spread(6), 2.5, now, 1.4 + this.rand(), 5 + this.rand() * 5, g, g, g * 1.05, 0.75);
    }
    for (let i = 0; i < 10; i++) {
      this.spawn(x, y, z, this.spread(16), this.rand() * 10 + 4, this.spread(16), -6, now, 0.5 + this.rand() * 0.4, 1.3, 1, 0.55 + this.rand() * 0.3, 0.15, 1);
    }
  }

  /** One trailing puff while a wreck falls or smoulders. */
  puff(x: number, y: number, z: number, now: number, strength: number): void {
    const g = 0.2 + this.rand() * 0.12;
    this.spawn(x, y, z, this.spread(1.5), 0, this.spread(1.5), 4 + this.rand() * 3, now, 2.2 + this.rand() * 1.5, (3 + this.rand() * 3) * strength, g, g, g, 0.5 * strength);
  }

  /** Advances the shared clock and uploads whatever was spawned since the last frame. */
  update(now: number): void {
    this.time.value = now;
    this.object.visible = now < this.quietAt;
    if (this.dirtyTo < 0) return;
    const count = this.dirtyTo - this.dirtyFrom + 1;
    for (const [name, size] of [['position', ORIGIN], ['aMotion', MOTION], ['aTiming', TIMING], ['aLook', LOOK]] as const) {
      const attr = this.geometry.getAttribute(name) as THREE.BufferAttribute;
      attr.clearUpdateRanges();
      attr.addUpdateRange(this.dirtyFrom * size, count * size);
      attr.needsUpdate = true;
    }
    this.dirtyFrom = MAX_PARTICLES;
    this.dirtyTo = -1;
  }

  reset(): void {
    this.timing.fill(-1);
    const attr = this.geometry.getAttribute('aTiming') as THREE.BufferAttribute;
    attr.clearUpdateRanges();
    attr.needsUpdate = true;
    this.quietAt = 0;
    this.object.visible = false;
  }

  private spawn(
    x: number, y: number, z: number,
    vx: number, vy: number, vz: number, rise: number,
    now: number, life: number, size: number,
    r: number, g: number, b: number, a: number,
  ): void {
    const i = this.next;
    this.next = (i + 1) % MAX_PARTICLES;
    this.origin.set([x, y, z], i * ORIGIN);
    this.motion.set([vx, vy, vz, rise], i * MOTION);
    this.timing.set([now, life, size], i * TIMING);
    this.look.set([r, g, b, a], i * LOOK);
    // A wrap-around makes the dirty span the whole buffer; that is rare and cheap.
    this.dirtyFrom = Math.min(this.dirtyFrom, i);
    this.dirtyTo = Math.max(this.dirtyTo, i);
    this.quietAt = Math.max(this.quietAt, now + life);
  }

  private spread(v: number): number {
    return (this.rand() * 2 - 1) * v;
  }

  private rand(): number {
    // xorshift32
    let s = this.seed;
    s ^= s << 13;
    s ^= s >>> 17;
    s ^= s << 5;
    this.seed = s >>> 0;
    return this.seed / 4294967296;
  }
}

function dyn(array: Float32Array, size: number): THREE.BufferAttribute {
  return new THREE.BufferAttribute(array, size).setUsage(THREE.DynamicDrawUsage);
}

const VERTEX = /* glsl */ `
  uniform float uTime;
  uniform float uScale;
  attribute vec4 aMotion;
  attribute vec3 aTiming;
  attribute vec4 aLook;
  varying vec4 vLook;
  varying float vAge;
  void main() {
    float age = (uTime - aTiming.x) / aTiming.y;
    if (aTiming.x < 0.0 || age < 0.0 || age > 1.0) {
      gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
      gl_PointSize = 0.0;
      return;
    }
    float t = age * aTiming.y;
    // Velocity decays (drag); 'rise' is buoyancy for smoke, gravity for sparks.
    vec3 drift = aMotion.xyz * (1.0 - exp(-t * 1.6)) / 1.6;
    vec3 p = position + drift + vec3(0.0, aMotion.w * t * t * 0.5 * (aMotion.w < 0.0 ? 1.0 : 0.25), 0.0);
    if (aMotion.w > 0.0) p.y += aMotion.w * t * 0.6;
    vec4 mv = modelViewMatrix * vec4(p, 1.0);
    gl_Position = projectionMatrix * mv;
    float grow = aMotion.w > 0.0 ? 0.6 + age * 1.8 : 1.0 - age * 0.5;
    gl_PointSize = aTiming.z * grow * uScale / -mv.z;
    vLook = aLook;
    vAge = age;
  }
`;

const FRAGMENT = /* glsl */ `
  varying vec4 vLook;
  varying float vAge;
  void main() {
    vec2 d = gl_PointCoord - 0.5;
    float r = length(d) * 2.0;
    if (r > 1.0) discard;
    float soft = smoothstep(1.0, 0.2, r);
    float fade = smoothstep(0.0, 0.12, vAge) * (1.0 - smoothstep(0.45, 1.0, vAge));
    gl_FragColor = vec4(vLook.rgb, vLook.a * soft * fade);
  }
`;
