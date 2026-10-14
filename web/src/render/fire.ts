import * as THREE from 'three';
import { FireCell } from '../protocol';
import type { Terrain } from './terrain';

/** Particle budgets: hard caps, whatever the size of the fire. */
const MAX_EMBERS = 640;
const MAX_SMOKE = 200;
/** Particles per burning cell, so a small fire is not one bright blob. */
const EMBERS_PER_CELL = 10;
const SMOKE_PER_CELL = 2;

/**
 * Draws the fire grid on the terrain and the embers and smoke above it.
 *
 * The grid is uploaded as a small RGBA texture (R = burning, G = burnt) that
 * the terrain's own material samples by world xz, so the fire costs no extra
 * geometry or draw calls. Particles are animated entirely on the GPU from a
 * per-particle seed; the CPU only reassigns origins when the grid changes
 * (at most 4 Hz), never per frame.
 */
export class FireLayer {
  readonly object = new THREE.Group();
  /** Burning cells in the latest grid, and their centroid in world xz. */
  burning = 0;
  readonly centroid = new THREE.Vector2();

  private readonly uniforms = {
    uFire: { value: blankTexture() as THREE.DataTexture },
    uFireExtent: { value: 1200 },
    uTime: { value: 0 },
  };
  private size = 0;
  private texels = new Uint8Array(4);
  private cells = new Uint8Array(0);
  private burningList = new Int32Array(0);
  private terrain: Terrain | null = null;
  private readonly embers: Particles;
  private readonly smoke: Particles;
  private readonly t0 = performance.now();

  constructor() {
    this.embers = new Particles(MAX_EMBERS, EMBERS_PER_CELL, EMBER_SHADER, THREE.AdditiveBlending, this.uniforms.uTime);
    this.smoke = new Particles(MAX_SMOKE, SMOKE_PER_CELL, SMOKE_SHADER, THREE.NormalBlending, this.uniforms.uTime);
    this.smoke.points.renderOrder = 1;
    this.embers.points.renderOrder = 2;
    this.object.add(this.smoke.points, this.embers.points);
  }

  /** Extends the terrain material to show burning and burnt ground. */
  readonly patch = (material: THREE.MeshStandardMaterial): void => {
    material.onBeforeCompile = (shader) => {
      Object.assign(shader.uniforms, this.uniforms);
      shader.vertexShader = shader.vertexShader
        .replace('#include <common>', '#include <common>\nvarying vec2 vFireXZ;')
        .replace('#include <project_vertex>', '#include <project_vertex>\nvFireXZ = (modelMatrix * vec4(transformed, 1.0)).xz;');
      shader.fragmentShader = shader.fragmentShader
        .replace('#include <common>', `#include <common>\n${TERRAIN_FIRE_PARS}`)
        .replace('#include <color_fragment>', `#include <color_fragment>\n${TERRAIN_FIRE_COLOR}`)
        .replace('#include <emissivemap_fragment>', `#include <emissivemap_fragment>\n${TERRAIN_FIRE_EMISSIVE}`);
    };
  };

  setTerrain(terrain: Terrain | null): void {
    this.terrain = terrain;
    if (terrain) this.uniforms.uFireExtent.value = terrain.extent;
  }

  /** Converts world-space point-size units to pixels; call on resize. */
  setViewport(heightPx: number, fovDeg: number): void {
    const scale = heightPx / (2 * Math.tan(THREE.MathUtils.degToRad(fovDeg) / 2));
    this.embers.scale.value = scale;
    this.smoke.scale.value = scale;
  }

  /** Takes a new grid from the worker (row-major, rows along +z). */
  setCells(size: number, cells: Uint8Array): void {
    if (size !== this.size) this.resize(size);
    const prev = this.cells;
    const extent = this.uniforms.uFireExtent.value;
    const cell = extent / size;
    let n = 0;
    let fresh = 0;
    let sx = 0;
    let sz = 0;
    for (let i = 0; i < cells.length; i++) {
      const v = cells[i];
      this.texels[i * 4] = v === FireCell.Burning ? 255 : 0;
      this.texels[i * 4 + 1] = v === FireCell.Burnt ? 255 : 0;
      if (v !== FireCell.Burning) continue;
      this.burningList[n++] = i;
      if (prev[i] !== FireCell.Burning) fresh++;
      sx += (i % size) + 0.5;
      sz += Math.floor(i / size) + 0.5;
    }
    prev.set(cells);
    this.uniforms.uFire.value.needsUpdate = true;
    this.burning = n;
    if (n > 0) this.centroid.set(-extent / 2 + (sx / n) * cell, -extent / 2 + (sz / n) * cell);

    const now = this.time();
    const place = (cellIndex: number, out: Float32Array, o: number) => {
      const x = -extent / 2 + ((cellIndex % size) + Math.random()) * cell;
      const z = -extent / 2 + (Math.floor(cellIndex / size) + Math.random()) * cell;
      out[o] = x;
      out[o + 1] = (this.terrain?.heightAt(x, z) ?? 0) + 1;
      out[o + 2] = z;
    };
    this.embers.assign(this.burningList, n, fresh, this.cells, place, now);
    this.smoke.assign(this.burningList, n, fresh, this.cells, place, now);
  }

  update(): void {
    this.uniforms.uTime.value = this.time();
  }

  /** Back to an unburnt map, e.g. on a simulation reset. */
  reset(): void {
    this.texels.fill(0);
    this.cells.fill(0);
    this.uniforms.uFire.value.needsUpdate = true;
    this.burning = 0;
    this.embers.hideAll();
    this.smoke.hideAll();
  }

  private resize(size: number): void {
    this.size = size;
    this.uniforms.uFire.value.dispose();
    this.texels = new Uint8Array(size * size * 4);
    this.cells = new Uint8Array(size * size);
    this.burningList = new Int32Array(size * size);
    const texture = new THREE.DataTexture(this.texels, size, size, THREE.RGBAFormat, THREE.UnsignedByteType);
    // Linear filtering softens cell edges into a ragged-looking front.
    texture.magFilter = THREE.LinearFilter;
    texture.minFilter = THREE.LinearFilter;
    texture.wrapS = THREE.ClampToEdgeWrapping;
    texture.wrapT = THREE.ClampToEdgeWrapping;
    this.uniforms.uFire.value = texture;
  }

  private time(): number {
    return (performance.now() - this.t0) / 1000;
  }
}

type Place = (cellIndex: number, out: Float32Array, offset: number) => void;

/** One capped pool of GPU-animated point sprites tied to burning cells. */
class Particles {
  readonly points: THREE.Points;
  readonly scale = { value: 800 };
  private readonly origin: Float32Array;
  private readonly born: Float32Array;
  /** The cell each particle is attached to, or -1. */
  private readonly cellOf: Int32Array;
  private readonly geometry = new THREE.BufferGeometry();

  constructor(
    private readonly max: number,
    private readonly perCell: number,
    shader: { vertex: string; fragment: string },
    blending: THREE.Blending,
    time: { value: number },
  ) {
    this.origin = new Float32Array(max * 3);
    this.born = new Float32Array(max).fill(-1);
    this.cellOf = new Int32Array(max).fill(-1);
    const seed = new Float32Array(max * 4);
    for (let i = 0; i < seed.length; i++) seed[i] = Math.random();
    this.geometry.setAttribute('position', new THREE.BufferAttribute(this.origin, 3));
    this.geometry.setAttribute('aSeed', new THREE.BufferAttribute(seed, 4));
    this.geometry.setAttribute('aBorn', new THREE.BufferAttribute(this.born, 1));
    const material = new THREE.ShaderMaterial({
      uniforms: { uTime: time, uScale: this.scale },
      vertexShader: shader.vertex,
      fragmentShader: shader.fragment,
      transparent: true,
      depthWrite: false,
      blending,
    });
    this.points = new THREE.Points(this.geometry, material);
    this.points.frustumCulled = false;
  }

  /**
   * Keeps particles on cells that still burn, moves those whose cell went out,
   * and moves a share matching the newly lit cells so the spread is followed.
   */
  assign(list: Int32Array, n: number, fresh: number, cells: Uint8Array, place: Place, now: number): void {
    const active = Math.min(this.max, n * this.perCell);
    const churn = n > 0 ? fresh / n : 0;
    for (let p = 0; p < this.max; p++) {
      if (p >= active) {
        this.born[p] = -1;
        this.cellOf[p] = -1;
        continue;
      }
      const c = this.cellOf[p];
      if (c >= 0 && cells[c] === FireCell.Burning && Math.random() >= churn) continue;
      const next = list[Math.floor(Math.random() * n)];
      this.cellOf[p] = next;
      place(next, this.origin, p * 3);
      this.born[p] = now;
    }
    this.geometry.getAttribute('position').needsUpdate = true;
    this.geometry.getAttribute('aBorn').needsUpdate = true;
  }

  hideAll(): void {
    this.born.fill(-1);
    this.cellOf.fill(-1);
    this.geometry.getAttribute('aBorn').needsUpdate = true;
  }
}

function blankTexture(): THREE.DataTexture {
  const texture = new THREE.DataTexture(new Uint8Array(4), 1, 1, THREE.RGBAFormat, THREE.UnsignedByteType);
  texture.needsUpdate = true;
  return texture;
}

const NOISE_GLSL = /* glsl */ `
float fireHash(vec2 p) {
  return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}
float fireNoise(vec2 p) {
  vec2 i = floor(p);
  vec2 f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(fireHash(i), fireHash(i + vec2(1.0, 0.0)), f.x),
             mix(fireHash(i + vec2(0.0, 1.0)), fireHash(i + vec2(1.0, 1.0)), f.x), f.y);
}`;

const TERRAIN_FIRE_PARS = /* glsl */ `
uniform sampler2D uFire;
uniform float uFireExtent;
uniform float uTime;
varying vec2 vFireXZ;
vec2 fireState;
float fireFlicker;
${NOISE_GLSL}`;

/** Chars the ground under burning and burnt cells. */
const TERRAIN_FIRE_COLOR = /* glsl */ `
fireState = texture2D(uFire, vFireXZ / uFireExtent + 0.5).rg;
fireFlicker = fireNoise(vFireXZ * 0.09 + vec2(uTime * 1.7, -uTime * 2.3)) * 0.65
            + fireNoise(vFireXZ * 0.31 - uTime * 4.0) * 0.35;
diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.045, 0.04, 0.038),
                       clamp(max(fireState.g, fireState.r * 0.85) * 1.3, 0.0, 0.92));`;

/** Burning cells glow and flicker; burnt ground keeps a faint smoulder. */
const TERRAIN_FIRE_EMISSIVE = /* glsl */ `
{
  float heat = smoothstep(0.05, 0.6, fireState.r) * (0.55 + 0.75 * fireFlicker);
  vec3 flame = mix(vec3(0.75, 0.07, 0.01), vec3(1.0, 0.52, 0.08), fireFlicker * fireState.r);
  totalEmissiveRadiance += flame * heat * 1.5;
  totalEmissiveRadiance += vec3(0.35, 0.05, 0.0) * fireState.g * smoothstep(0.82, 1.0, fireFlicker) * 0.5;
}`;

/** Shared vertex shader: rise, drift downwind, fade in and out over a life. */
function particleVertex(
  life: [number, number],
  rise: number,
  drift: number,
  spread: number,
  size: number,
  grow: number,
): string {
  return /* glsl */ `
uniform float uTime;
uniform float uScale;
attribute vec4 aSeed;
attribute float aBorn;
varying float vT;
varying float vA;
void main() {
  if (aBorn < 0.0) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    gl_PointSize = 0.0;
    vT = 0.0;
    vA = 0.0;
    return;
  }
  float life = mix(${life[0].toFixed(2)}, ${life[1].toFixed(2)}, aSeed.y);
  float age = uTime - aBorn;
  float t = fract((age + aSeed.x * life) / life);
  float a = aSeed.z * 6.2831853;
  vec3 p = position;
  p.y += ${rise.toFixed(1)} * t * (0.6 + 0.4 * aSeed.w);
  p.xz += vec2(0.8, 0.35) * ${drift.toFixed(1)} * t * t + vec2(cos(a), sin(a)) * ${spread.toFixed(1)} * t;
  p.x += sin(uTime * 2.3 + aSeed.x * 40.0) * ${(spread * 0.4).toFixed(1)} * t;
  vec4 mv = modelViewMatrix * vec4(p, 1.0);
  gl_Position = projectionMatrix * mv;
  float s = ${size.toFixed(1)} * mix(1.0, ${grow.toFixed(1)}, t) * (0.6 + 0.8 * aSeed.w);
  gl_PointSize = min(s * uScale / -mv.z, 160.0);
  vT = t;
  vA = smoothstep(0.0, 0.08, t) * (1.0 - smoothstep(0.55, 1.0, t)) * clamp(age * 1.5, 0.0, 1.0);
}`;
}

const EMBER_SHADER = {
  vertex: particleVertex([1.2, 2.6], 70, 25, 10, 2.6, 0.5),
  fragment: /* glsl */ `
varying float vT;
varying float vA;
void main() {
  float d = length(gl_PointCoord - 0.5) * 2.0;
  if (d > 1.0) discard;
  vec3 c = mix(vec3(1.0, 0.85, 0.45), vec3(1.0, 0.22, 0.04), vT);
  gl_FragColor = vec4(c * vA * (1.0 - d * d), 1.0);
}`,
};

const SMOKE_SHADER = {
  vertex: particleVertex([4.0, 7.0], 150, 35, 14, 22, 3.0),
  fragment: /* glsl */ `
varying float vT;
varying float vA;
void main() {
  float d = length(gl_PointCoord - 0.5) * 2.0;
  if (d > 1.0) discard;
  vec3 c = mix(vec3(0.22, 0.2, 0.19), vec3(0.5, 0.5, 0.53), vT);
  gl_FragColor = vec4(c, vA * 0.32 * pow(1.0 - d, 1.4));
}`,
};
