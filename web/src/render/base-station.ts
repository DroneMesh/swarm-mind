// The ground station that drone 0 (the network sink) lives in: a landing
// pad with an H marking, a lattice mast with a rotating dish and a beacon.
// Static parts are merged by material, so the station costs a handful of
// draw calls however detailed it looks.

import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

/** The sim places the base this far above the ground (see `base_position`). */
const SIM_BASE_AGL = 3;
const PAD_RADIUS = 9;
/** Mast foot, relative to the pad centre: on the rim, out of the drones' way. */
const MAST_X = 10.5;
const MAST_Z = -7;
const MAST_HEIGHT = 24;
const EDGE_LIGHTS = 10;
/** Dish turns once every this many seconds. */
const DISH_PERIOD_S = 9;

export class BaseStation {
  readonly object = new THREE.Group();
  /** Meshes a ray may hit to pick the base. */
  readonly pickables: THREE.Object3D[];
  private readonly dish = new THREE.Group();
  private readonly beacon: THREE.Mesh<THREE.SphereGeometry, THREE.MeshBasicMaterial>;
  private readonly glow: THREE.Sprite;
  private readonly edge: THREE.Mesh<THREE.BufferGeometry, THREE.MeshBasicMaterial>;
  private readonly structureMaterial: THREE.MeshStandardMaterial;
  private alive = true;

  constructor() {
    const padMat = new THREE.MeshStandardMaterial({ color: 0x3a3f46, roughness: 0.85, metalness: 0.1 });
    // A deep skirt hides the gap where the flat pad meets sloping ground.
    const pad = new THREE.Mesh(
      mergeAll([
        new THREE.CylinderGeometry(PAD_RADIUS, PAD_RADIUS + 0.6, 1.0, 40).translate(0, 0.5, 0),
        new THREE.CylinderGeometry(PAD_RADIUS + 0.6, PAD_RADIUS + 2.5, 7, 40, 1, true).translate(0, -3.5, 0),
      ]),
      padMat,
    );

    const markings = new THREE.Mesh(
      mergeAll([
        tint(new THREE.RingGeometry(PAD_RADIUS - 1.4, PAD_RADIUS - 0.8, 48).rotateX(-Math.PI / 2), 0xffc93c),
        tint(new THREE.BoxGeometry(1.1, 0.06, 7).translate(-2.2, 0, 0), 0xf2f4f7),
        tint(new THREE.BoxGeometry(1.1, 0.06, 7).translate(2.2, 0, 0), 0xf2f4f7),
        tint(new THREE.BoxGeometry(3.3, 0.06, 1.1), 0xf2f4f7),
      ]).translate(0, 1.03, 0),
      new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.6, emissive: 0x222222 }),
    );

    this.structureMaterial = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.45 });
    const structure = new THREE.Mesh(mergeAll(this.structureParts()), this.structureMaterial);

    const dishMat = new THREE.MeshStandardMaterial({ color: 0xdfe3e8, roughness: 0.4, metalness: 0.3, side: THREE.DoubleSide });
    const bowl = new THREE.Mesh(dishGeometry(), dishMat);
    this.dish.position.set(MAST_X, MAST_HEIGHT - 3.5, MAST_Z);
    this.dish.add(bowl);

    this.beacon = new THREE.Mesh(new THREE.SphereGeometry(0.75, 10, 8), new THREE.MeshBasicMaterial({ color: 0xff3b30, toneMapped: false }));
    this.beacon.position.set(MAST_X, MAST_HEIGHT + 1.6, MAST_Z);
    this.glow = new THREE.Sprite(
      new THREE.SpriteMaterial({ map: glowTexture(), color: 0xff4a3a, blending: THREE.AdditiveBlending, depthWrite: false, transparent: true }),
    );
    this.glow.scale.setScalar(9);
    this.glow.position.copy(this.beacon.position);

    const lights: THREE.BufferGeometry[] = [];
    for (let i = 0; i < EDGE_LIGHTS; i++) {
      const a = (i / EDGE_LIGHTS) * Math.PI * 2;
      lights.push(new THREE.SphereGeometry(0.28, 6, 4).translate(Math.cos(a) * (PAD_RADIUS - 0.3), 1.2, Math.sin(a) * (PAD_RADIUS - 0.3)));
    }
    this.edge = new THREE.Mesh(mergeAll(lights), new THREE.MeshBasicMaterial({ color: 0x9fe8ff, toneMapped: false }));

    this.object.add(pad, markings, structure, this.dish, this.edge, this.beacon, this.glow);
    this.pickables = [pad, structure];
    this.object.visible = false;
  }

  /** Places the station under the base drone's sim position and animates it. */
  update(x: number, y: number, z: number, timeS: number, alive: boolean): void {
    this.object.visible = true;
    this.object.position.set(x, y - SIM_BASE_AGL, z);
    if (alive !== this.alive) {
      this.alive = alive;
      this.structureMaterial.color.setHex(alive ? 0xffffff : 0x55504c);
      this.edge.visible = alive;
    }
    if (alive) this.dish.rotation.y = (timeS / DISH_PERIOD_S) * Math.PI * 2;
    // Beacon: a slow double blink, like an aviation obstruction light.
    const phase = timeS % 1.6;
    const on = alive && (phase < 0.12 || (phase > 0.3 && phase < 0.42));
    const level = on ? 1 : 0.12;
    this.beacon.material.color.setRGB(1.6 * level + 0.1, 0.25 * level, 0.2 * level);
    this.glow.material.opacity = alive ? (on ? 1 : 0.08) : 0;
    const e = 0.55 + 0.45 * Math.sin(timeS * 2.2);
    this.edge.material.color.setRGB(0.55 * e + 0.2, 0.95 * e + 0.25, 1.3 * e + 0.25);
  }

  hide(): void {
    this.object.visible = false;
  }

  private structureParts(): THREE.BufferGeometry[] {
    const steel = 0x8a939e;
    const dark = 0x2b3038;
    const parts: THREE.BufferGeometry[] = [];
    // Three-legged lattice mast, tapering toward the top.
    const legs = 3;
    const footR = 1.6;
    const topR = 0.55;
    const corners: THREE.Vector3[][] = [];
    for (let l = 0; l < legs; l++) {
      const a = (l / legs) * Math.PI * 2;
      const foot = new THREE.Vector3(MAST_X + Math.cos(a) * footR, 0, MAST_Z + Math.sin(a) * footR);
      const top = new THREE.Vector3(MAST_X + Math.cos(a) * topR, MAST_HEIGHT, MAST_Z + Math.sin(a) * topR);
      parts.push(tint(strut(foot, top, 0.16), steel));
      corners.push([foot, top]);
    }
    const levels = 7;
    for (let k = 0; k <= levels; k++) {
      const t0 = k / levels;
      const t1 = (k + 1) / levels;
      for (let l = 0; l < legs; l++) {
        const a = corners[l];
        const b = corners[(l + 1) % legs];
        const p0 = a[0].clone().lerp(a[1], t0);
        const q0 = b[0].clone().lerp(b[1], t0);
        parts.push(tint(strut(p0, q0, 0.07), steel));
        if (k < levels) parts.push(tint(strut(p0, b[0].clone().lerp(b[1], t1), 0.06), steel));
      }
    }
    // Mast head with the beacon post, and a short arm to the dish.
    parts.push(tint(new THREE.CylinderGeometry(0.75, 0.75, 0.6, 10).translate(MAST_X, MAST_HEIGHT, MAST_Z), dark));
    parts.push(tint(new THREE.CylinderGeometry(0.16, 0.16, 1.4, 6).translate(MAST_X, MAST_HEIGHT + 0.9, MAST_Z), dark));
    parts.push(tint(new THREE.CylinderGeometry(0.35, 0.35, 1.6, 8).translate(MAST_X, MAST_HEIGHT - 3.5, MAST_Z), dark));
    // Equipment shelter beside the mast, and a ground-plane antenna on the pad rim.
    parts.push(tint(new THREE.BoxGeometry(5.5, 3.6, 3.8).translate(MAST_X + 1.5, 1.8, MAST_Z + 5.2), 0xd6d2c4));
    parts.push(tint(new THREE.BoxGeometry(5.9, 0.35, 4.2).translate(MAST_X + 1.5, 3.75, MAST_Z + 5.2), dark));
    parts.push(tint(new THREE.BoxGeometry(1.2, 2.2, 0.1).translate(MAST_X + 0.2, 1.1, MAST_Z + 7.15), dark));
    parts.push(tint(new THREE.BoxGeometry(1.5, 0.9, 0.12).translate(MAST_X + 2.8, 2.5, MAST_Z + 7.15), 0x4fc3f7));
    // Concrete foundation under the mast.
    parts.push(tint(new THREE.CylinderGeometry(2.6, 3, 0.8, 12).translate(MAST_X, 0.2, MAST_Z), 0x5d6168));
    return parts;
  }
}

/** A dish on a yoke: a shallow paraboloid bowl tilted 35 degrees toward the sky, with a feed horn. */
function dishGeometry(): THREE.BufferGeometry {
  const points: THREE.Vector2[] = [];
  const r = 3.4;
  for (let i = 0; i <= 10; i++) {
    const x = (i / 10) * r;
    points.push(new THREE.Vector2(x, (x * x) / (4 * 2.6)));
  }
  const bowl = new THREE.LatheGeometry(points, 28);
  const feedArm = new THREE.CylinderGeometry(0.08, 0.08, 2.6, 4).translate(0, 1.3, 0);
  const feed = new THREE.CylinderGeometry(0.28, 0.2, 0.5, 8).translate(0, 2.6, 0);
  const g = mergeAll([bowl, feedArm, feed]);
  // Tilt back and push out so the bowl rotates around the mast without hitting it.
  g.rotateX(-0.95).translate(0, 0.8, 2.2);
  return g;
}

/** A thin box from `a` to `b`. */
function strut(a: THREE.Vector3, b: THREE.Vector3, thickness: number): THREE.BufferGeometry {
  const len = a.distanceTo(b);
  const g = new THREE.BoxGeometry(thickness, len, thickness);
  const dir = b.clone().sub(a).normalize();
  g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir));
  g.translate((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
  return g;
}

function tint(g: THREE.BufferGeometry, hex: number): THREE.BufferGeometry {
  const c = new THREE.Color(hex);
  const n = g.getAttribute('position').count;
  const colors = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) c.toArray(colors, i * 3);
  g.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  return g;
}

function mergeAll(parts: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const flat = parts.map((p) => {
    const g = p.index ? p.toNonIndexed() : p;
    g.deleteAttribute('uv');
    return g;
  });
  const merged = mergeGeometries(flat, false);
  if (!merged) throw new Error('base-station: incompatible part attributes');
  merged.computeBoundingSphere();
  return merged;
}

let glowTex: THREE.Texture | null = null;

/** Soft radial falloff, drawn once and shared. */
export function glowTexture(): THREE.Texture {
  if (glowTex) return glowTex;
  const size = 64;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');
  if (ctx) {
    const g = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    g.addColorStop(0, 'rgba(255,255,255,1)');
    g.addColorStop(0.25, 'rgba(255,255,255,0.45)');
    g.addColorStop(1, 'rgba(255,255,255,0)');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, size, size);
  }
  glowTex = new THREE.CanvasTexture(canvas);
  return glowTex;
}
