// Procedural low-poly quadcopter, split by material so the whole swarm draws
// with one InstancedMesh per part. Local frame: +Y up, +Z forward (nose),
// so port (left) is +X and starboard (right) is -X. Units are world units;
// the airframe spans about 12.8 rotor tip to rotor tip.

import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

/** Distance from the hull centre to each motor axis. */
export const ARM_REACH = 5.2;
export const ROTOR_RADIUS = 2.7;
/** Rotor disc height above the hull centre. */
const ROTOR_Y = 1.62;
/** Lowest point of the landing legs below the hull centre: a landed drone sits this high. */
export const SKID_DEPTH = 2.25;
/** Rotor tip to rotor tip, used for the minimum on-screen size. */
export const DRONE_SPAN = 2 * (ARM_REACH * Math.SQRT1_2 + ROTOR_RADIUS);

/** Motor axes on the diagonals; front pair first (port, starboard), then rear. */
export const MOTORS: readonly [number, number][] = [
  [ARM_REACH * Math.SQRT1_2, ARM_REACH * Math.SQRT1_2],
  [-ARM_REACH * Math.SQRT1_2, ARM_REACH * Math.SQRT1_2],
  [ARM_REACH * Math.SQRT1_2, -ARM_REACH * Math.SQRT1_2],
  [-ARM_REACH * Math.SQRT1_2, -ARM_REACH * Math.SQRT1_2],
];

/** Light slots per drone, in the order the swarm's light buffer stores them. */
export const LIGHT_SLOTS = 4;
export const LIGHT = { port: 0, starboard: 1, strobe: 2, antenna: 3 } as const;
/** Local offsets of each light slot. */
export const LIGHT_OFFSETS: readonly THREE.Vector3[] = [
  new THREE.Vector3(MOTORS[0][0] + 0.7, 0.35, MOTORS[0][1] + 0.7),
  new THREE.Vector3(MOTORS[1][0] - 0.7, 0.35, MOTORS[1][1] + 0.7),
  new THREE.Vector3(0, 1.25, -0.9),
  new THREE.Vector3(0, 7.4, -2.2),
];

/** Shade multiplied with the per-drone body tint: accents stay dark whatever the role. */
const SHELL = 1;
const ACCENT = 0.32;
const GLASS = 0.12;

/** Hull, top shell, battery and sensor bar. Vertex colours are multiplied by the instance tint. */
export function bodyGeometry(): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  parts.push(shade(new THREE.SphereGeometry(1, 18, 10).scale(2.25, 0.85, 3.1), SHELL));
  // Raised spine: the dome reads as "drone" rather than "pebble" at a distance.
  parts.push(
    shade(
      new THREE.SphereGeometry(1, 16, 6, 0, Math.PI * 2, 0, Math.PI / 2).scale(1.55, 0.95, 2.35).translate(0, 0.3, -0.25),
      SHELL,
    ),
  );
  parts.push(shade(new THREE.BoxGeometry(1.7, 0.7, 3.2).translate(0, -0.78, -0.3), ACCENT));
  // Front obstacle-sensor bar and rear vent: dark glass.
  parts.push(shade(new THREE.BoxGeometry(1.7, 0.42, 0.5).translate(0, 0.12, 2.95), GLASS));
  parts.push(shade(new THREE.BoxGeometry(1.3, 0.3, 0.4).translate(0, 0.25, -3.0), GLASS));
  return merge(parts);
}

/** Arms, motor pods, landing legs and the camera gimbal: one dark material. */
export function frameGeometry(): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  for (const [x, z] of MOTORS) {
    const angle = Math.atan2(x, z);
    const inner = 1.4;
    const len = ARM_REACH - inner;
    parts.push(
      new THREE.BoxGeometry(0.55, 0.42, len)
        .translate(0, 0.15, inner + len / 2)
        .rotateY(angle),
    );
    parts.push(new THREE.CylinderGeometry(0.72, 0.85, 1.25, 12).translate(x, 0.65, z));
    parts.push(new THREE.CylinderGeometry(0.22, 0.22, 0.5, 6).translate(x, 1.45, z));
    // Leg under each motor, splayed slightly outward, with a foot pad.
    const leg = new THREE.CylinderGeometry(0.14, 0.2, 2.3, 5).translate(0, -1.15, 0);
    leg.rotateZ(x > 0 ? -0.12 : 0.12).rotateX(z > 0 ? 0.12 : -0.12).translate(x * 0.82, 0.05, z * 0.82);
    parts.push(leg);
    parts.push(new THREE.SphereGeometry(0.2, 6, 4).translate(x * 0.82 + Math.sign(x) * 0.27, -SKID_DEPTH + 0.28, z * 0.82 + Math.sign(z) * 0.27));
  }
  parts.push(new THREE.SphereGeometry(0.62, 10, 8).translate(0, -1.05, 2.4));
  parts.push(new THREE.CylinderGeometry(0.32, 0.32, 0.3, 10).rotateX(Math.PI / 2).translate(0, -1.05, 2.98));
  // Short whip antennas at the rear: every drone has a radio.
  for (const sx of [-1, 1]) parts.push(new THREE.CylinderGeometry(0.06, 0.08, 1.6, 4).translate(sx * 0.9, 1.2, -2.6));
  return merge(parts.map(plain));
}

/**
 * Role LED strip: an elliptical band around the hull plus a glow under each
 * motor, drawn unlit in the role colour so it reads from any angle.
 */
export function ringGeometry(): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  parts.push(new THREE.TorusGeometry(1, 0.1, 5, 48).rotateX(Math.PI / 2).scale(2.3, 1.2, 3.14).translate(0, 0.02, 0));
  for (const [x, z] of MOTORS) parts.push(new THREE.CylinderGeometry(0.62, 0.62, 0.22, 10).translate(x, 0.0, z));
  return merge(parts.map(plain));
}

/** Tall mast with a blinking tip: fitted to relays only. */
export function antennaGeometry(): THREE.BufferGeometry {
  const parts = [
    new THREE.CylinderGeometry(0.1, 0.16, 6.2, 5).translate(0, 4.2, -2.2),
    new THREE.CylinderGeometry(0.42, 0.42, 0.5, 8).translate(0, 1.2, -2.2),
    new THREE.CylinderGeometry(0.5, 0.06, 0.25, 8).translate(0, 5.6, -2.2),
  ];
  return merge(parts.map(plain));
}

/**
 * Four rotor discs. `aLocal` holds the position on the disc in rotor radii
 * and `aDir` the spin sense (diagonal pairs counter-rotate, as on a real quad).
 */
export function rotorGeometry(): THREE.BufferGeometry {
  const parts: THREE.BufferGeometry[] = [];
  MOTORS.forEach(([x, z], i) => {
    const disc = new THREE.CircleGeometry(ROTOR_RADIUS, 32).rotateX(-Math.PI / 2);
    const pos = disc.getAttribute('position') as THREE.BufferAttribute;
    const local = new Float32Array(pos.count * 2);
    const dir = new Float32Array(pos.count).fill(i === 0 || i === 3 ? 1 : -1);
    for (let v = 0; v < pos.count; v++) {
      local[v * 2] = pos.getX(v) / ROTOR_RADIUS;
      local[v * 2 + 1] = pos.getZ(v) / ROTOR_RADIUS;
    }
    disc.setAttribute('aLocal', new THREE.BufferAttribute(local, 2));
    disc.setAttribute('aDir', new THREE.BufferAttribute(dir, 1));
    disc.deleteAttribute('uv');
    disc.deleteAttribute('normal');
    disc.translate(x, ROTOR_Y, z);
    parts.push(disc);
  });
  return merge(parts);
}

/**
 * Downward searchlight: an open cone with its apex at the origin and its
 * base at y = -1, plus a ground pool at the base. `aPart` is 0 on the
 * cone, 1 on the pool; `aT` runs 0 at the apex to 1 at the base.
 */
export function searchlightGeometry(): THREE.BufferGeometry {
  const cone = new THREE.ConeGeometry(1, 1, 28, 6, true).rotateX(Math.PI).translate(0, -0.5, 0);
  const pool = new THREE.CircleGeometry(1.15, 28).rotateX(-Math.PI / 2).translate(0, -0.985, 0);
  const tag = (g: THREE.BufferGeometry, part: number) => {
    const pos = g.getAttribute('position') as THREE.BufferAttribute;
    const t = new Float32Array(pos.count);
    for (let v = 0; v < pos.count; v++) t[v] = part === 0 ? -pos.getY(v) : Math.hypot(pos.getX(v), pos.getZ(v));
    g.setAttribute('aT', new THREE.BufferAttribute(t, 1));
    g.setAttribute('aPart', new THREE.BufferAttribute(new Float32Array(pos.count).fill(part), 1));
    g.deleteAttribute('uv');
    return g;
  };
  return merge([tag(cone, 0), tag(pool, 1)]);
}

/** A camera-facing quad for glow sprites; `position.xy` is the corner in -1..1. */
export function spriteGeometry(): THREE.BufferGeometry {
  const g = new THREE.PlaneGeometry(2, 2);
  g.deleteAttribute('normal');
  g.deleteAttribute('uv');
  return g;
}

function shade(g: THREE.BufferGeometry, v: number): THREE.BufferGeometry {
  const n = g.getAttribute('position').count;
  g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3).fill(v), 3));
  g.deleteAttribute('uv');
  return g;
}

function plain(g: THREE.BufferGeometry): THREE.BufferGeometry {
  g.deleteAttribute('uv');
  return g;
}

/** Merges non-indexed copies, so primitives with and without an index mix freely. */
function merge(parts: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const flat = parts.map((p) => (p.index ? p.toNonIndexed() : p));
  const merged = mergeGeometries(flat, false);
  if (!merged) throw new Error('drone-model: incompatible part attributes');
  for (const p of parts) p.dispose();
  merged.computeBoundingSphere();
  return merged;
}
