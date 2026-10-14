import * as THREE from 'three';
import { PRESETS, type PresetInfo } from '../presets';

const ROCK_LINE = 130;
const SNOW_LINE = 195;
/** 1 - normal.y above which a slope reads as cliff. */
const CLIFF = 0.35;
/** Height above the sea that is beach rather than grass. */
const SHORE_BAND = 5;
/** Depth over which the seabed fades from sand to the deep-water colour. */
const SEABED_FADE = 22;
/** Thickness of one sandstone layer on strata maps. */
const STRATUM = 17;
/** Strata maps: ground below this is the gorge floor's sand. */
const FLOOR_LINE = 28;
/** The sea plane reaches this many map widths, so islands sit in an ocean, not a pool. */
const SEA_SCALE = 5;

/**
 * The terrain mesh built from the simulator's heightmap, plus a height
 * sampler for placing things on the ground. Vertex order matches the
 * simulator's grid: row r runs along +z, column c along +x.
 *
 * Colours follow the preset's palette. A map with a `waterLevel` also gets
 * one flat, semi-transparent sea plane, a child of `mesh` (so adding `mesh`
 * to the scene adds it too): one draw call for the ground, one for the sea.
 */
export class Terrain {
  readonly mesh: THREE.Mesh<THREE.PlaneGeometry, THREE.MeshStandardMaterial>;
  /** The sea surface, or null on a dry map. */
  readonly water: THREE.Mesh<THREE.PlaneGeometry, THREE.MeshStandardMaterial> | null = null;
  readonly preset: PresetInfo;
  private readonly cell: number;
  private readonly floor: number;

  /**
   * `patch` may extend the material's shader (the fire layer does). `preset`
   * is a `PresetInfo` or a swarm-sim `Preset` value; it defaults to Alpine
   * Ridge, whose colours are exactly the original ones.
   */
  constructor(
    readonly size: number,
    readonly extent: number,
    private readonly heights: Float32Array,
    patch?: (material: THREE.MeshStandardMaterial) => void,
    preset: PresetInfo | number = 0,
  ) {
    this.preset = typeof preset === 'number' ? (PRESETS[preset] ?? PRESETS[0]) : preset;
    this.cell = extent / (size - 1);
    const sea = this.preset.waterLevel;
    this.floor = sea ?? -Infinity;

    const geometry = new THREE.PlaneGeometry(extent, extent, size - 1, size - 1);
    // Lying flat, PlaneGeometry's row 0 sits at z = -extent/2, like the sim's.
    geometry.rotateX(-Math.PI / 2);

    const position = geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < position.count; i++) position.setY(i, heights[i]);
    geometry.computeVertexNormals();
    geometry.computeBoundingSphere();

    const normal = geometry.getAttribute('normal') as THREE.BufferAttribute;
    geometry.setAttribute('color', new THREE.BufferAttribute(paint(this.preset, heights, normal), 3));

    const material = new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 0.95 });
    patch?.(material);
    this.mesh = new THREE.Mesh(geometry, material);

    if (sea !== undefined) {
      const span = extent * SEA_SCALE;
      const waterGeometry = new THREE.PlaneGeometry(span, span);
      waterGeometry.rotateX(-Math.PI / 2);
      const waterMaterial = new THREE.MeshStandardMaterial({
        color: this.preset.palette.water ?? 0x1f5a7a,
        transparent: true,
        opacity: 0.78,
        roughness: 0.2,
        metalness: 0.15,
      });
      this.water = new THREE.Mesh(waterGeometry, waterMaterial);
      // A hair above the surface height so the shoreline doesn't z-fight.
      this.water.position.y = sea + 0.3;
      // Drawn after the ground so the seabed shows through.
      this.water.renderOrder = 1;
      this.water.name = 'sea';
      this.mesh.add(this.water);
    }
  }

  /**
   * Height of what stands at world `(x, z)`, clamped to the map: the
   * ground, or the sea surface over the seabed (as the sim's `surface_at`).
   */
  heightAt(x: number, z: number): number {
    return Math.max(this.groundAt(x, z), this.floor);
  }

  /** Bilinear ground (or seabed) height at world `(x, z)`, clamped to the map. */
  groundAt(x: number, z: number): number {
    const max = this.size - 1;
    const fc = clamp((x + this.extent / 2) / this.cell, 0, max);
    const fr = clamp((z + this.extent / 2) / this.cell, 0, max);
    const c = Math.min(Math.floor(fc), max - 1);
    const r = Math.min(Math.floor(fr), max - 1);
    const tc = fc - c;
    const tr = fr - r;
    const h = this.heights;
    const i = r * this.size + c;
    const top = h[i] + (h[i + 1] - h[i]) * tc;
    const bottom = h[i + this.size] + (h[i + this.size + 1] - h[i + this.size]) * tc;
    return top + (bottom - top) * tr;
  }

  dispose(): void {
    this.mesh.removeFromParent();
    this.mesh.geometry.dispose();
    this.mesh.material.dispose();
    this.water?.geometry.dispose();
    this.water?.material.dispose();
  }
}

/** Per-vertex colours for `preset`'s palette, from height and steepness. */
function paint(preset: PresetInfo, heights: Float32Array, normal: THREE.BufferAttribute): Float32Array {
  const pal = preset.palette;
  const low = new THREE.Color(pal.low);
  const mid = new THREE.Color(pal.mid);
  const rock = new THREE.Color(pal.rock);
  const snow = new THREE.Color(pal.snow);
  const sand = new THREE.Color(pal.sand ?? pal.mid);
  // The seabed fades toward a darker shade of the sea itself.
  const deep = new THREE.Color(pal.water ?? 0x1f5a7a).multiplyScalar(0.45);
  const rockLine = preset.rockLine ?? ROCK_LINE;
  const snowLine = preset.snowLine ?? SNOW_LINE;
  const sea = preset.waterLevel;

  const colors = new Float32Array(normal.count * 3);
  const color = new THREE.Color();
  for (let i = 0; i < normal.count; i++) {
    const h = heights[i];
    const steep = 1 - normal.getY(i);
    if (sea !== undefined && h <= sea) {
      color.lerpColors(sand, deep, Math.min((sea - h) / SEABED_FADE, 1));
    } else if (sea !== undefined && h < sea + SHORE_BAND && steep < CLIFF) {
      color.copy(sand);
    } else if (h > snowLine && steep < CLIFF) {
      color.copy(snow);
    } else if (preset.strata) {
      paintStrata(color, h, steep, sand, low, mid, rock);
    } else if (h > rockLine || steep > CLIFF) {
      color.copy(rock);
    } else {
      color.lerpColors(low, mid, h / rockLine);
    }
    color.toArray(colors, i * 3);
  }
  return colors;
}

/**
 * Desert colouring: sand on the gorge floor, cliffs banded in alternating
 * sandstone layers by height, the plateau top a sun-baked mix.
 */
function paintStrata(
  out: THREE.Color,
  h: number,
  steep: number,
  sand: THREE.Color,
  low: THREE.Color,
  mid: THREE.Color,
  rock: THREE.Color,
): void {
  if (h < FLOOR_LINE && steep < CLIFF) {
    out.lerpColors(sand, low, h / FLOOR_LINE);
  } else if (steep > CLIFF * 0.6) {
    const layer = Math.floor(h / STRATUM);
    out.lerpColors(rock, mid, layer % 2 === 0 ? 0.15 : 0.7);
  } else {
    // Benches and plateau: lighter the higher they are.
    out.lerpColors(low, mid, clamp((h - FLOOR_LINE) / 140, 0, 1));
  }
}

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}
