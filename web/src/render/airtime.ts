import * as THREE from 'three';
import { TLM, tlmCount, tlmField } from '../telemetry';

/** Height sampler for the ground under world `(x, z)`. */
type GroundFn = (x: number, z: number) => number;

const TREE_COLOR = new THREE.Color(0xff8c1a);
/** A tree link to a parent that is already down: painlessMesh has not noticed yet (NODE_TIMEOUT). */
const TREE_STALE = new THREE.Color(0xff3b3b);
/** Busy heat ramp: clear, contended, saturated. */
const HEAT_LOW = new THREE.Color(0x2fd27a);
const HEAT_MID = new THREE.Color(0xffb52e);
const HEAT_HIGH = new THREE.Color(0xff3030);
const HEAT_MID_AT = 0.35;
const HEAT_HIGH_AT = 0.7;
/** Disc radius (world units) at zero busy, and its growth at full busy. */
const DISC_R0 = 16;
const DISC_R1 = 26;
/** Tree lines sit a little above the brain's links so the two stay apart. */
const TREE_LIFT = 2.5;
const DISC_LIFT = 2;

/**
 * Map overlays for the telemetry / channel models: the painlessMesh tree as
 * dashed orange segments (drone to its mesh parent) and a ground disc per
 * drone tinted by the channel busy fraction heard there. Everything is
 * preallocated; per frame it only rewrites a few hundred floats.
 */
export class AirtimeView {
  readonly object = new THREE.Group();
  showTree = true;
  showHeat = true;

  private data: Float32Array | null = null;
  private channelOn = false;
  private telemetryOn = false;
  private readonly treeGeometry = new THREE.BufferGeometry();
  private readonly treePos: Float32Array;
  private readonly treeCol: Float32Array;
  private readonly treeDist: Float32Array;
  private readonly tree: THREE.LineSegments;
  private readonly discs: THREE.InstancedMesh;
  private readonly m = new THREE.Matrix4();
  private readonly q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
  private readonly v = new THREE.Vector3();
  private readonly sc = new THREE.Vector3();
  private readonly c = new THREE.Color();

  constructor(private readonly maxDrones: number) {
    this.treePos = new Float32Array(maxDrones * 6);
    this.treeCol = new Float32Array(maxDrones * 6);
    this.treeDist = new Float32Array(maxDrones * 2);
    this.treeGeometry.setAttribute('position', new THREE.BufferAttribute(this.treePos, 3).setUsage(THREE.DynamicDrawUsage));
    this.treeGeometry.setAttribute('color', new THREE.BufferAttribute(this.treeCol, 3).setUsage(THREE.DynamicDrawUsage));
    this.treeGeometry.setAttribute('lineDistance', new THREE.BufferAttribute(this.treeDist, 1).setUsage(THREE.DynamicDrawUsage));
    this.treeGeometry.setDrawRange(0, 0);
    this.tree = new THREE.LineSegments(
      this.treeGeometry,
      new THREE.LineDashedMaterial({ vertexColors: true, dashSize: 7, gapSize: 5, transparent: true, opacity: 0.95, depthWrite: false }),
    );
    this.tree.frustumCulled = false;
    this.tree.renderOrder = 2;

    this.discs = new THREE.InstancedMesh(
      new THREE.CircleGeometry(1, 40),
      new THREE.MeshBasicMaterial({
        transparent: true,
        opacity: 0.2,
        depthWrite: false,
        toneMapped: false,
        polygonOffset: true,
        polygonOffsetFactor: -2,
        polygonOffsetUnits: -2,
      }),
      maxDrones,
    );
    this.discs.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    for (let i = 0; i < maxDrones; i++) {
      this.discs.setMatrixAt(i, this.m.makeScale(0, 0, 0));
      this.discs.setColorAt(i, HEAT_LOW);
    }
    this.discs.frustumCulled = false;
    this.discs.count = 0;
    this.object.add(this.discs, this.tree);
  }

  /** Latest 'tlmstats'; `channelOn` / `telemetryOn` from its fleet summary. */
  setStats(data: Float32Array | null, channelOn: boolean, telemetryOn: boolean): void {
    this.data = data;
    this.channelOn = channelOn;
    this.telemetryOn = telemetryOn;
  }

  reset(): void {
    this.data = null;
    this.treeGeometry.setDrawRange(0, 0);
    this.discs.count = 0;
  }

  /** Redraws at the frame's interpolated `positions` (xyz per drone id). */
  update(positions: Float32Array, count: number, alive: (id: number) => boolean, groundAt: GroundFn): void {
    const d = this.data;
    const n = d ? Math.min(tlmCount(d), count, this.maxDrones) : 0;
    this.updateTree(d, n, positions, alive);
    this.updateHeat(d, n, positions, alive, groundAt);
  }

  private updateTree(d: Float32Array | null, n: number, pos: Float32Array, alive: (id: number) => boolean): void {
    let segs = 0;
    if (d && this.showTree && this.telemetryOn) {
      const p = this.treePos;
      for (let i = 0; i < n; i++) {
        const parent = tlmField(d, i, TLM.treeParent);
        if (parent < 0 || parent >= n || !alive(i)) continue;
        const a = i * 3;
        const b = parent * 3;
        const o = segs * 6;
        p[o] = pos[a];
        p[o + 1] = pos[a + 1] + TREE_LIFT;
        p[o + 2] = pos[a + 2];
        p[o + 3] = pos[b];
        p[o + 4] = pos[b + 1] + TREE_LIFT;
        p[o + 5] = pos[b + 2];
        this.treeDist[segs * 2] = 0;
        this.treeDist[segs * 2 + 1] = Math.hypot(p[o + 3] - p[o], p[o + 4] - p[o + 1], p[o + 5] - p[o + 2]);
        const color = alive(parent) ? TREE_COLOR : TREE_STALE;
        color.toArray(this.treeCol, o);
        color.toArray(this.treeCol, o + 3);
        segs++;
      }
    }
    this.tree.visible = segs > 0;
    this.treeGeometry.setDrawRange(0, segs * 2);
    if (segs > 0) {
      this.treeGeometry.getAttribute('position').needsUpdate = true;
      this.treeGeometry.getAttribute('color').needsUpdate = true;
      this.treeGeometry.getAttribute('lineDistance').needsUpdate = true;
    }
  }

  private updateHeat(d: Float32Array | null, n: number, pos: Float32Array, alive: (id: number) => boolean, groundAt: GroundFn): void {
    const on = d !== null && this.showHeat && this.channelOn;
    this.discs.visible = on;
    if (!on || !d) return;
    for (let i = 0; i < n; i++) {
      const busy = Math.min(1, Math.max(0, tlmField(d, i, TLM.busyFrac)));
      const x = pos[i * 3];
      const z = pos[i * 3 + 2];
      if (!alive(i)) {
        this.m.makeScale(0, 0, 0);
      } else {
        const r = DISC_R0 + DISC_R1 * busy;
        // Highest ground under the rim, so a slope never buries half the disc.
        const y = Math.max(groundAt(x, z), groundAt(x + r, z), groundAt(x - r, z), groundAt(x, z + r), groundAt(x, z - r));
        this.m.compose(this.v.set(x, y + DISC_LIFT, z), this.q, this.sc.set(r, r, r));
      }
      this.discs.setMatrixAt(i, this.m);
      heat(busy, this.c);
      this.discs.setColorAt(i, this.c);
    }
    this.discs.count = n;
    this.discs.instanceMatrix.needsUpdate = true;
    if (this.discs.instanceColor) this.discs.instanceColor.needsUpdate = true;
  }
}

/** Busy fraction to the heat ramp (also used for the legend swatch). */
function heat(busy: number, out: THREE.Color): THREE.Color {
  if (busy <= HEAT_MID_AT) return out.copy(HEAT_LOW).lerp(HEAT_MID, busy / HEAT_MID_AT);
  return out.copy(HEAT_MID).lerp(HEAT_HIGH, Math.min(1, (busy - HEAT_MID_AT) / (HEAT_HIGH_AT - HEAT_MID_AT)));
}
