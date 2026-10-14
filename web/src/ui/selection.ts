// Highlight for the drone the console follows: a camera-facing bracket ring
// plus a thin plumb line to the ground, drawn over everything so it stays
// visible behind ridges. Two draw calls, no per-frame allocation.

import * as THREE from 'three';

const COLOR = 0x9dffb0;

export class SelectionMarker {
  readonly object = new THREE.Group();
  private readonly ring: THREE.Mesh<THREE.BufferGeometry, THREE.MeshBasicMaterial>;
  private readonly plumb: THREE.Line<THREE.BufferGeometry, THREE.LineBasicMaterial>;
  private readonly plumbPos: Float32Array;
  private id = -1;

  constructor() {
    // Four arcs read as a targeting bracket rather than a solid halo.
    const ring = new THREE.RingGeometry(16, 19, 48, 1);
    const arcs = new THREE.BufferGeometry();
    const index: number[] = [];
    const src = ring.getIndex();
    if (src) {
      for (let i = 0; i < src.count; i += 3) {
        // Keep triangles whose first vertex falls in the four 50-degree arcs.
        const v = src.getX(i) % 49;
        const deg = (v / 48) * 360;
        if (deg % 90 < 50) index.push(src.getX(i), src.getX(i + 1), src.getX(i + 2));
      }
    }
    arcs.setAttribute('position', ring.getAttribute('position'));
    arcs.setIndex(index);
    ring.dispose();
    this.ring = new THREE.Mesh(
      arcs,
      new THREE.MeshBasicMaterial({ color: COLOR, transparent: true, opacity: 0.95, depthTest: false, side: THREE.DoubleSide }),
    );
    this.ring.renderOrder = 6;

    this.plumbPos = new Float32Array(6);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(this.plumbPos, 3).setUsage(THREE.DynamicDrawUsage));
    this.plumb = new THREE.Line(g, new THREE.LineBasicMaterial({ color: COLOR, transparent: true, opacity: 0.45, depthTest: false }));
    this.plumb.frustumCulled = false;
    this.plumb.renderOrder = 6;
    this.object.add(this.ring, this.plumb);
    this.object.visible = false;
  }

  /** Drone to mark, or -1 to hide. */
  set(id: number): void {
    this.id = id;
    this.object.visible = id >= 0;
  }

  get selected(): number {
    return this.id;
  }

  update(positions: Float32Array, count: number, camera: THREE.Camera, groundAt: (x: number, z: number) => number, timeS: number): void {
    const id = this.id;
    if (id < 0 || id >= count) {
      this.object.visible = false;
      return;
    }
    this.object.visible = true;
    const x = positions[id * 3];
    const y = positions[id * 3 + 1];
    const z = positions[id * 3 + 2];
    this.ring.position.set(x, y, z);
    this.ring.quaternion.copy(camera.quaternion);
    this.ring.rotateZ(timeS * 0.8);
    // Constant on-screen size: scale with distance to the camera.
    const d = camera.position.distanceTo(this.ring.position);
    this.ring.scale.setScalar(Math.max(0.6, d / 500) * (1 + 0.06 * Math.sin(timeS * 5)));
    const p = this.plumbPos;
    p[0] = x;
    p[1] = y;
    p[2] = z;
    p[3] = x;
    p[4] = groundAt(x, z);
    p[5] = z;
    this.plumb.geometry.getAttribute('position').needsUpdate = true;
  }
}
