import * as THREE from 'three';

const HEIGHT = 140;
const RADIUS = 22;

/** A ground ring and beam marking the operator's custom target. */
export class TargetMarker {
  readonly object = new THREE.Group();
  private readonly ring: THREE.Mesh;

  constructor() {
    const material = new THREE.MeshBasicMaterial({
      color: 0x7fe7ff,
      transparent: true,
      opacity: 0.85,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    this.ring = new THREE.Mesh(new THREE.RingGeometry(RADIUS * 0.8, RADIUS, 40).rotateX(-Math.PI / 2), material);
    const beamGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, HEIGHT, 0)]);
    const beam = new THREE.Line(beamGeometry, new THREE.LineBasicMaterial({ color: 0x7fe7ff, transparent: true, opacity: 0.6 }));
    this.object.add(this.ring, beam);
    this.object.visible = false;
  }

  show(x: number, y: number, z: number): void {
    this.object.position.set(x, y + 1.5, z);
    this.object.visible = true;
  }

  hide(): void {
    this.object.visible = false;
  }

  update(timeS: number): void {
    if (this.object.visible) this.ring.scale.setScalar(1 + 0.15 * Math.sin(timeS * 4));
  }
}
