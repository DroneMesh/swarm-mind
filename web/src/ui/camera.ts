// Cinematic camera: eased shots (overview, follow a drone, frame a point)
// layered over OrbitControls. A shot moves the orbit target and the camera
// together; the controls keep running, so damping and limits still apply.
// Any user input on the controls cancels whatever shot is playing.

import * as THREE from 'three';
import type { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export type ShotKind = 'idle' | 'move' | 'follow';

/** Ease-in-out cubic: gentle start and stop, like a camera operator. */
function ease(t: number): number {
  return t < 0.5 ? 4 * t * t * t : 1 - (-2 * t + 2) ** 3 / 2;
}

export class CameraDirector {
  /** What the director is doing; 'idle' means the user has the camera. */
  kind: ShotKind = 'idle';
  /** Drone being followed, or -1. */
  followId = -1;
  /** Called when a shot is cancelled by user input (not when it completes). */
  onCancel: (() => void) | null = null;

  private readonly fromPos = new THREE.Vector3();
  private readonly fromTarget = new THREE.Vector3();
  private readonly toPos = new THREE.Vector3();
  private readonly toTarget = new THREE.Vector3();
  /** Camera offset from the target kept while following. */
  private readonly followOffset = new THREE.Vector3();
  private readonly tmp = new THREE.Vector3();
  private startAt = 0;
  private durationMs = 0;
  /** While easing into a follow, the move ends on the drone's live position. */
  private moveThenFollow = false;
  /** Set while we write the controls ourselves, so their events are not mistaken for the user. */
  private driving = false;

  constructor(
    private readonly camera: THREE.PerspectiveCamera,
    private readonly controls: OrbitControls,
    /** Interpolated drone positions, xyz per id (SwarmView.positions). */
    private readonly positions: () => Float32Array,
  ) {
    controls.addEventListener('start', () => {
      if (!this.driving && this.kind !== 'idle') this.cancel(true);
    });
  }

  /** Eases to an absolute camera position and orbit target. */
  shot(pos: THREE.Vector3Like, target: THREE.Vector3Like, durationMs = 2200): void {
    this.begin(durationMs);
    this.toPos.copy(pos);
    this.toTarget.copy(target);
    this.followId = -1;
    this.moveThenFollow = false;
  }

  /**
   * Frames a ground point from `distance` away, keeping the current compass
   * bearing so the move reads as a push-in rather than a jump.
   */
  frame(x: number, y: number, z: number, distance = 420, elevation = 0.62, durationMs = 2200): void {
    this.tmp.copy(this.camera.position).sub(this.controls.target);
    this.tmp.y = 0;
    if (this.tmp.lengthSq() < 1) this.tmp.set(0, 0, 1);
    this.tmp.normalize();
    const flat = Math.cos(elevation) * distance;
    this.shot(
      { x: x + this.tmp.x * flat, y: y + Math.sin(elevation) * distance, z: z + this.tmp.z * flat },
      { x, y, z },
      durationMs,
    );
  }

  /** Eases in on drone `id` and then keeps it centred until cancelled. */
  follow(id: number, distance = 240, durationMs = 1600): void {
    const p = this.positions();
    if (id < 0 || id * 3 + 2 >= p.length) return;
    this.frame(p[id * 3], p[id * 3 + 1], p[id * 3 + 2], distance, 0.5, durationMs);
    this.followId = id;
    this.moveThenFollow = true;
  }

  /** Stops any shot; `byUser` reports it through `onCancel`. */
  cancel(byUser = false): void {
    const was = this.kind;
    this.kind = 'idle';
    this.followId = -1;
    this.moveThenFollow = false;
    if (byUser && was !== 'idle') this.onCancel?.();
  }

  /** Advances the shot; call once per frame before `controls.update()`. */
  update(now: number): void {
    if (this.kind === 'idle') return;
    this.driving = true;
    const p = this.positions();
    const id = this.followId;
    const hasDrone = id >= 0 && id * 3 + 2 < p.length;
    if (this.kind === 'move') {
      const t = Math.min((now - this.startAt) / this.durationMs, 1);
      const k = ease(t);
      if (this.moveThenFollow && hasDrone) {
        // Track the moving drone: the end of the move slides with it.
        this.tmp.set(p[id * 3], p[id * 3 + 1], p[id * 3 + 2]).sub(this.toTarget);
        this.toTarget.add(this.tmp);
        this.toPos.add(this.tmp);
      }
      this.controls.target.lerpVectors(this.fromTarget, this.toTarget, k);
      this.camera.position.lerpVectors(this.fromPos, this.toPos, k);
      if (t >= 1) {
        if (this.moveThenFollow && hasDrone) {
          this.kind = 'follow';
          this.followOffset.copy(this.camera.position).sub(this.controls.target);
        } else {
          this.kind = 'idle';
        }
      }
    } else if (this.kind === 'follow') {
      if (!hasDrone) {
        this.cancel();
      } else {
        // Exponential smoothing hides the 20 Hz snapshot cadence.
        this.tmp.set(p[id * 3], p[id * 3 + 1], p[id * 3 + 2]);
        this.controls.target.lerp(this.tmp, 0.12);
        this.camera.position.copy(this.controls.target).add(this.followOffset);
      }
    }
    this.driving = false;
  }

  private begin(durationMs: number): void {
    this.kind = 'move';
    this.startAt = performance.now();
    this.durationMs = Math.max(1, durationMs);
    this.fromPos.copy(this.camera.position);
    this.fromTarget.copy(this.controls.target);
  }
}
