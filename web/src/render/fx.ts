// Post-processing. 'high' renders through a bloom composer so emissive
// lights, rotor tips and fire glow; 'low' renders straight to the screen
// and also tells the swarm view to skip its searchlights.
//
// Auto-downgrade: while quality is 'high', frame times are averaged over
// 3 s windows after a 3 s warm-up (shader compiles and the first terrain
// upload stutter); two consecutive windows under 45 fps switch to 'low' for
// the rest of the session (it never switches back by itself, which would
// oscillate).
// Windows containing a long stall (tab hidden, debugger) are discarded.
// Call `setQuality('high')` to opt back in; that also disarms the monitor.

import * as THREE from 'three';
import { EffectComposer } from 'three/examples/jsm/postprocessing/EffectComposer.js';
import { OutputPass } from 'three/examples/jsm/postprocessing/OutputPass.js';
import { RenderPass } from 'three/examples/jsm/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/examples/jsm/postprocessing/UnrealBloomPass.js';

export type Quality = 'high' | 'low';

export interface Fx {
  render(): void;
  resize(w: number, h: number): void;
  setQuality(q: Quality): void;
  readonly quality: Quality;
}

const WINDOW_MS = 3000;
const WARMUP_MS = 3000;
/** Below the spec's 50 on purpose: a 50 Hz display (or a throttled shared browser) sits at exactly 50. */
const MIN_FPS = 45;
/** A gap longer than this means the page was not being animated. */
const STALL_MS = 250;
/** Only pixels brighter than this (linear) bloom: unlit lights, not sunlit terrain. */
const BLOOM_THRESHOLD = 0.82;
const BLOOM_STRENGTH = 0.85;
const BLOOM_RADIUS = 0.45;

let current: Quality = 'high';

/** The quality of the most recently created Fx; render code polls it to drop costly extras. */
export function renderQuality(): Quality {
  return current;
}

export function createFx(renderer: THREE.WebGLRenderer, scene: THREE.Scene, camera: THREE.Camera): Fx {
  let quality: Quality = 'high';
  let composer: EffectComposer | null = null;
  let bloom: UnrealBloomPass | null = null;
  const size = new THREE.Vector2();

  // Built lazily: a session that starts (or drops) to 'low' never pays for the targets.
  const ensureComposer = (): EffectComposer => {
    if (composer) return composer;
    renderer.getSize(size);
    // MSAA on the scene target, so 'high' is not more jagged than 'low'; half float keeps HDR lights.
    const target = new THREE.WebGLRenderTarget(size.x, size.y, { type: THREE.HalfFloatType, samples: 4 });
    composer = new EffectComposer(renderer, target);
    composer.setPixelRatio(renderer.getPixelRatio());
    composer.setSize(size.x, size.y);
    composer.addPass(new RenderPass(scene, camera));
    bloom = new UnrealBloomPass(new THREE.Vector2(size.x, size.y), BLOOM_STRENGTH, BLOOM_RADIUS, BLOOM_THRESHOLD);
    composer.addPass(bloom);
    composer.addPass(new OutputPass());
    return composer;
  };

  const warmUntil = performance.now() + WARMUP_MS;
  let windowStart = -1;
  let windowFrames = 0;
  let slowWindows = 0;
  let last = -1;
  let monitoring = true;

  const watchFps = (now: number) => {
    if (!monitoring || quality !== 'high' || now < warmUntil) return;
    if (last >= 0 && now - last > STALL_MS) windowStart = -1;
    last = now;
    if (windowStart < 0) {
      windowStart = now;
      windowFrames = 0;
      return;
    }
    windowFrames++;
    const elapsed = now - windowStart;
    if (elapsed < WINDOW_MS) return;
    const fps = (windowFrames * 1000) / elapsed;
    windowStart = -1;
    slowWindows = fps < MIN_FPS ? slowWindows + 1 : 0;
    if (slowWindows >= 2) {
      console.info(`[fx] ${fps.toFixed(0)} fps over ${WINDOW_MS / 1000} s: switching to low quality`);
      fx.setQuality('low');
      monitoring = false;
    }
  };

  const fx: Fx = {
    render: () => {
      watchFps(performance.now());
      if (quality === 'high') ensureComposer().render();
      else renderer.render(scene, camera);
    },
    resize: (w, h) => {
      renderer.setSize(w, h);
      if (composer) {
        composer.setPixelRatio(renderer.getPixelRatio());
        composer.setSize(w, h);
      }
    },
    setQuality: (q) => {
      if (q === 'high' && quality === 'low') monitoring = false; // an explicit opt-in sticks
      quality = q;
      current = q;
    },
    get quality() {
      return quality;
    },
  };
  // Console hook for debugging the post chain; not part of the Fx contract.
  Object.defineProperty(fx, 'composer', { get: () => composer });
  current = quality;
  return fx;
}
