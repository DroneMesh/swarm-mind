// How a brain link looks for its measured quality (snapshot v3 `quality`, the
// delivery ratio of brain frames over ~4 s, worse direction). Pure and
// allocation-free so the renderer can call it per link per frame, and tests can
// check the mapping without three.js. The legend in index.html mirrors these.

/** At or above this a link draws in its healthy colour. */
export const Q_HEALTHY = 0.95;
/** Fully amber here; the ramp from healthy runs Q_HEALTHY → Q_WARN. */
export const Q_WARN = 0.8;
/** Fully red, and dotted + flickering below (swarm-sim LINKS_DEGRADED threshold). */
export const Q_BAD = 0.5;

/** sRGB hex → linear RGB, as THREE.Color(hex) does with colour management on. */
function linear(hex: number): [number, number, number] {
  const f = (c: number) => (c < 0.04045 ? c * 0.0773993808 : Math.pow(c * 0.9478672986 + 0.0521327014, 2.4));
  return [f(((hex >> 16) & 255) / 255), f(((hex >> 8) & 255) / 255), f((hex & 255) / 255)];
}

function mix(a: readonly number[], b: readonly number[], t: number): [number, number, number] {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

/** Healthy colours, unchanged from before quality existed. */
export const LINK_HEX = { route: 0x7fe7ff, neighbor: 0x2c5f80, warn: 0xffb020, bad: 0xff4d4d } as const;

const ROUTE = linear(LINK_HEX.route);
const NEIGHBOR = linear(LINK_HEX.neighbor);
const WARN = linear(LINK_HEX.warn);
const BAD = linear(LINK_HEX.bad);
const WHITE = [1, 1, 1] as const;
/** The parent route keeps a lighter tint of the warning colours, so it stays distinct. */
const ROUTE_WARN = mix(WARN, WHITE, 0.3);
const ROUTE_BAD = mix(BAD, WHITE, 0.25);

/** Alpha at quality 1 / 0. Neighbour links at 1 match the old 0.75 opacity. */
const NEIGHBOR_ALPHA = [0.75, 0.18] as const;
const ROUTE_ALPHA = [0.95, 0.45] as const;
/** A link to a downed drone (still in the neighbour table) never fades below this. */
const STALE_MIN_ALPHA = 0.5;

/**
 * Style of one link into `out` = [r, g, b, a] (linear RGB). Returns whether it is
 * degraded: drawn dotted and flickering (quality < Q_BAD, or an end is down).
 * `flicker` is 0..1 (a per-link oscillator); it only dims degraded links.
 */
export function linkStyle(q: number, route: boolean, stale: boolean, flicker: number, out: Float32Array | number[]): boolean {
  if (!(q >= 0)) q = 0;
  else if (q > 1) q = 1;
  const healthy = route ? ROUTE : NEIGHBOR;
  const warn = route ? ROUTE_WARN : WARN;
  const bad = route ? ROUTE_BAD : BAD;
  let from: readonly number[];
  let to: readonly number[];
  let t: number;
  if (stale || q < Q_BAD) {
    from = bad;
    to = bad;
    t = 0;
  } else if (q < Q_WARN) {
    from = warn;
    to = bad;
    t = (Q_WARN - q) / (Q_WARN - Q_BAD);
  } else if (q < Q_HEALTHY) {
    from = healthy;
    to = warn;
    t = (Q_HEALTHY - q) / (Q_HEALTHY - Q_WARN);
  } else {
    from = healthy;
    to = healthy;
    t = 0;
  }
  out[0] = from[0] + (to[0] - from[0]) * t;
  out[1] = from[1] + (to[1] - from[1]) * t;
  out[2] = from[2] + (to[2] - from[2]) * t;
  const range = route ? ROUTE_ALPHA : NEIGHBOR_ALPHA;
  let a = range[1] + (range[0] - range[1]) * q;
  const degraded = stale || q < Q_BAD;
  if (stale && a < STALE_MIN_ALPHA) a = STALE_MIN_ALPHA;
  if (degraded) a *= 0.55 + 0.45 * flicker;
  out[3] = a;
  return degraded;
}
