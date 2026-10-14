// Brain-link look by measured quality (render/link-style.ts). Pure, no three.js.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Q_BAD, Q_HEALTHY, Q_WARN, linkStyle } from '../src/render/link-style.ts';

const out = new Float32Array(4);
const style = (q: number, route = false, stale = false, flicker = 1) => {
  const degraded = linkStyle(q, route, stale, flicker, out);
  return { degraded, r: out[0], g: out[1], b: out[2], a: out[3] };
};

test('a lossless neighbour link looks as before quality existed', () => {
  const s = style(1);
  assert.equal(s.degraded, false);
  assert.ok(Math.abs(s.a - 0.75) < 1e-6);
  // #2c5f80 in linear RGB: blue dominant.
  assert.ok(s.b > s.g && s.g > s.r);
});

test('colour runs healthy → amber (0.8) → red (0.5); below 0.5 it is degraded', () => {
  assert.deepEqual([Q_HEALTHY, Q_WARN, Q_BAD], [0.95, 0.8, 0.5]);
  const amber = style(Q_WARN);
  assert.equal(amber.degraded, false);
  assert.ok(amber.r > 0.9 && amber.b < 0.05, 'amber at 0.8');
  const red = style(Q_BAD + 1e-4);
  assert.equal(red.degraded, false);
  assert.ok(red.r > 0.9 && red.g < 0.1, 'red by 0.5');
  assert.equal(style(Q_BAD - 1e-4).degraded, true);
  assert.equal(style(0).degraded, true);
});

test('opacity falls with quality and the parent route stays more visible', () => {
  let prev = Infinity;
  for (let q = 1; q >= 0; q -= 0.1) {
    const a = style(q, false, false, 1).a;
    assert.ok(a <= prev + 1e-6);
    prev = a;
    assert.ok(style(q, true, false, 1).a > a);
  }
});

test('degraded links flicker; healthy ones do not', () => {
  assert.ok(style(0.3, false, false, 0).a < style(0.3, false, false, 1).a);
  assert.equal(style(0.9, false, false, 0).a, style(0.9, false, false, 1).a);
});

test('a link to a downed drone is degraded red and never fades out while listed', () => {
  const s = style(0, false, true);
  assert.equal(s.degraded, true);
  assert.ok(s.r > 0.9);
  assert.ok(s.a >= 0.5 - 1e-6);
  // Legacy snapshots report quality 1: a stale link is still flagged.
  assert.equal(style(1, false, true).degraded, true);
});
