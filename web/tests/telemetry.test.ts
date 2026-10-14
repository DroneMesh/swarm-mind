// Regression tests for the panel-side telemetry summaries (pure, no DOM).
// Run: npm test (node --test with native type stripping).

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CHANNEL_LEN, TLM, TLM_STRIDE, fleetLossText, fleetLossTone, fleetTlm, lossAgreesWithRate, packageSeq } from '../src/telemetry.ts';

/** Stats for drones 0..n-1 (0 = mesh root): [requestedHz, deliveredHz, lossPct]. */
function stats(rows: [number, number, number][]): Float32Array {
  const d = new Float32Array(rows.length * TLM_STRIDE);
  rows.forEach(([req, gcs, loss], i) => {
    d[i * TLM_STRIDE + TLM.id] = i;
    d[i * TLM_STRIDE + TLM.requestedHz] = req;
    d[i * TLM_STRIDE + TLM.deliveredHz] = gcs;
    d[i * TLM_STRIDE + TLM.lossPct] = loss;
  });
  return d;
}
const alive = () => true;
const channel = new Float32Array(CHANNEL_LEN);

test('silent senders are not a 0 % loss: no bridge reads red, not green', () => {
  // Root dead or every drone orphaned: packages never arrive, so rx = gaps = 0 and the
  // per-drone loss is the sim's 0.0 placeholder.
  const f = fleetTlm(stats([[0, 0, 0], [25, 0, 0], [25, 0, 0], [25, 0, 0]]), channel, alive);
  assert.equal(f.senders, 3);
  assert.equal(f.heard, 0);
  assert.equal(f.silent, 3);
  assert.equal(fleetLossTone(f), 'r');
  assert.match(fleetLossText(f), /none reach the GCS/);
  assert.doesNotMatch(fleetLossText(f), /loss 0%/);
});

test('fleet loss median skips unheard senders, as the sim and backend do', () => {
  // 2 heard at 20 % / 30 %, 3 silent (would pull a naive median to 0).
  const f = fleetTlm(stats([[0, 0, 0], [25, 20, 20], [25, 17, 30], [25, 0, 0], [25, 0, 0], [25, 0, 0]]), channel, alive);
  assert.equal(f.heard, 2);
  assert.equal(f.silent, 3);
  assert.equal(f.lossPct, 25);
  assert.equal(fleetLossTone(f), 'r');
  assert.match(fleetLossText(f), /loss 25% \(med 10 s\) · 3 not heard/);
});

test('a clean fleet with one unheard drone is amber, not green', () => {
  const f = fleetTlm(stats([[0, 0, 0], [10, 10, 0], [10, 10, 1], [10, 10, 0], [10, 0, 0]]), channel, alive);
  assert.equal(f.lossPct, 0);
  assert.equal(fleetLossTone(f), 'a');
});

test('offered pps is senders × requested Hz (the field calibration unit), not frames on air', () => {
  const ch = new Float32Array(CHANNEL_LEN);
  ch[0] = 1500; // ppsOnAir: every hop, ACK, sync and brain frame
  const f = fleetTlm(stats([[0, 0, 0], [25, 25, 0], [25, 25, 0], [25, 25, 0], [25, 25, 0], [25, 25, 0], [25, 25, 0]]), ch, alive);
  assert.equal(f.offeredPps, 150);
  assert.equal(f.ppsOnAir, 1500);
});

test('packageSeq reads "s" from a mesh package (escaped) and from bare JSON', () => {
  const inner = '{"type":"drone","n":3,"s":4711,"t":120,"ft":0,"ekf":0,"f":0,"sa":0}';
  const pkg = `{"type":9,"dest":0,"from":3,"msg":"${inner.replace(/"/g, '\\"')}"}`;
  assert.equal(packageSeq(pkg), 4711);
  assert.equal(packageSeq(inner), 4711);
  // "sa" / "gsa" must not be taken for "s".
  assert.equal(packageSeq('{"sa":9,"gsa":7}'), -1);
});

test('loss agrees with the delivered shortfall only once both windows see the same mode', () => {
  // Steady 5 Hz, all delivered, 0 % loss: agrees.
  assert.equal(lossAgreesWithRate(stats([[0, 0, 0], [5, 5, 0], [5, 4.9, 1], [5, 5, 0]]), alive, 12), true);
  // Just after leaving a saturated 25 Hz: 2 s rate is clean, the 10 s loss still holds the old gaps.
  assert.equal(lossAgreesWithRate(stats([[0, 0, 0], [5, 5, 43], [5, 5, 40], [5, 5, 45]]), alive, 12), false);
  // Steady collapse: 11 of 25 Hz delivered and ~56 % loss tell the same story.
  assert.equal(lossAgreesWithRate(stats([[0, 0, 0], [25, 11, 55], [25, 12, 50], [25, 10, 62]]), alive, 12), true);
  // Silent senders have no loss to compare and are skipped.
  assert.equal(lossAgreesWithRate(stats([[0, 0, 0], [25, 0, 0], [25, 0, 0]]), alive, 12), true);
  assert.equal(lossAgreesWithRate(null, alive, 12), false);
});
