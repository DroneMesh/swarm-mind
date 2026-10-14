// Snapshot v3 / channel summary layouts (contract with crates/swarm-sim) and the
// legacy fallbacks the page keeps until the WASM is rebuilt. Pure, no DOM.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { HEADER_LEN, DRONE_STRIDE, L, LEGACY_VERSION, LINK_STRIDE, PULSE_STRIDE, Snapshot, VERSION, newLinkHealth } from '../src/snapshot.ts';
import {
  CH,
  CHANNEL_LEN,
  LEGACY_CHANNEL_LEN,
  TLM,
  TLM_STRIDE,
  airBytes,
  airHeader,
  airMixText,
  fmtK,
  hasAirBytes,
  linkHealthText,
  newAirBytes,
} from '../src/telemetry.ts';

/** A snapshot with `drones` drones (all alive), the given links and one pulse. */
function snapshot(version: number, drones: number, links: number[][]): Float32Array {
  const stride = version === VERSION ? LINK_STRIDE : 3;
  const d = new Float32Array(HEADER_LEN + drones * DRONE_STRIDE + links.length * stride + PULSE_STRIDE);
  d[0] = version;
  d[3] = drones;
  d[4] = links.length;
  d[5] = 1;
  for (let i = 0; i < drones; i++) {
    d[HEADER_LEN + i * DRONE_STRIDE] = i;
    d[HEADER_LEN + i * DRONE_STRIDE + 4] = 1;
  }
  links.forEach((l, k) => d.set(l.slice(0, stride), HEADER_LEN + drones * DRONE_STRIDE + k * stride));
  d.set([2, 255, 0], HEADER_LEN + drones * DRONE_STRIDE + links.length * stride); // pulse
  return d;
}

test('snapshot v3: links are [a, b, kind, quality] and pulses follow them', () => {
  assert.equal(VERSION, 3);
  assert.equal(LINK_STRIDE, 4);
  assert.deepEqual(L, { a: 0, b: 1, kind: 2, quality: 3 });
  const s = new Snapshot(snapshot(3, 3, [[0, 1, 1, 1], [1, 2, 0, 0.42], [0, 2, 0, 0.7]]));
  assert.equal(s.hasQuality, true);
  assert.equal(s.data[s.link(1) + L.b], 2);
  assert.ok(Math.abs(s.linkQuality(1) - 0.42) < 1e-6);
  assert.equal(s.data[s.pulse(0)], 2);
  const h = s.linkHealth(newLinkHealth());
  assert.equal(h.links, 3);
  assert.equal(h.degraded, 1);
  assert.ok(Math.abs(h.mean - (1 + 0.42 + 0.7) / 3) < 1e-6);
  assert.equal(h.measured, true);
});

test('snapshot quality is clamped to 0..1 (NaN reads 0)', () => {
  const s = new Snapshot(snapshot(3, 2, [[0, 1, 0, 1.3], [0, 1, 0, -0.2], [0, 1, 0, NaN]]));
  assert.equal(s.linkQuality(0), 1);
  assert.equal(s.linkQuality(1), 0);
  assert.equal(s.linkQuality(2), 0);
});

test('a legacy v2 snapshot still reads: stride 3, quality 1, no measured health', () => {
  const s = new Snapshot(snapshot(LEGACY_VERSION, 3, [[0, 1, 1], [1, 2, 0]]));
  assert.equal(s.hasQuality, false);
  assert.equal(s.data[s.link(1) + L.a], 1);
  assert.equal(s.data[s.link(1) + L.b], 2);
  assert.equal(s.linkQuality(1), 1);
  assert.equal(s.data[s.pulse(0)], 2);
  const h = s.linkHealth(newLinkHealth());
  assert.equal(h.measured, false);
  assert.equal(h.degraded, 0);
});

test('an unknown snapshot version is rejected', () => {
  assert.throws(() => new Snapshot(snapshot(7, 1, [])), /snapshot version 7/);
});

test('channel summary v3 offsets mirror Rust telemetry::chan', () => {
  assert.equal(CHANNEL_LEN, 28);
  assert.equal(LEGACY_CHANNEL_LEN, 16);
  assert.equal(CH.online, 15);
  assert.deepEqual(
    [CH.bytesOnAir, CH.bytesBrain, CH.bytesTlmData, CH.bytesAck, CH.bytesMeshControl, CH.bytesUplink, CH.bytesBeacon, CH.bytesRetry, CH.framesOnAir, CH.linkQualityMean, CH.linksDegraded, CH.bytesMeshControl60s],
    [16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27],
  );
  assert.equal(TLM_STRIDE, 20);
  assert.equal(TLM.txAllBytesPerS, 17);
  assert.equal(TLM.ackBytesPerS, 18);
});

test('a legacy 16-float channel summary has no byte counters', () => {
  const out = newAirBytes();
  assert.equal(hasAirBytes(new Float32Array(LEGACY_CHANNEL_LEN)), false);
  assert.equal(hasAirBytes(null), false);
  assert.equal(airBytes(new Float32Array(LEGACY_CHANNEL_LEN), out), false);
  assert.equal(out.total, 0);
  // The console then counts brain frames only, and says so.
  const h = airHeader(null, 41, 1234, true);
  assert.equal(h.text, 'brain 41 fr/s · 1.2 kB/s');
  assert.equal(h.mix, '');
  assert.match(h.title, /npm run wasm/);
});

test('the console header counts all traffic on air, with a per-class breakdown', () => {
  const ch = new Float32Array(CHANNEL_LEN);
  ch[CH.bytesOnAir] = 74_400;
  ch[CH.bytesBrain] = 800;
  ch[CH.bytesTlmData] = 52_000;
  ch[CH.bytesAck] = 9000;
  ch[CH.bytesMeshControl] = 0; // 2 s window: usually empty between 60 s syncs
  ch[CH.bytesMeshControl60s] = 300;
  ch[CH.bytesUplink] = 6000;
  ch[CH.bytesBeacon] = 4000;
  ch[CH.bytesRetry] = 3000;
  ch[CH.framesOnAir] = 830;
  ch[CH.linkQualityMean] = 0.93;
  ch[CH.linksDegraded] = 2;
  const a = newAirBytes();
  assert.equal(airBytes(ch, a), true);
  assert.equal(airMixText(a), 'brain 0.8k · tlm 52k · ack 9.0k · sync 0.3k · uplink 6.0k · beacon 4.0k · retry 3.0k');
  const h = airHeader(a, 41, 800);
  assert.equal(h.text, 'air 830 fr/s · 74.4 kB/s');
  assert.equal(h.mix, airMixText(a));
  assert.match(h.title, /retries included/);
  assert.equal(linkHealthText(a.linkQualityMean, a.linksDegraded), 'q 0.93 · 2 degraded');
  assert.equal(linkHealthText(1, 0), 'q 1.00 · none degraded');
});

test('fmtK keeps small rates exact and large ones short', () => {
  assert.equal(fmtK(0), '0');
  assert.equal(fmtK(NaN), '0');
  assert.equal(fmtK(42), '42');
  assert.equal(fmtK(800), '0.8k');
  assert.equal(fmtK(52_400), '52k');
});
