//! Acceptance suite for the shared-airtime channel, the painlessMesh
//! telemetry model and Phase 5 (brain-managed telemetry airtime).
//!
//! Public API only. The thresholds below were fixed when the suite was
//! written; do not loosen them to make a test pass. Fix the model or the
//! brain instead.
//!
//! | test | what it checks |
//! |------|----------------|
//! | T1   | telemetry/channel off is bit-identical to before (S8 digest) |
//! | T2   | 1 drone at 25 Hz: delivered rate, payload bytes/s, slow frames/s, C6 > S3 |
//! | T3   | painlessMesh escaping + envelope; Broadcast costs ≥ (N-1)× Single |
//! | T4   | backend budget rule: 2/3/4/6 online → 25/20/15/10 Hz, AIMD |
//! | T5   | field calibration: ~70 pps → ≤ 3 % loss, ~150 pps → 20-40 % |
//! | T6   | 20 drones at fixed 25 Hz saturate the channel (expected physics) |
//! | T7   | the same with BackendBudget stays healthy |
//! | T8   | relay kill under telemetry, every map: brain ≤ 5 s, telemetry ≤ 20 s |
//! | T9   | Phase 5: RateMode::Brain + 30 % external busy stays healthy |
//! | T10  | base killed: telemetry has no bridge, the brain still elects a sink |
//! | T11  | step time with everything on (release only) |
//! | T12  | every byte on air booked once: per-drone sums = fleet totals |
//!
//! Run: `cargo test --release -p swarm-sim --test telemetry -- --nocapture`

mod harness;

use harness::{Desc, World, deployed, relays_by_descendants, secs};
use swarm_brain::{Intent, Role, Vec3};
use swarm_sim::channel::{self, AirFrame, AirKind, Channel, Dst, NodeRadio};
use swarm_sim::telemetry::{self, BackendBudget, chan, tlm};
use swarm_sim::{
    snapshot, BASE_ID, ChannelConfig, Preset, RadioConfig, RateMode, Routing, Sim, SimConfig, TelemetryConfig, Terrain,
    Variant, chatter, metrics, painless,
};

// ---- thresholds (fixed) -----------------------------------------------------

const STEP_MS: u32 = harness::STEP_MS;
const DEPLOY_LIMIT_MS: u32 = 120_000;
const CONNECTED_HOLD_MS: u32 = 10_000;

// T1
/// The whole v3 snapshot (link records carry `quality` since v3; the v2
/// digest was 0x8b94_cbfc_d261_ec2f).
const S8_DIGEST: u64 = 0x0027_f287_b0b3_f240;
/// Header after the version, drone records, link `[a, b, kind]` and pulses:
/// computed by the same fold on the v2 code before the channel rework, so
/// equality proves the brain-visible world did not move.
const S8_WORLD_DIGEST: u64 = 0x8c17_7873_7777_d6af;
const S8_RUN_MS: u32 = 60_000;

// T2
const T2_HZ: f32 = 25.0;
const T2_DELIVERED_HZ: (f64, f64) = (24.0, 25.5);
const T2_PAYLOAD_BPS: (f64, f64) = (4_500.0, 6_500.0);
const T2_SLOW_PER_S: (f64, f64) = (1.0, 3.0);

// T3
/// Allowance for the 1 s smoothing ripple of the airtime fractions.
const T3_RATIO_SLACK: f64 = 0.98;

// T4
const T4_EXPECT: [(usize, f32); 4] = [(2, 25.0), (3, 20.0), (4, 15.0), (6, 10.0)];

// T5 (field calibration, UDP_VS_MQTT.md "Where the ceiling sits")
const T5_FRAME_BYTES: u32 = 400;
const T5_CLEAN_MAX_LOSS: f64 = 0.03;
const T5_SATURATED_LOSS: (f64, f64) = (0.20, 0.40);
/// The field sessions lasted minutes: measure a steady 40 s after 20 s.
const T5_RUN_MS: u32 = 60_000;
const T5_WARM_MS: u32 = 20_000;

// T6
const T6_BUSY_MAX: f32 = 0.6;
const T6_MEAN_LOSS_PCT: f64 = 15.0;
/// Steady-state report (not asserted): the last 30 s of a 180 s run.
const T6_STEADY_END_MS: u32 = 180_000;
const T6_STEADY_MS: u32 = 30_000;

// T7
const T7_SETTLE_MS: u32 = 60_000;
const T7_MAX_MEDIAN_LOSS_PCT: f64 = 10.0;
const T7_MIN_GCS_HZ: f64 = 4.5;

// T8
const T8_BRAIN_LIMIT_MS: u32 = 5_000;
/// NODE_TIMEOUT (10 s) + rescan/reconnect.
const T8_TLM_LIMIT_MS: u32 = 20_000;

// T9
const T9_REQUESTED_HZ: f32 = 25.0;
const T9_EXTERNAL_BUSY: f32 = 0.3;
const T9_STEADY_MS: u32 = 60_000;
const T9_MIN_CONNECTED: f64 = 0.90;
const T9_HEAL_MS: u32 = 5_000;
/// 95 % of the brain's 5 Hz floor: a drone held at the floor may miss a
/// couple of its 50 packages in the 10 s window without failing.
const T9_MIN_GCS_HZ: f64 = 4.75;
const T9_MAX_FLEET_LOSS_PCT: f64 = 15.0;
const T9_MAX_BRAIN_RATE: f64 = 2.0;

// T10
const T10_SINK_LIMIT_MS: u32 = 10_000;
/// The GCS rate stats use a 2 s window; allow one more second.
const T10_STATS_DRAIN_MS: u32 = 3_000;

// T11
const T11_BUDGET_20_MS: f64 = 2.0;
const T11_BUDGET_100_MS: f64 = 15.0;

/// The last part of every measurement: GCS rates are averaged over this.
const GCS_WINDOW_MS: u32 = 10_000;

// ---- helpers ----------------------------------------------------------------

fn tlm_on(rate: RateMode, routing: Routing, variant: Variant, requested_hz: f32) -> TelemetryConfig {
    TelemetryConfig { enabled: true, variant, rate, routing, requested_hz, chatter_every_frame: true }
}

fn channel_on(external_busy: f32) -> ChannelConfig {
    ChannelConfig { enabled: true, external_busy, ..ChannelConfig::default() }
}

/// `preset` with `drones` (base included), channel and telemetry on.
fn cfg_on(preset: Preset, drones: usize, external: f32, t: TelemetryConfig) -> SimConfig {
    SimConfig { channel: channel_on(external), telemetry: t, ..SimConfig::for_preset(preset, 1, drones) }
}

fn label(cfg: &SimConfig, test: &str) -> String {
    format!("{test} {}", cfg.preset.name())
}

fn row(sim: &Sim, id: u16) -> &[f32] {
    let i = usize::from(id) * telemetry::TLM_STRIDE;
    &sim.telemetry_stats()[i..i + telemetry::TLM_STRIDE]
}

fn summ(sim: &Sim, k: usize) -> f32 {
    sim.channel_summary()[k]
}

/// Telemetry chatter records of the latest step, as `(kind, src, f)`.
fn tlm_records(sim: &Sim) -> impl Iterator<Item = (f32, u16, &[f32])> + '_ {
    sim.chatter().as_chunks::<{ chatter::CHATTER_STRIDE }>().0.iter().filter_map(|r| {
        let k = r[chatter::KIND];
        (k == chatter::CHATTER_TLM || k == chatter::CHATTER_ROOT_UPLINK).then(|| {
            (k, r[chatter::SRC] as u16, &r[chatter::F0..chatter::F0 + 8])
        })
    })
}

/// Live, non-root drone with a painlessMesh path to the root.
fn tlm_routed(sim: &Sim, id: u16) -> bool {
    id != BASE_ID && sim.is_alive(id) && sim.painless().connected(id)
}

/// Every live drone whose painlessMesh parent chain passes through `via`.
fn painless_subtree(sim: &Sim, via: u16) -> Vec<u16> {
    let n = sim.drone_count();
    (0..n as u16)
        .filter(|&id| id != via && sim.is_alive(id))
        .filter(|&id| {
            let mut at = id;
            for _ in 0..n {
                match sim.painless().parent(at) {
                    Some(p) if p == via => return true,
                    Some(p) => at = p,
                    None => return false,
                }
            }
            false
        })
        .collect()
}

/// Ground-station meter fed from per-frame chatter (`chatter_every_frame`):
/// exactly what was sent and what reached the WiFi station, per drone.
struct GcsMeter {
    steps: u32,
    sent: Vec<u32>,
    delivered: Vec<u32>,
    slow: Vec<u32>,
    frame_bytes: Vec<u64>,
    uplink_bytes: u64,
    /// Routed (painlessMesh) on every step of the window.
    always_routed: Vec<bool>,
    /// Summary sums, for means.
    pps: f64,
    busy_mean: f64,
    busy_max: f64,
    busy_max_peak: f32,
    median_loss: f64,
    connected_steps: u32,
}

impl GcsMeter {
    fn new(sim: &Sim) -> Self {
        let n = sim.drone_count();
        Self {
            steps: 0,
            sent: vec![0; n],
            delivered: vec![0; n],
            slow: vec![0; n],
            frame_bytes: vec![0; n],
            uplink_bytes: 0,
            always_routed: vec![true; n],
            pps: 0.0,
            busy_mean: 0.0,
            busy_max: 0.0,
            busy_max_peak: 0.0,
            median_loss: 0.0,
            connected_steps: 0,
        }
    }

    fn sample(&mut self, sim: &Sim) {
        self.steps += 1;
        for (kind, src, f) in tlm_records(sim) {
            if kind == chatter::CHATTER_TLM {
                let i = usize::from(src);
                self.sent[i] += 1;
                self.frame_bytes[i] += f[0] as u64;
                self.slow[i] += u32::from(f[4] > 0.5);
            } else {
                let origin = f[1] as usize;
                if let Some(d) = self.delivered.get_mut(origin) {
                    *d += 1;
                }
                self.uplink_bytes += f[0] as u64;
            }
        }
        for id in 0..sim.drone_count() as u16 {
            if !tlm_routed(sim, id) {
                self.always_routed[usize::from(id)] = false;
            }
        }
        self.pps += f64::from(summ(sim, chan::PPS_ON_AIR));
        self.busy_mean += f64::from(summ(sim, chan::BUSY_MEAN));
        self.busy_max += f64::from(summ(sim, chan::BUSY_MAX));
        self.busy_max_peak = self.busy_max_peak.max(summ(sim, chan::BUSY_MAX));
        self.median_loss += f64::from(summ(sim, chan::MEDIAN_LOSS_PCT));
        self.connected_steps += u32::from(metrics::scouts_connected(sim));
    }

    fn seconds(&self) -> f64 {
        f64::from(self.steps * STEP_MS) / 1000.0
    }

    fn mean(&self, sum: f64) -> f64 {
        if self.steps == 0 { 0.0 } else { sum / f64::from(self.steps) }
    }

    fn gcs_hz(&self, id: u16) -> f64 {
        f64::from(self.delivered[usize::from(id)]) / self.seconds()
    }

    fn loss_pct(&self, id: u16) -> f64 {
        let i = usize::from(id);
        if self.sent[i] == 0 { 0.0 } else { 100.0 * (1.0 - f64::from(self.delivered[i]) / f64::from(self.sent[i])) }
    }

    fn routed(&self) -> Vec<u16> {
        (0..self.sent.len() as u16).filter(|&id| self.always_routed[usize::from(id)]).collect()
    }

    fn fleet(&self) -> FleetLoss {
        let mut v: Vec<f64> = self.routed().iter().map(|&id| self.loss_pct(id)).collect();
        v.sort_by(f64::total_cmp);
        let median = if v.is_empty() {
            0.0
        } else if v.len() % 2 == 1 {
            v[v.len() / 2]
        } else {
            0.5 * (v[v.len() / 2 - 1] + v[v.len() / 2])
        };
        let mean = if v.is_empty() { 0.0 } else { v.iter().sum::<f64>() / v.len() as f64 };
        let (s, d): (u64, u64) = self
            .routed()
            .iter()
            .map(|&id| (u64::from(self.sent[usize::from(id)]), u64::from(self.delivered[usize::from(id)])))
            .fold((0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
        let pooled = if s == 0 { 0.0 } else { 100.0 * (1.0 - d as f64 / s as f64) };
        FleetLoss { median, mean, pooled, routed: v.len() }
    }

    fn report(&self, sim: &Sim, sc: &str) {
        let f = self.fleet();
        println!(
            "[{sc}] window {:.0} s: pps_on_air {:.0}, busy mean {:.3} / max {:.3} (peak {:.3}), sim median loss {:.1} %, \
             measured loss median {:.1} % / mean {:.1} % / pooled {:.1} % over {} routed, gcs {:.0} B/s, scouts_connected {:.0} %",
            self.seconds(),
            self.mean(self.pps),
            self.mean(self.busy_mean),
            self.mean(self.busy_max),
            self.busy_max_peak,
            self.mean(self.median_loss),
            f.median,
            f.mean,
            f.pooled,
            f.routed,
            self.uplink_bytes as f64 / self.seconds(),
            100.0 * f64::from(self.connected_steps) / f64::from(self.steps.max(1)),
        );
        println!("[{sc}]   id role      routed hops req_hz gcs_hz loss% sent deliv busy  air   qdrop/s");
        for id in 0..sim.drone_count() as u16 {
            if id == BASE_ID || !sim.is_alive(id) {
                continue;
            }
            let r = row(sim, id);
            println!(
                "[{sc}]   {id:>2} {:<9} {:>6} {:>4} {:>6.1} {:>6.2} {:>5.1} {:>4} {:>5} {:.2}  {:.3} {:.1}",
                format!("{:?}", sim.brain(id).expect("id").role()),
                self.always_routed[usize::from(id)],
                sim.painless().hops(id).map_or("-".into(), |h| h.to_string()),
                r[tlm::REQUESTED_HZ],
                self.gcs_hz(id),
                self.loss_pct(id),
                self.sent[usize::from(id)],
                self.delivered[usize::from(id)],
                r[tlm::BUSY_FRAC],
                r[tlm::AIRTIME_FRAC],
                r[tlm::QUEUE_DROPS_PER_S],
            );
        }
    }
}

struct FleetLoss {
    median: f64,
    mean: f64,
    pooled: f64,
    routed: usize,
}

/// Step `ms`, feeding a fresh meter every step.
fn meter_run(w: &mut World, ms: u32) -> GcsMeter {
    let mut m = GcsMeter::new(&w.sim);
    for _ in 0..ms / STEP_MS {
        w.step();
        m.sample(&w.sim);
    }
    m
}

/// MonitorFire at the ignition point and wait for "deployed". `Some(ms)`
/// after the command at which the 10 s window began.
fn try_deploy(w: &mut World, sc: &str) -> Option<u32> {
    let (fx, fz) = w.ignition();
    w.command(Intent::MonitorFire, fx, fz);
    let t = w.time_until_held(DEPLOY_LIMIT_MS, CONNECTED_HOLD_MS, deployed);
    match t {
        Some(t) => println!("[{sc}] deployed: window began {:.2} s after command", secs(t)),
        None => println!("[{sc}] NOT deployed within {} s (continuing at t={:.1} s)", secs(DEPLOY_LIMIT_MS), secs(w.now_ms())),
    }
    t
}

fn deploy_or_fail(w: &mut World, sc: &str) {
    if try_deploy(w, sc).is_none() {
        w.fail(sc, &format!("deploy: no {} s deployed window within {} s", secs(CONNECTED_HOLD_MS), secs(DEPLOY_LIMIT_MS)));
    }
}

/// The relay to kill: of the brain's relays carrying a scout, the one with
/// the largest painlessMesh subtree (ties: most brain descendants), so the
/// telemetry half of the test is not vacuous. `None` if no relay carries a
/// scout.
fn pick_victim(sim: &Sim) -> Option<(u16, Desc, Vec<u16>)> {
    relays_by_descendants(sim)
        .into_iter()
        .filter(|(_, d)| d.scouts > 0)
        .map(|(id, d)| (id, d, painless_subtree(sim, id)))
        .max_by_key(|(id, d, sub)| (sub.len(), d.all, std::cmp::Reverse(*id)))
}

fn json_unescape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut it = s.chars();
    while let Some(c) = it.next() {
        if c == '\\' {
            match it.next() {
                Some('"') => out.push('"'),
                Some('\\') => out.push('\\'),
                Some('n') => out.push('\n'),
                Some('t') => out.push('\t'),
                Some('r') => out.push('\r'),
                Some(o) => {
                    out.push('\\');
                    out.push(o);
                }
                None => out.push('\\'),
            }
        } else {
            out.push(c);
        }
    }
    out
}

// ---- T1 telemetry-off parity -------------------------------------------------

/// FNV-1a over the bits of `vals`.
fn fnv(digest: &mut u64, vals: &[f32]) {
    for v in vals {
        for b in v.to_bits().to_le_bytes() {
            *digest = (*digest ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
}

/// The S8 script (scenarios.rs `scripted`), digesting every step's snapshot.
/// Returns (whole-snapshot digest, world digest, final snapshot, fire, model
/// chatter seen). The world digest covers what the brains do and is
/// independent of the snapshot's link layout: the header after the version
/// (tick, time, counts, frames, bytes, drops, fire, intent), every drone
/// record (position, liveness, role, hops, parent), each link's `[a, b,
/// kind]` and every pulse.
fn s8_script(cfg: SimConfig) -> (u64, u64, Vec<u32>, Vec<u8>, bool) {
    let mut w = World::new(cfg);
    let (fx, fz) = w.ignition();
    let mut digest = 0xcbf2_9ce4_8422_2325u64; // FNV-1a
    let mut world = 0xcbf2_9ce4_8422_2325u64;
    let mut model_chatter = false;
    let mut fold = |s: &Sim| {
        let snap = s.snapshot();
        fnv(&mut digest, snap);
        let (drones, links) = (snap[3] as usize, snap[4] as usize);
        let at = snapshot::HEADER_LEN + drones * snapshot::DRONE_STRIDE;
        fnv(&mut world, &snap[1..at]);
        for k in 0..links {
            fnv(&mut world, &snap[at + k * snapshot::LINK_STRIDE..][..3]);
        }
        fnv(&mut world, &snap[at + links * snapshot::LINK_STRIDE..]);
        model_chatter |= tlm_records(s).next().is_some()
            || s.chatter().as_chunks::<{ chatter::CHATTER_STRIDE }>().0.iter().any(|r| {
                r[chatter::KIND] == chatter::CHATTER_PM_SYNC || r[chatter::KIND] == chatter::CHATTER_PM_TIME
            });
    };
    w.command(Intent::MonitorFire, fx, fz);
    w.measure(20_000, &mut fold);
    w.sim.set_loss(0.2);
    w.measure(10_000, &mut fold);
    w.kill(&[7]);
    w.measure(10_000, &mut fold);
    w.sim.set_loss(0.0);
    w.command(Intent::MapPerimeter, fx, fz);
    w.measure(S8_RUN_MS - 40_000, &mut fold);
    assert_eq!(w.now_ms(), S8_RUN_MS);
    let snap = w.sim.snapshot().iter().map(|v| v.to_bits()).collect();
    (digest, world, snap, w.sim.fire().cells().to_vec(), model_chatter)
}

#[test]
fn t1_telemetry_off_parity() {
    let default = s8_script(SimConfig::default());
    // Explicitly off, with every other knob of both models moved away from
    // its default: off must mean off.
    let explicit = SimConfig {
        channel: ChannelConfig {
            enabled: false,
            external_busy: 0.5,
            espnow_rate_mbps: 2.0,
            min_rate_mbps: 24.0,
            cs_range_factor: 2.0,
            tx_queue_frames: 3,
            gcs_on_wifi: false,
        },
        telemetry: TelemetryConfig {
            enabled: false,
            variant: Variant::C6Rtk,
            rate: RateMode::BackendBudget,
            routing: Routing::Broadcast,
            requested_hz: 10.0,
            chatter_every_frame: true,
        },
        ..SimConfig::default()
    };
    let off = s8_script(explicit);
    println!(
        "[T1 parity] default digest {:016x}, explicit-off digest {:016x}, expected {S8_DIGEST:016x}; world digest {:016x} \
         (expected {S8_WORLD_DIGEST:016x}, the v2 snapshot's minus its version)",
        default.0, off.0, default.1
    );
    assert_eq!(default.1, S8_WORLD_DIGEST, "T1: drones, routes or traffic changed: the world is no longer bit-identical");
    assert_eq!(default.0, S8_DIGEST, "T1: default-config S8 digest changed: the world is no longer bit-identical");
    assert_eq!(off.0, default.0, "T1: explicit-off config diverged from the default config");
    assert!(off.2 == default.2, "T1: final snapshots differ");
    assert!(off.3 == default.3, "T1: fire grids differ");
    assert!(!default.4 && !off.4, "T1: telemetry/painlessMesh chatter appeared while the models are off");

    // Stats stay zero-filled while off.
    let mut sim = Sim::new(SimConfig::default());
    for _ in 0..100 {
        sim.step(STEP_MS);
    }
    assert_eq!(sim.telemetry_stats().len(), sim.drone_count() * telemetry::TLM_STRIDE);
    assert_eq!(sim.channel_summary().len(), telemetry::CHANNEL_LEN);
    for id in 0..sim.drone_count() as u16 {
        let r = row(&sim, id);
        assert_eq!(r[tlm::ID], f32::from(id));
        assert!(r[1..].iter().all(|&v| v == 0.0 || v == -1.0), "T1: drone {id} stats not zero while off: {r:?}");
        assert_eq!(sim.tlm_sample(id), "", "T1: drone {id} has a package while telemetry is off");
    }
    assert_eq!(summ(&sim, chan::TELEMETRY_ON), 0.0);
    assert_eq!(summ(&sim, chan::CHANNEL_ON), 0.0);
    assert_eq!(summ(&sim, chan::GCS_PPS), 0.0);
}

// ---- T2 byte budget ----------------------------------------------------------

struct OneDrone {
    gcs_hz: f64,
    payload_bps: f64,
    slow_per_s: f64,
    mean_frame: f64,
    sim_delivered_hz: f32,
    sample: String,
    last_frame: f32,
    last_package: f32,
}

fn one_drone(variant: Variant) -> OneDrone {
    let sc = format!("T2 byte budget {variant:?}");
    let cfg = cfg_on(Preset::AlpineRidge, 2, 0.0, tlm_on(RateMode::Fixed(T2_HZ), Routing::Single, variant, T2_HZ));
    let mut w = World::new(cfg);
    w.run(5_000);
    let m = meter_run(&mut w, GCS_WINDOW_MS);
    let s = m.seconds();
    let r = row(&w.sim, 1);
    let out = OneDrone {
        gcs_hz: m.gcs_hz(1),
        payload_bps: m.frame_bytes[1] as f64 / s,
        slow_per_s: f64::from(m.slow[1]) / s,
        mean_frame: m.frame_bytes[1] as f64 / f64::from(m.sent[1].max(1)),
        sim_delivered_hz: r[tlm::DELIVERED_HZ],
        sample: w.sim.tlm_sample(1).to_string(),
        last_frame: r[tlm::LAST_FRAME_BYTES],
        last_package: r[tlm::LAST_PACKAGE_BYTES],
    };
    println!(
        "[{sc}] sent {} delivered {} in {s:.0} s: {:.2} Hz at GCS (sim stat {:.2}), payload {:.0} B/s, \
         uplink {:.0} B/s, {:.2} slow frames/s, mean frame {:.1} B, last frame {} B / package {} B, \
         tx {:.0} B/s, busy {:.3}, req {} Hz, operator {} Hz",
        m.sent[1],
        m.delivered[1],
        out.gcs_hz,
        out.sim_delivered_hz,
        out.payload_bps,
        m.uplink_bytes as f64 / s,
        out.slow_per_s,
        out.mean_frame,
        out.last_frame,
        out.last_package,
        r[tlm::TX_BYTES_PER_S],
        r[tlm::BUSY_FRAC],
        r[tlm::REQUESTED_HZ],
        r[tlm::OPERATOR_HZ],
    );
    assert_eq!(r[tlm::REQUESTED_HZ], T2_HZ, "{sc}: requested_hz stat");
    assert_eq!(r[tlm::OPERATOR_HZ], T2_HZ, "{sc}: operator_hz stat");
    out
}

#[test]
fn t2_byte_budget_one_drone() {
    let s3 = one_drone(Variant::XiaoS3);
    let in_range = |v: f64, (lo, hi): (f64, f64)| (lo..=hi).contains(&v);
    assert!(in_range(s3.gcs_hz, T2_DELIVERED_HZ), "T2: delivered {:.2} Hz, need {T2_DELIVERED_HZ:?}", s3.gcs_hz);
    assert!(
        in_range(f64::from(s3.sim_delivered_hz), T2_DELIVERED_HZ),
        "T2: sim delivered_hz stat {:.2}, need {T2_DELIVERED_HZ:?}",
        s3.sim_delivered_hz
    );
    assert!(in_range(s3.payload_bps, T2_PAYLOAD_BPS), "T2: payload {:.0} B/s, need {T2_PAYLOAD_BPS:?}", s3.payload_bps);
    assert!(in_range(s3.slow_per_s, T2_SLOW_PER_S), "T2: {:.2} slow frames/s, need {T2_SLOW_PER_S:?}", s3.slow_per_s);
    assert!(!s3.sample.contains("\\\"gq\\\""), "T2: XIAO S3 frame carries GNSS keys: {}", s3.sample);

    let c6 = one_drone(Variant::C6Rtk);
    assert!(in_range(c6.gcs_hz, T2_DELIVERED_HZ), "T2: C6 delivered {:.2} Hz, need {T2_DELIVERED_HZ:?}", c6.gcs_hz);
    assert!(c6.mean_frame > s3.mean_frame, "T2: C6 mean frame {:.1} B not > S3 {:.1} B", c6.mean_frame, s3.mean_frame);
    assert!(c6.payload_bps > s3.payload_bps, "T2: C6 payload {:.0} B/s not > S3 {:.0}", c6.payload_bps, s3.payload_bps);
    assert!(c6.sample.contains("\\\"gq\\\""), "T2: C6 frame lacks the GNSS keys: {}", c6.sample);
}

// ---- T3 escaping / package overhead, Broadcast vs Single -----------------------

#[test]
fn t3_package_escaping_overhead() {
    let mut pkg = String::new();
    painless::encode_package(&mut pkg, painless::SINGLE, 7, 9, "{\"a\":\"b\\c\"}");
    assert_eq!(pkg, r#"{"type":9,"dest":7,"from":9,"msg":"{\"a\":\"b\\c\"}"}"#);
    assert_eq!(painless::escaped_len("{\"a\":\"b\\c\"}"), "{\\\"a\\\":\\\"b\\\\c\\\"}".len());

    for variant in [Variant::XiaoS3, Variant::C6Rtk] {
        let sc = format!("T3 escaping {variant:?}");
        let cfg = cfg_on(Preset::AlpineRidge, 2, 0.0, tlm_on(RateMode::Fixed(25.0), Routing::Single, variant, 25.0));
        let mut w = World::new(cfg);
        w.run(3_000);
        let sample = w.sim.tlm_sample(1).to_string();
        let r = row(&w.sim, 1);
        let prefix = format!(
            "{{\"type\":{},\"dest\":{},\"from\":{},\"msg\":\"",
            painless::SINGLE,
            painless::node_id(BASE_ID),
            painless::node_id(1)
        );
        assert!(sample.starts_with(&prefix), "{sc}: package {sample:?} does not start with {prefix:?}");
        assert!(sample.ends_with("\"}"), "{sc}: package does not end with \"}}: {sample:?}");
        let msg = json_unescape(&sample[prefix.len()..sample.len() - 2]);
        let head = format!("{{\"type\":\"drone\",\"n\":{},\"s\":", painless::node_id(1));
        assert!(msg.starts_with(&head), "{sc}: frame {msg:?}");
        assert!(msg.ends_with('}'), "{sc}: frame {msg:?}");
        let mut envelope = String::new();
        painless::encode_package(&mut envelope, painless::SINGLE, painless::node_id(BASE_ID), painless::node_id(1), "");
        let escaping = painless::escaped_len(&msg) - msg.len();
        let expect_pkg = msg.len() + escaping + envelope.len() + 1;
        println!(
            "[{sc}] frame {} B, package {} B = frame + escaping {escaping} + envelope {} + '\\0'",
            msg.len(),
            r[tlm::LAST_PACKAGE_BYTES],
            envelope.len()
        );
        assert_eq!(r[tlm::LAST_FRAME_BYTES] as usize, msg.len(), "{sc}: last_frame_bytes vs the sample's msg");
        assert_eq!(r[tlm::LAST_PACKAGE_BYTES] as usize, sample.len() + 1, "{sc}: last_package_bytes vs the sample");
        assert_eq!(sample.len() + 1, expect_pkg, "{sc}: package != frame + escaping + envelope + 1");
        assert!(escaping >= 20, "{sc}: only {escaping} escape bytes in a JSON frame");
    }
}

/// Mesh data + 802.11 ACK airtime per second, averaged over `ms`, plus the
/// tree shape: (airtime frac, connected nodes incl. root, Σ hops of senders,
/// senders).
fn mesh_air(routing: Routing, drones: usize, ms: u32) -> (f64, f64, f64, f64) {
    let cfg = cfg_on(Preset::Foothills, drones, 0.0, tlm_on(RateMode::Fixed(10.0), routing, Variant::XiaoS3, 10.0));
    let mut w = World::new(cfg);
    w.run(10_000);
    let (mut air, mut conn, mut hops, mut senders) = (0.0, 0.0, 0.0, 0.0);
    let steps = ms / STEP_MS;
    for _ in 0..steps {
        w.step();
        let ch = w.sim.channel();
        air += f64::from(ch.kind_frac(AirKind::MeshData) + ch.kind_frac(AirKind::Ack));
        for id in 0..drones as u16 {
            if w.sim.painless().connected(id) || id == BASE_ID {
                conn += 1.0;
            }
            if tlm_routed(&w.sim, id) {
                senders += 1.0;
                hops += f64::from(w.sim.painless().hops(id).unwrap_or(0));
            }
        }
    }
    let n = f64::from(steps);
    (air / n, conn / n, hops / n, senders / n)
}

#[test]
fn t3_broadcast_costs_n_minus_1_singles() {
    let drones = 6;
    let (single, conn, hops, senders) = mesh_air(Routing::Single, drones, 10_000);
    let (bcast, conn_b, _, _) = mesh_air(Routing::Broadcast, drones, 10_000);
    // Single: each frame crosses `hops` links; Broadcast: the flood reaches
    // each of the other N-1 connected nodes once.
    let need = T3_RATIO_SLACK * senders * (conn - 1.0) / hops.max(1e-9);
    let ratio = bcast / single.max(1e-12);
    println!(
        "[T3 broadcast] connected {conn:.2}/{conn_b:.2} nodes, {senders:.2} senders, Σhops {hops:.2}: \
         mesh airtime single {single:.5}, broadcast {bcast:.5}, ratio {ratio:.2} (need ≥ {need:.2}; literal N-1 = {:.0})",
        conn - 1.0
    );
    assert!(single > 0.0, "T3: Single telemetry put no mesh data airtime on the channel");
    assert!(conn >= drones as f64 - 0.01, "T3: mesh not fully connected ({conn:.2} of {drones})");
    assert!(ratio >= need, "T3: Broadcast/Single airtime {ratio:.2} < {need:.2}");
}

// ---- T4 backend budget ----------------------------------------------------------

#[test]
fn t4_backend_budget_rule() {
    let b = BackendBudget::default();
    assert_eq!(b.budget_pps, telemetry::BUDGET_PPS);
    assert_eq!(b.hz(1, 25.0), 25.0);
    for (online, hz) in T4_EXPECT {
        assert_eq!(b.hz(online, 25.0), hz, "T4: hz({online})");
    }
    assert_eq!(b.hz(19, 25.0), telemetry::MIN_HZ, "T4: clamp to MIN_HZ");
    assert_eq!(b.hz(2, 10.0), 10.0, "T4: operator cap");

    // AIMD: while loss > 10 %, cut ×0.8 every 10 s down to the 20 pps floor.
    let mut b = BackendBudget::default();
    let mut cut_at = Vec::new();
    let mut last = b.budget_pps;
    for t in (0..=60_000).step_by(STEP_MS as usize) {
        b.update(t, 25.0);
        if b.budget_pps < last {
            assert!((b.budget_pps - (last * 0.8).max(telemetry::BUDGET_FLOOR_PPS)).abs() < 1e-3, "T4: cut is ×0.8");
            cut_at.push(t);
            last = b.budget_pps;
        }
    }
    println!("[T4 AIMD] cuts at {cut_at:?} ms, budget {} pps", b.budget_pps);
    // 60 → 48 → 38.4 → 30.7 → 24.6 → 20 (floor): five cuts.
    assert_eq!(cut_at.len(), 5, "T4: cuts at {cut_at:?}");
    assert!(cut_at[0] <= 10_000, "T4: first cut only at {} ms", cut_at[0]);
    assert!(
        cut_at.windows(2).all(|w| (10_000..=10_000 + STEP_MS).contains(&(w[1] - w[0]))),
        "T4: cuts must be 10 s apart: {cut_at:?}"
    );
    for t in (60_000..=200_000).step_by(1_000) {
        b.update(t, 50.0);
    }
    assert_eq!(b.budget_pps, telemetry::BUDGET_FLOOR_PPS, "T4: floor");
    // Recovery: +5 pps per 30 s after 30 s below 5 %.
    let t0 = 300_000;
    for t in (t0..t0 + 29_000).step_by(1_000) {
        b.update(t, 1.0);
    }
    assert_eq!(b.budget_pps, telemetry::BUDGET_FLOOR_PPS, "T4: no increase before 30 s of good loss");
    for t in (t0 + 29_000..=t0 + 95_000).step_by(1_000) {
        b.update(t, 1.0);
    }
    let gained = b.budget_pps - telemetry::BUDGET_FLOOR_PPS;
    assert!((10.0..=15.0).contains(&gained), "T4: gained {gained} pps in 95 s good (need +5 per 30 s)");
    for t in (t0 + 96_000..=t0 + 2_000_000).step_by(1_000) {
        b.update(t, 1.0);
    }
    assert_eq!(b.budget_pps, telemetry::BUDGET_PPS, "T4: recovery caps at the initial budget");

    // In the sim: the ground station counts online drones and every drone
    // sends at the share.
    for (online, hz) in T4_EXPECT {
        let sc = format!("T4 sim {online} online");
        let cfg = cfg_on(Preset::Foothills, online + 1, 0.0, tlm_on(RateMode::BackendBudget, Routing::Single, Variant::XiaoS3, 25.0));
        let mut w = World::new(cfg);
        w.run(15_000);
        let m = meter_run(&mut w, GCS_WINDOW_MS);
        let reqs: Vec<f32> = (1..=online as u16).map(|id| row(&w.sim, id)[tlm::REQUESTED_HZ]).collect();
        let gcs: Vec<String> = (1..=online as u16).map(|id| format!("{:.1}", m.gcs_hz(id))).collect();
        println!(
            "[{sc}] online stat {}, budget {} pps, requested {reqs:?}, gcs Hz {gcs:?}, median loss {:.1} %",
            summ(&w.sim, chan::ONLINE),
            summ(&w.sim, chan::BUDGET_PPS),
            summ(&w.sim, chan::MEDIAN_LOSS_PCT)
        );
        assert_eq!(summ(&w.sim, chan::ONLINE) as usize, online, "{sc}: online count");
        assert_eq!(summ(&w.sim, chan::MODE), 1.0, "{sc}: mode code");
        assert!(reqs.iter().all(|&r| (r - hz).abs() < 0.01), "{sc}: requested {reqs:?}, need {hz} Hz each");
    }
}

// ---- T5 field calibration ---------------------------------------------------------

/// The field cell of `UDP_VS_MQTT.md`: one AP (node 0), the ground-station
/// laptop on the same 2.4 GHz AP (node 1), and one station per entry of
/// `hz`, a few metres apart, each sending `T5_FRAME_BYTES` UDP datagrams.
/// Every frame crosses the air twice (station → AP, AP → laptop), driven
/// exactly like the world drives the channel: `admit` + `account` per
/// transmission and `unicast` (every MAC attempt drawn from the SINR model)
/// per delivery. Returns the end-to-end loss
/// fraction over the last 40 s of 60 s, offered pps, and the AP's busy.
fn ap_loss(hz: &[f64], rate_mbps: f32, external: f32) -> (f64, f64, f32) {
    const AP: u16 = 0;
    const LAPTOP: u16 = 1;
    let nodes_n = hz.len() + 2;
    let cfg = ChannelConfig { min_rate_mbps: rate_mbps, ..channel_on(external) };
    let mut ch = Channel::new(cfg, nodes_n, 1);
    let size = 9;
    let terrain = Terrain::from_heights(size, 400.0, vec![0.0; size * size]);
    let radio = RadioConfig::default();
    let nodes: Vec<NodeRadio> = (0..nodes_n)
        .map(|i| {
            let a = i as f32 * 1.1;
            let r = if i == 0 { 0.0 } else { 3.0 + i as f32 };
            NodeRadio { pos: Vec3::new(r * a.cos(), 10.0, r * a.sin()), alive: true }
        })
        .collect();
    let mpdu = channel::QOS_MAC_OVERHEAD + channel::LLC_SNAP + channel::UDP_IP_OVERHEAD + T5_FRAME_BYTES;
    let rate = ch.config().min_rate_mbps;
    let mut due = vec![0.0f64; hz.len()];
    let (mut sent, mut lost) = (0u64, 0u64);
    let total_steps = T5_RUN_MS / STEP_MS;
    let warm = T5_WARM_MS / STEP_MS;
    let mut now = 0;
    for step in 0..total_steps {
        now += STEP_MS;
        ch.begin_step(now, STEP_MS);
        for (k, d) in due.iter_mut().enumerate() {
            let src = (k + 2) as u16;
            *d += hz[k] * f64::from(STEP_MS) / 1000.0;
            while *d >= 1.0 {
                *d -= 1.0;
                let counted = step >= warm;
                sent += u64::from(counted);
                let up = ch.admit(src)
                    && ch.unicast(now, &AirFrame::new(src, AirKind::Uplink, mpdu, rate), Dst::Node(AP), true).delivered;
                let ok = up
                    && ch.admit(AP)
                    && ch.unicast(now, &AirFrame::new(AP, AirKind::Uplink, mpdu, rate), Dst::Node(LAPTOP), true).delivered;
                lost += u64::from(counted && !ok);
            }
        }
        ch.end_step(&nodes, &terrain, &radio);
    }
    let secs_measured = f64::from((total_steps - warm) * STEP_MS) / 1000.0;
    (lost as f64 / sent.max(1) as f64, sent as f64 / secs_measured, ch.busy(AP))
}

/// The field sessions ran with 802.11b rates still on (frames, ACKs and
/// retries at 1-11 Mbps DSSS, `UDP_VS_MQTT.md` "The second look"); 2 Mbps
/// stands for that mix.
const T5_FIELD_RATE_MBPS: f32 = 2.0;
/// The 70 pps session: 2 nodes at 10 Hz, 2 at 25 Hz.
const T5_CLEAN_SESSION: [f64; 4] = [10.0, 10.0, 25.0, 25.0];
/// The 150 pps session: 6 nodes at 25 Hz.
const T5_COLLAPSE_SESSION: [f64; 6] = [25.0; 6];

#[test]
fn t5_field_calibration() {
    let (clean, pps_c, busy_c) = ap_loss(&T5_CLEAN_SESSION, T5_FIELD_RATE_MBPS, 0.0);
    let (sat, pps_s, busy_s) = ap_loss(&T5_COLLAPSE_SESSION, T5_FIELD_RATE_MBPS, 0.0);
    // After radio hygiene (11b off, 12 Mbps minimum): the doc's target is
    // 6-7 nodes x 10 Hz under 2 %, not yet measured in the field, so it is
    // reported, not asserted.
    let (hyg, pps_h, _) = ap_loss(&[10.0; 6], 12.0, 0.0);
    let (hyg150, _, _) = ap_loss(&T5_COLLAPSE_SESSION, 12.0, 0.0);
    println!(
        "[T5 calibration] field cell at {T5_FIELD_RATE_MBPS} Mbps, {T5_FRAME_BYTES} B frames crossing the air twice: \
         {pps_c:.0} pps → loss {:.1} % (AP busy {busy_c:.3}; field ~1 %), {pps_s:.0} pps → loss {:.1} % \
         (AP busy {busy_s:.3}; field 26-32 %). At 12 Mbps: {pps_h:.0} pps → {:.1} % (doc target < 2 %), \
         150 pps → {:.1} %",
        100.0 * clean,
        100.0 * sat,
        100.0 * hyg,
        100.0 * hyg150
    );
    assert!(clean <= T5_CLEAN_MAX_LOSS, "T5: {pps_c:.0} pps loss {:.1} % > {:.0} %", 100.0 * clean, 100.0 * T5_CLEAN_MAX_LOSS);
    assert!(
        (T5_SATURATED_LOSS.0..=T5_SATURATED_LOSS.1).contains(&sat),
        "T5: {pps_s:.0} pps loss {:.1} %, need {:.0}-{:.0} %",
        100.0 * sat,
        100.0 * T5_SATURATED_LOSS.0,
        100.0 * T5_SATURATED_LOSS.1
    );
}

#[test]
fn t5_external_busy_costs_delivery() {
    // The slider is airtime from elsewhere: the same 70 pps session must
    // lose more with it up, and the busy fraction must include it.
    let (base, _, busy0) = ap_loss(&T5_CLEAN_SESSION, T5_FIELD_RATE_MBPS, 0.0);
    let (loaded, _, busy5) = ap_loss(&T5_CLEAN_SESSION, T5_FIELD_RATE_MBPS, 0.5);
    println!(
        "[T5 external] 70 pps field cell: loss {:.1} % at external 0 (AP busy {busy0:.3}), {:.1} % at external 0.5 \
         (AP busy {busy5:.3})",
        100.0 * base,
        100.0 * loaded
    );
    assert!(busy5 >= (busy0 + 0.5).min(1.0) - 0.02, "T5: busy {busy5:.3} does not include the external 0.5");
    assert!(loaded > base, "T5: external busy 0.5 did not increase loss ({:.2} % vs {:.2} %)", 100.0 * loaded, 100.0 * base);
}

// ---- T6 / T7 saturation and the budget rule ----------------------------------------

#[test]
fn t6_fixed_25hz_saturates_alpine() {
    let cfg = cfg_on(Preset::AlpineRidge, 20, 0.0, tlm_on(RateMode::Fixed(25.0), Routing::Single, Variant::XiaoS3, 25.0));
    let sc = &label(&cfg, "T6 saturation");
    let mut w = World::new(cfg);
    try_deploy(&mut w, sc);
    let m = meter_run(&mut w, GCS_WINDOW_MS);
    m.report(&w.sim, sc);
    let f = m.fleet();
    let routed = m.routed();
    let mean_hz = routed.iter().map(|&id| m.gcs_hz(id)).sum::<f64>() / routed.len().max(1) as f64;
    let busy_max = summ(&w.sim, chan::BUSY_MAX);
    println!(
        "[{sc}] RESULT pps {:.0}, busy_max {busy_max:.3} (window mean {:.3}), mean {mean_hz:.2} Hz at GCS, mean loss {:.1} % \
         (expected physics: saturated)",
        m.mean(m.pps),
        m.mean(m.busy_max),
        f.mean
    );
    assert!(
        m.mean(m.busy_max) > f64::from(T6_BUSY_MAX) || f.mean > T6_MEAN_LOSS_PCT,
        "T6: 20 drones x 25 Hz did not saturate the channel: busy_max {:.3} (need > {T6_BUSY_MAX}) and mean loss {:.1} % \
         (need > {T6_MEAN_LOSS_PCT} %)",
        m.mean(m.busy_max),
        f.mean
    );

    // Steady state, reported only: the window above opens right after
    // deployment (~75 s) and averages over always-routed drones; the UI
    // shows every drone minutes later. The last 30 s of a 180 s run, over
    // every live drone.
    w.run((T6_STEADY_END_MS - T6_STEADY_MS).saturating_sub(w.now_ms()));
    let mut q = (0.0f64, 0.0f64);
    let mut steady = GcsMeter::new(&w.sim);
    for _ in 0..T6_STEADY_MS / STEP_MS {
        w.step();
        steady.sample(&w.sim);
        q.0 += f64::from(summ(&w.sim, chan::LINK_QUALITY_MEAN));
        q.1 += f64::from(summ(&w.sim, chan::LINKS_DEGRADED));
    }
    let ids: Vec<u16> = (1..w.sim.drone_count() as u16).filter(|&id| w.sim.is_alive(id)).collect();
    let mut hz: Vec<f64> = ids.iter().map(|&id| steady.gcs_hz(id)).collect();
    hz.sort_by(f64::total_cmp);
    let (sent, deliv): (u32, u32) = ids.iter().map(|&id| (steady.sent[usize::from(id)], steady.delivered[usize::from(id)])).fold((0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
    let n = f64::from(steady.steps);
    println!(
        "[{sc}] STEADY t={:.0}-{:.0} s, all {} live drones: {:.1} Hz median / {:.1} mean at GCS, {} not heard, pooled loss {:.1} %, \
         busy mean {:.3} / max {:.3}, link quality {:.2}, {:.1} links degraded",
        f64::from(T6_STEADY_END_MS - T6_STEADY_MS) / 1000.0,
        f64::from(T6_STEADY_END_MS) / 1000.0,
        ids.len(),
        hz[hz.len() / 2],
        hz.iter().sum::<f64>() / hz.len() as f64,
        hz.iter().filter(|&&h| h == 0.0).count(),
        100.0 * (1.0 - f64::from(deliv) / f64::from(sent.max(1))),
        steady.mean(steady.busy_mean),
        steady.mean(steady.busy_max),
        q.0 / n,
        q.1 / n,
    );
}

#[test]
fn t7_backend_budget_keeps_alpine_healthy() {
    let cfg = cfg_on(Preset::AlpineRidge, 20, 0.0, tlm_on(RateMode::BackendBudget, Routing::Single, Variant::XiaoS3, 25.0));
    let sc = &label(&cfg, "T7 budget");
    let mut w = World::new(cfg);
    try_deploy(&mut w, sc);
    w.run(T7_SETTLE_MS - GCS_WINDOW_MS);
    let m = meter_run(&mut w, GCS_WINDOW_MS);
    m.report(&w.sim, sc);
    let f = m.fleet();
    let routed = m.routed();
    let slow: Vec<String> = routed
        .iter()
        .filter(|&&id| m.gcs_hz(id) < T7_MIN_GCS_HZ)
        .map(|&id| format!("#{id} {:.2} Hz", m.gcs_hz(id)))
        .collect();
    println!(
        "[{sc}] RESULT budget {} pps, median loss {:.1} % (sim stat {:.1} %), {} routed, below {T7_MIN_GCS_HZ} Hz: {slow:?}",
        summ(&w.sim, chan::BUDGET_PPS),
        f.median,
        m.mean(m.median_loss),
        routed.len()
    );
    assert!(!routed.is_empty(), "T7: no drone routed in the painlessMesh tree");
    assert!(f.median <= T7_MAX_MEDIAN_LOSS_PCT, "T7: fleet median loss {:.1} % > {T7_MAX_MEDIAN_LOSS_PCT} %", f.median);
    assert!(slow.is_empty(), "T7: routed drones below {T7_MIN_GCS_HZ} Hz at GCS: {slow:?}");
}

// ---- T8 relay kill under telemetry (every map) ---------------------------------------

fn t8(preset: Preset) {
    let cfg = SimConfig {
        channel: channel_on(0.0),
        telemetry: tlm_on(RateMode::BackendBudget, Routing::Single, Variant::XiaoS3, 25.0),
        ..SimConfig::for_preset(preset, 1, 20)
    };
    let sc = &label(&cfg, "T8 relay kill");
    let mut w = World::new(cfg);
    deploy_or_fail(&mut w, sc);
    let Some((victim, d, subtree)) = pick_victim(&w.sim) else {
        w.fail(sc, "no live relay carrying a scout to kill");
    };
    println!(
        "[{sc}] killing relay #{victim} ({:?}, brain desc={} scouts={}, painless hops {:?}, painless subtree {subtree:?})",
        w.sim.brain(victim).expect("id").role(),
        d.all,
        d.scouts,
        w.sim.painless().hops(victim)
    );
    // Scouts the base cannot hear directly must reach it through someone;
    // if one exists and no relay carries any painlessMesh subtree, the tree
    // is not following the radio (or the test would be vacuous).
    let unheard: Vec<u16> =
        metrics::with_role(&w.sim, Role::Scout).filter(|&id| !w.sim.in_contact(id, BASE_ID)).collect();
    if subtree.is_empty() {
        println!(
            "[{sc}] NOTE: victim has no painlessMesh children; scouts out of the base's radio contact: {unheard:?}"
        );
        if !unheard.is_empty() {
            w.fail(
                sc,
                &format!(
                    "no brain relay carries a painlessMesh subtree although scouts {unheard:?} cannot hear the base \
                     directly (painlessMesh tree ignores radio range?)"
                ),
            );
        }
    }
    w.kill(&[victim]);
    let start = w.now_ms();
    let mut brain_at: Option<u32> = None;
    let mut brain_held = false;
    let mut brain_broke: Option<u32> = None;
    let mut resumed: Vec<Option<u32>> = vec![None; subtree.len()];
    let horizon = T8_TLM_LIMIT_MS.max(T8_BRAIN_LIMIT_MS + CONNECTED_HOLD_MS);
    while w.now_ms() - start < horizon {
        w.step();
        let t = w.now_ms() - start;
        let ok = metrics::scouts_connected(&w.sim);
        match brain_at {
            None if ok && t <= T8_BRAIN_LIMIT_MS => brain_at = Some(t),
            Some(t0) if !brain_held && brain_broke.is_none() => {
                if !ok {
                    brain_broke = Some(t - t0);
                } else if t - t0 >= CONNECTED_HOLD_MS {
                    brain_held = true;
                }
            }
            _ => {}
        }
        for (_, _, f) in tlm_records(&w.sim).filter(|r| r.0 == chatter::CHATTER_ROOT_UPLINK) {
            let origin = f[1] as u16;
            if let Some(k) = subtree.iter().position(|&id| id == origin) {
                resumed[k].get_or_insert(t);
            }
        }
    }
    let alive_sub: Vec<(u16, Option<u32>)> =
        subtree.iter().zip(&resumed).filter(|(id, _)| w.sim.is_alive(**id)).map(|(&id, &r)| (id, r)).collect();
    let worst = alive_sub.iter().map(|&(_, r)| r.unwrap_or(u32::MAX)).max();
    let shown: Vec<String> = alive_sub
        .iter()
        .map(|&(id, r)| format!("#{id}:{}", r.map_or("never".into(), |t| format!("{:.2}s", secs(t)))))
        .collect();
    println!(
        "[{sc}] RESULT brain scouts_connected restored {} (held {} s: {}), painless subtree telemetry resumed [{}] worst {}",
        brain_at.map_or("never".into(), |t| format!("in {:.2} s", secs(t))),
        secs(CONNECTED_HOLD_MS),
        if brain_held { "yes".to_string() } else { format!("no, broke after {:?} ms", brain_broke) },
        shown.join(" "),
        match worst {
            None => "n/a (empty)".to_string(),
            Some(u32::MAX) => "never".to_string(),
            Some(t) => format!("{:.2} s", secs(t)),
        }
    );
    if brain_at.is_none() {
        w.fail(sc, &format!("brain: scouts_connected not restored within {} s", secs(T8_BRAIN_LIMIT_MS)));
    }
    if !brain_held {
        w.fail(sc, &format!("brain: restored but did not hold {} s", secs(CONNECTED_HOLD_MS)));
    }
    if let Some(t) = worst
        && t > T8_TLM_LIMIT_MS
    {
        w.fail(sc, &format!("telemetry from the cut-off subtree not resumed within {} s: {shown:?}", secs(T8_TLM_LIMIT_MS)));
    }
}

/// The painlessMesh tree must follow the radio: every connected node's
/// parent is in radio contact, hop counts match the parent chain, and the
/// root has no parent.
fn tree_is_physical(preset: Preset) {
    let cfg = SimConfig {
        channel: channel_on(0.0),
        telemetry: tlm_on(RateMode::BackendBudget, Routing::Single, Variant::XiaoS3, 25.0),
        ..SimConfig::for_preset(preset, 1, 20)
    };
    let sc = &label(&cfg, "T8 tree physical");
    let mut w = World::new(cfg);
    try_deploy(&mut w, sc);
    let mesh = w.sim.painless();
    let mut errs = Vec::new();
    let mut shape = Vec::new();
    assert_eq!(mesh.root(), BASE_ID, "{sc}: root");
    if mesh.parent(BASE_ID).is_some() {
        errs.push("root has a parent".to_string());
    }
    for id in 1..w.sim.drone_count() as u16 {
        if !w.sim.is_alive(id) || !mesh.connected(id) {
            shape.push(format!("#{id}:-"));
            continue;
        }
        let Some(p) = mesh.parent(id) else {
            errs.push(format!("#{id} connected without a parent"));
            continue;
        };
        shape.push(format!("#{id}->{p}"));
        if !w.sim.in_contact(id, p) {
            let d = harness::dist_xz(w.sim.position(id).expect("id"), w.sim.position(p).expect("id"));
            errs.push(format!("#{id} -> parent #{p} not in radio contact ({d:.0} units xz)"));
        }
        if !w.sim.is_alive(p) {
            errs.push(format!("#{id} -> dead parent #{p}"));
        }
        let ph = if p == BASE_ID { Some(0) } else { mesh.hops(p) };
        if mesh.hops(id) != ph.map(|h| h + 1) {
            errs.push(format!("#{id} hops {:?} but parent #{p} hops {ph:?}", mesh.hops(id)));
        }
    }
    println!("[{sc}] tree: {}", shape.join(" "));
    if !errs.is_empty() {
        w.fail(sc, &format!("painlessMesh tree is not physical: {errs:?}"));
    }
}

#[test]
fn t8_tree_is_physical_alpine() {
    tree_is_physical(Preset::AlpineRidge);
}
#[test]
fn t8_tree_is_physical_canyon() {
    tree_is_physical(Preset::Canyon);
}

#[test]
fn t8_relay_kill_under_telemetry_alpine() {
    t8(Preset::AlpineRidge);
}
#[test]
fn t8_relay_kill_under_telemetry_canyon() {
    t8(Preset::Canyon);
}
#[test]
fn t8_relay_kill_under_telemetry_twin_valleys() {
    t8(Preset::TwinValleys);
}
#[test]
fn t8_relay_kill_under_telemetry_archipelago() {
    t8(Preset::Archipelago);
}
#[test]
fn t8_relay_kill_under_telemetry_foothills() {
    t8(Preset::Foothills);
}

// ---- T9 Phase 5: brain-managed telemetry airtime ---------------------------------------

fn t9(preset: Preset) {
    let cfg = SimConfig {
        channel: channel_on(T9_EXTERNAL_BUSY),
        telemetry: tlm_on(RateMode::Brain, Routing::Single, Variant::XiaoS3, T9_REQUESTED_HZ),
        ..SimConfig::for_preset(preset, 1, 20)
    };
    let sc = &label(&cfg, "T9 brain-managed");
    let mut w = World::new(cfg);
    deploy_or_fail(&mut w, sc);
    // 60 s steady: connectivity and brain traffic over all of it, GCS rates
    // and loss over its last 10 s.
    let mut connected = 0u32;
    let early = w.measure(T9_STEADY_MS - GCS_WINDOW_MS, |s| connected += u32::from(metrics::scouts_connected(s)));
    let mut m = GcsMeter::new(&w.sim);
    let late = w.measure(GCS_WINDOW_MS, |s| m.sample(s));
    m.report(&w.sim, sc);
    connected += m.connected_steps;
    let steps = early.steps + late.steps;
    let conn_frac = f64::from(connected) / f64::from(steps);
    let brain_rate = (early.frames + late.frames) as f64
        / ((early.live_drone_steps + late.live_drone_steps) as f64 * f64::from(STEP_MS) / 1000.0);
    let f = m.fleet();
    let routed = m.routed();
    let slow: Vec<String> = routed
        .iter()
        .filter(|&&id| m.gcs_hz(id) < T9_MIN_GCS_HZ)
        .map(|&id| format!("#{id} {:.2} Hz (req {:.1})", m.gcs_hz(id), row(&w.sim, id)[tlm::REQUESTED_HZ]))
        .collect();
    let intervals: Vec<String> =
        routed.iter().map(|&id| format!("{:.1}", row(&w.sim, id)[tlm::REQUESTED_HZ])).collect();
    println!(
        "[{sc}] RESULT scouts_connected {:.1} % of steps, brain {brain_rate:.3} frames/drone/s, fleet loss mean {:.1} % \
         (median {:.1} %), {} routed, brain-chosen Hz {intervals:?}, below {T9_MIN_GCS_HZ} Hz: {slow:?}",
        100.0 * conn_frac,
        f.mean,
        f.median,
        routed.len()
    );
    if conn_frac < T9_MIN_CONNECTED {
        w.fail(sc, &format!("scouts_connected {:.1} % of steps < {:.0} %", 100.0 * conn_frac, 100.0 * T9_MIN_CONNECTED));
    }
    if brain_rate > T9_MAX_BRAIN_RATE {
        w.fail(sc, &format!("brain traffic {brain_rate:.3} frames/drone/s > {T9_MAX_BRAIN_RATE}"));
    }
    if routed.is_empty() {
        w.fail(sc, "no drone routed in the painlessMesh tree");
    }
    if !slow.is_empty() {
        w.fail(sc, &format!("routed drones below {T9_MIN_GCS_HZ} Hz at GCS: {slow:?}"));
    }
    if f.mean > T9_MAX_FLEET_LOSS_PCT {
        w.fail(sc, &format!("fleet loss {:.1} % > {T9_MAX_FLEET_LOSS_PCT} %", f.mean));
    }

    // Relay kill still heals within the PRD limit with Phase 5 on.
    let Some((victim, d, _)) = pick_victim(&w.sim) else {
        w.fail(sc, "no live relay carrying a scout to kill");
    };
    println!("[{sc}] killing relay #{victim} (desc={} scouts={})", d.all, d.scouts);
    w.kill(&[victim]);
    let Some(t) = w.time_until(T9_HEAL_MS, metrics::scouts_connected) else {
        w.fail(sc, &format!("relay kill: scouts_connected not restored within {} s", secs(T9_HEAL_MS)));
    };
    if let Err(at) = w.holds_for(CONNECTED_HOLD_MS, metrics::scouts_connected) {
        w.fail(sc, &format!("relay kill: restored after {:.2} s but lost {:.2} s later", secs(t), secs(at)));
    }
    println!("[{sc}] RESULT relay kill healed in {:.2} s and held {} s", secs(t), secs(CONNECTED_HOLD_MS));
}

#[test]
fn t9_brain_managed_alpine() {
    t9(Preset::AlpineRidge);
}
#[test]
fn t9_brain_managed_canyon() {
    t9(Preset::Canyon);
}

// ---- T10 root death -------------------------------------------------------------------

#[test]
fn t10_root_death_no_bridge() {
    let cfg = cfg_on(Preset::AlpineRidge, 20, 0.0, tlm_on(RateMode::BackendBudget, Routing::Single, Variant::XiaoS3, 25.0));
    let sc = &label(&cfg, "T10 root death");
    let mut w = World::new(cfg);
    deploy_or_fail(&mut w, sc);
    let before = meter_run(&mut w, 2_000);
    let delivered_before: u32 = before.delivered.iter().sum();
    let gcs_before = summ(&w.sim, chan::GCS_PPS);
    let connected_before = (0..20u16).filter(|&id| tlm_routed(&w.sim, id)).count();
    println!(
        "[{sc}] before: {delivered_before} frames reached GCS in 2 s (gcs_pps {gcs_before:.1}), {connected_before} drones in the tree"
    );
    assert!(delivered_before > 0, "T10: no telemetry reached the GCS before the kill (precondition)");
    w.kill(&[BASE_ID]);
    let start = w.now_ms();
    let mut uplinks_after = 0u32;
    let mut sent_after = 0u32;
    let mut sink: Option<(u32, u16)> = None;
    let mut stat_zero_at: Option<u32> = None;
    while w.now_ms() - start < T10_SINK_LIMIT_MS.max(T10_STATS_DRAIN_MS + 1_000) {
        w.step();
        let t = w.now_ms() - start;
        for (kind, _, _) in tlm_records(&w.sim) {
            if kind == chatter::CHATTER_ROOT_UPLINK {
                uplinks_after += 1;
            } else {
                sent_after += 1;
            }
        }
        if summ(&w.sim, chan::GCS_PPS) == 0.0 {
            stat_zero_at.get_or_insert(t);
        } else {
            stat_zero_at = None;
        }
        if sink.is_none() {
            let sinks: Vec<u16> = metrics::with_role(&w.sim, Role::Sink).collect();
            if sinks.len() == 1 && sinks[0] != BASE_ID && w.sim.is_alive(sinks[0]) {
                sink = Some((t, sinks[0]));
            }
        }
    }
    let connected_after = (0..20u16).filter(|&id| tlm_routed(&w.sim, id)).count();
    println!(
        "[{sc}] RESULT after base death: {sent_after} packages still sent, {uplinks_after} reached GCS, gcs_pps {:.1}, \
         bridge_up {}, painless tree connected {connected_after} (no root election in painlessMesh), \
         acting sink {}, stats zero from {}",
        summ(&w.sim, chan::GCS_PPS),
        summ(&w.sim, chan::BRIDGE_UP),
        sink.map_or("none".into(), |(t, id)| format!("#{id} after {:.2} s", secs(t))),
        stat_zero_at.map_or("never".into(), |t| format!("{:.2} s", secs(t))),
    );
    assert_eq!(uplinks_after, 0, "T10: {uplinks_after} datagrams reached the GCS with the bridge dead");
    assert_eq!(summ(&w.sim, chan::BRIDGE_UP), 0.0, "T10: bridge_up still 1");
    assert!(stat_zero_at.is_some_and(|t| t <= T10_STATS_DRAIN_MS), "T10: gcs_pps not 0 within {} s", secs(T10_STATS_DRAIN_MS));
    for id in 1..20u16 {
        assert_eq!(row(&w.sim, id)[tlm::DELIVERED_HZ], 0.0, "T10: drone {id} delivered_hz not 0 without a bridge");
    }
    if sink.is_none_or(|(t, _)| t > T10_SINK_LIMIT_MS) {
        w.fail(sc, &format!("brain: no single acting sink elected within {} s", secs(T10_SINK_LIMIT_MS)));
    }
}

// ---- T12 byte accounting ----------------------------------------------------------

/// Every byte on air is booked once, to the node that sends it: the
/// per-drone `tx_all_bytes_per_s` sum to the fleet's `bytes_on_air`, the
/// fleet breakdown sums to it, and the per-drone brain / telemetry / uplink
/// figures sum to their fleet categories.
#[test]
fn t12_bytes_on_air_are_consistent() {
    let cfg = cfg_on(Preset::AlpineRidge, 20, 0.2, tlm_on(RateMode::Fixed(25.0), Routing::Single, Variant::XiaoS3, 25.0));
    let mut w = World::new(cfg);
    w.run(20_000);
    let (mut worst, mut ctl, mut acks, mut retry) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for _ in 0..200 {
        w.step();
        let ch = w.sim.channel_summary();
        let st = w.sim.telemetry_stats();
        let rows = st.as_chunks::<{ telemetry::TLM_STRIDE }>().0;
        let total = f64::from(ch[chan::BYTES_ON_AIR]);
        let sum_all: f64 = rows.iter().map(|r| f64::from(r[tlm::TX_ALL_BYTES_PER_S])).sum();
        let kinds: f64 = [
            chan::BYTES_BRAIN,
            chan::BYTES_TLM_DATA,
            chan::BYTES_ACK,
            chan::BYTES_MESH_CONTROL,
            chan::BYTES_UPLINK,
            chan::BYTES_BEACON,
        ]
        .iter()
        .map(|&k| f64::from(ch[k]))
        .sum();
        let brain: f64 = rows.iter().map(|r| f64::from(r[tlm::BRAIN_BYTES_PER_S])).sum();
        let ack: f64 = rows.iter().map(|r| f64::from(r[tlm::ACK_BYTES_PER_S])).sum();
        let tlm_data: f64 = rows.iter().map(|r| f64::from(r[tlm::TX_BYTES_PER_S] + r[tlm::RELAY_BYTES_PER_S])).sum();
        assert!(total > 0.0);
        let rel = |a: f64, b: f64| (a - b).abs() / b.max(1.0);
        worst = worst.max(rel(sum_all, total)).max(rel(kinds, total)).max(rel(brain, f64::from(ch[chan::BYTES_BRAIN])));
        worst = worst.max(rel(ack, f64::from(ch[chan::BYTES_ACK])));
        // Telemetry data + the root's uplink (in its relay figure).
        worst = worst.max(rel(tlm_data, f64::from(ch[chan::BYTES_TLM_DATA] + ch[chan::BYTES_UPLINK])));
        assert!(ch[chan::BYTES_RETRY] <= ch[chan::BYTES_ON_AIR]);
        assert!(ch[chan::FRAMES_ON_AIR] >= ch[chan::PPS_ON_AIR]);
        ctl += f64::from(ch[chan::BYTES_MESH_CONTROL]);
        acks += f64::from(ch[chan::BYTES_ACK]);
        retry += f64::from(ch[chan::BYTES_RETRY]);
    }
    let ch = w.sim.channel_summary();
    println!(
        "[T12 bytes] on air {:.0} B/s = brain {:.0} + tlm {:.0} + ack {:.0} + ctl {:.0} + uplink {:.0} + beacon {:.0}; retry {:.0}; \
         {:.0} frames/s; worst relative mismatch {worst:.2e}",
        ch[chan::BYTES_ON_AIR],
        ch[chan::BYTES_BRAIN],
        ch[chan::BYTES_TLM_DATA],
        ch[chan::BYTES_ACK],
        ch[chan::BYTES_MESH_CONTROL],
        ch[chan::BYTES_UPLINK],
        ch[chan::BYTES_BEACON],
        ch[chan::BYTES_RETRY],
        ch[chan::FRAMES_ON_AIR]
    );
    assert!(worst < 1e-3, "T12: byte stats disagree by {worst:.2e}");
    assert!(ctl > 0.0 && acks > 0.0 && retry > 0.0, "T12: ctl {ctl} ack {acks} retry {retry}");
}

// ---- T11 performance ---------------------------------------------------------------

fn step_ms(drones: usize) -> (f64, f64) {
    let cfg = cfg_on(
        Preset::AlpineRidge,
        drones,
        0.3,
        TelemetryConfig { chatter_every_frame: false, ..tlm_on(RateMode::Fixed(25.0), Routing::Single, Variant::Mixed, 25.0) },
    );
    let mut sim = Sim::new(cfg);
    for _ in 0..40 {
        sim.step(STEP_MS);
    }
    sim.command(Intent::MonitorFire, sim.config().ignition.0, sim.config().ignition.1);
    for _ in 0..600 {
        sim.step(STEP_MS);
    }
    let mut best = f64::INFINITY;
    let mut worst = 0.0f64;
    for _ in 0..3 {
        let t = std::time::Instant::now();
        for _ in 0..400 {
            let s = std::time::Instant::now();
            sim.step(STEP_MS);
            worst = worst.max(s.elapsed().as_secs_f64() * 1e3);
        }
        best = best.min(t.elapsed().as_secs_f64() * 1e3 / 400.0);
    }
    println!(
        "[T11 perf] {drones:>3} drones, channel + telemetry 25 Hz Mixed: {best:.3} ms/step (best of 3 x 400), worst step {worst:.3} ms, \
         pps {:.0}, live {}",
        summ(&sim, chan::PPS_ON_AIR),
        (0..drones as u16).filter(|&i| sim.is_alive(i)).count()
    );
    (best, worst)
}

#[test]
#[cfg_attr(debug_assertions, ignore = "timing; run with --release")]
fn t11_step_time_everything_on() {
    let (t20, _) = step_ms(20);
    let (t100, _) = step_ms(100);
    assert!(t20 < T11_BUDGET_20_MS, "T11: 20 drones {t20:.3} ms/step >= {T11_BUDGET_20_MS} ms");
    assert!(t100 < T11_BUDGET_100_MS, "T11: 100 drones {t100:.3} ms/step >= {T11_BUDGET_100_MS} ms");
}
