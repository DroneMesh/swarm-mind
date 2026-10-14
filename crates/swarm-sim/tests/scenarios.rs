//! PRD acceptance gate: the swarm scenarios, run natively and
//! deterministically against the real brains.
//!
//! Thresholds below are fixed by the PRD. Do not loosen them to make a test
//! pass; fix the brain instead.
//!
//! Conventions used by every scenario:
//! - Default `SimConfig` (20 drones including the base, seed 1), 50 ms steps.
//!   The unprefixed tests are Alpine Ridge (`SimConfig::default()`); every
//!   other map runs the same scenarios, with the same thresholds, from
//!   `SimConfig::for_preset(p, 1, 20)` as `<map>_<scenario>` tests.
//! - The fire target is always the config's ignition point, never a literal.
//! - "Deployed" = at least 2 scouts within 100 units (xz) of a burning cell
//!   and `metrics::scouts_connected`, held continuously for 10 s. The whole
//!   10 s window must complete within 120 s of the `MonitorFire` command.
//! - "Restored within N s" = `scouts_connected` first holds within N s of
//!   the kill and then holds on every step for the following 10 s, so a
//!   flicker does not count as repair.
//! - Traffic rates are frames per *live* drone per second of sim time.
//!
//! Run: `cargo test --release -p swarm-sim --test scenarios -- --nocapture`

mod harness;

use harness::{
    Desc, RoleWatch, World, burning_cells, deployed, dist_xz, fire_centroid, live_count,
    relays_by_descendants, scouts_near_fire, secs, top_relay,
};
use swarm_brain::{Intent, NO_ROUTE, OPERATOR, Role};
use swarm_sim::{BASE_ID, Preset, Sim, SimConfig, base_position, chatter, metrics, snapshot};

// ---- PRD thresholds (fixed) -------------------------------------------------

const DEPLOY_LIMIT_MS: u32 = 120_000;
const CONNECTED_HOLD_MS: u32 = 10_000;
const NEAR_FIRE: f32 = 100.0;
const MIN_SCOUTS_ON_FIRE: usize = 2;
const STEADY_WINDOW_MS: u32 = 20_000;
const STEADY_MAX_RATE: f64 = 2.0;

const SINGLE_KILL_LIMIT_MS: u32 = 5_000;
const SINGLE_KILL_REPEATS: usize = 3;
/// Time allowed to get back to "deployed" between successive kills.
const RESTABILISE_LIMIT_MS: u32 = 60_000;
const MULTI_KILL_LIMIT_MS: u32 = 15_000;
const PARTITION_LIMIT_MS: u32 = 30_000;

const BASE_OFFLINE_LIMIT_MS: u32 = 10_000;
const BASE_OFFLINE_WINDOW_MS: u32 = 30_000;
const BASE_OFFLINE_MAX_RATE: f64 = 3.0;

const NEW_INTENT_DELAY_MS: u32 = 1_000;
const NEW_INTENT_LIMIT_MS: u32 = 15_000;

const LOSS: f32 = 0.3;
const LOSS_WINDOW_MS: u32 = 60_000;
const LOSS_MIN_CONNECTED: f64 = 0.90;
const LOSS_MAX_RATE: f64 = 2.5;
const LOSS_MAX_ROLE_CHANGES: u32 = 2;

const DETERMINISM_MS: u32 = 60_000;

const IDLE_WINDOW_MS: u32 = 30_000;
const IDLE_MAX_RATE: f64 = 1.1;

const RTB_LIMIT_MS: u32 = 120_000;
const RTB_RADIUS: f32 = 60.0;
const RTB_MAX_AGL: f32 = 15.0;

// ---- shared steps -----------------------------------------------------------

/// Failure/print prefix: plain for Alpine Ridge (the historical names),
/// prefixed with the map otherwise.
fn scenario_label(cfg: &SimConfig, scenario: &str) -> String {
    match cfg.preset {
        Preset::AlpineRidge => scenario.to_string(),
        p => format!("{} {scenario}", p.name()),
    }
}

/// Command `MonitorFire` at the ignition point and wait for "deployed".
/// Returns the sim time (ms since the command) at which the 10 s window began.
fn deploy(w: &mut World, scenario: &str) -> u32 {
    let (fx, fz) = w.ignition();
    w.command(Intent::MonitorFire, fx, fz);
    let start = w.now_ms();
    match w.time_until_held(DEPLOY_LIMIT_MS, CONNECTED_HOLD_MS, deployed) {
        Some(t) => {
            println!(
                "[{scenario}] deployed: window began {:.2} s after command, complete at {:.2} s",
                secs(t),
                secs(w.now_ms() - start)
            );
            t
        }
        None => w.fail(
            scenario,
            &format!(
                "deploy: no {} s window of (>= {MIN_SCOUTS_ON_FIRE} scouts within {NEAR_FIRE} of fire \
                 && scouts_connected) within {} s",
                secs(CONNECTED_HOLD_MS),
                secs(DEPLOY_LIMIT_MS)
            ),
        ),
    }
}

/// Require `scouts_connected` within `limit_ms`, then for 10 s more.
/// Returns the recovery time in ms.
fn expect_restored(w: &mut World, scenario: &str, what: &str, limit_ms: u32) -> u32 {
    let Some(t) = w.time_until(limit_ms, metrics::scouts_connected) else {
        w.fail(scenario, &format!("{what}: scouts_connected not restored within {} s", secs(limit_ms)));
    };
    if let Err(at) = w.holds_for(CONNECTED_HOLD_MS, metrics::scouts_connected) {
        w.fail(
            scenario,
            &format!(
                "{what}: restored after {:.2} s but lost again {:.2} s later (needs to hold {} s)",
                secs(t),
                secs(at),
                secs(CONNECTED_HOLD_MS)
            ),
        );
    }
    println!("[{scenario}] {what}: restored in {:.2} s and held {} s", secs(t), secs(CONNECTED_HOLD_MS));
    t
}

fn describe(victims: &[(u16, Desc)], sim: &Sim) -> String {
    victims
        .iter()
        .map(|&(id, d)| {
            let b = sim.brain(id).expect("id");
            format!("#{id}({:?} hops={} desc={} scouts={})", b.role(), b.hops(), d.all, d.scouts)
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// Kill the relay carrying the most traffic, requiring that it actually
/// carries at least one scout (otherwise the chain isn't built from relays).
fn kill_top_relay(w: &mut World, scenario: &str) -> u16 {
    let Some((id, d)) = top_relay(&w.sim) else {
        w.fail(scenario, "no live relay to kill: the chain to the fire uses no relays");
    };
    if d.scouts == 0 {
        w.fail(scenario, &format!("top relay #{id} carries no scouts (desc={})", d.all));
    }
    println!("[{scenario}] killing top relay {}", describe(&[(id, d)], &w.sim));
    w.kill(&[id]);
    id
}

// ---- harness self-checks (must pass regardless of brain quality) ------------

#[test]
fn harness_self_check() {
    let mut w = World::default_demo();
    assert_eq!(w.time_until(1_000, |_| true), Some(0), "already-true predicate");
    let t0 = w.now_ms();
    assert_eq!(w.time_until(1_000, |s| s.now_ms() >= t0 + 500), Some(500));
    assert_eq!(w.time_until(1_000, |_| false), None);
    assert_eq!(w.now_ms() - t0, 1_500, "time_until(false) must run exactly max_ms");

    let t0 = w.now_ms();
    assert_eq!(w.time_until_held(5_000, 1_000, |_| true), Some(0));
    assert_eq!(w.now_ms() - t0, 1_000, "held window is exactly hold_ms");
    assert_eq!(w.time_until_held(2_000, 1_000, |s| s.now_ms() >= t0 + 1_500), Some(500));
    assert_eq!(w.time_until_held(1_000, 1_000, |_| false), None);

    let traffic = w.measure(2_000, |_| {});
    assert_eq!(traffic.steps, 40);
    assert_eq!(traffic.live_drone_steps, 40 * 20);

    // Landed ring: every drone should be routed to the base by now, and the
    // descendant walk must agree with the ground-truth judge.
    w.run(6_000);
    let desc = harness::descendants(&w.sim);
    let routed = metrics::routed_count(&w.sim);
    assert_eq!(desc[usize::from(BASE_ID)].all as usize, routed, "{}", w.diagnostics());
    assert!(metrics::route_ok(&w.sim, BASE_ID), "the live base is its own route");

    assert_eq!(scouts_near_fire(&w.sim, NEAR_FIRE), 0, "no scouts while landed");

    // Fire geometry, before any spread: one burning cell under the ignition point.
    let fresh = World::default_demo();
    let cells = burning_cells(&fresh.sim);
    assert_eq!(cells.len(), 1, "exactly the ignition cell burns at t=0");
    let cell = fresh.sim.config().extent / fresh.sim.fire().size() as f32;
    let (cx, cz) = cells[0];
    let (ix, iz) = fresh.ignition();
    assert!((cx - ix).abs() <= cell / 2.0 && (cz - iz).abs() <= cell / 2.0, "ignition cell centre ({cx}, {cz})");
    assert_eq!(fire_centroid(&fresh.sim), Some((cx, cz)));
}

// ---- S1 ---------------------------------------------------------------------

#[test]
fn s1_deploy_to_fire() {
    s1(SimConfig::default());
}

fn s1(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S1 deploy");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    let mut connected_steps = 0u32;
    let traffic = w.measure(STEADY_WINDOW_MS, |s| connected_steps += u32::from(metrics::scouts_connected(s)));
    println!("[{sc}] steady: {traffic}; connected on {connected_steps}/{} steps", traffic.steps);
    if traffic.per_drone_per_s() > STEADY_MAX_RATE {
        w.fail(sc, &format!("steady traffic {traffic} exceeds {STEADY_MAX_RATE}"));
    }
}

// ---- S2 ---------------------------------------------------------------------

#[test]
fn s2_single_relay_kill_repeated() {
    s2(SimConfig::default());
}

fn s2(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S2 single relay kill");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    for round in 1..=SINGLE_KILL_REPEATS {
        if round > 1 {
            let Some(t) = w.time_until_held(RESTABILISE_LIMIT_MS, CONNECTED_HOLD_MS, deployed) else {
                w.fail(sc, &format!("round {round}: could not re-stabilise (deployed) within {} s", secs(RESTABILISE_LIMIT_MS)));
            };
            println!("[{sc}] round {round}: re-stabilised, window began after {:.2} s", secs(t));
        }
        kill_top_relay(&mut w, sc);
        expect_restored(&mut w, sc, &format!("round {round}"), SINGLE_KILL_LIMIT_MS);
    }
}

// ---- S3 ---------------------------------------------------------------------

#[test]
fn s3_multiple_simultaneous_kills() {
    s3(SimConfig::default());
}

fn s3(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S3 multiple kills");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    let relays = relays_by_descendants(&w.sim);
    if relays.len() < 2 {
        w.fail(sc, &format!("need >= 2 live relays to kill, have {}", relays.len()));
    }
    let mut victims: Vec<(u16, Desc)> = relays.iter().copied().take(3).collect();
    if victims.len() < 3 {
        // Fill with the busiest other non-sink drone.
        let desc = harness::descendants(&w.sim);
        let extra = (0..w.sim.drone_count() as u16)
            .filter(|&id| w.sim.is_alive(id) && id != BASE_ID)
            .filter(|&id| w.sim.brain(id).is_some_and(|b| b.role() != Role::Sink))
            .filter(|&id| victims.iter().all(|v| v.0 != id))
            .max_by_key(|&id| (desc[usize::from(id)].all, std::cmp::Reverse(id)))
            .expect("plenty of drones");
        victims.push((extra, desc[usize::from(extra)]));
    }
    println!("[{sc}] killing {}", describe(&victims, &w.sim));
    let ids: Vec<u16> = victims.iter().map(|v| v.0).collect();
    w.kill(&ids);
    expect_restored(&mut w, sc, "3 simultaneous kills", MULTI_KILL_LIMIT_MS);
}

// ---- S4 ---------------------------------------------------------------------

#[test]
fn s4_partition_then_merge() {
    s4(SimConfig::default());
}

fn s4(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S4 partition/merge");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);

    // Hop levels of the relays that actually carry scouts.
    let mut levels: Vec<u8> = relays_by_descendants(&w.sim)
        .iter()
        .filter(|(_, d)| d.scouts > 0)
        .map(|&(id, _)| w.sim.brain(id).expect("id").hops())
        .filter(|&h| h != NO_ROUTE)
        .collect();
    levels.sort_unstable();
    levels.dedup();
    if levels.is_empty() {
        w.fail(sc, "no relay carries a scout: nothing to partition");
    }
    let mid = levels.len() / 2;
    let middle: Vec<u8> = if levels.len() % 2 == 1 { vec![levels[mid]] } else { vec![levels[mid - 1], levels[mid]] };
    let desc = harness::descendants(&w.sim);
    let victims: Vec<(u16, Desc)> = metrics::with_role(&w.sim, Role::Relay)
        .filter(|&id| middle.contains(&w.sim.brain(id).expect("id").hops()))
        .map(|id| (id, desc[usize::from(id)]))
        .collect();
    println!("[{sc}] chain relay levels {levels:?}, middle {middle:?}; killing {}", describe(&victims, &w.sim));
    let ids: Vec<u16> = victims.iter().map(|v| v.0).collect();
    w.kill(&ids);
    if metrics::scouts_connected(&w.sim) {
        w.fail(sc, "killing the middle relay level(s) did not cut the scouts off (no partition formed)");
    }
    expect_restored(&mut w, sc, "partition merge", PARTITION_LIMIT_MS);
}

// ---- S5 ---------------------------------------------------------------------

#[test]
fn s5_base_offline() {
    s5(SimConfig::default());
}

fn s5(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S5 base offline");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    w.kill(&[BASE_ID]);
    let elected = |s: &Sim| {
        metrics::with_role(s, Role::Sink).count() == 1
            && metrics::with_role(s, Role::Scout).any(|id| metrics::route_ok(s, id))
    };
    let Some(t) = w.time_until(BASE_OFFLINE_LIMIT_MS, elected) else {
        w.fail(
            sc,
            &format!("within {} s: need exactly one live Sink and a routed scout", secs(BASE_OFFLINE_LIMIT_MS)),
        );
    };
    let sink = metrics::with_role(&w.sim, Role::Sink).next().expect("one sink");
    println!("[{sc}] drone {sink} became sink after {:.2} s", secs(t));
    let mut sink_counts = [0u32; 3]; // steps with 0, 1, 2+ sinks
    let traffic = w.measure(BASE_OFFLINE_WINDOW_MS, |s| {
        sink_counts[metrics::with_role(s, Role::Sink).count().min(2)] += 1;
    });
    println!("[{sc}] window: {traffic}; steps with 0/1/2+ sinks: {sink_counts:?}");
    if traffic.per_drone_per_s() > BASE_OFFLINE_MAX_RATE {
        w.fail(sc, &format!("traffic {traffic} exceeds {BASE_OFFLINE_MAX_RATE}"));
    }
}

// ---- S6 ---------------------------------------------------------------------

#[test]
fn s6_new_intent_mid_repair() {
    s6(SimConfig::default());
}

fn s6(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S6 new intent mid-repair");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    kill_top_relay(&mut w, sc);
    w.run(NEW_INTENT_DELAY_MS);
    let (cx, cz) = fire_centroid(&w.sim).unwrap_or(w.ignition());
    w.command(Intent::MapPerimeter, cx, cz);
    let settled = |s: &Sim| {
        (0..s.drone_count() as u16)
            .filter(|&id| s.is_alive(id))
            .all(|id| s.brain(id).is_some_and(|b| b.intent() == Intent::MapPerimeter))
            && metrics::scouts_connected(s)
    };
    let Some(t) = w.time_until(NEW_INTENT_LIMIT_MS, settled) else {
        w.fail(
            sc,
            &format!(
                "within {} s of MapPerimeter: every live drone must carry it and scouts_connected hold",
                secs(NEW_INTENT_LIMIT_MS)
            ),
        );
    };
    if let Err(at) = w.holds_for(CONNECTED_HOLD_MS, settled) {
        w.fail(sc, &format!("settled after {:.2} s but lost it {:.2} s later", secs(t), secs(at)));
    }
    println!("[{sc}] MapPerimeter adopted and connected {:.2} s after command, held {} s", secs(t), secs(CONNECTED_HOLD_MS));
}

// ---- S7 ---------------------------------------------------------------------

#[test]
fn s7_packet_loss() {
    s7(SimConfig::default());
}

fn s7(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S7 packet loss");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    w.sim.set_loss(LOSS);
    w.note(format!("t={:.2}s LOSS {LOSS}", secs(w.now_ms())));
    let mut roles = RoleWatch::new(&w.sim);
    let mut connected = 0u32;
    let traffic = w.measure(LOSS_WINDOW_MS, |s| {
        connected += u32::from(metrics::scouts_connected(s));
        roles.sample(s);
    });
    let share = f64::from(connected) / f64::from(traffic.steps);
    let worst = roles.changes.iter().copied().enumerate().max_by_key(|&(i, c)| (c, std::cmp::Reverse(i))).unwrap_or((0, 0));
    println!(
        "[{sc}] connected {:.1}% of steps; {traffic}; role changes per drone {:?} (worst #{} = {})",
        share * 100.0,
        roles.changes,
        worst.0,
        worst.1
    );
    let mut errs = Vec::new();
    if share < LOSS_MIN_CONNECTED {
        errs.push(format!("connected on {:.1}% of steps (< {:.0}%)", share * 100.0, LOSS_MIN_CONNECTED * 100.0));
    }
    if traffic.per_drone_per_s() > LOSS_MAX_RATE {
        errs.push(format!("traffic {traffic} exceeds {LOSS_MAX_RATE}"));
    }
    if worst.1 > LOSS_MAX_ROLE_CHANGES {
        errs.push(format!("drone {} changed role {} times (> {LOSS_MAX_ROLE_CHANGES}): thrash", worst.0, worst.1));
    }
    if !errs.is_empty() {
        w.fail(sc, &errs.join("; "));
    }
}

// ---- S8 ---------------------------------------------------------------------

/// A scripted run touching every source of randomness and every input.
/// Returns a digest of every step's snapshot plus the final snapshot and fire.
fn scripted(seed: u64) -> (u64, Vec<u32>, Vec<u8>) {
    let mut w = World::new(SimConfig { seed, ..SimConfig::default() });
    let (fx, fz) = w.ignition();
    let mut digest = 0xcbf2_9ce4_8422_2325u64; // FNV-1a
    let mut fold = |s: &Sim| {
        for v in s.snapshot() {
            for b in v.to_bits().to_le_bytes() {
                digest = (digest ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
            }
        }
    };
    w.command(Intent::MonitorFire, fx, fz);
    w.measure(20_000, &mut fold);
    w.sim.set_loss(0.2);
    w.measure(10_000, &mut fold);
    w.kill(&[7]);
    w.measure(10_000, &mut fold);
    w.sim.set_loss(0.0);
    w.command(Intent::MapPerimeter, fx, fz);
    w.measure(DETERMINISM_MS - 40_000, &mut fold);
    assert_eq!(w.now_ms(), DETERMINISM_MS);
    let snap = w.sim.snapshot().iter().map(|v| v.to_bits()).collect();
    (digest, snap, w.sim.fire().cells().to_vec())
}

#[test]
fn s8_determinism() {
    let a = scripted(1);
    let b = scripted(1);
    assert_eq!(a.1.len(), b.1.len(), "final snapshot lengths differ");
    if let Some(i) = a.1.iter().zip(&b.1).position(|(x, y)| x != y) {
        panic!("S8: final snapshots differ first at index {i}: {:?} vs {:?}", f32::from_bits(a.1[i]), f32::from_bits(b.1[i]));
    }
    assert!(a.2 == b.2, "S8: fire grids differ");
    assert_eq!(a.0, b.0, "S8: per-step snapshot digests differ (diverged mid-run)");
    // Guard against a vacuous comparison: the seed must matter.
    let c = scripted(2);
    assert_ne!(a.0, c.0, "S8: seeds 1 and 2 gave identical runs; the digest is not sensitive");
    println!("[S8 determinism] identical over {} s (digest {:016x}, {} floats)", secs(DETERMINISM_MS), a.0, a.1.len());
}

// ---- S9 ---------------------------------------------------------------------

#[test]
fn s9_idle_efficiency() {
    s9(SimConfig::default());
}

fn s9(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S9 idle");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    let traffic = w.measure(IDLE_WINDOW_MS, |_| {});
    println!("[{sc}] {traffic}");
    if traffic.per_drone_per_s() > IDLE_MAX_RATE || traffic.dropped > 0 {
        w.fail(sc, &format!("idle traffic {traffic}: need <= {IDLE_MAX_RATE} frames/drone/s and no drops"));
    }
}

// ---- S10 --------------------------------------------------------------------

#[test]
fn s10_return_to_base() {
    s10(SimConfig::default());
}

fn s10(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S10 return to base");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    let base = base_position(w.sim.config(), w.sim.terrain());
    w.command(Intent::ReturnToBase, base.x, base.z);
    let home = |s: &Sim| {
        (0..s.drone_count() as u16).filter(|&id| s.is_alive(id)).all(|id| {
            let p = s.position(id).expect("id");
            let agl = p.y - s.terrain().height_at(p.x, p.z);
            dist_xz(p, base) <= RTB_RADIUS && agl < RTB_MAX_AGL
        })
    };
    let Some(t) = w.time_until(RTB_LIMIT_MS, home) else {
        let away: Vec<String> = (0..w.sim.drone_count() as u16)
            .filter(|&id| w.sim.is_alive(id))
            .filter_map(|id| {
                let p = w.sim.position(id).expect("id");
                let agl = p.y - w.sim.terrain().height_at(p.x, p.z);
                let d = dist_xz(p, base);
                (d > RTB_RADIUS || agl >= RTB_MAX_AGL).then(|| format!("#{id} d={d:.0} agl={agl:.1}"))
            })
            .collect();
        w.fail(
            sc,
            &format!(
                "within {} s all {} live drones must be within {RTB_RADIUS} (xz) of base and below {RTB_MAX_AGL} AGL; not home: {}",
                secs(RTB_LIMIT_MS),
                live_count(&w.sim),
                away.join(", ")
            ),
        );
    };
    println!("[{sc}] all drones home after {:.2} s", secs(t));
}

// ---- map sanity (S0) ---------------------------------------------------------

/// Before the swarm is judged on a map, check the map itself: the fire
/// ignites in exactly one cell under the configured point, the base sits on
/// the map at its preset position, and everything is routed while landed.
/// A failure here is a map bug, not a brain bug.
fn s0(cfg: SimConfig) {
    let label = scenario_label(&cfg, "S0 map sanity");
    let sc: &str = &label;
    let preset = cfg.preset;
    let mut w = World::new(cfg);
    assert_eq!(w.sim.preset(), preset, "[{sc}] Sim::preset");
    let half = w.sim.config().extent * 0.5;
    let (ix, iz) = w.ignition();
    if !(ix.abs() < half && iz.abs() < half) {
        w.fail(sc, &format!("ignition ({ix}, {iz}) is off the map (half extent {half})"));
    }
    let cells = burning_cells(&w.sim);
    let cell = w.sim.config().extent / w.sim.fire().size() as f32;
    match cells.as_slice() {
        [(cx, cz)] if (cx - ix).abs() <= cell / 2.0 && (cz - iz).abs() <= cell / 2.0 => {}
        _ => w.fail(sc, &format!("expected exactly the ignition cell burning at t=0, got {cells:?}")),
    }
    let base = base_position(w.sim.config(), w.sim.terrain());
    let actual = w.sim.position(BASE_ID).expect("base");
    if dist_xz(base, actual) > 1.0 || !(base.x.abs() < half && base.z.abs() < half) {
        w.fail(sc, &format!("base at {actual:?}, base_position says {base:?}"));
    }
    w.run(6_000);
    let routed = metrics::routed_count(&w.sim);
    if routed + 1 != live_count(&w.sim) {
        w.fail(
            sc,
            &format!("landed swarm: only {routed}/{} drones routed to the base after 6 s", live_count(&w.sim) - 1),
        );
    }
    println!("[{sc}] ignition ({ix:.0}, {iz:.0}), base ({:.0}, {:.0}, {:.0}), {routed} routed", base.x, base.y, base.z);
}

// ---- chatter feed (C1, C2) ---------------------------------------------------

/// Chatter must be a faithful, complete decode of the air: over a steady
/// 20 s window after deploy, one record per frame counted in `stats()`,
/// every one decodable and stamped with the step time, in the same order
/// and with the same src/dst as the snapshot's transmission list.
fn c1(cfg: SimConfig) {
    let label = scenario_label(&cfg, "C1 chatter");
    let sc: &str = &label;
    let mut w = World::new(cfg);
    deploy(&mut w, sc);
    let stride = chatter::CHATTER_STRIDE;
    let (mut records, mut frames, mut heartbeats, mut steps) = (0u64, 0u64, 0u64, 0u32);
    let mut errs: Vec<String> = Vec::new();
    w.measure(STEADY_WINDOW_MS, |s| {
        steps += 1;
        let c = s.chatter();
        let st = s.stats();
        let now = s.now_ms();
        frames += u64::from(st.frames);
        if c.len() % stride != 0 {
            errs.push(format!("t={now} chatter len {} not a multiple of {stride}", c.len()));
            return;
        }
        let n = c.len() / stride;
        records += n as u64;
        if n != st.frames as usize {
            errs.push(format!("t={now} {n} chatter records vs {} frames sent", st.frames));
        }
        let snap = s.snapshot();
        let (drones, links, sent) = (snap[3] as usize, snap[4] as usize, snap[5] as usize);
        let pulses = snapshot::HEADER_LEN + drones * snapshot::DRONE_STRIDE + links * snapshot::LINK_STRIDE;
        if sent != n {
            errs.push(format!("t={now} snapshot lists {sent} transmissions, chatter {n}"));
        }
        for (i, r) in c.chunks_exact(stride).enumerate() {
            let kind = r[chatter::KIND];
            if kind == chatter::CHATTER_UNDECODABLE {
                errs.push(format!("t={now} record {i} undecodable (src {})", r[chatter::SRC]));
            }
            heartbeats += u64::from(kind == chatter::CHATTER_HEARTBEAT);
            if r[chatter::T_MS] != now as f32 {
                errs.push(format!("t={now} record {i} stamped {}", r[chatter::T_MS]));
            }
            if r[chatter::SRC] == f32::from(OPERATOR) {
                errs.push(format!("t={now} operator record in a window with no commands"));
            }
            if i < sent {
                let p = &snap[pulses + i * snapshot::PULSE_STRIDE..];
                if (p[0], p[1]) != (r[chatter::SRC], r[chatter::DST]) {
                    errs.push(format!(
                        "t={now} record {i} src/dst ({}, {}) != snapshot pulse ({}, {})",
                        r[chatter::SRC],
                        r[chatter::DST],
                        p[0],
                        p[1]
                    ));
                }
            }
        }
    });
    println!("[{sc}] {records} records / {frames} frames over {steps} steps, {heartbeats} heartbeats");
    if records == 0 || heartbeats == 0 {
        errs.push(format!("no traffic decoded ({records} records, {heartbeats} heartbeats)"));
    }
    if !errs.is_empty() {
        let n = errs.len();
        errs.truncate(10);
        w.fail(sc, &format!("{n} chatter mismatches, first: {}", errs.join("; ")));
    }
}

/// An operator command shows up exactly once as an intent record from the
/// console to the base, and the base then floods it as a mission carrying
/// the same seq, intent and target.
#[test]
fn c2_chatter_operator_intent() {
    const SC: &str = "C2 chatter intent";
    let mut w = World::default_demo();
    w.run(2_000);
    let (fx, fz) = w.ignition();
    w.command(Intent::MonitorFire, fx, fz);
    let stride = chatter::CHATTER_STRIDE;
    let mut intents: Vec<Vec<f32>> = Vec::new();
    let mut mission: Option<Vec<f32>> = None;
    for _ in 0..40 {
        w.step();
        for r in w.sim.chatter().chunks_exact(stride) {
            let kind = r[chatter::KIND];
            if kind == chatter::CHATTER_INTENT {
                intents.push(r.to_vec());
            } else if kind == chatter::CHATTER_MISSION && mission.is_none() {
                mission = Some(r.to_vec());
            }
        }
    }
    let [intent] = intents.as_slice() else {
        w.fail(SC, &format!("expected exactly one intent record, got {}", intents.len()));
    };
    let f = chatter::F0;
    if intent[chatter::SRC] != f32::from(OPERATOR) || intent[chatter::DST] != f32::from(BASE_ID) {
        w.fail(SC, &format!("intent record src/dst {intent:?}"));
    }
    if intent[f + 1] != f32::from(Intent::MonitorFire as u8)
        || (intent[f + 2] - fx).abs() > 1e-3
        || (intent[f + 3] - fz).abs() > 1e-3
        || !intent[f + 4].is_nan()
        || !intent[f + 5].is_nan()
    {
        w.fail(SC, &format!("intent fields {intent:?}"));
    }
    let Some(m) = mission else { w.fail(SC, "no mission flood within 2 s of the command") };
    if m[f..f + 4] != intent[f..f + 4] || !m[f + 4].is_finite() || !m[f + 5].is_finite() {
        w.fail(SC, &format!("mission {m:?} does not carry intent {intent:?}"));
    }
    println!("[{SC}] intent seq {} at t={} ms, first mission from #{} at t={} ms", intent[f], intent[0], m[1], m[0]);
}

#[test]
fn alpine_s0_map_sanity() {
    s0(SimConfig::default());
}

#[test]
fn c1_chatter_matches_air() {
    c1(SimConfig::default());
}

// ---- every other map -----------------------------------------------------------

/// One `#[test]` per map x scenario so a failure names both. Alpine Ridge is
/// covered by the unprefixed tests above (its `for_preset` is the default).
macro_rules! preset_tests {
    ($($preset:ident => { $($name:ident = $runner:ident;)* })*) => {
        $($(
            #[test]
            fn $name() {
                $runner(SimConfig::for_preset(Preset::$preset, 1, 20));
            }
        )*)*
    };
}

preset_tests! {
    Canyon => {
        canyon_s0_map_sanity = s0;
        canyon_s1_deploy = s1;
        canyon_s2_single_relay_kill = s2;
        canyon_s3_multiple_kills = s3;
        canyon_s4_partition_merge = s4;
        canyon_s5_base_offline = s5;
        canyon_s6_new_intent_mid_repair = s6;
        canyon_s7_packet_loss = s7;
        canyon_s9_idle = s9;
        canyon_s10_return_to_base = s10;
        canyon_c1_chatter = c1;
    }
    TwinValleys => {
        twin_valleys_s0_map_sanity = s0;
        twin_valleys_s1_deploy = s1;
        twin_valleys_s2_single_relay_kill = s2;
        twin_valleys_s3_multiple_kills = s3;
        twin_valleys_s4_partition_merge = s4;
        twin_valleys_s5_base_offline = s5;
        twin_valleys_s6_new_intent_mid_repair = s6;
        twin_valleys_s7_packet_loss = s7;
        twin_valleys_s9_idle = s9;
        twin_valleys_s10_return_to_base = s10;
        twin_valleys_c1_chatter = c1;
    }
    Archipelago => {
        archipelago_s0_map_sanity = s0;
        archipelago_s1_deploy = s1;
        archipelago_s2_single_relay_kill = s2;
        archipelago_s3_multiple_kills = s3;
        archipelago_s4_partition_merge = s4;
        archipelago_s5_base_offline = s5;
        archipelago_s6_new_intent_mid_repair = s6;
        archipelago_s7_packet_loss = s7;
        archipelago_s9_idle = s9;
        archipelago_s10_return_to_base = s10;
        archipelago_c1_chatter = c1;
    }
    Foothills => {
        foothills_s0_map_sanity = s0;
        foothills_s1_deploy = s1;
        foothills_s2_single_relay_kill = s2;
        foothills_s3_multiple_kills = s3;
        foothills_s4_partition_merge = s4;
        foothills_s5_base_offline = s5;
        foothills_s6_new_intent_mid_repair = s6;
        foothills_s7_packet_loss = s7;
        foothills_s9_idle = s9;
        foothills_s10_return_to_base = s10;
        foothills_c1_chatter = c1;
    }
}

/// Guard against a map silently dropping out of the matrix above.
#[test]
fn preset_matrix_covers_every_map() {
    assert_eq!(Preset::ALL.len(), 5, "a preset was added: add its row to preset_tests!");
}
