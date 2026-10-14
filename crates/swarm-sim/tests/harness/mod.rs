//! Shared scenario harness: stepping, timing, traffic accounting, chain
//! analysis and failure diagnostics.
//!
//! Everything here uses only the public `swarm_sim` API and the
//! ground-truth judges in `swarm_sim::metrics`. The harness never steers the
//! swarm; it only watches it, so a failing scenario is the brain's fault, not
//! the harness's.

#![allow(dead_code)] // Each test binary uses a different subset.

use std::fmt::Write as _;

use swarm_brain::{Intent, NO_ROUTE, Role, Vec3};
use swarm_sim::{BASE_ID, Preset, Sim, SimConfig, fire, metrics};

/// The sim's tick, fixed by the world facts.
pub const STEP_MS: u32 = 50;
/// Longest parent chain worth following; anything longer is a loop.
const MAX_CHAIN: usize = 64;
/// Trace a one-line summary this often, so failures show how we got there.
const TRACE_EVERY_MS: u32 = 5_000;
/// Keep only the most recent trace lines.
const TRACE_KEEP: usize = 80;

/// Radio traffic over a window of steps.
#[derive(Clone, Copy, Debug, Default)]
pub struct Traffic {
    pub steps: u32,
    pub frames: u64,
    pub bytes: u64,
    pub dropped: u64,
    /// Sum over steps of live drones, so averages respect kills mid-window.
    pub live_drone_steps: u64,
}

impl Traffic {
    /// Average frames per live drone per second of sim time.
    pub fn per_drone_per_s(&self) -> f64 {
        if self.live_drone_steps == 0 {
            return 0.0;
        }
        let drone_seconds = self.live_drone_steps as f64 * f64::from(STEP_MS) / 1000.0;
        self.frames as f64 / drone_seconds
    }

    pub fn seconds(&self) -> f64 {
        f64::from(self.steps) * f64::from(STEP_MS) / 1000.0
    }
}

impl std::fmt::Display for Traffic {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:.3} frames/drone/s over {:.1} s ({} frames, {} bytes, {} dropped)",
            self.per_drone_per_s(),
            self.seconds(),
            self.frames,
            self.bytes,
            self.dropped
        )
    }
}

/// A sim plus the bookkeeping the scenarios need: a running traffic meter
/// and a coarse trace of how the swarm evolved.
pub struct World {
    pub sim: Sim,
    meter: Traffic,
    trace: Vec<String>,
    next_trace_ms: u32,
}

impl World {
    pub fn new(cfg: SimConfig) -> Self {
        Self { sim: Sim::new(cfg), meter: Traffic::default(), trace: Vec::new(), next_trace_ms: 0 }
    }

    /// The demo default: 20 drones including the base, seed 1.
    pub fn default_demo() -> Self {
        Self::new(SimConfig::default())
    }

    /// The standard setup of a map: 20 drones including the base, seed 1.
    pub fn for_preset(preset: Preset) -> Self {
        Self::new(SimConfig::for_preset(preset, 1, 20))
    }

    pub fn now_ms(&self) -> u32 {
        self.sim.now_ms()
    }

    /// Where this world's fire ignites, as `(x, z)`. Taken from the config so
    /// every preset's scenarios aim at its own fire.
    pub fn ignition(&self) -> (f32, f32) {
        self.sim.config().ignition
    }

    /// One 50 ms step, with traffic accounting and tracing.
    pub fn step(&mut self) {
        self.sim.step(STEP_MS);
        let s = self.sim.stats();
        self.meter.steps += 1;
        self.meter.frames += u64::from(s.frames);
        self.meter.bytes += u64::from(s.bytes);
        self.meter.dropped += u64::from(s.dropped);
        self.meter.live_drone_steps += live_count(&self.sim) as u64;
        if self.sim.now_ms() >= self.next_trace_ms {
            self.next_trace_ms = self.sim.now_ms() + TRACE_EVERY_MS;
            let line = summary_line(&self.sim);
            self.note(line);
        }
    }

    /// Step for `ms` of sim time.
    pub fn run(&mut self, ms: u32) {
        for _ in 0..ms / STEP_MS {
            self.step();
        }
    }

    /// Step until `pred` holds, for at most `max_ms`. Returns the elapsed
    /// sim time when it first held (0 if it already held), or `None`.
    pub fn time_until(&mut self, max_ms: u32, mut pred: impl FnMut(&Sim) -> bool) -> Option<u32> {
        let start = self.now_ms();
        if pred(&self.sim) {
            return Some(0);
        }
        while self.now_ms() - start < max_ms {
            self.step();
            if pred(&self.sim) {
                return Some(self.now_ms() - start);
            }
        }
        None
    }

    /// Step for `ms`, requiring `pred` after every step. On failure returns
    /// the elapsed ms at the first step where it did not hold.
    pub fn holds_for(&mut self, ms: u32, mut pred: impl FnMut(&Sim) -> bool) -> Result<(), u32> {
        let start = self.now_ms();
        while self.now_ms() - start < ms {
            self.step();
            if !pred(&self.sim) {
                return Err(self.now_ms() - start);
            }
        }
        Ok(())
    }

    /// Wait at most `max_ms` for `pred` to have held continuously for
    /// `hold_ms` (every step sample, both ends included). The whole window
    /// must finish inside `max_ms`. Returns the elapsed ms at which the
    /// window *started*.
    pub fn time_until_held(&mut self, max_ms: u32, hold_ms: u32, mut pred: impl FnMut(&Sim) -> bool) -> Option<u32> {
        let start = self.now_ms();
        let mut streak: Option<u32> = pred(&self.sim).then_some(start);
        loop {
            if let Some(t0) = streak
                && self.now_ms() - t0 >= hold_ms
            {
                return Some(t0 - start);
            }
            if self.now_ms() - start >= max_ms {
                return None;
            }
            self.step();
            if pred(&self.sim) {
                streak.get_or_insert(self.now_ms());
            } else {
                streak = None;
            }
        }
    }

    /// Step for `ms` and return the traffic over just that window, calling
    /// `each` after every step.
    pub fn measure(&mut self, ms: u32, mut each: impl FnMut(&Sim)) -> Traffic {
        let before = self.meter;
        for _ in 0..ms / STEP_MS {
            self.step();
            each(&self.sim);
        }
        let after = self.meter;
        Traffic {
            steps: after.steps - before.steps,
            frames: after.frames - before.frames,
            bytes: after.bytes - before.bytes,
            dropped: after.dropped - before.dropped,
            live_drone_steps: after.live_drone_steps - before.live_drone_steps,
        }
    }

    /// Kill drones and log it.
    pub fn kill(&mut self, ids: &[u16]) {
        for &id in ids {
            let role = self.sim.brain(id).map(|b| b.role());
            assert!(self.sim.kill(id), "harness: drone {id} was already dead");
            self.note(format!("t={:.2}s KILL drone {id} ({role:?})", secs(self.now_ms())));
        }
    }

    /// Issue an operator command and log it. Panics if the base rejected it,
    /// because every scenario issues commands while the base is alive.
    pub fn command(&mut self, intent: Intent, x: f32, z: f32) {
        let ok = self.sim.command(intent, x, z);
        self.note(format!("t={:.2}s COMMAND {intent:?} at ({x:.0}, {z:.0}) accepted={ok}", secs(self.now_ms())));
        assert!(ok, "harness: command {intent:?} rejected (base dead or console full)");
    }

    pub fn note(&mut self, line: String) {
        if self.trace.len() >= TRACE_KEEP {
            self.trace.remove(0);
        }
        self.trace.push(line);
    }

    /// Fail the scenario with the full diagnostic dump.
    #[track_caller]
    pub fn fail(&self, scenario: &str, why: &str) -> ! {
        panic!("\n=== {scenario} FAILED: {why}\n{}", self.diagnostics());
    }

    /// Per-drone table, fire state and the recent trace.
    pub fn diagnostics(&self) -> String {
        let sim = &self.sim;
        let mut out = String::new();
        let _ = writeln!(
            out,
            "t={:.2}s  scouts_connected={}  routed={}/{}  sinks={}  scouts_near_fire={}  burning_cells={}  meter: {}",
            secs(sim.now_ms()),
            metrics::scouts_connected(sim),
            metrics::routed_count(sim),
            live_count(sim).saturating_sub(metrics::with_role(sim, Role::Sink).count()),
            metrics::with_role(sim, Role::Sink).count(),
            scouts_near_fire(sim, 100.0),
            sim.fire().burning_count(),
            self.meter,
        );
        let _ = writeln!(
            out,
            "  id alive role     hops parent      x       y       z    agl  fire_d  intent        route_ok desc"
        );
        let desc = descendants(sim);
        for id in 0..sim.drone_count() as u16 {
            let b = sim.brain(id).expect("id in range");
            let p = sim.position(id).expect("id in range");
            let agl = p.y - sim.terrain().height_at(p.x, p.z);
            let hops = if b.hops() == NO_ROUTE { "-".to_string() } else { b.hops().to_string() };
            let parent = b.parent().map_or("-".to_string(), |p| p.to_string());
            let fire_d = nearest_burning_xz(sim, p).map_or("-".to_string(), |d| format!("{d:.0}"));
            let _ = writeln!(
                out,
                "  {id:>2} {:>5} {:<8} {hops:>4} {parent:>6} {:>7.1} {:>7.1} {:>7.1} {agl:>6.1} {fire_d:>7}  {:<13} {:>8} {:>4}",
                sim.is_alive(id),
                format!("{:?}", b.role()),
                p.x,
                p.y,
                p.z,
                format!("{:?}", b.intent()),
                metrics::route_ok(sim, id),
                desc[usize::from(id)].all,
            );
        }
        let _ = writeln!(out, "--- trace (most recent last)");
        for line in &self.trace {
            let _ = writeln!(out, "  {line}");
        }
        out
    }
}

pub fn secs(ms: u32) -> f64 {
    f64::from(ms) / 1000.0
}

pub fn live_count(sim: &Sim) -> usize {
    (0..sim.drone_count() as u16).filter(|&id| sim.is_alive(id)).count()
}

fn summary_line(sim: &Sim) -> String {
    let count = |r| metrics::with_role(sim, r).count();
    let s = sim.stats();
    format!(
        "t={:>6.2}s live={:>2} sink={} scout={} relay={} reserve={} connected={} routed={} near_fire={} step_frames={} dropped={}",
        secs(sim.now_ms()),
        live_count(sim),
        count(Role::Sink),
        count(Role::Scout),
        count(Role::Relay),
        count(Role::Reserve),
        metrics::scouts_connected(sim),
        metrics::routed_count(sim),
        scouts_near_fire(sim, 100.0),
        s.frames,
        s.dropped,
    )
}

/// Centres `(x, z)` of every burning fire cell.
pub fn burning_cells(sim: &Sim) -> Vec<(f32, f32)> {
    let grid = sim.fire();
    let n = grid.size();
    let cell = sim.config().extent / n as f32;
    let half = sim.config().extent * 0.5;
    grid.cells()
        .iter()
        .enumerate()
        .filter(|&(_, &c)| c == fire::BURNING)
        .map(|(i, _)| {
            let (r, c) = (i / n, i % n);
            (-half + (c as f32 + 0.5) * cell, -half + (r as f32 + 0.5) * cell)
        })
        .collect()
}

/// Mean of the burning cell centres, or `None` if nothing burns.
pub fn fire_centroid(sim: &Sim) -> Option<(f32, f32)> {
    let cells = burning_cells(sim);
    if cells.is_empty() {
        return None;
    }
    let n = cells.len() as f32;
    let (sx, sz) = cells.iter().fold((0.0, 0.0), |(a, b), &(x, z)| (a + x, b + z));
    Some((sx / n, sz / n))
}

/// Horizontal distance from `p` to the nearest burning cell centre.
pub fn nearest_burning_xz(sim: &Sim, p: Vec3) -> Option<f32> {
    burning_cells(sim)
        .iter()
        .map(|&(x, z)| ((x - p.x).powi(2) + (z - p.z).powi(2)).sqrt())
        .min_by(f32::total_cmp)
}

pub fn dist_xz(a: Vec3, b: Vec3) -> f32 {
    ((a.x - b.x).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

/// Live scouts within `radius` (xz) of some burning cell.
pub fn scouts_near_fire(sim: &Sim, radius: f32) -> usize {
    let cells = burning_cells(sim);
    metrics::with_role(sim, Role::Scout)
        .filter(|&id| {
            let p = sim.position(id).expect("live id");
            cells.iter().any(|&(x, z)| (x - p.x).powi(2) + (z - p.z).powi(2) <= radius * radius)
        })
        .count()
}

/// The S1 "deployed" condition: at least two scouts over the fire and every
/// scout routed to a sink.
pub fn deployed(sim: &Sim) -> bool {
    scouts_near_fire(sim, 100.0) >= 2 && metrics::scouts_connected(sim)
}

/// How many live drones route through each drone.
#[derive(Clone, Copy, Debug, Default)]
pub struct Desc {
    pub all: u32,
    pub scouts: u32,
}

/// For every drone, the live drones whose parent chain passes through it
/// (following believed parent pointers; a chain stops at a dead drone, a
/// missing parent or a loop).
pub fn descendants(sim: &Sim) -> Vec<Desc> {
    let n = sim.drone_count();
    let mut out = vec![Desc::default(); n];
    for id in 0..n as u16 {
        if !sim.is_alive(id) {
            continue;
        }
        let is_scout = sim.brain(id).is_some_and(|b| b.role() == Role::Scout);
        let mut seen = vec![false; n];
        seen[usize::from(id)] = true;
        let mut at = id;
        for _ in 0..MAX_CHAIN {
            let Some(parent) = sim.brain(at).and_then(|b| b.parent()) else { break };
            let pi = usize::from(parent);
            if pi >= n || seen[pi] || !sim.is_alive(parent) {
                break;
            }
            seen[pi] = true;
            out[pi].all += 1;
            out[pi].scouts += u32::from(is_scout);
            at = parent;
        }
    }
    out
}

/// Live relays ordered by descendants, most first (ties: lowest id).
pub fn relays_by_descendants(sim: &Sim) -> Vec<(u16, Desc)> {
    let desc = descendants(sim);
    let mut relays: Vec<(u16, Desc)> =
        metrics::with_role(sim, Role::Relay).map(|id| (id, desc[usize::from(id)])).collect();
    relays.sort_by_key(|&(id, d)| (std::cmp::Reverse(d.all), id));
    relays
}

/// The relay with the most descendants, if any relay exists.
pub fn top_relay(sim: &Sim) -> Option<(u16, Desc)> {
    relays_by_descendants(sim).into_iter().next()
}

/// Number of role changes per drone, sampled once per step.
pub struct RoleWatch {
    last: Vec<Role>,
    pub changes: Vec<u32>,
}

impl RoleWatch {
    pub fn new(sim: &Sim) -> Self {
        let last = (0..sim.drone_count() as u16).map(|id| sim.brain(id).expect("id").role()).collect();
        Self { last, changes: vec![0; sim.drone_count()] }
    }

    pub fn sample(&mut self, sim: &Sim) {
        for id in 0..sim.drone_count() as u16 {
            if !sim.is_alive(id) {
                continue;
            }
            let r = sim.brain(id).expect("id").role();
            let i = usize::from(id);
            if r != self.last[i] {
                self.changes[i] += 1;
                self.last[i] = r;
            }
        }
    }
}

/// True if `id` is the base station and still alive.
pub fn base_alive(sim: &Sim) -> bool {
    sim.is_alive(BASE_ID)
}
