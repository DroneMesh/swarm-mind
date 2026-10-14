//! Mechanism tests on a tiny in-crate radio: flat ground, explicit links,
//! perfect delivery unless a test drops frames. The scenario suite in
//! `swarm-sim` covers the same behaviour on real terrain.

extern crate std;

use std::vec::Vec;

use crate::plan::{NO_SLOT, Slot};
use crate::wire::{BROADCAST, Beacon, FLAG_SETTLED, Intent, Message, OPERATOR, Payload, Role, Rx};
use crate::{Brain, Config, FireReading, MAX_FRAME, NO_ROUTE, NodeId, Outbox, Sensors, Vec3};

const STEP_MS: u32 = 50;

/// A handful of brains, a link predicate and kinematic bodies.
struct Net {
    brains: Vec<Brain>,
    pos: Vec<Vec3>,
    alive: Vec<bool>,
    inbox: Vec<Vec<Rx>>,
    /// `links[i][j]`: can `j` hear `i`?
    links: Vec<Vec<bool>>,
    now: u32,
    frames: u32,
    /// Drop every n-th delivery (0 = none), deterministically.
    drop_every: u32,
    deliveries: u32,
    /// Bodies follow their brains' targets at this speed (0 = pinned).
    speed: f32,
    /// Called on every frame sent: `(src, message)`.
    sent: Vec<(usize, Message)>,
    /// When non-zero, links follow distance (this range) instead of `links`.
    range: f32,
}

impl Net {
    fn new(positions: &[Vec3]) -> Self {
        let n = positions.len();
        let brains = (0..n).map(|i| Brain::new(i as NodeId, i == 0, positions[i], Config::default())).collect();
        Self {
            brains,
            pos: positions.to_vec(),
            alive: std::vec![true; n],
            inbox: std::vec![Vec::new(); n],
            links: std::vec![std::vec![true; n]; n],
            now: 0,
            frames: 0,
            drop_every: 0,
            deliveries: 0,
            speed: 0.0,
            sent: Vec::new(),
            range: 0.0,
        }
    }

    /// Landed drones a few units apart, all within range of each other.
    fn cluster(n: usize) -> Self {
        let positions: Vec<Vec3> = (0..n).map(|i| Vec3::new(i as f32 * 3.0, 5.0, 0.0)).collect();
        Self::new(&positions)
    }

    /// Only consecutive drones (`i`, `i + 1`) hear each other.
    fn line(n: usize) -> Self {
        let mut net = Self::new(&(0..n).map(|i| Vec3::new(i as f32 * 100.0, 50.0, 0.0)).collect::<Vec<_>>());
        for i in 0..n {
            for j in 0..n {
                net.links[i][j] = i.abs_diff(j) == 1;
            }
        }
        net
    }

    fn link(&mut self, a: usize, b: usize, on: bool) {
        self.links[a][b] = on;
        self.links[b][a] = on;
    }

    fn step(&mut self) {
        self.now += STEP_MS;
        let n = self.brains.len();
        let mut out: Vec<(usize, Outbox)> = Vec::new();
        for i in 0..n {
            if !self.alive[i] {
                continue;
            }
            let sensors = Sensors {
                now_ms: self.now,
                pos: self.pos[i],
                ground_agl: self.pos[i].y,
                fire: FireReading::default(),
                ..Sensors::default()
            };
            let mut outbox = Outbox::new();
            let inbox = core::mem::take(&mut self.inbox[i]);
            let act = self.brains[i].tick(&sensors, &inbox, &mut outbox);
            if self.speed > 0.0 && !self.brains[i].is_sink() {
                let to = act.target - self.pos[i];
                let d = crate::math::sqrt(to.length_sq());
                let max = self.speed * STEP_MS as f32 / 1000.0;
                self.pos[i] = if d <= max { act.target } else { self.pos[i] + to * (max / d) };
                self.pos[i].y = self.pos[i].y.max(5.0);
            }
            out.push((i, outbox));
        }
        if self.range > 0.0 {
            for a in 0..n {
                for b in 0..n {
                    self.links[a][b] = a != b && (self.pos[a] - self.pos[b]).length_sq() < self.range * self.range;
                }
            }
        }
        for (src, outbox) in out {
            for frame in outbox.iter() {
                self.frames += 1;
                assert!(frame.as_bytes().len() <= MAX_FRAME);
                if let Some(m) = Message::decode(frame) {
                    self.sent.push((src, m));
                }
                for j in 0..n {
                    if j == src || !self.alive[j] || !self.links[src][j] {
                        continue;
                    }
                    self.deliveries += 1;
                    if self.drop_every > 0 && self.deliveries.is_multiple_of(self.drop_every) {
                        continue;
                    }
                    let d = crate::math::sqrt((self.pos[src] - self.pos[j]).length_sq());
                    let rssi = (-40.0 - 22.8 * if d > 1.0 { log10(d) } else { 0.0 }) as i8;
                    self.inbox[j].push(Rx { frame: *frame, rssi_dbm: rssi });
                }
            }
        }
    }

    fn run(&mut self, ms: u32) {
        for _ in 0..ms / STEP_MS {
            self.step();
        }
    }

    /// Run until `pred` holds; returns the elapsed ms.
    fn until(&mut self, max_ms: u32, mut pred: impl FnMut(&Self) -> bool) -> Option<u32> {
        let start = self.now;
        while self.now - start < max_ms {
            if pred(self) {
                return Some(self.now - start);
            }
            self.step();
        }
        pred(self).then_some(self.now - start)
    }

    fn command(&mut self, intent: Intent, target: Vec3) {
        let seq = self.brains[0].intent_seq.wrapping_add(1);
        let payload = Payload::Intent { seq, intent, target };
        let frame = Message { src: OPERATOR, dst: 0, payload }.encode();
        self.inbox[0].push(Rx { frame, rssi_dbm: -20 });
    }

    fn b(&self, i: usize) -> &Brain {
        &self.brains[i]
    }

    /// Follow parent pointers from `i`; true if they reach a live sink over
    /// existing links without revisiting anyone.
    fn routed(&self, i: usize) -> bool {
        let mut at = i;
        for _ in 0..self.brains.len() {
            if !self.alive[at] {
                return false;
            }
            if self.brains[at].role() == Role::Sink {
                return true;
            }
            let Some(p) = self.brains[at].parent() else { return false };
            let p = usize::from(p);
            if !self.links[p][at] {
                return false;
            }
            at = p;
        }
        false
    }

    fn has_two_cycle(&self) -> bool {
        (0..self.brains.len()).filter(|&i| self.alive[i]).any(|i| {
            self.brains[i].parent().is_some_and(|p| {
                let p = usize::from(p);
                self.alive[p] && self.brains[p].parent() == Some(i as NodeId)
            })
        })
    }

    fn sinks(&self) -> Vec<usize> {
        (0..self.brains.len()).filter(|&i| self.alive[i] && self.brains[i].role() == Role::Sink).collect()
    }
}

/// log10 for the test radio's RSSI, from the crate's sqrt-free tools:
/// ln via the bit pattern, good to a few hundredths, plenty for RSSI.
fn log10(x: f32) -> f32 {
    let bits = x.to_bits();
    let exp = ((bits >> 23) & 0xff) as f32 - 127.0;
    let mant = f32::from_bits((bits & 0x007f_ffff) | 0x3f80_0000); // [1, 2)
    let t = (mant - 1.0) / (mant + 1.0);
    let ln_m = 2.0 * (t + t * t * t / 3.0 + t * t * t * t * t / 5.0);
    (exp * core::f32::consts::LN_2 + ln_m) / core::f32::consts::LN_10
}

fn heartbeat(src: NodeId, hops: u8, parent: NodeId, sink_seq: u16) -> Rx {
    let b = Beacon {
        congestion_up: 0,
        hops,
        parent,
        role: Role::Relay,
        pos: Vec3::ZERO,
        sink: 0,
        sink_seq,
        counter: 1,
        intent_seq: 0,
        slot: NO_SLOT,
        need: NO_SLOT,
        need_hops: 0,
        flags: FLAG_SETTLED,
        interval_50ms: 20,
        heard_slots: 0,
    };
    Rx { frame: Message { src, dst: BROADCAST, payload: Payload::Heartbeat(b) }.encode(), rssi_dbm: -60 }
}

fn tick(brain: &mut Brain, now_ms: u32, inbox: &[Rx]) -> Outbox {
    let mut out = Outbox::new();
    let sensors = Sensors { now_ms, pos: Vec3::new(0.0, 50.0, 0.0), ground_agl: 50.0, ..Sensors::default() };
    brain.tick(&sensors, inbox, &mut out);
    out
}

// ---- footprint --------------------------------------------------------------

#[test]
fn fits_on_a_microcontroller() {
    let size = core::mem::size_of::<Brain>();
    assert!(size <= 4096, "Brain is {size} bytes");
}

// ---- routing ----------------------------------------------------------------

#[test]
fn sink_is_the_root() {
    let mut sink = Brain::new(0, true, Vec3::ZERO, Config::default());
    tick(&mut sink, 50, &[heartbeat(4, 1, 0, 1)]);
    assert_eq!((sink.hops(), sink.parent(), sink.role()), (0, None, Role::Sink));
}

#[test]
fn gradient_follows_the_line() {
    let mut net = Net::line(6);
    net.run(10_000);
    for i in 0..6 {
        assert_eq!(net.b(i).hops(), i as u8, "drone {i}");
        assert!(net.routed(i));
    }
}

#[test]
fn ties_break_on_lowest_id_and_split_horizon_holds() {
    let mut b = Brain::new(5, false, Vec3::ZERO, Config::default());
    tick(&mut b, 50, &[heartbeat(9, 1, 0, 3), heartbeat(4, 1, 0, 3)]);
    assert_eq!(b.parent(), Some(4));
    // A neighbour that routes through us is never our route.
    let mut c = Brain::new(5, false, Vec3::ZERO, Config::default());
    tick(&mut c, 50, &[heartbeat(6, 2, 5, 3)]);
    assert_eq!(c.hops(), NO_ROUTE);
}

#[test]
fn weak_links_lose_to_a_longer_solid_path() {
    let mut b = Brain::new(5, false, Vec3::ZERO, Config::default());
    let mut far = heartbeat(3, 1, 0, 3);
    far.rssi_dbm = -95;
    let near = heartbeat(4, 2, 1, 3);
    tick(&mut b, 50, &[far, near]);
    assert_eq!(b.parent(), Some(4), "a 2-hop solid route beats a 1-hop edge-of-range link");
}

#[test]
fn ladder_repairs_a_dead_relay_in_seconds_without_loops() {
    // 0 - {1,2} - {3,4} - 5 : a two-rail ladder.
    let mut net = Net::new(&[
        Vec3::new(0.0, 50.0, 0.0),
        Vec3::new(100.0, 50.0, -30.0),
        Vec3::new(100.0, 50.0, 30.0),
        Vec3::new(200.0, 50.0, -30.0),
        Vec3::new(200.0, 50.0, 30.0),
        Vec3::new(300.0, 50.0, 0.0),
    ]);
    let rows: [&[usize]; 4] = [&[0], &[1, 2], &[3, 4], &[5]];
    for a in 0..6 {
        for b in 0..6 {
            let ra = rows.iter().position(|r| r.contains(&a)).unwrap();
            let rb = rows.iter().position(|r| r.contains(&b)).unwrap();
            net.links[a][b] = a != b && ra.abs_diff(rb) <= 1;
        }
    }
    net.run(10_000);
    assert!(net.routed(5));
    // Kill whichever relay the tail's route uses.
    let victim = usize::from(net.b(usize::from(net.b(5).parent().unwrap())).parent().unwrap());
    net.alive[victim] = false;
    let mut looped = false;
    let t = net.until(5_000, |n| {
        looped |= n.has_two_cycle();
        n.routed(5)
    });
    assert!(!looped, "repair formed a routing loop");
    assert!(t.is_some_and(|t| t <= 3_500), "repaired after {t:?} ms");
}

#[test]
fn a_cut_off_drone_poisons_its_route_instead_of_counting_up() {
    let mut net = Net::line(5);
    net.run(10_000);
    net.alive[1] = false;
    net.run(15_000);
    for i in 2..5 {
        assert_eq!(net.b(i).hops(), NO_ROUTE, "drone {i} still believes in a route");
    }
    // Nobody ever advertised a hop count beyond the cap while cut off.
    assert!(net.sent.iter().all(|(_, m)| match m.payload {
        Payload::Heartbeat(b) => b.hops == NO_ROUTE || b.hops < crate::MAX_HOPS,
        _ => true,
    }));
}

#[test]
fn lowest_witness_becomes_the_one_acting_sink() {
    // 1, 2 and 3 hear the base; 4 and 5 hear only them.
    let mut net = Net::new(&[Vec3::new(0.0, 5.0, 0.0); 6]);
    for a in 0..6 {
        for b in 0..6 {
            net.links[a][b] = a != b && !(a == 0 && b >= 4) && !(b == 0 && a >= 4);
        }
    }
    // During a mission the sink beacons fast, so every witness has dropped
    // its route (and stopped advertising one hop) long before election time.
    net.command(Intent::MonitorFire, Vec3::new(800.0, 0.0, 0.0));
    net.run(8_000);
    net.alive[0] = false;
    let mut most = 0;
    let t = net.until(10_000, |n| {
        most = most.max(n.sinks().len());
        n.sinks() == [1] && (2..6).all(|i| n.routed(i))
    });
    assert!(t.is_some(), "sinks {:?}", net.sinks());
    assert_eq!(most, 1, "the other witnesses must defer, not declare and step down");
    // And it stays that way: no second sink, no flapping.
    for _ in 0..400 {
        net.step();
        assert_eq!(net.sinks(), [1]);
    }
}

/// Base 0, its witness 1, and 2 that hears only 1, on a mission. Then 1
/// loses the base while (if `moved`) it has flown 60 units on; returns the
/// sinks 20 s later.
fn sinks_after_witness_loses_base(moved: bool) -> Vec<usize> {
    let mut net = Net::new(&[Vec3::new(0.0, 5.0, 0.0), Vec3::new(10.0, 5.0, 0.0), Vec3::new(20.0, 5.0, 0.0)]);
    net.link(0, 2, false);
    net.command(Intent::MonitorFire, Vec3::new(800.0, 0.0, 0.0));
    net.run(8_000);
    assert!(net.b(1).parent() == Some(0) && net.routed(2), "set-up: 1 hears the base directly");
    net.link(0, 1, false);
    if moved {
        net.pos[1].x += 60.0;
        net.pos[2].x += 60.0;
    }
    net.run(20_000);
    net.sinks()
}

#[test]
fn a_witness_that_flew_out_of_sight_does_not_split_off_a_live_base() {
    // Standing still, the base's silence means it died: take over at once.
    assert_eq!(sinks_after_witness_loses_base(false), [0, 1]);
    // Having flown on, it most likely just lost sight of the base (a hill
    // in between). A second sink that stays put would keep the whole swarm
    // routed away from the live base for good; regrouping finds it again.
    assert_eq!(sinks_after_witness_loses_base(true), [0], "a witness in transit elected itself");
}

#[test]
fn a_witness_that_takes_over_far_from_the_base_goes_back_to_check() {
    // 1 hears the base from 150 units out, standing still, and 2 hears
    // only 1. A hill comes between 1 and the live base.
    let mut net = Net::new(&[Vec3::new(0.0, 5.0, 0.0), Vec3::new(150.0, 50.0, 0.0), Vec3::new(160.0, 50.0, 0.0)]);
    net.link(0, 2, false);
    net.command(Intent::MonitorFire, Vec3::new(800.0, 0.0, 0.0));
    net.run(8_000);
    assert!(net.b(1).parent() == Some(0), "set-up: 1 hears the base directly");
    net.link(0, 1, false);
    assert!(net.until(10_000, |n| n.sinks() == [0, 1]).is_some(), "the standing witness takes over");
    assert!(net.b(1).rt.verify, "away from the base's spot it must go and check");
    // It flies back alone (2 keeps to its slot), hears the base and hands back.
    net.speed = 12.0;
    let t = net.until(40_000, |n| n.pos[1].dist_xz(n.pos[0]) < 60.0);
    assert!(t.is_some(), "the acting sink never went back: {:?}", net.pos[1]);
    assert!(net.pos[2].x > 200.0, "its group keeps to the formation");
    net.link(0, 1, true);
    let t = net.until(15_000, |n| n.sinks() == [0] && n.routed(2));
    assert!(t.is_some(), "never handed back: sinks {:?}", net.sinks());
}

#[test]
fn acting_sink_steps_down_when_partitions_merge() {
    // Two groups: {0, 1, 2} with the base, {3, 4, 5} cut off.
    let mut net = Net::new(&[Vec3::new(0.0, 5.0, 0.0); 6]);
    for a in 0..6 {
        for b in 0..6 {
            net.links[a][b] = a != b && (a < 3) == (b < 3);
        }
    }
    // Long enough for the cut-off group to elect its own sink.
    net.run(40_000);
    assert_eq!(net.sinks(), [0, 3], "the orphans elected their lowest id");
    net.link(2, 3, true);
    let t = net.until(15_000, |n| n.sinks() == [0] && (1..6).all(|i| n.routed(i)));
    assert!(t.is_some(), "after merge: sinks {:?}", net.sinks());
}

#[test]
fn loss_is_measured_and_widens_tolerance() {
    let mut net = Net::cluster(6);
    net.run(20_000);
    let clean = net.b(3).link_loss();
    net.drop_every = 3; // a third of all deliveries
    net.run(60_000);
    let lossy = net.b(3).link_loss();
    assert!(clean < 0.02 && (0.2..0.5).contains(&lossy), "clean {clean}, lossy {lossy}");
    assert!(net.b(3).misses_tolerated() >= 4);
    assert!((1..6).all(|i| net.routed(i)), "routes held under loss");
}

// ---- radio discipline -------------------------------------------------------

#[test]
fn landed_drones_beacon_slowly_and_never_in_lockstep() {
    let mut net = Net::cluster(20);
    net.run(5_000);
    let before = net.frames;
    let mut worst = 0;
    for _ in 0..600 {
        let f = net.frames;
        net.step();
        worst = worst.max(net.frames - f);
    }
    let rate = f64::from(net.frames - before) / 20.0 / 30.0;
    assert!(rate <= 1.0, "{rate} frames/drone/s while landed");
    assert!(worst <= 4, "{worst} frames in one step: beacons are synchronised");
}

#[test]
fn intent_floods_once_and_reaches_everyone() {
    let mut net = Net::line(6);
    net.run(5_000);
    net.sent.clear();
    net.command(Intent::MonitorFire, Vec3::new(1000.0, 0.0, 0.0));
    net.run(3_000);
    for i in 0..6 {
        assert_eq!(net.b(i).intent(), Intent::MonitorFire, "drone {i}");
    }
    let floods = net.sent.iter().filter(|(_, m)| matches!(m.payload, Payload::Mission { .. })).count();
    assert_eq!(floods, 6, "each drone repeats a new intent exactly once");
}

#[test]
fn a_drone_that_missed_the_flood_catches_up() {
    let mut net = Net::line(4);
    net.run(5_000);
    net.alive[3] = false; // asleep during the flood
    net.command(Intent::MapPerimeter, Vec3::new(1000.0, 0.0, 0.0));
    net.run(2_000);
    net.alive[3] = true;
    let t = net.until(5_000, |n| n.b(3).intent() == Intent::MapPerimeter);
    assert!(t.is_some(), "anti-entropy never re-sent the intent");
}

#[test]
fn stale_or_foreign_intents_are_ignored() {
    let mut b = Brain::new(5, false, Vec3::ZERO, Config::default());
    let console = |seq, intent| {
        let payload = Payload::Intent { seq, intent, target: Vec3::ZERO };
        Rx { frame: Message { src: OPERATOR, dst: 5, payload }.encode(), rssi_dbm: -20 }
    };
    tick(&mut b, 50, &[console(1, Intent::ReturnToBase)]);
    assert_eq!(b.intent(), Intent::Hold, "only the sink is wired to the console");
    let mission = |seq, intent| {
        let payload = Payload::Mission { seq, intent, target: Vec3::new(500.0, 0.0, 0.0), origin: Vec3::ZERO };
        Rx { frame: Message { src: 0, dst: BROADCAST, payload }.encode(), rssi_dbm: -60 }
    };
    tick(&mut b, 100, &[mission(5, Intent::MonitorFire)]);
    tick(&mut b, 150, &[mission(4, Intent::ReturnToBase)]);
    assert_eq!(b.intent(), Intent::MonitorFire, "older sequence numbers lose");
}

// ---- formation ----------------------------------------------------------------

/// A landed cluster told to monitor a fire 800 units away.
fn deployed_cluster(n: usize) -> Net {
    let mut net = Net::cluster(n);
    net.run(3_000);
    net.command(Intent::MonitorFire, Vec3::new(800.0, 0.0, 0.0));
    net.run(3_000);
    net
}

#[test]
fn ranking_gives_every_drone_a_distinct_slot_without_messages() {
    let net = deployed_cluster(20);
    let mut slots: Vec<u8> = (1..20).map(|i| net.b(i).slot()).collect();
    assert!(slots.iter().all(|&s| s != NO_SLOT));
    slots.sort_unstable();
    slots.dedup();
    assert_eq!(slots.len(), 19, "duplicate slot claims");
    let count = |r| (1..20).filter(|&i| net.b(i).role() == r).count();
    assert_eq!((count(Role::Relay), count(Role::Scout), count(Role::Reserve)), (8, 4, 7));
    // Lowest ids take the most important slots: rail A from the sink out.
    assert_eq!(net.b(1).slot(), Slot::Relay { row: 1, rail: 0 }.id());
}

#[test]
fn duplicate_claims_resolve_to_one_holder() {
    let mut net = deployed_cluster(6);
    let slot = net.b(2).slot();
    // Force drone 4 onto drone 2's slot; one of them must back off.
    net.brains[4].m.slot = slot;
    net.run(3_000);
    let holders = (1..6).filter(|&i| net.b(i).slot() == slot).count();
    assert_eq!(holders, 1);
    assert_eq!(net.b(2).slot(), slot, "the lower id keeps it when neither is settled");
}

#[test]
fn a_vacated_relay_slot_is_noticed_and_filled_by_a_reserve() {
    // Everyone hears everyone, so vacancies are the only thing to repair.
    let mut net = deployed_cluster(20);
    net.speed = 12.0;
    net.run(90_000);
    let victim = (1..20).find(|&i| net.b(i).role() == Role::Relay).unwrap();
    let slot = net.b(victim).slot();
    net.alive[victim] = false;
    let t = net.until(40_000, |n| {
        (1..20).any(|i| n.alive[i] && n.b(i).slot() == slot && n.b(i).role() == Role::Relay)
    });
    assert!(t.is_some(), "slot {slot} never refilled");
    let holders = (1..20).filter(|&i| net.alive[i] && net.b(i).slot() == slot).count();
    assert_eq!(holders, 1);
}

#[test]
fn hold_keeps_landed_drones_down_and_return_lands_them_home() {
    let mut net = Net::cluster(5);
    net.speed = 12.0;
    let home: Vec<Vec3> = net.pos.clone();
    net.run(5_000);
    assert!(net.pos.iter().zip(&home).all(|(p, h)| (*p - *h).length_sq() < 1.0), "Hold moved a landed drone");
    net.command(Intent::MonitorFire, Vec3::new(400.0, 0.0, 0.0));
    net.run(30_000);
    assert!(net.pos[1].x > 50.0, "the mission never started");
    net.command(Intent::ReturnToBase, Vec3::ZERO);
    let t = net.until(90_000, |n| (1..5).all(|i| n.pos[i].dist_xz(home[i]) < 10.0 && n.pos[i].y <= 5.5));
    assert!(t.is_some(), "not home: {:?}", net.pos);
    assert!((1..5).all(|i| net.b(i).role() == Role::Reserve));
}

#[test]
fn scouts_follow_the_heat() {
    let mut net = deployed_cluster(8);
    let scout = (1..8).find(|&i| net.b(i).role() == Role::Scout).unwrap();
    let r0 = net.b(scout).m.scout_r;
    // Pretend the scout sees fire further out along its sector.
    let plan = net.b(scout).plan.unwrap();
    let Some(Slot::Scout(k)) = Slot::decode(net.b(scout).slot()) else { panic!() };
    let (dx, dz) = plan.scout_dir(k);
    let spot = net.b(scout).slot_spot_for_test();
    let mut b = core::mem::replace(&mut net.brains[scout], Brain::new(99, false, Vec3::ZERO, Config::default()));
    let fire = FireReading { burning: 5, centroid: Vec3::new(spot.x + dx * 30.0, 0.0, spot.z + dz * 30.0) };
    for i in 0..40 {
        let s = Sensors { now_ms: net.now + i * 50, pos: Vec3::new(spot.x, 40.0, spot.z), ground_agl: 40.0, fire, ..Sensors::default() };
        b.tick(&s, &[], &mut Outbox::new());
    }
    assert!(b.m.scout_r > r0 + 3.0, "radius {r0} -> {}", b.m.scout_r);
}

/// A settled relay at its slot whose only parent (drone 3, one hop from the
/// sink) is heard on a weak link from `parent_y`; returns the extra height
/// it gained over 20 s.
fn relay_climb_with_parent_at(parent_y: f32) -> f32 {
    let target = Vec3::new(800.0, 0.0, 0.0);
    let mut b = Brain::new(5, false, Vec3::ZERO, Config::default());
    let mission = Payload::Mission { seq: 1, intent: Intent::MonitorFire, target, origin: Vec3::ZERO };
    tick(&mut b, 50, &[Rx { frame: Message { src: 0, dst: BROADCAST, payload: mission }.encode(), rssi_dbm: -60 }]);
    let slot = Slot::Relay { row: 2, rail: 0 }.id();
    b.m.slot = slot;
    let spot = b.slot_spot_for_test();
    let me = Vec3::new(spot.x, 250.0, spot.z);
    for k in 1..400u32 {
        let now = 100 + k * 50;
        let mut inbox = Vec::new();
        if k % 20 == 0 {
            let mut rx = heartbeat(3, 1, 0, (k / 20) as u16);
            let Some(Message { payload: Payload::Heartbeat(mut hb), .. }) = Message::decode(&rx.frame) else {
                unreachable!()
            };
            hb.pos = Vec3::new(spot.x - 150.0, parent_y, spot.z);
            hb.slot = Slot::Relay { row: 1, rail: 0 }.id();
            let frame = Message { src: 3, dst: BROADCAST, payload: Payload::Heartbeat(hb) }.encode();
            rx = Rx { frame, rssi_dbm: -95 };
            inbox.push(rx);
        }
        let s = Sensors { now_ms: now, pos: me, ground_agl: 100.0, ..Sensors::default() };
        b.tick(&s, &inbox, &mut Outbox::new());
    }
    assert!(b.m.settled && b.parent() == Some(3), "set-up: settled relay routed through 3");
    b.m.climb
}

#[test]
fn a_relay_climbs_toward_a_higher_parent_but_never_away_from_a_lower_one() {
    // Signal follows 3D distance: above a lower parent, climbing only
    // stretches the weak link further until it breaks.
    assert_eq!(relay_climb_with_parent_at(100.0), 0.0, "climbed away from a parent below");
    assert!(relay_climb_with_parent_at(400.0) > 20.0, "should climb toward a parent above");
}

/// Base 0, its neighbour 1, and 2 and 3 that hear the base only through 1
/// (so they are no witnesses of it), all told to monitor a fire at `fire_x`.
fn orphan_candidates(fire_x: f32) -> Net {
    let mut net = Net::new(&[
        Vec3::new(0.0, 5.0, 0.0),
        Vec3::new(10.0, 5.0, 0.0),
        Vec3::new(20.0, 5.0, 0.0),
        Vec3::new(30.0, 5.0, 0.0),
    ]);
    net.link(0, 2, false);
    net.link(0, 3, false);
    net.run(3_000);
    net.command(Intent::MonitorFire, Vec3::new(fire_x, 0.0, 0.0));
    net.run(4_000);
    assert!((1..4).all(|i| net.b(i).slot() != NO_SLOT), "set-up: everyone has a slot");
    assert!(net.routed(3) && net.b(3).hops() == 2, "set-up: 3 routes through 1");
    net
}

#[test]
fn a_group_that_elected_its_own_sink_heads_home_and_merges() {
    let mut net = orphan_candidates(300.0);
    // Cut 2 and 3 off, far out of range, long enough to elect a sink.
    net.pos[1] = Vec3::new(100.0, 50.0, 0.0);
    net.pos[2] = Vec3::new(900.0, 50.0, 0.0);
    net.pos[3] = Vec3::new(950.0, 50.0, 0.0);
    net.range = 260.0;
    net.run(36_000);
    assert_eq!(net.sinks(), [0, 2], "set-up: the cut-off pair elected its own sink");
    // Free to fly, the pair must not settle for its own sink while the
    // base is alive: it heads back and merges into the base's gradient.
    net.speed = 12.0;
    let t = net.until(120_000, |n| n.sinks() == [0] && (1..4).all(|i| n.routed(i)));
    assert!(t.is_some(), "never merged: sinks {:?}, positions {:?}", net.sinks(), net.pos);
}

#[test]
fn an_orphan_sink_that_finds_no_base_carries_on_without_it() {
    let mut net = orphan_candidates(200.0);
    net.alive[0] = false;
    net.alive[1] = false;
    net.pos[2] = Vec3::new(900.0, 50.0, 0.0);
    net.pos[3] = Vec3::new(950.0, 50.0, 0.0);
    net.range = 260.0;
    net.speed = 12.0;
    // They regroup at the base's spot, elect, find nobody, and carry on.
    net.run(180_000);
    assert_eq!(net.sinks(), [2]);
    assert!(!net.b(2).rt.orphan, "the acting sink gave up looking for the base");
    let origin = net.b(2).plan.unwrap().origin;
    assert!(net.pos[2].dist_xz(origin) < 60.0, "the acting sink stays near the base's spot");
    assert!(net.routed(3), "the mission carries on through the new sink");
}

#[test]
fn a_far_holder_waits_its_turn_before_moving_up() {
    // Drone 5 holds B4, the least important relay slot, and hears only its
    // parent 3, which carries traffic (so can't move) and relays a need for
    // A2 from two hops away. Drones nearer the gap may be heading there
    // unheard, so 5 must not set off at the plain promotion delay.
    let target = Vec3::new(800.0, 0.0, 0.0);
    let mut b = Brain::new(5, false, Vec3::ZERO, Config::default());
    let mission = Payload::Mission { seq: 1, intent: Intent::MonitorFire, target, origin: Vec3::ZERO };
    tick(&mut b, 50, &[Rx { frame: Message { src: 0, dst: BROADCAST, payload: mission }.encode(), rssi_dbm: -60 }]);
    let rows = b.plan.unwrap().rows;
    assert!(rows >= 4, "set-up: {rows} rows");
    b.m.slot = Slot::Relay { row: 4, rail: 1 }.id();
    let spot = b.slot_spot_for_test();
    let need = Slot::Relay { row: 2, rail: 0 }.id();
    let mut claimed_at = None;
    for k in 1..(60_000 / STEP_MS) {
        let now = 100 + k * STEP_MS;
        let mut inbox = Vec::new();
        if k % 20 == 0 {
            let payload = Payload::Heartbeat(Beacon {
                congestion_up: 0,
                hops: 1,
                parent: 0,
                role: Role::Relay,
                pos: Vec3::new(spot.x, 50.0, spot.z - 60.0),
                sink: 0,
                sink_seq: (k / 20) as u16,
                counter: (k / 20) as u8,
                intent_seq: 1,
                slot: Slot::Relay { row: 4, rail: 0 }.id(),
                need,
                need_hops: 1,
                flags: FLAG_SETTLED | crate::wire::FLAG_CARRYING,
                interval_50ms: 20,
                heard_slots: 0,
            });
            inbox.push(Rx { frame: Message { src: 3, dst: BROADCAST, payload }.encode(), rssi_dbm: -70 });
        }
        let s = Sensors { now_ms: now, pos: Vec3::new(spot.x, 50.0, spot.z), ground_agl: 50.0, ..Sensors::default() };
        b.tick(&s, &inbox, &mut Outbox::new());
        if b.slot() == need && claimed_at.is_none() {
            claimed_at = Some(now);
        }
    }
    let t = claimed_at.expect("never moved up");
    // 8 s base wait, 2 hops x 4 s, and about 330 units x 40 ms.
    assert!(t > 25_000, "moved up after only {t} ms");
}

#[test]
fn a_filler_that_loses_to_another_filler_stays_away() {
    let mut net = deployed_cluster(6);
    let slot = net.b(2).slot();
    // Both on their way to fill the same slot: the lower id wins, and the
    // loser must not volunteer for it again straight away.
    for i in [2, 4] {
        net.brains[i].m.slot = slot;
        net.brains[i].m.filling = true;
    }
    net.run(3_000);
    assert_eq!(net.b(2).slot(), slot);
    assert_ne!(net.b(4).slot(), slot);
    assert_eq!(net.b(4).m.refused.map(|r| r.0), Some(slot), "the loser keeps away from the slot");
}

#[test]
fn a_filler_that_arrived_and_then_loses_to_the_holder_stays_away() {
    let mut net = deployed_cluster(6);
    let slot = net.b(2).slot();
    // 4 came to fill 2's slot and arrived (arrival clears `filling`), then
    // 2, which had only dropped out of earshot, is heard holding it again.
    // 4 loses on id and must not answer the next false alarm for it.
    net.brains[4].m.slot = slot;
    net.brains[4].m.came_as_filler = true;
    net.run(3_000);
    assert_eq!(net.b(2).slot(), slot);
    assert_ne!(net.b(4).slot(), slot);
    assert_eq!(net.b(4).m.refused.map(|r| r.0), Some(slot), "the loser keeps away from the slot");
}

#[test]
fn a_scout_over_a_cliff_edge_holds_the_height_of_its_link() {
    // A settled scout on a solid link to a parent 60 units back on a
    // plateau, out over ground that falls away (a gorge): flying at its
    // usual height above that ground would put the rim between them.
    let target = Vec3::new(800.0, 0.0, 0.0);
    let mut b = Brain::new(5, false, Vec3::ZERO, Config::default());
    let mission = Payload::Mission { seq: 1, intent: Intent::MonitorFire, target, origin: Vec3::ZERO };
    tick(&mut b, 50, &[Rx { frame: Message { src: 0, dst: BROADCAST, payload: mission }.encode(), rssi_dbm: -60 }]);
    b.m.slot = Slot::Scout(0).id();
    let spot = b.slot_spot_for_test();
    let mut me = Vec3::new(spot.x, 180.0, spot.z);
    let mut lowest = me.y;
    for k in 1..400u32 {
        let mut inbox = Vec::new();
        if k % 20 == 1 {
            let rx = heartbeat(3, 1, 0, (k / 20 + 1) as u16);
            let Some(Message { payload: Payload::Heartbeat(mut hb), .. }) = Message::decode(&rx.frame) else {
                unreachable!()
            };
            hb.pos = Vec3::new(spot.x - 60.0, 180.0, spot.z);
            let frame = Message { src: 3, dst: BROADCAST, payload: Payload::Heartbeat(hb) }.encode();
            inbox.push(Rx { frame, rssi_dbm: -80 });
        }
        // The ground is at 10, far below the plateau; the body follows the
        // target's height at 12 units/s.
        let s = Sensors { now_ms: 100 + k * 50, pos: me, ground_agl: me.y - 10.0, ..Sensors::default() };
        let y = b.tick(&s, &inbox, &mut Outbox::new()).target.y;
        me.y += (y - me.y).clamp(-0.6, 0.6);
        lowest = lowest.min(me.y);
    }
    assert_eq!(b.parent(), Some(3), "set-up: routed through the parent on the plateau");
    // Slope floor: 180 - 0.25 x 60 = 165 (a little under, one step's travel).
    assert!(lowest >= 160.0, "the scout sank to {lowest}, below its parent's horizon");
}

#[test]
fn a_swarm_taking_off_together_does_not_beacon_in_lockstep() {
    // Drones that hear nobody (so nothing triggers extra heartbeats) sit
    // landed long enough to spread their phases, then all lift off in the
    // same tick and switch to the shorter flying interval together.
    let n = 40u32;
    let mut brains: Vec<Brain> =
        (0..n).map(|i| Brain::new(i as NodeId + 1, false, Vec3::ZERO, Config::default())).collect();
    // (Under 30 s: alone that long, every drone would elect itself sink.)
    let (mut worst, mut now) = (0, 0);
    for k in 0..(25_000 / STEP_MS) {
        now += STEP_MS;
        let agl = if k < 200 { 5.0 } else { 50.0 };
        let mut sent = 0;
        for b in &mut brains {
            let mut out = Outbox::new();
            let s = Sensors { now_ms: now, pos: Vec3::new(0.0, agl, 0.0), ground_agl: agl, ..Sensors::default() };
            b.tick(&s, &[], &mut out);
            sent += out.len();
        }
        if k >= 200 {
            worst = worst.max(sent);
        }
    }
    // Two per step on average, a few more by chance (the jitter is seeded,
    // so this is deterministic). Re-anchoring every drone whose shorter
    // wait had already passed to "now" sent a fifth of the swarm at once
    // (11 here), and with no jitter they stayed locked like that.
    assert!(worst <= 7, "{worst} heartbeats in one step: phases locked together");
}
