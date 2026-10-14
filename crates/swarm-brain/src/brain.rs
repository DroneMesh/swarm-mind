//! The decision loop each drone runs, and its radio discipline.
//!
//! [`Brain::tick`] runs once per control step: read the inbox, update
//! beliefs about neighbours, maintain the route to the sink
//! (`routing.rs`), play the drone's part in the formation (`mission.rs`),
//! then decide what to transmit and where to fly.

use crate::airtime::Airtime;
use crate::mission::Mission;
use crate::plan::Plan;
use crate::routing::Routing;
use crate::wire::{
    BROADCAST, Beacon, FLAG_CARRYING, FLAG_LANDED, FLAG_ORPHAN, Frame, Intent, Message, NodeId,
    OPERATOR, Payload, Role, Rx,
};
use crate::{FixedVec, MAX_NEIGHBORS, MAX_OUTBOX, Outbox, Vec3};

/// Hop count meaning "no known route to the sink".
pub const NO_ROUTE: u8 = u8::MAX;

// `tick` queues at most an intent flood and a heartbeat.
const _: () = assert!(MAX_OUTBOX >= 2);

/// The sink beacons this fast during a mission, so its loss is noticed
/// quickly and route freshness propagates promptly.
const SINK_MISSION_INTERVAL_MS: u32 = 500;
/// Parked drones (landed, or reserves holding a parking spot) beacon this
/// slowly: nothing depends on them minute to minute.
const PARKED_INTERVAL_MS: u32 = 1250;
/// Fastest a relay beacons to protect the swarm's control on a busy
/// channel (see `Brain::protect_control`).
const PROTECTED_INTERVAL_MS: u32 = 500;
/// A triggered heartbeat (route changed) never follows the previous one
/// sooner than this, which bounds bursts during repair.
const MIN_BEACON_GAP_MS: u32 = 250;
/// Each periodic heartbeat comes up to this fraction of the interval early,
/// at random. Drones that start or change interval together would
/// otherwise keep transmitting in the same instant for ever (collisions on
/// a real radio, overflowing inboxes in a crowd). Early only, never late:
/// a neighbour's gap between beacons never exceeds its advertised interval,
/// so missed-beacon counting stays exact; the cost is a few % more frames.
const JITTER_DIV: u32 = 8;
/// Below this height above ground a drone counts as landed.
pub(crate) const LANDED_AGL: f32 = 8.0;

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Time between heartbeats for a drone with a job to do. Parked drones
    /// beacon a little slower, the sink faster during missions.
    pub beacon_interval_ms: u32,
    /// A neighbour beaconing at `beacon_interval_ms` is forgotten after this
    /// long without a heartbeat (scaled for neighbours on other intervals,
    /// and stretched on lossy channels).
    pub neighbor_timeout_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { beacon_interval_ms: 1000, neighbor_timeout_ms: 3500 }
    }
}

/// What this drone currently believes about a drone it can hear, as of
/// that drone's latest heartbeat.
#[derive(Clone, Copy, Debug, Default)]
pub struct Neighbor {
    pub id: NodeId,
    pub hops: u8,
    pub parent: NodeId,
    pub role: Role,
    pub pos: Vec3,
    pub last_heard_ms: u32,
    /// Sink its gradient leads to, and that sink's freshness counter.
    pub sink: NodeId,
    pub sink_seq: u16,
    /// Signal strength of its latest heartbeat.
    pub rssi_dbm: i8,
    /// Its advertised beacon interval.
    pub interval_ms: u16,
    /// Formation slot it claims, and its `FLAG_*` bits.
    pub slot: u8,
    pub flags: u8,
    /// Its [`crate::Beacon::congestion_up`].
    pub congestion_up: u8,
    pub intent_seq: u16,
    pub counter: u8,
    /// Slots whose holders it heard recently (`Beacon::heard_slots`).
    pub heard_slots: u32,
    /// The sink it last routed to directly, kept while it has no route
    /// ([`BROADCAST`] if none): a witness that may take over if that sink
    /// falls silent. Remembered here because by election time every
    /// witness has already poisoned its route.
    pub witness_of: NodeId,
}

/// Readings from the drone's own hardware.
#[derive(Clone, Copy, Debug, Default)]
pub struct Sensors {
    pub now_ms: u32,
    /// Own position (GNSS).
    pub pos: Vec3,
    /// Height above the ground directly below (rangefinder).
    pub ground_agl: f32,
    /// Downward thermal sensor.
    pub fire: FireReading,
    /// Local sends refused since the last tick: by the radio (its queue was
    /// full or the channel too busy; what an ESP32 learns from failed
    /// `esp_now_send` callbacks), and by the telemetry transport running on
    /// this node (painlessMesh `sendSingle`/`sendBroadcast` returning false,
    /// and packages this node had to drop, its own or relayed, because a
    /// connection's send buffer was full). The transport refusals are most
    /// of the evidence at the relays near the base, so a host running
    /// telemetry should count them. 0 when the host models neither.
    pub tx_dropped: u16,
    /// Telemetry interval the operator asked for, in ms (0 = none / no
    /// telemetry). The brain may pick a longer one; see
    /// [`Actuation::telemetry_interval_ms`].
    pub telemetry_requested_ms: u16,
}

/// What the downward thermal sensor sees. It covers a disc of fixed radius
/// on the ground below the drone.
#[derive(Clone, Copy, Debug, Default)]
pub struct FireReading {
    /// Burning cells in view; 0 means no fire seen.
    pub burning: u16,
    /// Ground-plane centroid of the burning cells in view (`x`, `z`; `y` is
    /// 0). Only meaningful when `burning > 0`.
    pub centroid: Vec3,
}

/// What the brain asks the flight controller to do.
#[derive(Clone, Copy, Debug)]
pub struct Actuation {
    /// Position to fly to. Getting there is the flight controller's job.
    pub target: Vec3,
    /// Telemetry interval the brain chose, in ms, for the host's telemetry
    /// task to use (0 = none). The brain manages its own airtime with it.
    pub telemetry_interval_ms: u16,
}

/// Transmit-side state.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Radio {
    pub next_beacon_ms: u32,
    pub last_beacon_ms: Option<u32>,
    pub counter: u8,
    /// The interval `next_beacon_ms` was scheduled with.
    pub scheduled_ms: u32,
    /// Jitter source (xorshift; never 0).
    pub rng: u32,
    /// Send an extra heartbeat at this time (route changed). The periodic
    /// schedule is left alone, so triggers can't synchronise neighbours.
    pub trigger_at: Option<u32>,
    /// Estimated delivery loss, from gaps in neighbours' beacon counters.
    pub loss: f32,
    /// Pending re-broadcast of the current intent, and how many copies of
    /// it were overheard meanwhile (enough copies cancel it).
    pub flood_at: Option<u32>,
    pub flood_copies: u8,
    pub last_flood_ms: Option<u32>,
}

pub struct Brain {
    pub(crate) id: NodeId,
    pub(crate) cfg: Config,
    pub(crate) is_sink: bool,
    pub(crate) home: Vec3,
    pub(crate) neighbors: FixedVec<Neighbor, MAX_NEIGHBORS>,
    pub(crate) now: u32,
    pub(crate) boot_ms: Option<u32>,
    pub(crate) last_tick_ms: u32,
    pub(crate) pos: Vec3,
    pub(crate) agl: f32,
    pub(crate) rt: Routing,
    pub(crate) radio: Radio,
    pub(crate) m: Mission,
    pub(crate) role: Role,
    pub(crate) intent: Intent,
    pub(crate) intent_target: Vec3,
    pub(crate) intent_seq: u16,
    /// Formation geometry of the current mission, if it has one.
    pub(crate) plan: Option<Plan>,
    /// Telemetry airtime controller (`airtime.rs`).
    pub(crate) air: Airtime,
}

impl Brain {
    pub fn new(id: NodeId, is_sink: bool, home: Vec3, cfg: Config) -> Self {
        let cfg = Config {
            beacon_interval_ms: cfg.beacon_interval_ms.max(100),
            neighbor_timeout_ms: cfg.neighbor_timeout_ms.max(200),
        };
        Self {
            id,
            cfg,
            is_sink,
            home,
            neighbors: FixedVec::new(),
            now: 0,
            boot_ms: None,
            last_tick_ms: 0,
            pos: home,
            agl: 0.0,
            rt: Routing::new(id, is_sink),
            radio: Radio {
                // Stagger the first beacon so drones powered on together
                // don't all transmit at the same instant.
                next_beacon_ms: stagger(id, PARKED_INTERVAL_MS.min(cfg.beacon_interval_ms)),
                scheduled_ms: PARKED_INTERVAL_MS.min(cfg.beacon_interval_ms),
                rng: u32::from(id).wrapping_mul(2_654_435_761) | 1,
                last_beacon_ms: None,
                counter: 0,
                trigger_at: None,
                loss: 0.0,
                flood_at: None,
                flood_copies: 0,
                last_flood_ms: None,
            },
            m: Mission::new(),
            role: if is_sink { Role::Sink } else { Role::Reserve },
            intent: Intent::Hold,
            intent_target: home,
            intent_seq: 0,
            plan: None,
            air: Airtime::default(),
        }
    }

    /// One decision step: read the inbox, update beliefs, queue outgoing
    /// frames and decide where to fly.
    pub fn tick(&mut self, sensors: &Sensors, inbox: &[Rx], outbox: &mut Outbox) -> Actuation {
        let now = sensors.now_ms;
        let first = self.boot_ms.is_none();
        self.boot_ms.get_or_insert(now);
        let dt_ms = if first { 0 } else { now.wrapping_sub(self.last_tick_ms).min(1000) };
        self.last_tick_ms = now;
        self.now = now;
        if sensors.pos.is_finite() {
            self.pos = sensors.pos;
        }
        self.agl = if sensors.ground_agl.is_finite() { sensors.ground_agl } else { 0.0 };

        for rx in inbox {
            // Corrupt or unknown frames are ignored, as a real radio would.
            if let Some(msg) = Message::decode(&rx.frame) {
                self.handle(msg, rx.rssi_dbm);
            }
        }

        self.expire_neighbors();
        self.update_route();
        self.update_mission(sensors);
        self.update_role();
        let target = self.flight_target(sensors, dt_ms as f32 / 1000.0);
        self.transmit(outbox);
        Actuation { target, telemetry_interval_ms: self.telemetry_interval(sensors) }
    }

    pub fn id(&self) -> NodeId {
        self.id
    }

    /// True for the base station, the network's designated sink. An acting
    /// sink elected after the base goes silent reports `role() == Sink`
    /// but is not the base.
    pub fn is_sink(&self) -> bool {
        self.is_sink
    }

    pub fn hops(&self) -> u8 {
        self.rt.hops
    }

    pub fn parent(&self) -> Option<NodeId> {
        self.rt.parent
    }

    pub fn role(&self) -> Role {
        self.role
    }

    pub fn intent(&self) -> Intent {
        self.intent
    }

    pub fn intent_target(&self) -> Vec3 {
        self.intent_target
    }

    pub fn neighbors(&self) -> &[Neighbor] {
        &self.neighbors
    }

    /// The sink this drone's route leads to, if it has a route (its own id
    /// when it is the sink).
    pub fn sink(&self) -> Option<NodeId> {
        (self.rt.hops != NO_ROUTE).then_some(self.rt.sink)
    }

    /// The formation slot this drone claims (0 = none).
    pub fn slot(&self) -> u8 {
        self.m.slot
    }

    /// Estimated fraction of neighbours' beacons that never arrive.
    pub fn link_loss(&self) -> f32 {
        self.radio.loss
    }

    pub(crate) fn acts_as_sink(&self) -> bool {
        self.is_sink || self.rt.acting
    }

    /// True if some live neighbour routes through us.
    pub(crate) fn carrying(&self) -> bool {
        let id = self.id;
        self.neighbors.iter().any(|n| n.parent == id && !self.suspect(n))
    }

    pub(crate) fn neighbor(&self, id: NodeId) -> Option<&Neighbor> {
        self.neighbors.iter().find(|n| n.id == id)
    }

    pub(crate) fn age(&self, t: u32) -> u32 {
        self.now.wrapping_sub(t)
    }

    /// Consecutive beacons a neighbour may miss before we suspect it is
    /// gone. Two on a clean channel keeps repair fast; lossy channels need
    /// more, or every unlucky streak would look like a death.
    pub(crate) fn misses_tolerated(&self) -> u32 {
        match self.radio.loss {
            l if l < 0.08 => 2,
            l if l < 0.2 => 3,
            l if l < 0.35 => 4,
            _ => 5,
        }
    }

    /// True once `n` has missed enough beacons to be suspected dead.
    pub(crate) fn suspect(&self, n: &Neighbor) -> bool {
        let interval = u32::from(n.interval_ms.max(50));
        self.age(n.last_heard_ms) > interval * self.misses_tolerated() + 150
    }

    /// Time without a beacon after which `n` is forgotten entirely.
    fn expiry_ms(&self, n: &Neighbor) -> u32 {
        let interval = u32::from(n.interval_ms.max(50));
        let base = self.cfg.neighbor_timeout_ms * interval / self.cfg.beacon_interval_ms;
        base + (self.misses_tolerated() - 2) * interval
    }

    fn handle(&mut self, msg: Message, rssi_dbm: i8) {
        if msg.src == self.id || (msg.dst != BROADCAST && msg.dst != self.id) {
            return;
        }
        match msg.payload {
            Payload::Heartbeat(b) => self.on_beacon(msg.src, b, rssi_dbm),
            Payload::Intent { seq, intent, target } => {
                // Only the base station is wired to the operator console.
                if msg.src != OPERATOR || !self.acts_as_sink() || !target.is_finite() {
                    return;
                }
                if seq_newer(seq, self.intent_seq) {
                    self.adopt_intent(seq, intent, target, self.pos);
                    // The sink floods at once; nobody else can be first.
                    self.radio.flood_at = Some(self.now);
                    self.radio.flood_copies = 0;
                }
            }
            Payload::Mission { seq, intent, target, origin } => {
                if msg.src == OPERATOR || !target.is_finite() || !origin.is_finite() {
                    return;
                }
                if seq_newer(seq, self.intent_seq) {
                    self.adopt_intent(seq, intent, target, origin);
                    self.schedule_flood();
                } else if seq == self.intent_seq {
                    // Trickle-style suppression: if enough neighbours already
                    // repeated it, our copy adds nothing.
                    self.radio.flood_copies = self.radio.flood_copies.saturating_add(1);
                    if self.radio.flood_copies >= 4 && !self.acts_as_sink() {
                        self.radio.flood_at = None;
                    }
                }
            }
        }
    }

    fn on_beacon(&mut self, src: NodeId, b: Beacon, rssi_dbm: i8) {
        let known = self.neighbor(src).map(|n| (n.counter, n.witness_of));
        let witness_of = if b.hops == 1 && b.parent == b.sink {
            b.sink
        } else if b.hops == NO_ROUTE {
            known.map_or(BROADCAST, |k| k.1)
        } else {
            BROADCAST
        };
        if let Some((prev, _)) = known {
            let gap = b.counter.wrapping_sub(prev);
            if (1..=10).contains(&gap) {
                self.observe_loss(u32::from(gap - 1));
            }
        }
        let n = Neighbor {
            id: src,
            hops: b.hops,
            parent: b.parent,
            role: b.role,
            pos: b.pos,
            last_heard_ms: self.now,
            sink: b.sink,
            sink_seq: b.sink_seq,
            rssi_dbm,
            interval_ms: u16::from(b.interval_50ms.max(1)) * 50,
            slot: b.slot,
            flags: b.flags,
            congestion_up: b.congestion_up,
            intent_seq: b.intent_seq,
            counter: b.counter,
            heard_slots: b.heard_slots,
            witness_of,
        };
        self.heard(n);
        self.mission_on_beacon(&n, b.need, b.need_hops);

        // Anti-entropy: a neighbour that missed the latest intent flood gets
        // it re-sent (rate-limited, and suppressed if others beat us to it).
        if self.intent_seq != 0
            && seq_newer(self.intent_seq, b.intent_seq)
            && self.radio.flood_at.is_none()
            && self.radio.last_flood_ms.is_none_or(|t| self.age(t) >= 1500)
        {
            self.schedule_flood();
        }
    }

    /// Exponentially weighted loss estimate, fed one sample per expected
    /// beacon: `missed` losses, then the one that arrived.
    fn observe_loss(&mut self, missed: u32) {
        const W: f32 = 1.0 / 24.0;
        for _ in 0..missed.min(9) {
            self.radio.loss += (1.0 - self.radio.loss) * W;
        }
        self.radio.loss -= self.radio.loss * W;
    }

    fn heard(&mut self, n: Neighbor) {
        if let Some(known) = self.neighbors.iter_mut().find(|e| e.id == n.id) {
            *known = n;
            return;
        }
        if let Err(n) = self.neighbors.push(n) {
            // Table full: evict whoever we heard from least recently, but
            // never the current parent.
            let parent = self.rt.parent;
            if let Some(stalest) = self
                .neighbors
                .iter_mut()
                .filter(|e| Some(e.id) != parent)
                .min_by_key(|e| e.last_heard_ms)
            {
                *stalest = n;
            }
        }
    }

    fn expire_neighbors(&mut self) {
        let mut i = 0;
        while i < self.neighbors.len() {
            let n = self.neighbors[i];
            if self.age(n.last_heard_ms) > self.expiry_ms(&n) {
                self.neighbors.swap_remove(i);
            } else {
                i += 1;
            }
        }
    }

    fn schedule_flood(&mut self) {
        // A small id-derived delay de-synchronises neighbours that heard the
        // same copy, so suppression has a chance to work.
        let backoff = 50 + (u32::from(self.id).wrapping_mul(37) % 7) * 50;
        self.radio.flood_at = Some(self.now.wrapping_add(backoff));
        self.radio.flood_copies = 0;
    }

    fn adopt_intent(&mut self, seq: u16, intent: Intent, target: Vec3, origin: Vec3) {
        self.intent_seq = seq;
        self.intent = intent;
        self.intent_target = target;
        self.adopt_mission(intent, target, origin);
    }

    fn update_role(&mut self) {
        self.role = if self.acts_as_sink() { Role::Sink } else { self.mission_role() };
    }

    pub(crate) fn beacon_interval(&self) -> u32 {
        if self.acts_as_sink() {
            return if self.mission_active() {
                SINK_MISSION_INTERVAL_MS
            } else {
                self.cfg.beacon_interval_ms
            };
        }
        if self.agl < LANDED_AGL || self.parked() {
            return PARKED_INTERVAL_MS.max(self.cfg.beacon_interval_ms);
        }
        if self.protect_control() {
            return (self.cfg.beacon_interval_ms / 2).max(PROTECTED_INTERVAL_MS);
        }
        self.cfg.beacon_interval_ms
    }

    fn transmit(&mut self, outbox: &mut Outbox) {
        let now = self.now;
        if let Some(at) = self.radio.flood_at
            && reached(now, at)
        {
            self.radio.flood_at = None;
            self.radio.last_flood_ms = Some(now);
            if let Some(origin) = self.plan.map(|p| p.origin).or(Some(self.pos)) {
                let mission = Payload::Mission {
                    seq: self.intent_seq,
                    intent: self.intent,
                    target: self.intent_target,
                    origin,
                };
                // The outbox has room for both frames (checked at compile time above).
                let _ = outbox.push(self.frame(BROADCAST, mission));
            }
        }

        let interval = self.beacon_interval();
        // A shorter interval takes effect at once rather than after the
        // long wait already scheduled: the wait left shrinks in proportion.
        // Re-anchoring to `last + interval` instead would send everyone
        // whose interval changed in the same tick (a swarm taking off
        // together) at that same instant, and keep them in lockstep.
        if interval < self.radio.scheduled_ms {
            if !reached(now, self.radio.next_beacon_ms) {
                let left = u64::from(self.radio.next_beacon_ms.wrapping_sub(now));
                let left = left * u64::from(interval) / u64::from(self.radio.scheduled_ms);
                self.radio.next_beacon_ms = now.wrapping_add(left as u32);
            }
            self.radio.scheduled_ms = interval;
        }
        let periodic = reached(now, self.radio.next_beacon_ms);
        let triggered = self.radio.trigger_at.is_some_and(|t| reached(now, t))
            && self.radio.last_beacon_ms.is_some_and(|t| self.age(t) >= MIN_BEACON_GAP_MS);
        if !(periodic || triggered) {
            return;
        }
        self.radio.trigger_at = None;
        self.radio.last_beacon_ms = Some(now);
        if periodic || self.radio.next_beacon_ms.wrapping_sub(now) < MIN_BEACON_GAP_MS {
            let early = self.next_random() % (interval / JITTER_DIV + 1);
            self.radio.next_beacon_ms = now.wrapping_add(interval - early);
            self.radio.scheduled_ms = interval;
        }
        self.radio.counter = self.radio.counter.wrapping_add(1);
        let sink_seq = if self.acts_as_sink() {
            self.rt.bump_sink_seq(self.id);
            self.rt.seq
        } else {
            let hold = self.hold_seq();
            self.rt.advertise(hold)
        };
        let (need, need_hops) = self.next_need();
        let mut flags = self.m.flags();
        if self.agl < LANDED_AGL {
            flags |= FLAG_LANDED;
        }
        if self.carrying() {
            flags |= FLAG_CARRYING;
        }
        if self.rt.orphan {
            flags |= FLAG_ORPHAN;
        }
        flags |= self.congestion_flags();
        let beacon = Beacon {
            congestion_up: self.air.up_level,
            hops: self.rt.hops,
            parent: self.rt.parent.unwrap_or(BROADCAST),
            role: self.role,
            pos: self.pos,
            sink: self.rt.sink,
            sink_seq,
            counter: self.radio.counter,
            intent_seq: self.intent_seq,
            slot: self.m.slot,
            need,
            need_hops,
            flags,
            interval_50ms: (interval / 50).min(255) as u8,
            heard_slots: self.heard_slots(),
        };
        let _ = outbox.push(self.frame(BROADCAST, Payload::Heartbeat(beacon)));
    }

    /// Ask for an early heartbeat, a little later for higher ids so that
    /// neighbours reacting to the same event don't all transmit at once. A
    /// drone that has never beaconed waits for its staggered first slot.
    pub(crate) fn trigger_beacon(&mut self) {
        if self.radio.last_beacon_ms.is_none() {
            return;
        }
        let at = self.now.wrapping_add(20 + (u32::from(self.id).wrapping_mul(53) % 11) * 20);
        if self.radio.trigger_at.is_none_or(|t| reached(t, at)) {
            self.radio.trigger_at = Some(at);
        }
    }

    fn next_random(&mut self) -> u32 {
        let mut x = self.radio.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.radio.rng = x;
        x
    }

    fn frame(&self, dst: NodeId, payload: Payload) -> Frame {
        Message { src: self.id, dst, payload }.encode()
    }
}

/// True if sequence number `a` is newer than `b`, allowing for wrap-around.
pub(crate) fn seq_newer(a: u16, b: u16) -> bool {
    (a.wrapping_sub(b) as i16) > 0
}

/// True once `now` has reached `deadline`, allowing for clock wrap-around.
pub(crate) fn reached(now: u32, deadline: u32) -> bool {
    (now.wrapping_sub(deadline) as i32) >= 0
}

fn stagger(id: NodeId, interval_ms: u32) -> u32 {
    u32::from(id).wrapping_mul(2_654_435_761) % interval_ms.max(1)
}
