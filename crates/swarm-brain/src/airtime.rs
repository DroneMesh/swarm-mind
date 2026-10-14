//! Phase 5: each drone manages the airtime of its own telemetry stream.
//!
//! The host's telemetry task (the firmware's `maybeBroadcastTelem`) sends
//! at whatever interval the brain hands back in
//! [`crate::Actuation::telemetry_interval_ms`]. On a shared 2.4 GHz channel
//! that stream is most of the traffic, so a fleet that all streams at the
//! operator's 25 Hz saturates the air and loses everything, its own control
//! traffic included. Each brain therefore picks its own rate, from local
//! evidence only:
//!
//! * **its own refused sends** (`Sensors::tx_dropped`): the radio's
//!   transmit queue was full, or the telemetry transport refused a package
//!   (painlessMesh `sendSingle` returning false, or a relay's connection
//!   buffer overflowing), the clearest local sign of saturation. Most
//!   of them happen at the relays near the base, which forward everyone's
//!   telemetry, not at the drones that cause them;
//! * **its own loss estimate** (gaps in neighbours' beacon counters, the
//!   same number that widens missed-beacon tolerance). It is noisy and
//!   partly geometry (hidden terminals), so it only ever steers this drone,
//!   and only when it is high;
//! * **congestion levels piggybacked on heartbeats**, in spare bits (no
//!   extra bytes or frames): a 2-bit level *reported up* (the top bits of
//!   the role byte), the largest of the drone's own and those reported by
//!   neighbours farther from the sink; and a 2-bit *verdict* passed down
//!   (`flags` bits 5-6), which the sink sets from what reaches it and
//!   everyone else copies from its parent (or, without a live parent, the
//!   highest of the neighbours nearer the sink).
//!
//! All telemetry funnels to the base, so congestion anywhere along the way
//! is everyone's. Acting on the common verdict rather than on its own
//! evidence, the whole fleet backs off and recovers together, and AIMD on a
//! common signal converges to equal rates without anyone knowing the fleet
//! size. (A relay acting on its own refusals would only starve itself: they
//! mostly come from traffic it forwards.) A drone cut off from the sink has
//! only its own evidence and uses that.
//!
//! The controller is AIMD with hysteresis. Every [`EVAL_MS`]: at level 2+
//! the sink cuts the rate above the floor by [`CUT`] (squared at level 3),
//! then waits [`CUT_HOLD_MS`] for the cut to show before cutting again.
//! Each cut flips a *cut epoch* bit in its heartbeat (`flags` bit 7), which
//! everyone copies from its parent and cuts once when it changes: so the
//! whole fleet cuts the same number of times, however much longer the
//! verdict lingers at depth (it moves about a hop per beacon). Evidence
//! that steers only this drone (its loss estimate; its own refusals when it
//! has no route) cuts on the same timer. After [`CLEAN_BEFORE_RAISE_MS`] of
//! level 0 it adds [`RAISE_HZ`] every [`RAISE_EVERY_MS`], [`PROBE_SLOWDOWN`]
//! times slower once it nears the rate of the last cut (the knee). Level 1
//! holds, but only for [`MILD_HOLD_MS`]: then the climb resumes at the
//! probing pace, so mild congestion slows the fleet but never freezes it,
//! and it keeps probing until the evidence says level 2.
//! A raised own level is
//! held [`LEVEL_HOLD_MS`] so once-a-second neighbours all see it. It starts
//! at the floor, never exceeds the operator's rate and never goes below
//! [`MIN_HZ`] (unless the operator asked for less).
//!
//! Protecting the swarm's own control while telemetry runs:
//! [`Brain::protect_control`] makes drones that carry traffic beacon twice
//! as often on a lossy channel, and [`Brain::routes_via_suspect`] stops a
//! drone adopting a route through a relay it already suspects dead.
//!
//! With telemetry off (`telemetry_requested_ms == 0`) none of this runs: it
//! returns 0, keeps no state and advertises nothing, so heartbeats and
//! behaviour are the same bit for bit.

use crate::brain::{Brain, Sensors, reached};
use crate::NO_ROUTE;
use crate::NodeId;
use crate::wire::{BROADCAST, FLAG_CONGESTION_MASK, FLAG_CONGESTION_SHIFT, FLAG_CUT_EPOCH};

/// Slowest the controller ever goes on its own: 5 Hz (200 ms).
pub const MIN_HZ: f32 = 5.0;
/// Rate a drone starts at when telemetry is switched on (or the operator's,
/// if lower): starting at 25 Hz everywhere would saturate the channel
/// before the first congestion signal came back, so start at the floor and
/// climb.
const START_HZ: f32 = 5.0;
/// The controller re-evaluates this often.
const EVAL_MS: u32 = 500;
/// After a cut, wait this long for its effect before cutting again
/// (loss estimates and hints lag by a beacon or two).
const CUT_HOLD_MS: u32 = 6_000;
/// Clean air needed before the rate may rise again.
const CLEAN_BEFORE_RAISE_MS: u32 = 4_000;
/// Level 1 holds the rate this long, then the climb resumes slowly.
const MILD_HOLD_MS: u32 = 30_000;
/// Rate rises at most once per this long.
const RAISE_EVERY_MS: u32 = 2_000;
/// Below this fraction of the last congested rate the rate rises at the
/// normal pace; above it, [`PROBE_SLOWDOWN`] times slower.
const CEILING_MARGIN: f32 = 0.85;
const PROBE_SLOWDOWN: u32 = 4;
/// Additive increase per raise, Hz (relays: half).
const RAISE_HZ: f32 = 1.0;
/// Multiplicative decrease per cut, of the rate above the floor.
const CUT: f32 = 0.6;
/// Loss-estimate thresholds of congestion levels 1, 2 and 3.
const LOSS_LEVELS: [f32; 3] = [0.5, 0.7, 0.8];
/// Refused-send rate (frames/s, smoothed) thresholds of levels 1, 2 and 3.
const DROP_LEVELS: [f32; 3] = [1.0, 3.0, 10.0];
/// Smoothing of the refused-send rate per evaluation (~2 s time constant).
const DROP_W: f32 = 0.25;
/// An own congestion level is advertised for at least this long.
const LEVEL_HOLD_MS: u32 = 2_500;
/// Control is protected for this long after the last sign that telemetry
/// congests the air (any congestion level above 0).
const PRESSURE_MEMORY_MS: u32 = 30_000;
/// Level at or above which the drone cuts its rate.
const CUT_LEVEL: u8 = 2;

/// Controller state; all zero / `None` while telemetry is off.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Airtime {
    /// Current rate, Hz (0 = telemetry off).
    pub hz: f32,
    pub next_eval_ms: Option<u32>,
    /// Refused sends since the last evaluation.
    pub drops: u32,
    /// Smoothed refused sends per second.
    pub drop_rate: f32,
    /// Congestion level from own evidence, and the one advertised.
    pub own_level: u8,
    pub adv_level: u8,
    /// Time of the last cut.
    pub last_cut_ms: Option<u32>,
    /// How long the air has been clean (level 0), and calm (level 1 or less).
    pub clean_ms: u32,
    pub calm_ms: u32,
    pub last_raise_ms: Option<u32>,
    /// Congestion reported upward ([`crate::Beacon::congestion_up`]).
    pub up_level: u8,
    /// Rate at the last cut (0 = none): where the knee was.
    pub ceiling_hz: f32,
    /// Telemetry was seen congesting the air until about this time.
    pub pressure_until_ms: Option<u32>,
    /// The own level may not fall before this.
    pub hold_until_ms: Option<u32>,
    /// Cut epoch advertised (the sink's own; everyone else's copied from
    /// its parent), and the parent it was copied from.
    pub epoch: bool,
    pub epoch_from: Option<NodeId>,
}

/// What [`Brain::heard_congestion`] found.
struct Heard {
    /// The sink's verdict as passed down.
    down: u8,
    /// The parent and the cut epoch it advertises, if it is live.
    epoch: Option<(NodeId, bool)>,
    /// Highest level reported up from farther out.
    up: u8,
}

fn level_of(value: f32, thresholds: &[f32; 3]) -> u8 {
    thresholds.iter().filter(|&&t| value >= t).count() as u8
}

/// Congestion level carried in a heartbeat's `flags`.
pub(crate) fn flags_level(flags: u8) -> u8 {
    (flags & FLAG_CONGESTION_MASK) >> FLAG_CONGESTION_SHIFT
}

impl Brain {
    /// Telemetry interval to ask the host for, in ms (0 = none).
    pub(crate) fn telemetry_interval(&mut self, sensors: &Sensors) -> u16 {
        let requested = sensors.telemetry_requested_ms;
        if requested == 0 {
            self.air = Airtime::default();
            return 0;
        }
        let req_hz = 1000.0 / f32::from(requested);
        let floor = MIN_HZ.min(req_hz);
        let now = self.now;
        let a = &mut self.air;
        a.drops = a.drops.saturating_add(u32::from(sensors.tx_dropped));
        if a.hz <= 0.0 {
            a.hz = START_HZ.min(req_hz);
            a.next_eval_ms = Some(now.wrapping_add(EVAL_MS));
        }
        a.hz = a.hz.clamp(floor, req_hz);
        if self.air.next_eval_ms.is_some_and(|t| reached(now, t)) {
            self.air.next_eval_ms = Some(now.wrapping_add(EVAL_MS));
            self.evaluate_airtime(req_hz, floor);
        }
        let ms = (1000.0 / self.air.hz + 0.5) as u32;
        ms.clamp(u32::from(requested), u32::from(u16::MAX)) as u16
    }

    /// One AIMD step, every [`EVAL_MS`].
    fn evaluate_airtime(&mut self, req_hz: f32, floor: f32) {
        let now = self.now;
        let per_s = self.air.drops as f32 * (1000.0 / EVAL_MS as f32);
        self.air.drops = 0;
        self.air.drop_rate += (per_s - self.air.drop_rate) * DROP_W;
        // Refusals are evidence about the air our telemetry crosses, so they
        // are advertised. The loss estimate is noisy (one sample per
        // neighbour beacon) and partly geometry (hidden terminals), so it
        // only steers this drone.
        let own = level_of(self.air.drop_rate, &DROP_LEVELS);
        let local = level_of(self.radio.loss, &LOSS_LEVELS);
        // A raised level is held for a while, so that neighbours beaconing
        // once a second all see it rather than a flicker.
        if own >= self.air.own_level || self.air.hold_until_ms.is_none_or(|t| reached(now, t)) {
            if own > 0 {
                self.air.hold_until_ms = Some(now.wrapping_add(LEVEL_HOLD_MS));
            }
            self.air.own_level = own;
        }
        // Own congestion is not acted on directly: it travels up the
        // gradient to the sink (`up`), and the sink's verdict comes back
        // down to everyone (`adv_level`). So every drone sharing the funnel
        // to the base reacts to the same signal at about the same time, and
        // AIMD on a common signal converges to equal rates without anyone
        // knowing the fleet size. A relay's own refusals mostly come from
        // the traffic it forwards; cutting only itself would starve it and
        // spare the senders.
        let heard = self.heard_congestion();
        let own = self.air.own_level;
        self.air.up_level = own.max(heard.up);
        let sink = self.acts_as_sink();
        let reported = if sink { self.air.up_level } else { 0 };
        self.air.adv_level = heard.down.max(reported);
        // Cut off from the sink, a drone has only its own evidence.
        let alone = if self.rt.hops == NO_ROUTE { own } else { 0 };
        // Evidence that steers only this drone.
        let solo = local.max(alone);
        let level = self.air.adv_level.max(solo);
        if own.max(self.air.up_level).max(self.air.adv_level) > 0 {
            self.air.pressure_until_ms = Some(now.wrapping_add(PRESSURE_MEMORY_MS));
        }

        // Cuts. The common verdict cuts once per decision of the sink, not
        // for as long as it is heard: it reaches a drone k hops out about k
        // beacons late and lingers as long after it has cleared, so cutting
        // on its level alone would cut the far drones (the scouts at the
        // fire) more often than the near ones, every episode. The sink
        // decides on the usual timer and flips its cut epoch; everyone
        // copies the epoch from its parent and cuts when it changes.
        let a = &mut self.air;
        let held = a.last_cut_ms.is_none_or(|t| now.wrapping_sub(t) >= CUT_HOLD_MS);
        let mut cut = None;
        if sink {
            if a.adv_level >= CUT_LEVEL && held {
                a.epoch = !a.epoch;
                cut = Some(a.adv_level);
            }
        } else if let Some((parent, epoch)) = heard.epoch {
            // A new parent's epoch is adopted as it is: it is no decision.
            // The sink flips it only with a verdict of 2+, and each drone
            // passes both on together.
            if a.epoch_from == Some(parent) && epoch != a.epoch && heard.down >= CUT_LEVEL {
                cut = Some(heard.down);
            }
            a.epoch = epoch;
            a.epoch_from = Some(parent);
        }
        if cut.is_none() && solo >= CUT_LEVEL && held {
            cut = Some(solo);
        }
        if let Some(level) = cut {
            a.clean_ms = 0;
            a.calm_ms = 0;
            // Level 3 (refusals piling up): cut twice as deep. The cut
            // applies to the part above the floor, so fast senders give up
            // more than slow ones and rates converge.
            let k = if level >= 3 { CUT * CUT } else { CUT };
            // The knee is the rate at the first cut of an episode; a
            // follow-up cut while the evidence lingers keeps it.
            let episode = a.last_cut_ms.is_some_and(|t| now.wrapping_sub(t) < 3 * CUT_HOLD_MS);
            a.ceiling_hz = if episode { a.ceiling_hz.max(a.hz) } else { a.hz };
            a.hz = floor + (a.hz - floor) * k;
            a.last_cut_ms = Some(now);
            return;
        }
        a.clean_ms = if level == 0 { a.clean_ms.saturating_add(EVAL_MS) } else { 0 };
        a.calm_ms = if level <= 1 { a.calm_ms.saturating_add(EVAL_MS) } else { 0 };
        // Level 1 (mild: a relay refusing a frame or two a second, or a
        // loss estimate creeping up) holds the rate for a while, but not
        // for ever, or one busy relay would freeze the whole fleet wherever
        // it happened to be: after [`MILD_HOLD_MS`] of it the climb resumes
        // at the probing pace. If the rate it allows is too much, the
        // evidence rises to level 2 and cuts.
        let mild = a.clean_ms < CLEAN_BEFORE_RAISE_MS;
        if mild && a.calm_ms < MILD_HOLD_MS {
            return;
        }
        let step = RAISE_HZ;
        // Back up to just below the rate that last congested quickly, then
        // probe beyond it slowly: most of the time is then spent under the
        // knee instead of see-sawing across it.
        let probing = mild || (a.ceiling_hz > 0.0 && a.hz + step > a.ceiling_hz * CEILING_MARGIN);
        let every = if probing { RAISE_EVERY_MS * PROBE_SLOWDOWN } else { RAISE_EVERY_MS };
        if a.last_raise_ms.is_none_or(|t| now.wrapping_sub(t) >= every) {
            a.hz = (a.hz + step).min(req_hz);
            a.last_raise_ms = Some(now);
            if a.hz > a.ceiling_hz {
                a.ceiling_hz = 0.0; // the knee moved: forget it
            }
        }
    }

    /// Congestion heard from others. Down: the sink's verdict and cut epoch
    /// as passed on by the parent, which is the freshest (it is one hop
    /// nearer the sink, and the one our telemetry goes through); without a
    /// live parent, the highest verdict of any neighbour nearer the sink,
    /// and no epoch. Up: the highest level reported by any neighbour
    /// farther from the sink.
    fn heard_congestion(&self) -> Heard {
        let hops = self.rt.hops;
        let parent = self.rt.parent.unwrap_or(BROADCAST);
        let mut nearer = 0;
        let mut up = 0;
        let mut from_parent = None;
        for n in self.neighbors.iter().filter(|n| !self.suspect(n)) {
            if n.id == parent {
                from_parent = Some(n.flags);
            } else if n.hops < hops {
                nearer = nearer.max(flags_level(n.flags));
            } else if n.hops > hops && n.hops != NO_ROUTE {
                up = up.max(n.congestion_up);
            }
        }
        match from_parent {
            Some(f) => Heard { down: flags_level(f), epoch: Some((parent, f & FLAG_CUT_EPOCH != 0)), up },
            None => Heard { down: nearer, epoch: None, up },
        }
    }

    /// True while this drone should beacon faster to keep the swarm's own
    /// control responsive: telemetry is on and has recently been seen
    /// congesting the air (it is what fills it), the measured loss is high
    /// enough that neighbours tolerate four or more missed beacons, and
    /// others route through us. Children notice a dead
    /// relay after `interval x misses`, so halving the interval of the few
    /// drones that carry traffic keeps repair within a couple of seconds on
    /// a busy channel, for a few hundred bytes a second.
    pub(crate) fn protect_control(&self) -> bool {
        self.under_pressure() && self.misses_tolerated() > 3 && self.carrying()
    }

    /// Telemetry runs and has recently been seen congesting the air (own
    /// refusals, or a level reported or passed down). Loss alone (a noisy
    /// link, another network) is not telemetry's doing and changes nothing.
    fn under_pressure(&self) -> bool {
        self.air.hz > 0.0 && self.air.pressure_until_ms.is_some_and(|t| !reached(self.now, t))
    }

    /// True if `n` still routes through a drone we already suspect dead.
    /// Its advertisement is stale (it has not noticed yet), and adopting it
    /// would put us back behind the dead relay until it does. Applied only
    /// under telemetry pressure (as [`Self::protect_control`]); otherwise
    /// the brain behaves exactly as before Phase 5.
    pub(crate) fn routes_via_suspect(&self, n: &crate::Neighbor) -> bool {
        self.under_pressure() && self.neighbor(n.parent).is_some_and(|p| self.suspect(p))
    }

    /// Heartbeat flag bits for the advertised congestion level.
    pub(crate) fn congestion_flags(&self) -> u8 {
        let epoch = if self.air.epoch { FLAG_CUT_EPOCH } else { 0 };
        ((self.air.adv_level.min(3) << FLAG_CONGESTION_SHIFT) & FLAG_CONGESTION_MASK) | epoch
    }

    /// Congestion level this drone advertises (0-3; 0 with telemetry off).
    pub fn congestion_level(&self) -> u8 {
        self.air.adv_level
    }

    /// Controller internals for tests and probes:
    /// `(refused sends/s smoothed, own level, level reported up)`.
    #[doc(hidden)]
    pub fn debug_airtime(&self) -> (f32, u8, u8) {
        (self.air.drop_rate, self.air.own_level, self.air.up_level)
    }

    /// Telemetry rate the controller currently chooses, Hz (0 = off).
    pub fn telemetry_hz(&self) -> f32 {
        self.air.hz
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::wire::{Beacon, FLAG_SETTLED, Message, Payload, Role, Rx};
    use crate::{Config, NodeId, Outbox, Vec3};

    const STEP: u32 = 50;
    /// 25 Hz, the operator's usual request.
    const REQ_MS: u16 = 40;
    /// Long enough on clean air to climb from the start to 25 Hz.
    const WARM_MS: u32 = 60_000;

    fn hb(src: NodeId, hops: u8, parent: NodeId, seq: u16, flags: u8, up: u8) -> Rx {
        let b = Beacon {
            hops,
            parent,
            role: if hops == 0 { Role::Sink } else { Role::Relay },
            congestion_up: up,
            pos: Vec3::ZERO,
            sink: 0,
            sink_seq: seq,
            counter: seq as u8,
            intent_seq: 0,
            slot: 0,
            need: 0,
            need_hops: 0,
            flags: FLAG_SETTLED | flags,
            interval_50ms: 20,
            heard_slots: 0,
        };
        Rx { frame: Message { src, dst: BROADCAST, payload: Payload::Heartbeat(b) }.encode(), rssi_dbm: -60 }
    }

    /// A drone at 50 above ground, ticked every 50 ms.
    struct Rig {
        b: Brain,
        now: u32,
    }

    impl Rig {
        fn new(id: NodeId, sink: bool) -> Self {
            Self { b: Brain::new(id, sink, Vec3::ZERO, Config::default()), now: 1_000 }
        }

        /// One tick; returns the chosen interval and the heartbeat sent, if any.
        fn tick(&mut self, req: u16, dropped: u16, inbox: &[Rx]) -> (u16, Option<Beacon>) {
            self.now += STEP;
            let s = Sensors {
                now_ms: self.now,
                pos: Vec3::new(0.0, 50.0, 0.0),
                ground_agl: 50.0,
                tx_dropped: dropped,
                telemetry_requested_ms: req,
                ..Sensors::default()
            };
            let mut out = Outbox::new();
            let act = self.b.tick(&s, inbox, &mut out);
            let beacon = out.iter().find_map(|f| match Message::decode(f) {
                Some(Message { payload: Payload::Heartbeat(b), .. }) => Some(b),
                _ => None,
            });
            (act.telemetry_interval_ms, beacon)
        }

        /// `ms` of ticks with `drops_per_s` refused sends; the last interval.
        fn run(&mut self, ms: u32, req: u16, drops_per_s: u16) -> u16 {
            let mut last = 0;
            for k in 0..ms / STEP {
                // Spread the refusals evenly over the second.
                let per_tick = u32::from(drops_per_s) * STEP;
                let d = (per_tick * (k + 1) / 1000 - per_tick * k / 1000) as u16;
                last = self.tick(req, d, &[]).0;
            }
            last
        }
    }

    #[test]
    fn off_means_zero_and_no_trace_on_the_wire() {
        let mut r = Rig::new(3, false);
        for _ in 0..100 {
            let (ms, beacon) = r.tick(0, 9, &[]);
            assert_eq!(ms, 0);
            if let Some(b) = beacon {
                assert_eq!(b.flags & FLAG_CONGESTION_MASK, 0);
                assert_eq!(b.congestion_up, 0);
            }
        }
        assert_eq!(r.b.telemetry_hz(), 0.0);
        assert_eq!(r.b.congestion_level(), 0);
        // Switching telemetry off again forgets everything.
        r.run(5_000, REQ_MS, 20);
        assert!(r.b.air.drop_rate > 0.0);
        r.tick(0, 0, &[]);
        assert_eq!(r.b.air.hz, 0.0);
        assert_eq!(r.b.air.drop_rate, 0.0);
    }

    #[test]
    fn starts_low_and_climbs_to_the_request_on_clean_air() {
        let mut r = Rig::new(3, false);
        let (first, _) = r.tick(REQ_MS, 0, &[]);
        assert_eq!(first, (1000.0 / START_HZ) as u16);
        let mut prev = first;
        let mut t = 0u32;
        while prev > REQ_MS && t < 120_000 {
            let ms = r.run(500, REQ_MS, 0);
            assert!(ms <= prev, "interval rose to {ms} ms on clean air");
            prev = ms;
            t += 500;
        }
        assert_eq!(prev, REQ_MS, "never reached the request");
        // Additive: +1 Hz per 2 s, after the first clean 4 s.
        let expect = CLEAN_BEFORE_RAISE_MS + ((25.0 - START_HZ) / RAISE_HZ) as u32 * RAISE_EVERY_MS;
        assert!(t.abs_diff(expect) <= 2_000, "took {t} ms, expected about {expect}");
        // And never beyond it.
        assert_eq!(r.run(20_000, REQ_MS, 0), REQ_MS);
    }

    #[test]
    fn never_faster_than_asked_and_never_below_the_floor() {
        let mut r = Rig::new(3, false);
        // A slow request is honoured as is, whatever happens.
        assert_eq!(r.run(10_000, 500, 0), 500);
        assert_eq!(r.run(10_000, 500, 50), 500);
        // Persistent heavy refusals bottom out at the floor, 5 Hz.
        let mut r = Rig::new(3, false);
        for _ in 0..60 {
            let ms = r.run(1_000, REQ_MS, 40);
            assert!((REQ_MS..=200).contains(&ms), "{ms} ms");
        }
        assert_eq!(r.run(1_000, REQ_MS, 40), 200);
    }

    #[test]
    fn cuts_are_multiplicative_and_spaced() {
        // A drone with no route acts on its own evidence.
        let mut r = Rig::new(3, false);
        r.run(WARM_MS, REQ_MS, 0);
        assert_eq!(r.b.air.hz, 25.0);
        // Sustained refusals (level 2): the first cut is immediate, ...
        let mut cuts = std::vec::Vec::new();
        let mut last = r.b.air.hz;
        for k in 0..(12_000 / STEP) {
            r.tick(REQ_MS, if k % 4 == 0 { 1 } else { 0 }, &[]);
            if r.b.air.hz < last {
                cuts.push((k * STEP, last, r.b.air.hz));
                last = r.b.air.hz;
            }
        }
        assert!(cuts.len() >= 2, "{cuts:?}");
        let (_, before, after) = cuts[0];
        let expect = MIN_HZ + (before - MIN_HZ) * CUT;
        assert!((after - expect).abs() < 1e-3, "{cuts:?}");
        // ... and the next waits for the first to take effect.
        for w in cuts.windows(2) {
            assert!(w[1].0 - w[0].0 >= CUT_HOLD_MS, "{cuts:?}");
        }
    }

    #[test]
    fn recovery_waits_for_clean_air_then_probes_slowly_near_the_old_knee() {
        let mut r = Rig::new(3, false);
        r.run(WARM_MS, REQ_MS, 0);
        r.run(2_000, REQ_MS, 30); // one cut
        let knee = r.b.air.ceiling_hz;
        assert_eq!(knee, 25.0);
        let cut = r.b.air.hz;
        assert!(cut < knee);
        // Hysteresis: the level is held and the air must stay clean for a
        // while, so nothing rises at once.
        r.run(CLEAN_BEFORE_RAISE_MS, REQ_MS, 0);
        assert_eq!(r.b.air.hz, cut);
        // Back up quickly to below the knee, then slowly.
        let mut t_fast = 0;
        while r.b.air.hz + RAISE_HZ <= knee * CEILING_MARGIN {
            r.run(500, REQ_MS, 0);
            t_fast += 500;
            assert!(t_fast < 60_000);
        }
        let near = r.b.air.hz;
        r.run(RAISE_EVERY_MS * 2, REQ_MS, 0);
        assert_eq!(r.b.air.hz, near, "probing past the old knee at the normal pace");
        r.run(RAISE_EVERY_MS * PROBE_SLOWDOWN, REQ_MS, 0);
        assert!(r.b.air.hz > near);
    }

    #[test]
    fn a_held_level_does_not_flicker() {
        let mut r = Rig::new(3, false);
        r.run(2_000, REQ_MS, 0);
        r.tick(REQ_MS, 2, &[]); // one burst
        r.run(EVAL_MS, REQ_MS, 0);
        assert!(r.b.air.own_level >= 1);
        r.run(LEVEL_HOLD_MS - 2 * EVAL_MS, REQ_MS, 0);
        assert!(r.b.air.own_level >= 1, "level dropped before the hold ran out");
        r.run(LEVEL_HOLD_MS + 2_000, REQ_MS, 0);
        assert_eq!(r.b.air.own_level, 0);
    }

    /// Relay 1 hears the sink (0) and a congested drone beyond it (2).
    #[test]
    fn congestion_goes_up_to_the_sink_and_its_verdict_comes_down() {
        let mut relay = Rig::new(1, false);
        let mut seq = 1;
        let calm = |seq| [hb(0, 0, BROADCAST, seq, 0, 0)];
        for _ in 0..(WARM_MS / STEP) {
            relay.tick(REQ_MS, 0, &calm(seq));
            seq += 1;
        }
        assert_eq!(relay.b.hops(), 1);
        let hz = relay.b.air.hz;
        assert_eq!(hz, 25.0);

        // A congested drone farther out: the relay passes it up, but does
        // not cut on hearsay; only the sink decides.
        let report = hb(2, 2, 1, 1, 0, 3);
        for _ in 0..(3_000 / STEP) {
            let (_, b) = relay.tick(REQ_MS, 0, &[calm(seq)[0], report]);
            seq += 1;
            if let Some(b) = b {
                assert_eq!(flags_level(b.flags), 0);
            }
        }
        assert_eq!(relay.b.air.up_level, 3);
        assert_eq!(relay.b.air.hz, hz);

        // The sink hears the report and passes a verdict down to everyone.
        let mut sink = Rig::new(0, true);
        let relay_up = hb(1, 1, 0, 1, 0, 3);
        for _ in 0..(1_000 / STEP) {
            sink.tick(REQ_MS, 0, &[relay_up]);
        }
        assert_eq!(sink.b.congestion_level(), 3);
        // The sink cut and flipped its epoch: its heartbeats say so.
        assert!(sink.b.air.hz < 25.0 && sink.b.air.epoch);
        let mut sent = None;
        for _ in 0..(2_000 / STEP) {
            if let (_, Some(b)) = sink.tick(REQ_MS, 0, &[relay_up]) {
                sent = Some(b.flags);
            }
        }
        let verdict = sent.expect("the sink beacons") & (FLAG_CONGESTION_MASK | FLAG_CUT_EPOCH);
        assert_eq!(verdict, (3 << FLAG_CONGESTION_SHIFT) | FLAG_CUT_EPOCH);
        let mut cut_at = None;
        for k in 0..(2_000 / STEP) {
            relay.tick(REQ_MS, 0, &[hb(0, 0, BROADCAST, seq, verdict, 0)]);
            seq += 1;
            if relay.b.air.hz < hz {
                cut_at.get_or_insert(k * STEP);
            }
        }
        assert!(cut_at.is_some_and(|t| t <= EVAL_MS), "{cut_at:?}");
        assert_eq!(relay.b.congestion_level(), 3, "the verdict travels on down");
        assert_eq!(relay.b.congestion_flags() & FLAG_CUT_EPOCH, FLAG_CUT_EPOCH, "and so does the epoch");
    }

    /// The verdict reaches a drone k hops out k beacons late and lingers as
    /// long after it cleared. However long it is heard, a drone cuts once
    /// per decision of the sink (epoch flip), so the far drones (the scouts
    /// at the fire) are not cut more often than the near ones.
    #[test]
    fn a_lingering_verdict_cuts_once_per_sink_decision() {
        let mut r = Rig::new(5, false);
        let mut seq = 1;
        let mut epoch = 0;
        let from_parent = |seq, flags| [hb(4, 3, 3, seq, flags, 0)];
        for _ in 0..(WARM_MS / STEP) {
            r.tick(REQ_MS, 0, &from_parent(seq, 0));
            seq += 1;
        }
        assert_eq!((r.b.hops(), r.b.parent()), (4, Some(4)));
        assert_eq!(r.b.air.hz, 25.0);
        let mut cuts = 0;
        let mut last = r.b.air.hz;
        // One decision, its verdict heard for 20 s (> 3 cut holds).
        epoch ^= FLAG_CUT_EPOCH;
        for _ in 0..(20_000 / STEP) {
            r.tick(REQ_MS, 0, &from_parent(seq, (2 << FLAG_CONGESTION_SHIFT) | epoch));
            seq += 1;
            if r.b.air.hz < last {
                cuts += 1;
                last = r.b.air.hz;
            }
        }
        assert_eq!(cuts, 1, "cut on the tail of a verdict");
        assert!((last - (MIN_HZ + (25.0 - MIN_HZ) * CUT)).abs() < 1e-3, "{last}");
        // A second decision: a second cut.
        epoch ^= FLAG_CUT_EPOCH;
        for _ in 0..(2_000 / STEP) {
            r.tick(REQ_MS, 0, &from_parent(seq, (2 << FLAG_CONGESTION_SHIFT) | epoch));
            seq += 1;
            if r.b.air.hz < last {
                cuts += 1;
                last = r.b.air.hz;
            }
        }
        assert_eq!(cuts, 2);
        // A neighbour nearer the sink that is not our parent may carry a
        // stale verdict and epoch of its own: neither cuts us.
        let other = |seq, flags| [hb(4, 3, 3, seq, 2 << FLAG_CONGESTION_SHIFT | epoch, 0), hb(6, 3, 2, seq, flags, 0)];
        for k in 0..(20_000 / STEP) {
            let e = if (k / 20) % 2 == 0 { 0 } else { FLAG_CUT_EPOCH };
            r.tick(REQ_MS, 0, &other(seq, (3 << FLAG_CONGESTION_SHIFT) | e));
            seq += 1;
            assert_eq!(r.b.parent(), Some(4));
        }
        assert_eq!(r.b.air.hz, last, "a non-parent's flags cut us");
    }

    /// The sink (hence everyone) says level 1 for a long time, with no
    /// other evidence: the rate keeps climbing, at the probing pace, rather
    /// than freezing the whole fleet wherever it happened to be.
    #[test]
    fn sink_level_one_slows_the_climb_but_never_freezes_it() {
        let mut r = Rig::new(1, false);
        let mut seq = 1;
        for _ in 0..(10_000 / STEP) {
            r.tick(REQ_MS, 0, &[hb(0, 0, BROADCAST, seq, 0, 0)]);
            seq += 1;
        }
        // A moment of level 2 (no cut decision): the air is not calm.
        for _ in 0..(1_000 / STEP) {
            r.tick(REQ_MS, 0, &[hb(0, 0, BROADCAST, seq, 2 << FLAG_CONGESTION_SHIFT, 0)]);
            seq += 1;
        }
        let hz = r.b.air.hz;
        assert!(hz < 25.0);
        let mut raises = std::vec::Vec::new();
        let mut last = hz;
        for k in 0..(90_000 / STEP) {
            r.tick(REQ_MS, 0, &[hb(0, 0, BROADCAST, seq, 1 << FLAG_CONGESTION_SHIFT, 0)]);
            seq += 1;
            if r.b.air.hz > last {
                raises.push(k * STEP);
                last = r.b.air.hz;
            }
        }
        assert!(raises.len() >= 5, "frozen at level 1: {raises:?}");
        assert!(raises[0] >= MILD_HOLD_MS - EVAL_MS, "level 1 did not hold at first: {raises:?}");
        for w in raises.windows(2) {
            assert!(w[1] - w[0] >= RAISE_EVERY_MS * PROBE_SLOWDOWN, "level 1 raised at the normal pace: {raises:?}");
        }
        // Level 2 still stops it.
        let before = r.b.air.hz;
        for _ in 0..(20_000 / STEP) {
            r.tick(REQ_MS, 0, &[hb(0, 0, BROADCAST, seq, 2 << FLAG_CONGESTION_SHIFT, 0)]);
            seq += 1;
        }
        assert!(r.b.air.hz <= before);
    }

    #[test]
    fn congestion_rides_in_spare_bits() {
        let b = Beacon { role: Role::Scout, congestion_up: 2, flags: FLAG_SETTLED | (3 << FLAG_CONGESTION_SHIFT), ..Beacon::default() };
        let m = Message { src: 4, dst: BROADCAST, payload: Payload::Heartbeat(b) };
        let f = m.encode();
        assert_eq!(Message::decode(&f), Some(m));
        let plain = Message { payload: Payload::Heartbeat(Beacon { role: Role::Scout, flags: FLAG_SETTLED, ..b }), ..m };
        let plain = plain.encode();
        assert_eq!(plain.as_bytes().len(), f.as_bytes().len(), "no extra bytes");
    }

    #[test]
    fn relays_beacon_faster_only_under_telemetry_pressure_on_a_lossy_channel() {
        let mut r = Rig::new(1, false);
        let mut seq = 1;
        // Drone 2 routes through us.
        let inbox = |seq| [hb(0, 0, BROADCAST, seq, 0, 0), hb(2, 2, 1, seq, 0, 0)];
        for _ in 0..(5_000 / STEP) {
            r.tick(0, 0, &inbox(seq));
            seq += 1;
        }
        r.b.radio.loss = 0.3;
        assert!(r.b.misses_tolerated() > 3 && r.b.carrying());
        assert_eq!(r.b.beacon_interval(), 1000, "telemetry off: unchanged");
        // Telemetry on but nothing congested: loss alone changes nothing.
        for _ in 0..(5_000 / STEP) {
            r.tick(REQ_MS, 0, &inbox(seq));
            seq += 1;
        }
        r.b.radio.loss = 0.3;
        assert_eq!(r.b.beacon_interval(), 1000, "no telemetry pressure: unchanged");
        // The sink says the air is congested.
        let verdict = 1 << FLAG_CONGESTION_SHIFT;
        for _ in 0..(1_000 / STEP) {
            r.tick(REQ_MS, 0, &[hb(0, 0, BROADCAST, seq, verdict, 0), hb(2, 2, 1, seq, 0, 0)]);
            seq += 1;
        }
        r.b.radio.loss = 0.3;
        assert_eq!(r.b.beacon_interval(), 500);
        r.b.radio.loss = 0.1;
        assert_eq!(r.b.beacon_interval(), 1000, "clean channel: unchanged");
        // The pressure is remembered for a while, then forgotten.
        for _ in 0..((PRESSURE_MEMORY_MS + 1_000) / STEP) {
            r.tick(REQ_MS, 0, &inbox(seq));
            seq += 1;
        }
        r.b.radio.loss = 0.3;
        assert_eq!(r.b.beacon_interval(), 1000);
    }
}
