//! Routing to the sink: a loop-free distance-vector gradient with sink
//! freshness, fast local repair and sink election.
//!
//! * **Gradient.** Each heartbeat advertises `(sink, sink_seq, hops)`. A
//!   drone's route is its parent's advertisement plus one hop; it always
//!   follows its current parent's updates.
//! * **Loop freedom** (the Babel/DSDV feasibility condition). Switching to a
//!   new parent is allowed only if that parent advertises *newer* sink
//!   freshness than we last advertised, or the same freshness and strictly
//!   fewer hops than the fewest we advertised for it. A drone's descendants
//!   can never satisfy this, even acting on stale information at the same
//!   instant, so repairs never form loops and there is no count-to-infinity;
//!   a hop cap backs it up.
//! * **Hold-down.** Freshness travels hop by hop, so a sideways neighbour is
//!   often one counter behind us and therefore never feasible. A drone that
//!   wants to leave a poor link for a neighbour with no more hops than its
//!   own, but is blocked only by that, keeps advertising its previous
//!   counter for about one beacon interval (always safe: it claims less
//!   than it knows) until the neighbour's news overtakes it. Never longer:
//!   while the counter stands still, the drone's whole subtree sees its
//!   route go stale and nobody can newly adopt it.
//! * **Freshness.** The sink bumps `sink_seq` every beacon. A route whose
//!   counter stops advancing is suspect, then stale: stale routes are
//!   dropped and advertised as `NO_ROUTE` (poison), and the sink is marked
//!   dead so its old advertisements can't be resurrected by echoes.
//! * **Fast repair.** A parent that misses two beacons (more on lossy
//!   channels, see `misses_tolerated`) is suspected; if a feasible
//!   alternative exists the drone switches at once, and a triggered
//!   heartbeat tells its children. The ladder formation guarantees such an
//!   alternative after any single relay loss.
//! * **Sink election.** Drones that heard the sink directly ("witnesses")
//!   notice when it falls silent. The lowest-id live witness that stayed
//!   about where it heard the sink (one that flew on may just have lost
//!   sight of it) declares itself acting sink; the others defer to it (each
//!   neighbour's witness status is remembered, since all of them poison
//!   their routes before election time). From afar a shadow and a death
//!   look the same, so an acting sink away from the base's spot flies there
//!   to check. Gradients from lower sink ids win, so if two partitions each
//!   elected one, they merge into the lower id when they meet, and the
//!   acting sink steps down for the real base. A group cut off for a long
//!   time elects its lowest id too, but marks that gradient as an orphan's
//!   (`FLAG_ORPHAN`): the base is probably alive, so the group keeps flying
//!   back toward it (`mission.rs`) instead of settling for its own sink.

use crate::brain::{Brain, NO_ROUTE, seq_newer};
use crate::wire::{BROADCAST, FLAG_ORPHAN, NodeId, Role};
use crate::{Neighbor, Vec3};

/// Routes this long are treated as loops.
pub const MAX_HOPS: u8 = 16;
/// Advertisements of a dead sink are ignored unless their counter is at
/// least this much newer than the last we saw (the live base passes this
/// within seconds; echoes of a dead one never do).
const DEAD_SLACK: u16 = 8;
/// How long a dead-sink mark is kept.
const DEAD_MEMORY_MS: u32 = 60_000;
/// Drones that lost every route elect a sink among themselves after this
/// long, so a swarm whose base and every base neighbour died still works.
const ORPHAN_ELECT_MS: u32 = 30_000;
/// A witness defers to a lower-id one for at most this long past its own
/// election time; two acting sinks resolve by step-down anyway.
const ELECT_GRACE_MS: u32 = 2_500;
/// A witness that has moved further than this (xz) since it last heard the
/// sink does not elect: the silence is more likely its own flight carrying
/// it out of sight (behind a hill) than the sink dying, and a second sink
/// that stays put would split the swarm off a live base for good. It keeps
/// regrouping toward the sink instead and, if still cut off, joins the
/// orphan election, whose sink flies back to look for the base. Settled and
/// parked drones barely move, so they still take over from a dead base at
/// once.
const WITNESS_DRIFT: f32 = 30.0;
/// An orphan sink this close (xz) to the mission origin stops looking for
/// the base there.
const ORIGIN_REACHED: f32 = 40.0;
/// Link quality worse than this counts as weak: the link is close to the
/// edge of range (about 225 of 260 units) and would break first.
pub const WEAK_RSSI_DBM: i8 = -94;
/// Extra cost, in quarter hops, for links at or below these signal levels
/// (roughly 195, 225 and 245 units). A marginal link must lose to a solid
/// one even when the solid one is a hop or two longer, or routes hug the
/// edge of range and snap as soon as anything moves.
const LINK_PENALTY: [(i8, u16); 3] = [(-95, 16), (WEAK_RSSI_DBM, 8), (-92, 4)];
/// Parent choice only switches for an improvement of at least this much
/// (4 = one hop), so near-equal parents don't flap.
const SWITCH_MARGIN: u16 = 3;

#[derive(Clone, Copy, Debug, Default)]
struct DeadSink {
    sink: NodeId,
    seq: u16,
    at_ms: u32,
    valid: bool,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Routing {
    /// Sink our route leads to ([`BROADCAST`] if we never had one).
    pub sink: NodeId,
    pub seq: u16,
    pub hops: u8,
    pub parent: Option<NodeId>,
    /// Feasibility distance: the best hops advertised for `(fd_sink, fd_seq)`.
    fd_sink: NodeId,
    fd_seq: u16,
    fd_hops: u8,
    /// When our route's sink counter last advanced.
    seq_at_ms: u32,
    /// Elected acting sink.
    pub acting: bool,
    /// Our route leads to an orphan-elected sink, or we are one (see
    /// [`crate::wire::FLAG_ORPHAN`]).
    pub orphan: bool,
    /// Witness-elected acting sink still away from the mission origin: it
    /// flies there to check that the base is really gone (see
    /// `Brain::maybe_elect`). Unlike `orphan`, its group stays put.
    pub verify: bool,
    /// The sink we heard directly, while we did.
    witness_of: Option<NodeId>,
    /// Where we were when we last heard it directly, and whether we have
    /// since moved away from there (see [`WITNESS_DRIFT`]); coming back
    /// doesn't undo it, only hearing the sink again does.
    witness_at: Vec3,
    witness_left: bool,
    dead: [DeadSink; 2],
    /// Last known position of our parent: where to look when it is lost.
    pub parent_pos: Option<Vec3>,
    pub no_route_since: Option<u32>,
    /// Since when a better parent has been blocked only by feasibility.
    pub blocked_since: Option<u32>,
}

impl Routing {
    pub fn new(id: NodeId, is_sink: bool) -> Self {
        Self {
            sink: if is_sink { id } else { BROADCAST },
            seq: 0,
            hops: if is_sink { 0 } else { NO_ROUTE },
            parent: None,
            fd_sink: BROADCAST,
            fd_seq: 0,
            fd_hops: NO_ROUTE,
            seq_at_ms: 0,
            acting: false,
            orphan: false,
            verify: false,
            witness_of: None,
            witness_at: Vec3::ZERO,
            witness_left: false,
            dead: [DeadSink::default(); 2],
            parent_pos: None,
            no_route_since: None,
            blocked_since: None,
        }
    }

    /// Called by a sink on every beacon it sends.
    pub fn bump_sink_seq(&mut self, id: NodeId) {
        self.sink = id;
        self.seq = self.seq.wrapping_add(1);
    }

    /// The freshness to put in the next heartbeat, recording it as the
    /// feasibility distance. That tracks what we have *advertised*, as in
    /// Babel: only what we told others can have reached our descendants
    /// (tracking what we received would put us a step ahead of every
    /// downstream neighbour). While `hold` is set we repeat the counter we
    /// last advertised instead of the newer one we know.
    pub fn advertise(&mut self, hold: bool) -> u16 {
        if self.hops == NO_ROUTE {
            return self.seq;
        }
        if self.fd_sink == self.sink && hold {
            self.fd_hops = self.fd_hops.min(self.hops);
            return self.fd_seq;
        }
        if self.fd_sink != self.sink || seq_newer(self.seq, self.fd_seq) {
            (self.fd_sink, self.fd_seq, self.fd_hops) = (self.sink, self.seq, self.hops);
        } else if self.fd_seq == self.seq {
            self.fd_hops = self.fd_hops.min(self.hops);
        }
        self.seq
    }
}

impl Brain {
    /// Route internals for tests and probes: `(seq, fd_seq, fd_hops)`.
    #[doc(hidden)]
    pub fn debug_route(&self) -> (u16, u16, u8) {
        (self.rt.seq, self.rt.fd_seq, self.rt.fd_hops)
    }

    /// Milliseconds without fresh sink news after which our route is
    /// suspect (another sink's gradient may replace it), a witness elects,
    /// and the route is dropped as stale.
    fn suspect_after_ms(&self) -> u32 {
        1000 * self.misses_tolerated() + 500
    }

    fn elect_after_ms(&self) -> u32 {
        1000 * self.misses_tolerated() + 1500
    }

    fn stale_after_ms(&self) -> u32 {
        1000 * self.misses_tolerated() + 4000
    }

    pub(crate) fn has_route(&self) -> bool {
        self.rt.hops != NO_ROUTE
    }

    fn is_dead(&self, sink: NodeId, seq: u16) -> bool {
        self.rt.dead.iter().any(|d| {
            d.valid
                && d.sink == sink
                && self.age(d.at_ms) < DEAD_MEMORY_MS
                && !seq_newer(seq, d.seq.wrapping_add(DEAD_SLACK))
        })
    }

    fn mark_dead(&mut self, sink: NodeId, seq: u16) {
        if sink == BROADCAST || sink == self.id {
            return;
        }
        let now = self.now;
        let slot = match self.rt.dead.iter().position(|d| d.valid && d.sink == sink) {
            Some(i) => i,
            None => (0..self.rt.dead.len())
                .min_by_key(|&i| (self.rt.dead[i].valid, self.rt.dead[i].at_ms))
                .unwrap_or(0),
        };
        let d = &mut self.rt.dead[slot];
        let seq = if d.valid && d.sink == sink && seq_newer(d.seq, seq) { d.seq } else { seq };
        *d = DeadSink { sink, seq, at_ms: now, valid: true };
    }

    /// Cost of routing through `n`, in quarter hops: hop count, plus a
    /// penalty for reserves (they may leave at any moment), weak links and
    /// drones still in transit.
    fn cost(&self, n: &Neighbor) -> u16 {
        let mut c = u16::from(n.hops) * 4;
        if n.role == Role::Reserve {
            c += 4;
        }
        if let Some(&(_, p)) = LINK_PENALTY.iter().find(|&&(dbm, _)| n.rssi_dbm <= dbm) {
            c += p;
        }
        if n.flags & crate::wire::FLAG_FILLING != 0 {
            // On its way somewhere else: it will leave us behind.
            c += 12;
        } else if n.role != Role::Sink && n.flags & crate::wire::FLAG_SETTLED == 0 {
            // In transit. A moving reserve is about the worst parent there
            // is; relays and scouts in transit are usually moving with us.
            c += if n.role == Role::Reserve { 8 } else { 2 };
        }
        c
    }

    /// Whether `n` may become our parent now.
    fn acceptable(&self, n: &Neighbor) -> bool {
        if n.hops >= MAX_HOPS
            || n.parent == self.id
            || n.sink == self.id
            || n.sink == BROADCAST
            || self.is_dead(n.sink, n.sink_seq)
            || self.suspect(n)
            || self.routes_via_suspect(n)
        {
            return false;
        }
        let mine_suspect = self.age(self.rt.seq_at_ms) > self.suspect_after_ms();
        if n.sink == self.rt.fd_sink {
            let newer = seq_newer(n.sink_seq, self.rt.fd_seq);
            // A suspect route only improves with genuinely fresh news.
            // Strictly fewer hops: with equal hops two siblings could pick
            // each other in the same instant and form a loop.
            return newer || (!mine_suspect && n.sink_seq == self.rt.fd_seq && n.hops < self.rt.fd_hops);
        }
        // Another sink: take it if we have nothing better, or it is
        // preferred (lower id), or ours has gone quiet.
        self.rt.parent.is_none() || mine_suspect || n.sink < self.rt.sink
    }

    fn best_candidate(&self) -> Option<Neighbor> {
        let parent = self.rt.parent;
        self.neighbors
            .iter()
            .filter(|n| Some(n.id) != parent && self.acceptable(n))
            .min_by_key(|n| (n.sink, self.cost(n), n.id))
            .copied()
    }

    /// Take `n`'s advertisement as our route (it is, or becomes, our parent).
    fn adopt(&mut self, n: &Neighbor) {
        let hops = n.hops + 1;
        if n.sink != self.rt.sink {
            // Our parent gave up on a sink we prefer: it must be dead.
            if self.rt.sink != BROADCAST && n.sink > self.rt.sink {
                self.mark_dead(self.rt.sink, self.rt.seq);
            }
            self.rt.sink = n.sink;
            self.rt.seq = n.sink_seq;
            self.rt.seq_at_ms = self.now;
        } else if seq_newer(n.sink_seq, self.rt.seq) || self.rt.parent.is_none() {
            if seq_newer(n.sink_seq, self.rt.seq) {
                self.rt.seq_at_ms = self.now;
            }
            self.rt.seq = n.sink_seq;
        }
        self.rt.hops = hops;
        self.rt.parent = Some(n.id);
        self.rt.parent_pos = Some(n.pos);
        self.rt.orphan = n.flags & FLAG_ORPHAN != 0;
        if n.id == n.sink {
            self.rt.witness_of = Some(n.sink);
            // Only on fresh news: we keep following a silent parent until
            // it expires, and that must not drag the mark along with us.
            if self.rt.seq_at_ms == self.now {
                self.rt.witness_at = self.pos;
                self.rt.witness_left = false;
            }
        } else if self.rt.seq_at_ms == self.now {
            // Fresh news through someone else: either the sink is alive, or
            // we are no longer next to it. Either way we are no witness.
            self.rt.witness_of = None;
        }
    }

    /// Note whether a clearly cheaper parent exists that feasibility alone
    /// rules out (see "Hold-down" above). Only sideways neighbours count
    /// (no more hops than us): one further down the gradient is usually our
    /// own descendant, which can never become feasible, and holding for it
    /// would only starve our whole subtree of fresh sink news.
    fn update_blocked(&mut self, parent_ok: bool) {
        let cur = self.rt.parent.and_then(|p| self.neighbor(p)).copied();
        let blocked = cur.filter(|_| parent_ok).is_some_and(|cur| {
            let limit = self.cost(&cur).saturating_sub(SWITCH_MARGIN);
            let id = self.id;
            self.neighbors.iter().any(|n| {
                n.id != cur.id
                    && n.sink == self.rt.sink
                    && n.parent != id
                    && n.hops <= self.rt.hops
                    && !self.suspect(n)
                    && self.cost(n) <= limit
                    && !self.acceptable(n)
            })
        });
        if !blocked {
            self.rt.blocked_since = None;
        } else if self.rt.blocked_since.is_none() {
            self.rt.blocked_since = Some(self.now);
        }
    }

    /// Hold the advertised counter back while blocked, but never for long:
    /// a sideways neighbour catches up within one of its beacons, while our
    /// children judge our route's health by that counter advancing and can't
    /// adopt us (nor can anyone else) as long as it stands still.
    pub(crate) fn hold_seq(&self) -> bool {
        let limit = self.cfg.beacon_interval_ms + 250;
        self.rt.blocked_since.is_some_and(|t| self.age(t) < limit)
    }

    fn drop_route(&mut self) {
        self.rt.parent = None;
        self.rt.hops = NO_ROUTE;
        self.rt.orphan = false;
    }

    pub(crate) fn update_route(&mut self) {
        if self.rt.witness_at.dist_xz_sq(self.pos) > WITNESS_DRIFT * WITNESS_DRIFT {
            self.rt.witness_left = true;
        }
        let before = (self.rt.sink, self.rt.hops, self.rt.parent);
        if self.acts_as_sink() {
            self.update_as_sink();
        } else {
            self.update_as_node();
        }
        let after = (self.rt.sink, self.rt.hops, self.rt.parent);
        if after != before {
            self.trigger_beacon();
        }
        if self.has_route() {
            self.rt.no_route_since = None;
        } else if self.rt.no_route_since.is_none() {
            self.rt.no_route_since = Some(self.now);
        }
    }

    fn update_as_sink(&mut self) {
        self.rt.hops = 0;
        self.rt.parent = None;
        self.rt.sink = self.id;
        if self.is_sink {
            return;
        }
        // An orphan sink that got back to where the base should be and
        // still hears nobody better concludes the base is gone, and stays.
        if (self.rt.orphan || self.rt.verify) && !self.away_from_origin() {
            self.rt.orphan = false;
            self.rt.verify = false;
        }
        // An acting sink steps down for a live gradient to a lower sink id:
        // the base reappeared, or two partitions met.
        let id = self.id;
        let better = self
            .neighbors
            .iter()
            .filter(|n| n.sink < id && n.hops < MAX_HOPS && n.parent != id)
            .filter(|n| !self.is_dead(n.sink, n.sink_seq) && !self.suspect(n))
            .min_by_key(|n| (n.sink, self.cost(n), n.id))
            .copied();
        if let Some(n) = better {
            self.rt.acting = false;
            self.rt.orphan = false;
            self.rt.verify = false;
            self.rt.fd_sink = BROADCAST;
            self.rt.sink = BROADCAST;
            self.adopt(&n);
        }
    }

    fn update_as_node(&mut self) {
        // Follow the current parent, or notice it is gone or poisoned.
        let mut parent_ok = false;
        if let Some(p) = self.rt.parent {
            match self.neighbor(p).copied() {
                Some(n)
                    if n.hops < MAX_HOPS && n.parent != self.id && !self.is_dead(n.sink, n.sink_seq) =>
                {
                    self.adopt(&n);
                    parent_ok = !self.suspect(&n);
                }
                _ => self.drop_route(),
            }
        }

        self.update_blocked(parent_ok);
        if let Some(best) = self.best_candidate() {
            let switch = match self.rt.parent.and_then(|p| self.neighbor(p)).copied() {
                None => true,
                Some(_) if !parent_ok => true,
                Some(cur) => {
                    let mine_suspect = self.age(self.rt.seq_at_ms) > self.suspect_after_ms();
                    best.sink < self.rt.sink
                        || (best.sink != self.rt.sink && mine_suspect)
                        || (best.sink == self.rt.sink
                            && self.cost(&best) + SWITCH_MARGIN <= self.cost(&cur))
                }
            };
            if switch {
                if best.sink != self.rt.sink && self.rt.sink != BROADCAST && self.has_route() {
                    self.mark_dead(self.rt.sink, self.rt.seq);
                }
                self.adopt(&best);
            }
        }

        // A route whose sink has gone quiet leads nowhere: poison it.
        if self.has_route() && self.age(self.rt.seq_at_ms) > self.stale_after_ms() {
            self.mark_dead(self.rt.sink, self.rt.seq);
            self.drop_route();
        }
        if self.rt.hops >= MAX_HOPS {
            self.drop_route();
        }
        if !self.has_route() {
            self.maybe_elect();
        }
    }

    /// Become the acting sink if the sink we heard directly has been silent
    /// long enough, we have stayed about where we heard it, and no lower-id
    /// witness is alive to take over; or, as a last resort, if we have been
    /// cut off for a long time and are the lowest id around.
    fn maybe_elect(&mut self) {
        let quiet = self.age(self.rt.seq_at_ms);
        if let Some(old) = self.rt.witness_of
            && quiet >= self.elect_after_ms()
            && !self.rt.witness_left
        {
            // Defer to a live lower-id witness of the same sink: it reaches
            // this point in the same tick or so and its Sink beacon wins
            // us over. Not for ever, in case it never declares.
            let id = self.id;
            let lower_witness = self.neighbors.iter().any(|m| m.id < id && !self.suspect(m) && m.witness_of == old);
            if !lower_witness || quiet >= self.elect_after_ms() + ELECT_GRACE_MS {
                self.become_sink(old, false);
                // Out here a dead base and a hill between us look the same,
                // and a second sink that stays put would hold the swarm off
                // a live base for good. Its gradient carries on as normal
                // (no FLAG_ORPHAN: the group keeps its slots) while it flies
                // back to look; it steps down on hearing the base, or stays
                // at the base's spot if the base really is gone.
                self.rt.verify = self.away_from_origin();
            }
            return;
        }
        let Some(since) = self.rt.no_route_since else { return };
        if self.age(since) < ORPHAN_ELECT_MS {
            return;
        }
        let id = self.id;
        let routed_or_lower = self
            .neighbors
            .iter()
            .any(|m| !self.suspect(m) && (m.hops < MAX_HOPS || (m.id < id && m.role != Role::Sink)));
        if !routed_or_lower {
            self.become_sink(self.rt.sink, true);
        }
    }

    /// Further than [`ORIGIN_REACHED`] from the mission origin (where the
    /// base is).
    fn away_from_origin(&self) -> bool {
        self.plan.is_some_and(|p| p.origin.dist_xz_sq(self.pos) >= ORIGIN_REACHED * ORIGIN_REACHED)
    }

    fn become_sink(&mut self, old: NodeId, orphan: bool) {
        let last = if seq_newer(self.rt.fd_seq, self.rt.seq) { self.rt.fd_seq } else { self.rt.seq };
        self.mark_dead(old, last);
        self.rt.acting = true;
        self.rt.orphan = orphan;
        self.rt.verify = false;
        self.rt.witness_of = None;
        self.rt.sink = self.id;
        self.rt.hops = 0;
        self.rt.parent = None;
        self.rt.seq = 0;
        self.trigger_beacon();
    }
}
