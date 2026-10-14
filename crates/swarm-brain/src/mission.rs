//! The drone's part in the formation: which slot it takes, how vacancies
//! are noticed and filled, and where to fly.
//!
//! * **Assignment without a coordinator.** When a mission starts, every
//!   drone ranks the unassigned drones it can hear by id and takes the slot
//!   at its rank in the plan's fill order (`plan.rs`). Drones that hear the
//!   same set compute the same assignment, so this costs no messages; the
//!   chosen slot is then claimed in every heartbeat.
//! * **Claims and tie-breaks.** If two drones that hear each other claim the
//!   same slot, a drone already in position beats one still on its way;
//!   between two on their way the clearly closer one wins; otherwise the
//!   lower id wins. The loser yields once and does not come back, so claims
//!   never oscillate.
//! * **Vacancies.** A drone in position knows which slots should be within
//!   earshot (`Plan::neighbours`). If none of its neighbours has claimed
//!   one of them for a few beacon intervals, and no neighbour vouches for
//!   it either (heartbeats carry a bitmap of slots whose holders the sender
//!   heard in the last few seconds), it advertises that slot as needed.
//!   The vouching makes a lost heartbeat or a blocked line of sight look
//!   different from a death without any extra messages. Needs travel a few
//!   hops by gossip (each relay re-advertises with a hop count, so echoes
//!   can't keep a need alive) and die out as soon as anyone hears a claim
//!   for the slot or a neighbour vouches for it.
//! * **Filling.** A parked reserve that knows of needs matches them, in
//!   priority order, to the closest available reserves it can hear; if that
//!   is itself, it claims the slot and flies there. It stays a reserve until
//!   it arrives, so a false alarm costs a short flight but no role change.
//!   Reserves that other drones route through don't volunteer.
//! * **Promotion.** A need nobody fills for a while is matched the same way
//!   against holders of less important slots (extra scouts, then the
//!   redundant rail), least important first, so small swarms still close
//!   gaps in their only chain. A holder waits longer to volunteer the more
//!   hops the need travelled and the further it would fly, so the side of
//!   a gap nearest the hole fills it before the far side, which can't hear
//!   its claim, sets off too.
//! * **Regrouping.** A drone with no route for a few seconds climbs (height
//!   clears the ridge) and flies toward where its parent was last heard,
//!   then toward the sink, until it hears the swarm again. A group that
//!   elected its own sink after a long cut-off (`FLAG_ORPHAN`) keeps
//!   flying toward the sink's position, its sink too, until it meets the
//!   base's gradient; finding nobody there, it carries on without it. A
//!   witness that took over from a silent base away from the base's spot
//!   flies there alone to check, while its group keeps its slots.

use crate::brain::{Brain, LANDED_AGL};
use crate::plan::{NO_SLOT, Plan, SLOT_COUNT, Slot, slot_bit};
use crate::routing::WEAK_RSSI_DBM;
use crate::wire::{FLAG_FILLING, FLAG_SETTLED, Intent, Role};
use crate::{FixedVec, Neighbor, Sensors, Vec3};

/// Uptime before ranking: long enough to have heard every drone parked
/// nearby at least twice.
const ASSEMBLY_MS: u32 = 2_600;
/// After a mission starts, unassigned drones rank themselves into any slot;
/// later they only take parking spots and wait to be needed.
const RANK_WINDOW_MS: u32 = 15_000;
/// Settled means really there: others pick settled drones as parents
/// because they won't move, so the radius is tight.
const SETTLE_RADIUS: f32 = 10.0;
const SCOUT_SETTLE_RADIUS: f32 = 50.0;
const MAX_NEEDS: usize = 4;
/// A relayed need is forgotten this long after it was last refreshed.
const NEED_TTL_MS: u32 = 5_000;
const MAX_NEED_HOPS: u8 = 4;
/// A filler that found its slot already held ignores needs for it this long.
const REFUSE_MS: u32 = 30_000;
/// A need nobody has filled for this long may be taken by the holder of a
/// less important slot (reserves get the first chance).
const PROMOTE_AFTER_MS: u32 = 8_000;
/// A holder volunteers for promotion later the further the need came from
/// and the further it would fly: drones on opposite sides of a gap can't
/// hear each other's claims, so the one nearest the gap must get there
/// first (it flies 12 units/s; this is a little under half that pace, so a
/// rival more than about three times as far never takes off in time).
const PROMOTE_PER_HOP_MS: u32 = 4_000;
const PROMOTE_MS_PER_UNIT: f32 = 40.0;
/// No route for this long and the drone starts to regroup.
const REGROUP_AFTER_MS: u32 = 3_000;

const RELAY_AGL: f32 = 50.0;
const SCOUT_AGL: f32 = 40.0;
const RESERVE_AGL: f32 = 45.0;
const TRANSIT_AGL: f32 = 50.0;
/// Highest a drone climbs above the ground while searching for a link.
const CEILING_AGL: f32 = 230.0;
const CLIMB_RATE: f32 = 3.0;
/// A relay climbs for its parent only while the parent is at least this
/// much higher, so the climb stops before it passes the parent's height.
const CLIMB_MARGIN: f32 = 5.0;
/// How fast a scout adjusts its distance from the fire centre.
const SCOUT_RATE: f32 = 4.0;
/// A parent link this weak (about 155 units) is long: a drone on it holds
/// its height rather than sink and lose the line of sight.
const MARGINAL_RSSI_DBM: i8 = -90;
/// A relay keeps the climb angle up to its parent below this slope (rise
/// over horizontal distance). A link that climbs steeply skims whatever
/// slope it rises over, and a few units of sideways motion put that slope
/// in the way; a drone has no map to tell, but the angle alone says the
/// link is at risk. Flying up toward the parent's height also shortens the
/// link, so it never costs signal.
const MAX_LINK_SLOPE: f32 = 0.25;

#[derive(Clone, Copy, Debug, Default)]
struct Need {
    slot: u8,
    hops: u8,
    /// Last refreshed, and first heard of.
    at_ms: u32,
    since_ms: u32,
}

pub(crate) struct Mission {
    pub slot: u8,
    /// Claimed through a vacancy notice and not there yet.
    pub filling: bool,
    /// The current slot was claimed through a vacancy notice. Unlike
    /// `filling` this survives arrival: a filler that then meets the
    /// slot's original holder (it had only dropped out of earshot) must
    /// still back off for [`REFUSE_MS`], or the next false alarm for the
    /// same slot sends it straight back.
    pub came_as_filler: bool,
    pub settled: bool,
    settled_since: u32,
    /// When the current formation geometry was adopted.
    plan_ms: u32,
    /// When each slot was last claimed by a neighbour.
    seen_at: [u32; SLOT_COUNT],
    seen: u128,
    needs: FixedVec<Need, MAX_NEEDS>,
    need_rr: u8,
    /// Extra height gained while hunting for links. Never given back during
    /// a mission: dropping again would just lose the link again. A settled
    /// relay only adds to it while that brings it closer to its parent
    /// (`climb_shortens_link`), so that climb never stretches the link it
    /// was meant to save.
    pub climb: f32,
    pub scout_r: f32,
    fire_seen_ms: Option<u32>,
    hold_pos: Option<Vec3>,
    pub refused: Option<(u8, u32)>,
}

impl Mission {
    pub fn new() -> Self {
        Self {
            slot: NO_SLOT,
            filling: false,
            came_as_filler: false,
            settled: false,
            settled_since: 0,
            plan_ms: 0,
            seen_at: [0; SLOT_COUNT],
            seen: 0,
            needs: FixedVec::new(),
            need_rr: 0,
            climb: 0.0,
            scout_r: 40.0,
            fire_seen_ms: None,
            hold_pos: None,
            refused: None,
        }
    }

    pub fn flags(&self) -> u8 {
        let mut f = 0;
        if self.settled {
            f |= FLAG_SETTLED;
        }
        if self.filling {
            f |= FLAG_FILLING;
        }
        f
    }

    fn clear_slot(&mut self) {
        self.slot = NO_SLOT;
        self.filling = false;
        self.came_as_filler = false;
        self.settled = false;
    }
}

fn is_park(slot: u8) -> bool {
    matches!(Slot::decode(slot), Some(Slot::Park(_)))
}

/// Where to aim to reach `goal` (ground plane) at `agl` above the ground.
/// When well below the wanted height, climb first: aiming straight at a
/// far goal would spend the whole speed budget horizontally and leave the
/// drone skimming up a slope at minimum clearance, radio-shadowed.
fn fly_to(pos: Vec3, goal: Vec3, ground: f32, agl: f32) -> Vec3 {
    let y = ground + agl;
    let deficit = y - pos.y;
    let (dx, dz) = (goal.x - pos.x, goal.z - pos.z);
    let d2 = dx * dx + dz * dz;
    let max = 0.5 * deficit;
    if deficit > 10.0 && d2 > max * max {
        let k = max / crate::math::sqrt(d2);
        return Vec3::new(pos.x + dx * k, y, pos.z + dz * k);
    }
    Vec3::new(goal.x, y, goal.z)
}

impl Brain {
    pub(crate) fn mission_active(&self) -> bool {
        self.plan.is_some() && matches!(self.intent, Intent::MonitorFire | Intent::MapPerimeter)
    }

    /// A reserve waiting at its parking spot: nothing urgent to say.
    pub(crate) fn parked(&self) -> bool {
        self.mission_active() && is_park(self.m.slot) && self.m.settled && self.has_route()
    }

    pub(crate) fn mission_role(&self) -> Role {
        match Slot::decode(self.m.slot) {
            Some(Slot::Relay { .. }) if !self.m.filling => Role::Relay,
            Some(Slot::Scout(_)) if !self.m.filling => Role::Scout,
            _ => Role::Reserve,
        }
    }

    fn vacancy_ms(&self) -> u32 {
        1000 * (self.misses_tolerated() + 2)
    }

    /// How recently we must have heard a slot's holder to vouch for it.
    fn vouch_ms(&self) -> u32 {
        500 * (self.misses_tolerated() + 2)
    }

    /// Slots whose holders we heard directly just now, for our heartbeat.
    pub(crate) fn heard_slots(&self) -> u32 {
        if !self.mission_active() {
            return 0;
        }
        let window = self.vouch_ms();
        let mut bits = slot_bit(self.m.slot);
        for slot in 1..SLOT_COUNT as u8 {
            let bit = slot_bit(slot);
            if bit != 0 && self.seen_within(slot, window) {
                bits |= bit;
            }
        }
        bits
    }

    /// True if a live neighbour has recently heard the holder of `slot`.
    fn vouched(&self, slot: u8) -> bool {
        let bit = slot_bit(slot);
        bit != 0 && self.neighbors.iter().any(|n| n.heard_slots & bit != 0 && !self.suspect(n))
    }

    fn seen_within(&self, slot: u8, ms: u32) -> bool {
        let s = usize::from(slot);
        s < SLOT_COUNT && (self.m.seen >> s) & 1 == 1 && self.age(self.m.seen_at[s]) <= ms
    }

    pub(crate) fn adopt_mission(&mut self, intent: Intent, target: Vec3, origin: Vec3) {
        let was_active = self.mission_active();
        match intent {
            Intent::MonitorFire | Intent::MapPerimeter => {
                let plan = Plan::new(origin, target);
                let same = self.plan.is_some_and(|p| {
                    p.rows == plan.rows
                        && p.origin.dist_xz_sq(origin) < 60.0 * 60.0
                        && p.target.dist_xz_sq(target) < 60.0 * 60.0
                });
                if !same {
                    self.m.climb = 0.0;
                    self.m.needs.clear();
                    self.m.seen = 0;
                    self.m.refused = None;
                    self.m.scout_r = 40.0;
                    if !plan.is_valid(self.m.slot) {
                        self.m.clear_slot();
                    }
                }
                if !same || !was_active {
                    self.m.plan_ms = self.now;
                }
                self.plan = Some(plan);
            }
            Intent::Hold => self.m.hold_pos = None,
            Intent::ReturnToBase => {
                self.m.clear_slot();
                self.m.needs.clear();
                self.m.climb = 0.0;
            }
        }
    }

    pub(crate) fn mission_on_beacon(&mut self, n: &Neighbor, need: u8, need_hops: u8) {
        let s = usize::from(n.slot);
        if n.slot != NO_SLOT && s < SLOT_COUNT {
            self.m.seen_at[s] = self.now;
            self.m.seen |= 1 << s;
        }
        let Some(plan) = self.plan else { return };
        if self.mission_active()
            && need != NO_SLOT
            && need_hops < MAX_NEED_HOPS
            && need != self.m.slot
            && plan.is_valid(need)
            && !self.seen_within(need, self.vacancy_ms())
            && !self.vouched(need)
        {
            self.add_need(&plan, need, need_hops + 1);
        }
    }

    pub(crate) fn update_mission(&mut self, s: &Sensors) {
        if !self.mission_active() {
            if self.intent == Intent::Hold && self.m.hold_pos.is_none() {
                self.m.hold_pos = Some(self.pos);
            }
            return;
        }
        let Some(plan) = self.plan else { return };
        let uptime = self.boot_ms.map_or(0, |b| self.age(b));
        if !self.is_sink {
            if self.m.slot == NO_SLOT && uptime >= ASSEMBLY_MS && !self.acts_as_sink() {
                self.choose_slot(&plan);
            }
            if !self.acts_as_sink() {
                self.resolve_conflict(&plan);
            }
            self.update_settled(&plan, s);
        }
        self.detect_vacancies(&plan);
        self.prune_needs();
        self.respond_to_needs(&plan);
    }

    fn slot_taken(&self, slot: u8) -> bool {
        self.neighbors.iter().any(|n| n.slot == slot && !self.suspect(n))
    }

    /// Pick a slot: by rank while the mission is starting, otherwise the
    /// first free parking spot.
    fn choose_slot(&mut self, plan: &Plan) {
        let parks = usize::from(plan.parks());
        if self.age(self.m.plan_ms) < RANK_WINDOW_MS {
            let id = self.id;
            let mut rank = self
                .neighbors
                .iter()
                .filter(|n| !self.suspect(n) && n.role != Role::Sink && n.slot == NO_SLOT && n.id < id)
                .count();
            let total = 2 * usize::from(plan.rows) + usize::from(crate::plan::SCOUTS) + parks;
            for i in 0..total {
                let slot = plan.slot_at(i);
                if self.slot_taken(slot) {
                    continue;
                }
                if rank == 0 {
                    self.m.slot = slot;
                    self.m.filling = false;
                    self.m.came_as_filler = false;
                    return;
                }
                rank -= 1;
            }
        }
        let free = (0..plan.parks()).map(|j| Slot::Park(j).id()).find(|&p| !self.slot_taken(p));
        let fallback = Slot::Park((self.id % u16::from(plan.parks().max(1))) as u8).id();
        self.m.slot = free.unwrap_or(fallback);
        self.m.filling = false;
        self.m.came_as_filler = false;
    }

    fn resolve_conflict(&mut self, plan: &Plan) {
        let slot = self.m.slot;
        if slot == NO_SLOT || is_park(slot) {
            return;
        }
        let Some(spot) = plan.slot_xz(slot) else {
            self.m.clear_slot();
            return;
        };
        let rival = self
            .neighbors
            .iter()
            .filter(|n| n.slot == slot && !self.suspect(n))
            .find(|n| self.loses_to(n, spot))
            .copied();
        if let Some(n) = rival {
            // A filler that lost keeps away for a while, whether the rival
            // is already there or merely closer, and whether or not we had
            // arrived yet: it must not come back.
            if self.m.filling || self.m.came_as_filler || n.flags & FLAG_FILLING != 0 {
                self.m.refused = Some((slot, self.now));
            }
            self.m.clear_slot();
        }
    }

    fn loses_to(&self, n: &Neighbor, spot: Vec3) -> bool {
        if n.role == Role::Sink {
            return true;
        }
        let theirs = n.flags & FLAG_SETTLED != 0;
        if theirs != self.m.settled {
            return theirs;
        }
        if !self.m.settled {
            let (dn, dm) = (n.pos.dist_xz(spot), self.pos.dist_xz(spot));
            if dm > dn + 20.0 {
                return true;
            }
            if dn > dm + 20.0 {
                return false;
            }
        }
        n.id < self.id
    }

    /// Where this drone's slot wants it, in the ground plane.
    fn slot_spot(&self, plan: &Plan) -> Option<Vec3> {
        match Slot::decode(self.m.slot)? {
            Slot::Scout(k) => {
                let (dx, dz) = plan.scout_dir(k);
                let r = self.m.scout_r;
                Some(Vec3::new(plan.target.x + dx * r, 0.0, plan.target.z + dz * r))
            }
            _ => plan.slot_xz(self.m.slot),
        }
    }

    #[cfg(test)]
    pub(crate) fn slot_spot_for_test(&self) -> Vec3 {
        self.plan.and_then(|p| self.slot_spot(&p)).unwrap_or(self.pos)
    }

    fn update_settled(&mut self, plan: &Plan, _s: &Sensors) {
        let radius = match Slot::decode(self.m.slot) {
            Some(Slot::Scout(_)) => SCOUT_SETTLE_RADIUS,
            _ => SETTLE_RADIUS,
        };
        let d2 = self.slot_spot(plan).map_or(f32::INFINITY, |p| p.dist_xz_sq(self.pos));
        // Hysteresis: settling is quick, unsettling needs a real departure.
        let settled = d2 < radius * radius || (self.m.settled && d2 < 4.0 * radius * radius);
        if settled && !self.m.settled {
            self.m.settled_since = self.now;
            // Arrived at a slot we came to fill: it is ours now.
            self.m.filling = false;
        }
        self.m.settled = settled;
    }

    fn detect_vacancies(&mut self, plan: &Plan) {
        let watched = if self.is_sink {
            NO_SLOT
        } else {
            match Slot::decode(self.m.slot) {
                Some(Slot::Relay { .. } | Slot::Scout(_)) if self.m.settled && !self.m.filling => {
                    self.m.slot
                }
                _ => return,
            }
        };
        let since = if self.is_sink { self.m.plan_ms } else { self.m.settled_since };
        let wait = self.vacancy_ms();
        if self.age(since) < wait {
            return;
        }
        let mut adj = [NO_SLOT; 8];
        let n = plan.neighbours(watched, &mut adj);
        for &slot in &adj[..n] {
            if !self.seen_within(slot, wait) && !self.vouched(slot) {
                self.add_need(plan, slot, 0);
            }
        }
    }

    fn add_need(&mut self, plan: &Plan, slot: u8, hops: u8) {
        let now = self.now;
        if let Some(e) = self.m.needs.iter_mut().find(|e| e.slot == slot) {
            if hops <= e.hops {
                e.hops = hops;
                e.at_ms = now;
            }
            return;
        }
        let need = Need { slot, hops, at_ms: now, since_ms: now };
        if self.m.needs.push(need).is_err()
            && let Some(worst) = self.m.needs.iter_mut().max_by_key(|e| plan.priority(e.slot))
            && plan.priority(slot) < plan.priority(worst.slot)
        {
            *worst = need;
        }
    }

    fn prune_needs(&mut self) {
        let (now, mine) = (self.now, self.m.slot);
        let refused = self.m.refused.filter(|&(_, t)| now.wrapping_sub(t) < REFUSE_MS).map(|r| r.0);
        let mut i = 0;
        while i < self.m.needs.len() {
            let e = self.m.needs[i];
            let keep = now.wrapping_sub(e.at_ms) <= NEED_TTL_MS
                && e.slot != mine
                && Some(e.slot) != refused
                && !self.seen_within(e.slot, 2_000)
                && !self.vouched(e.slot);
            if keep {
                i += 1;
            } else {
                self.m.needs.swap_remove(i);
            }
        }
    }

    /// The need to advertise in the next heartbeat, rotating through all we
    /// know so several vacancies spread in parallel.
    pub(crate) fn next_need(&mut self) -> (u8, u8) {
        if self.m.needs.is_empty() || !self.mission_active() {
            return (NO_SLOT, 0);
        }
        self.m.need_rr = self.m.need_rr.wrapping_add(1);
        let e = self.m.needs[usize::from(self.m.need_rr) % self.m.needs.len()];
        (e.slot, e.hops)
    }

    fn available_reserve(n: &Neighbor) -> bool {
        n.role == Role::Reserve
            && (n.slot == NO_SLOT || is_park(n.slot))
            && n.flags & FILLING_OR_LANDED == 0
            && n.hops < crate::routing::MAX_HOPS
    }

    /// Could `n` be moved up into `need`: settled in a less important slot,
    /// carrying nobody, and not just the other rail of the same row (that
    /// would only move the hole)?
    fn promotable(plan: &Plan, n_slot: u8, n_flags: u8, n_role: Role, need: u8) -> bool {
        let same_row = match (Slot::decode(n_slot), Slot::decode(need)) {
            (Some(Slot::Relay { row: a, .. }), Some(Slot::Relay { row: b, .. })) => a == b,
            _ => false,
        };
        matches!(n_role, Role::Relay | Role::Scout)
            && n_flags & FLAG_SETTLED != 0
            && n_flags & FILLING_OR_LANDED == 0
            && plan.priority(n_slot) > plan.priority(need)
            && !same_row
    }

    /// Match known needs, most important first, to the drones best placed
    /// to fill them: the closest available reserve, or once a need has gone
    /// unfilled for a while, the holder of the least important slot (closest
    /// first among equals). Claim a slot if it falls to us.
    fn respond_to_needs(&mut self, plan: &Plan) {
        if self.acts_as_sink()
            || self.m.needs.is_empty()
            || !self.has_route()
            || self.carrying()
            || self.m.filling
            || self.age(self.m.plan_ms) < ASSEMBLY_MS + 2_000
        {
            return;
        }
        let i_reserve = self.m.slot == NO_SLOT || is_park(self.m.slot);
        let my_flags = self.m.flags();
        if !i_reserve && !self.m.settled {
            return;
        }
        let mut order = [0usize; MAX_NEEDS];
        let count = self.m.needs.len();
        for (i, o) in order.iter_mut().enumerate().take(count) {
            *o = i;
        }
        order[..count].sort_unstable_by_key(|&i| plan.priority(self.m.needs[i].slot));

        // Bit per neighbour-table index (see the assertion below).
        let mut taken: u64 = 0;
        for &i in &order[..count] {
            let need = self.m.needs[i];
            let Some(spot) = plan.slot_xz(need.slot) else { continue };
            let waited = self.age(need.since_ms);
            let promote = waited >= PROMOTE_AFTER_MS;
            // Lower key wins: reserves first, then the least important slot,
            // then distance, then id.
            let key = |reserve: bool, slot: u8, pos: Vec3, id: u16| {
                (u8::from(!reserve), u8::MAX - plan.priority(slot), pos.dist_xz_sq(spot) as u32, id)
            };
            let me_candidate = promote && Self::promotable(plan, self.m.slot, my_flags, self.role, need.slot);
            let mine = (i_reserve || me_candidate).then(|| key(i_reserve, self.m.slot, self.pos, self.id));
            let mut best = mine.map(|k| (k, None::<usize>));
            for (j, n) in self.neighbors.iter().enumerate() {
                if taken >> j & 1 == 1 || self.suspect(n) || n.hops >= crate::routing::MAX_HOPS {
                    continue;
                }
                let reserve = Self::available_reserve(n);
                if !reserve && !(promote && Self::promotable(plan, n.slot, n.flags, n.role, need.slot)) {
                    continue;
                }
                let k = key(reserve, n.slot, n.pos, n.id);
                if best.is_none_or(|(b, _)| k < b) {
                    best = Some((k, Some(j)));
                }
            }
            match best {
                // Others are judged on the base wait (we defer to a better
                // one still waiting); our own promotion waits for the
                // backoff, and we keep ourselves for this need meanwhile.
                Some((_, None))
                    if !i_reserve
                        && waited
                            < PROMOTE_AFTER_MS
                                + u32::from(need.hops) * PROMOTE_PER_HOP_MS
                                + (self.pos.dist_xz(spot) * PROMOTE_MS_PER_UNIT) as u32 =>
                {
                    return;
                }
                Some((_, None)) => {
                    self.m.slot = need.slot;
                    self.m.filling = true;
                    self.m.came_as_filler = true;
                    self.m.settled = false;
                    return;
                }
                Some((_, Some(j))) => taken |= 1 << j,
                None => {}
            }
        }
    }

    pub(crate) fn flight_target(&mut self, s: &Sensors, dt: f32) -> Vec3 {
        let ground = self.pos.y - self.agl;
        if self.is_sink {
            return self.pos;
        }
        match self.intent {
            Intent::Hold => self.m.hold_pos.unwrap_or(self.pos),
            Intent::ReturnToBase => {
                let home = self.home;
                if self.pos.dist_xz_sq(home) > 10.0 * 10.0 {
                    fly_to(self.pos, home, ground, TRANSIT_AGL.max(self.agl.min(CEILING_AGL)))
                } else {
                    // Over the spot: descend; the airframe stops at the ground.
                    Vec3::new(home.x, ground - 10.0, home.z)
                }
            }
            Intent::MonitorFire | Intent::MapPerimeter => match self.plan {
                Some(plan) => self.mission_target(&plan, s, dt, ground),
                None => self.pos,
            },
        }
    }

    fn mission_target(&mut self, plan: &Plan, s: &Sensors, dt: f32, ground: f32) -> Vec3 {
        if self.acts_as_sink() && !self.rt.orphan && !self.rt.verify {
            return self.pos;
        }
        let base_agl = match Slot::decode(self.m.slot) {
            Some(Slot::Relay { .. }) => RELAY_AGL,
            Some(Slot::Scout(_)) => SCOUT_AGL,
            _ => RESERVE_AGL,
        };
        let max_climb = CEILING_AGL - base_agl;

        // Cut off: climb for line of sight and head back toward the swarm.
        // A group that elected its own sink while cut off is still cut off
        // from the base, which is most likely alive: the whole group heads
        // for it, sink included.
        let lost = self.rt.no_route_since.is_some_and(|since| self.age(since) >= REGROUP_AFTER_MS);
        if lost || self.rt.orphan || self.rt.verify {
            self.m.climb = (self.m.climb + CLIMB_RATE * dt).min(max_climb);
            let goal = match self.rt.parent_pos {
                Some(p) if lost && p.dist_xz_sq(self.pos) > 20.0 * 20.0 => p,
                Some(_) => {
                    // Reached where the parent was and still alone.
                    self.rt.parent_pos = None;
                    plan.origin
                }
                None => plan.origin,
            };
            return fly_to(self.pos, goal, ground, base_agl + self.m.climb);
        }

        let goal = match Slot::decode(self.m.slot) {
            Some(Slot::Scout(k)) => {
                self.track_fire(plan, k, s, dt);
                self.slot_spot(plan).unwrap_or(plan.target)
            }
            Some(slot) => {
                if matches!(slot, Slot::Relay { .. }) && self.m.settled && self.climb_shortens_link() {
                    self.m.climb = (self.m.climb + CLIMB_RATE * dt).min(max_climb);
                }
                plan.slot_xz(self.m.slot).unwrap_or(self.pos)
            }
            None if self.agl < LANDED_AGL && self.boot_ms.is_some_and(|b| self.age(b) < ASSEMBLY_MS) => {
                return self.pos;
            }
            None => return fly_to(self.pos, self.pos, ground, self.agl.max(20.0)),
        };
        let goal = if self.leashed(goal) { self.pos } else { goal };
        let mut agl = base_agl + self.m.climb;
        if let Some(floor) = self.link_floor() {
            agl = agl.max((floor - ground).min(CEILING_AGL));
        }
        let mut target = fly_to(self.pos, goal, ground, agl);
        if self.carrying() || self.parent_link_marginal() {
            // Sinking could put a ridge between us and those who rely on us,
            // or between us and a parent we can barely reach as it is.
            target.y = target.y.max(self.pos.y);
        }
        target
    }

    /// Connectivity-preserving motion: don't fly further from our parent,
    /// or from a drone that routes through us, when that link is already
    /// marginal. We wait (climbing is still allowed) until routing finds a
    /// better link.
    fn leashed(&self, goal: Vec3) -> bool {
        let id = self.id;
        let parent = self.rt.parent;
        self.neighbors
            .iter()
            .filter(|n| Some(n.id) == parent || (n.parent == id && !self.suspect(n)))
            .any(|n| n.rssi_dbm <= WEAK_RSSI_DBM && goal.dist_xz_sq(n.pos) > self.pos.dist_xz_sq(n.pos))
    }

    /// Whether a settled relay should gain height for its parent link: the
    /// link is weak, the parent is above us (signal follows 3D distance, so
    /// climbing toward a lower parent only stretches the link further, which
    /// is weaker still, a climb that feeds itself until the link snaps), and
    /// no drone below us that routes through us is itself on a weak link
    /// (climbing would stretch that one instead).
    fn climb_shortens_link(&self) -> bool {
        let Some(p) = self.rt.parent.and_then(|p| self.neighbor(p)) else { return false };
        if p.rssi_dbm > WEAK_RSSI_DBM || p.pos.y <= self.pos.y + CLIMB_MARGIN {
            return false;
        }
        let (id, y) = (self.id, self.pos.y);
        !self
            .neighbors
            .iter()
            .any(|c| c.parent == id && !self.suspect(c) && c.rssi_dbm <= WEAK_RSSI_DBM && c.pos.y < y)
    }

    /// The link to our parent is long enough that line of sight along it
    /// is easily lost: it skims whatever lies between.
    fn parent_link_marginal(&self) -> bool {
        self.rt.parent.and_then(|p| self.neighbor(p)).is_some_and(|p| p.rssi_dbm <= MARGINAL_RSSI_DBM)
    }

    /// Lowest height (absolute) a relay or scout should fly at to keep the
    /// links to its parent and to the drones routing through it shallower
    /// than [`MAX_LINK_SLOPE`], or `None` if rising would stretch a weak link
    /// to one of them below us (signal follows 3D distance). Scouts need it
    /// as much as relays: following the ground out over a cliff edge (a
    /// gorge rim) drops a scout below its parent's horizon.
    fn link_floor(&self) -> Option<f32> {
        if !matches!(Slot::decode(self.m.slot), Some(Slot::Relay { .. } | Slot::Scout(_))) {
            return None;
        }
        let (id, parent, y) = (self.id, self.rt.parent, self.pos.y);
        let linked = |n: &&Neighbor| Some(n.id) == parent || (n.parent == id && !self.suspect(n));
        let floor = self
            .neighbors
            .iter()
            .filter(linked)
            .map(|n| n.pos.y - MAX_LINK_SLOPE * n.pos.dist_xz(self.pos))
            .fold(f32::NEG_INFINITY, f32::max);
        if floor <= y {
            return None;
        }
        let stretches = self.neighbors.iter().filter(linked).any(|n| n.rssi_dbm <= WEAK_RSSI_DBM && n.pos.y < y);
        (!stretches).then_some(floor)
    }

    /// Keep the thermal sensor on the fire: move along this scout's sector
    /// ray so the burning cells in view sit under it (monitoring) or just
    /// inside it (perimeter), and search outward when the fire is lost.
    fn track_fire(&mut self, plan: &Plan, k: u8, s: &Sensors, dt: f32) {
        let (r_min, r_max, bias) = match self.intent {
            Intent::MapPerimeter => (30.0, 150.0, -15.0),
            _ => (25.0, 100.0, 0.0),
        };
        let (dx, dz) = plan.scout_dir(k);
        let near = self.slot_spot(plan).is_some_and(|p| p.dist_xz_sq(self.pos) < 30.0 * 30.0);
        let mut r = self.m.scout_r;
        if s.fire.burning > 0 && s.fire.centroid.is_finite() {
            self.m.fire_seen_ms = Some(self.now);
            if near {
                let c = s.fire.centroid;
                let outward = (c.x - self.pos.x) * dx + (c.z - self.pos.z) * dz;
                r += ((outward - bias) / 10.0).clamp(-1.0, 1.0) * SCOUT_RATE * dt;
            }
        } else if near && self.m.fire_seen_ms.is_none_or(|t| self.age(t) > 3_000) {
            r += 0.5 * SCOUT_RATE * dt;
            if r > r_max {
                r = r_min;
            }
        }
        self.m.scout_r = r.clamp(r_min, r_max);
    }
}

const _: () = assert!(crate::MAX_NEIGHBORS <= 64, "filler matching tracks neighbours in a u64");

const FILLING_OR_LANDED: u8 = FLAG_FILLING | crate::wire::FLAG_LANDED | crate::wire::FLAG_CARRYING;
