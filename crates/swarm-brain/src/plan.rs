//! Formation geometry, derived identically by every drone.
//!
//! A mission names two points: the sink's position (`origin`) and the area
//! of interest (`target`). From those alone each drone computes the same
//! template of numbered *slots*, so no drone needs a map or a coordinator to
//! know where the formation goes; it only has to agree with its neighbours
//! on who takes which slot (see `mission.rs`).
//!
//! ```text
//!   sink ── A1 ── A2 ── A3 ── A4 ──  scouts ring the fire
//!     ·  p  B1  p  B2  p  B3  p  B4 ·
//! ```
//!
//! * **Relay ladder.** `rows` relay rows evenly spaced from origin to target,
//!   each with two rails (A and B) a little either side of the line. Every
//!   relay hears both relays of the rows either side, so losing any single
//!   relay leaves a working route without anyone moving.
//! * **Scouts.** Up to [`SCOUTS`] sectors around the target.
//! * **Parking spots** (`p`) for reserves, between the rows and further out
//!   to either side, filled middle-first. Parked reserves are hot spares a
//!   short hop from any gap, and in a pinch a sparse third rail.

use crate::Vec3;
use crate::math::sqrt;

/// Target spacing between relay rows. The radio reaches 260; this leaves
/// margin for the diagonal links of the ladder and for terrain.
pub const HOP_SPACING: f32 = 165.0;
/// Rails sit this far either side of the origin-target line.
pub const RAIL_OFFSET: f32 = 30.0;
/// Parking spots sit this far either side of the line.
pub const PARK_OFFSET: f32 = 80.0;
/// Longest ladder a drone will plan. Targets further away still get a
/// ladder this long, ending short of the target.
pub const MAX_ROWS: u8 = 12;
/// Scout sectors around the target.
pub const SCOUTS: u8 = 4;

/// Slot ids. 0 means "no slot".
pub const NO_SLOT: u8 = 0;
const RELAY_BASE: u8 = 1;
const SCOUT_BASE: u8 = 32;
const PARK_BASE: u8 = 40;
/// Parking spots: two per gap between rows, plus the two ends.
pub const MAX_PARKS: u8 = 2 * (MAX_ROWS + 1);
/// One past the largest slot id.
pub const SLOT_COUNT: usize = (PARK_BASE + MAX_PARKS) as usize;
const _: () = assert!(RELAY_BASE as usize + 2 * MAX_ROWS as usize <= SCOUT_BASE as usize);
const _: () = assert!(SCOUT_BASE + SCOUTS <= PARK_BASE);
const _: () = assert!(SLOT_COUNT <= 128, "slot sightings are tracked in a u128 bitmask");

/// Bit for `slot` in a heartbeat's `heard_slots` map: relay slots use their
/// id (1..=24), scouts the top four bits. Parks have none.
pub fn slot_bit(slot: u8) -> u32 {
    match Slot::decode(slot) {
        Some(Slot::Relay { .. }) => 1 << slot,
        Some(Slot::Scout(k)) => 1 << (28 + u32::from(k)),
        _ => 0,
    }
}

/// What a slot is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    /// Relay `row` (1-based, counted from the sink) on `rail` 0 or 1.
    Relay { row: u8, rail: u8 },
    /// Scout sector `k`.
    Scout(u8),
    /// Parking spot `j` for a reserve.
    Park(u8),
}

impl Slot {
    pub fn decode(id: u8) -> Option<Self> {
        match id {
            NO_SLOT => None,
            _ if id < RELAY_BASE + 2 * MAX_ROWS => {
                let i = id - RELAY_BASE;
                Some(Self::Relay { row: i / 2 + 1, rail: i % 2 })
            }
            _ if (SCOUT_BASE..SCOUT_BASE + SCOUTS).contains(&id) => Some(Self::Scout(id - SCOUT_BASE)),
            _ if id < PARK_BASE => None,
            _ if (id as usize) < SLOT_COUNT => Some(Self::Park(id - PARK_BASE)),
            _ => None,
        }
    }

    pub fn id(self) -> u8 {
        match self {
            Self::Relay { row, rail } => RELAY_BASE + 2 * (row - 1) + rail,
            Self::Scout(k) => SCOUT_BASE + k,
            Self::Park(j) => PARK_BASE + j,
        }
    }
}

/// Unit vectors every 45 degrees, so directions need no trigonometry.
const H: f32 = core::f32::consts::FRAC_1_SQRT_2;
const DIRS: [(f32, f32); 8] = [
    (1.0, 0.0),
    (H, H),
    (0.0, 1.0),
    (-H, H),
    (-1.0, 0.0),
    (-H, -H),
    (0.0, -1.0),
    (H, -H),
];
/// Scout `k` looks this many eighths of a turn from "back toward the sink":
/// the first two face each other across the fire, the next two fill in.
const SCOUT_TURNS: [usize; SCOUTS as usize] = [0, 4, 2, 6];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plan {
    pub origin: Vec3,
    pub target: Vec3,
    /// Unit vector from origin to target in the ground plane, and its
    /// left-hand perpendicular.
    u: (f32, f32),
    v: (f32, f32),
    pub rows: u8,
    pub spacing: f32,
}

impl Plan {
    pub fn new(origin: Vec3, target: Vec3) -> Self {
        let (dx, dz) = (target.x - origin.x, target.z - origin.z);
        let len = sqrt(dx * dx + dz * dz);
        let u = if len > 1.0 { (dx / len, dz / len) } else { (1.0, 0.0) };
        // Rows such that no gap exceeds HOP_SPACING; scouts take the last gap.
        let q = len / HOP_SPACING;
        let gaps = q as u32 + u32::from((q as u32 as f32) < q);
        let rows = gaps.saturating_sub(1).min(u32::from(MAX_ROWS)) as u8;
        let spacing = if rows == 0 { len } else { (len / f32::from(rows + 1)).min(HOP_SPACING) };
        Self { origin, target, u, v: (-u.1, u.0), rows, spacing }
    }

    /// A point `along` the line from the origin and `side` to its left.
    fn at(&self, along: f32, side: f32) -> Vec3 {
        let o = self.origin;
        let (u, v) = (self.u, self.v);
        Vec3::new(o.x + u.0 * along + v.0 * side, 0.0, o.z + u.1 * along + v.1 * side)
    }

    pub fn is_valid(&self, slot: u8) -> bool {
        match Slot::decode(slot) {
            Some(Slot::Relay { row, .. }) => (1..=self.rows).contains(&row),
            Some(Slot::Scout(_)) => true,
            Some(Slot::Park(j)) => j < self.parks(),
            None => false,
        }
    }

    pub fn parks(&self) -> u8 {
        2 * (self.rows + 1)
    }

    /// Ground-plane direction scout `k` faces, away from the target.
    pub fn scout_dir(&self, k: u8) -> (f32, f32) {
        let (c, s) = DIRS[SCOUT_TURNS[usize::from(k % SCOUTS)]];
        let back = (-self.u.0, -self.u.1);
        (back.0 * c - back.1 * s, back.0 * s + back.1 * c)
    }

    /// Nominal ground position (`y` = 0) of a slot. Scouts use a nominal
    /// radius here; in flight they track the fire instead.
    pub fn slot_xz(&self, slot: u8) -> Option<Vec3> {
        if !self.is_valid(slot) {
            return None;
        }
        Some(match Slot::decode(slot)? {
            Slot::Relay { row, rail } => {
                let side = if rail == 0 { -RAIL_OFFSET } else { RAIL_OFFSET };
                self.at(f32::from(row) * self.spacing, side)
            }
            Slot::Scout(k) => {
                let (dx, dz) = self.scout_dir(k);
                Vec3::new(self.target.x + dx * 40.0, 0.0, self.target.z + dz * 40.0)
            }
            Slot::Park(j) => {
                let gap = middle_first(j / 2, self.rows + 1);
                let side = if j % 2 == 0 { PARK_OFFSET } else { -PARK_OFFSET };
                self.at((f32::from(gap) + 0.5) * self.spacing, side)
            }
        })
    }

    /// The `i`-th slot in fill order. Connectivity first (rail A), then the
    /// two scouts the mission can't do without, then redundancy (rail B),
    /// then the remaining scouts, then parking spots for everyone else.
    pub fn slot_at(&self, i: usize) -> u8 {
        let rows = usize::from(self.rows);
        let relay = |row: usize, rail| Slot::Relay { row: row as u8 + 1, rail }.id();
        let mut i = i;
        if i < rows {
            return relay(i, 0);
        }
        i -= rows;
        if i < 2 {
            return Slot::Scout(i as u8).id();
        }
        i -= 2;
        if i < rows {
            return relay(i, 1);
        }
        i -= rows;
        let extra = usize::from(SCOUTS - 2);
        if i < extra {
            return Slot::Scout(2 + i as u8).id();
        }
        i -= extra;
        Slot::Park((i % usize::from(self.parks())) as u8).id()
    }

    /// Position of a slot in fill order; lower is more important. Parks
    /// come last.
    pub fn priority(&self, slot: u8) -> u8 {
        let rows = self.rows;
        match Slot::decode(slot) {
            Some(Slot::Relay { row, rail: 0 }) => row - 1,
            Some(Slot::Scout(k)) if k < 2 => rows + k,
            Some(Slot::Relay { row, .. }) => rows + 2 + row - 1,
            Some(Slot::Scout(k)) => 2 * rows + k,
            _ => u8::MAX,
        }
    }

    /// Slots whose holders should be able to hear the holder of `slot`, and
    /// so are the ones to notice when it falls silent. The sink watches the
    /// first row (`slot` = [`NO_SLOT`] stands for the sink).
    pub fn neighbours(&self, slot: u8, out: &mut [u8; 8]) -> usize {
        let mut n = 0;
        let mut push = |s: Slot| {
            let id = s.id();
            if self.is_valid(id) && id != slot && n < out.len() {
                out[n] = id;
                n += 1;
            }
        };
        match Slot::decode(slot) {
            None => {
                if self.rows > 0 {
                    push(Slot::Relay { row: 1, rail: 0 });
                    push(Slot::Relay { row: 1, rail: 1 });
                }
            }
            Some(Slot::Relay { row, rail }) => {
                push(Slot::Relay { row, rail: 1 - rail });
                for r in [row.wrapping_sub(1), row + 1] {
                    if (1..=self.rows).contains(&r) {
                        push(Slot::Relay { row: r, rail: 0 });
                        push(Slot::Relay { row: r, rail: 1 });
                    }
                }
                if row == self.rows {
                    push(Slot::Scout(0));
                    push(Slot::Scout(1));
                }
            }
            Some(Slot::Scout(_)) => {
                for k in 0..SCOUTS {
                    push(Slot::Scout(k));
                }
                if self.rows > 0 {
                    push(Slot::Relay { row: self.rows, rail: 0 });
                    push(Slot::Relay { row: self.rows, rail: 1 });
                }
            }
            Some(Slot::Park(_)) => {}
        }
        n
    }
}

/// The `k`-th of `n` positions counted outward from the middle:
/// for `n` = 5 that is 2, 1, 3, 0, 4.
fn middle_first(k: u8, n: u8) -> u8 {
    let n = n.max(1);
    let mid = (n - 1) / 2;
    let mut seen = 0;
    for step in 0..=n {
        for cand in [mid.checked_sub(step), mid.checked_add(step).filter(|_| step > 0)] {
            if let Some(c) = cand.filter(|&c| c < n) {
                if seen == k % n {
                    return c;
                }
                seen += 1;
            }
        }
    }
    mid
}

#[cfg(test)]
mod tests {
    use super::*;

    fn demo() -> Plan {
        Plan::new(Vec3::new(-456.0, 35.0, 0.0), Vec3::new(330.0, 12.0, 40.0))
    }

    #[test]
    fn ladder_spans_the_gap_with_margin() {
        let p = demo();
        assert_eq!(p.rows, 4);
        assert!(p.spacing <= HOP_SPACING);
        // Consecutive rows, diagonals included, are well inside radio range.
        let a1 = p.slot_xz(Slot::Relay { row: 1, rail: 0 }.id()).unwrap();
        let b2 = p.slot_xz(Slot::Relay { row: 2, rail: 1 }.id()).unwrap();
        assert!(a1.dist_xz(b2) < 200.0);
        let a4 = p.slot_xz(Slot::Relay { row: 4, rail: 0 }.id()).unwrap();
        assert!(a4.dist_xz(p.target) < 200.0);
        assert!(p.origin.dist_xz(a1) < 200.0);
    }

    #[test]
    fn short_and_degenerate_missions() {
        let near = Plan::new(Vec3::ZERO, Vec3::new(100.0, 0.0, 0.0));
        assert_eq!(near.rows, 0);
        assert_eq!(near.slot_at(0), Slot::Scout(0).id());
        let same = Plan::new(Vec3::ZERO, Vec3::ZERO);
        assert_eq!(same.rows, 0);
        let far = Plan::new(Vec3::ZERO, Vec3::new(1e9, 0.0, -1e9));
        assert_eq!(far.rows, MAX_ROWS);
        for i in 0..64 {
            assert!(far.slot_xz(far.slot_at(i)).unwrap().is_finite());
        }
    }

    #[test]
    fn heard_bits_are_distinct() {
        let mut all = 0u32;
        for id in 1..SLOT_COUNT as u8 {
            let bit = slot_bit(id);
            assert_eq!(all & bit, 0, "slot {id} shares a bit");
            all |= bit;
        }
        assert_eq!(slot_bit(Slot::Park(0).id()), 0);
    }

    #[test]
    fn slot_ids_round_trip() {
        let mut valid = 0;
        for id in 1..SLOT_COUNT as u8 {
            if let Some(slot) = Slot::decode(id) {
                assert_eq!(slot.id(), id);
                valid += 1;
            }
        }
        assert_eq!(valid, 2 * MAX_ROWS as usize + SCOUTS as usize + MAX_PARKS as usize);
        assert_eq!(Slot::decode(NO_SLOT), None);
        assert_eq!(Slot::decode(SLOT_COUNT as u8), None);
    }

    #[test]
    fn fill_order_puts_connectivity_first() {
        let p = demo();
        let order: [u8; 12] = core::array::from_fn(|i| p.slot_at(i));
        let relay = |row, rail| Slot::Relay { row, rail }.id();
        assert_eq!(&order[..4], &[relay(1, 0), relay(2, 0), relay(3, 0), relay(4, 0)]);
        assert_eq!(&order[4..6], &[Slot::Scout(0).id(), Slot::Scout(1).id()]);
        assert_eq!(&order[6..10], &[relay(1, 1), relay(2, 1), relay(3, 1), relay(4, 1)]);
        for (i, &s) in order.iter().enumerate() {
            assert_eq!(usize::from(p.priority(s)), i);
        }
        assert!(matches!(Slot::decode(p.slot_at(12)), Some(Slot::Park(0))));
    }

    #[test]
    fn parking_fills_from_the_middle() {
        assert_eq!([0, 1, 2, 3, 4].map(|k| middle_first(k, 5)), [2, 1, 3, 0, 4]);
        assert_eq!([0, 1, 2, 3].map(|k| middle_first(k, 4)), [1, 0, 2, 3]);
        assert_eq!(middle_first(0, 1), 0);
        let p = demo();
        let mid = p.at(2.5 * p.spacing, 0.0);
        assert!(p.slot_xz(Slot::Park(0).id()).unwrap().dist_xz(mid) <= PARK_OFFSET + 1.0);
    }

    #[test]
    fn ladder_neighbours() {
        let p = demo();
        let mut out = [0; 8];
        let n = p.neighbours(Slot::Relay { row: 2, rail: 0 }.id(), &mut out);
        assert_eq!(n, 5);
        let n = p.neighbours(Slot::Relay { row: 4, rail: 1 }.id(), &mut out);
        assert!(out[..n].contains(&Slot::Scout(0).id()));
        let n = p.neighbours(NO_SLOT, &mut out);
        let first_row = [Slot::Relay { row: 1, rail: 0 }.id(), Slot::Relay { row: 1, rail: 1 }.id()];
        assert_eq!(&out[..n], &first_row);
    }
}
