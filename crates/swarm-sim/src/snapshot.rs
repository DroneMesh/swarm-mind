//! Flat `f32` snapshot layout shared with the web renderer
//! (`web/src/snapshot.ts`; keep the two in sync).
//!
//! ```text
//! header  [version, tick, time_s, drones, links, pulses, frames, bytes,
//!          dropped, burning_cells, base_intent, reserved]
//! drone   [id, x, y, z, alive, role, hops, parent]  x drones  (parent -1 = none)
//! link    [a, b, kind, quality]                      x links   (kind 1 = route to parent)
//! pulse   [src, dst, kind]                           x pulses  (dst -1 = broadcast)
//! ```
//!
//! Links come from what the drones believe (their neighbor tables), not from
//! the radio model, so a link to a dead drone persists until its neighbors
//! notice the silence. `quality` (0..1, since v3) is what the radio really
//! delivers on it: the measured share of brain frames each end heard of
//! those the other end sent it over the last 3-4 s, the worse direction
//! ([`crate::linkq`]). Exactly 1 with no loss; a link that is physically
//! gone (out of range, behind terrain, a dead node) decays to 0 while the
//! tables still list it. With the channel model off it reflects range,
//! line of sight and `sim_set_loss`; with it on, every frame's SINR. The fire grid is exported separately (see
//! [`crate::FireGrid::cells`]) because it changes far less often.
//!
//! v3: `LINK_STRIDE` 3 → 4 (the `quality` field); everything else as v2.
//!
//! Additive since v2 (layout unchanged): the wind that steers the fire is
//! set with `sim_set_wind(sim, x, z)` (units/s, default `(-3, 0)`) and is not
//! echoed in the snapshot; the `reserved` header slot is still always 0.

pub const VERSION: f32 = 3.0;
pub const HEADER_LEN: usize = 12;
pub const DRONE_STRIDE: usize = 8;
pub const LINK_STRIDE: usize = 4;
pub const PULSE_STRIDE: usize = 3;
