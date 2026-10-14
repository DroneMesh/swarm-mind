//! Per-drone swarm brain.
//!
//! Every drone runs its own [`Brain`]. It knows only what its own sensors
//! report and the frames that reach its inbox; there is no shared state
//! between drones, no map and no coordinator. The crate is `no_std` with
//! fixed-size buffers and no allocator, so the same code runs in the
//! browser (WASM) and on microcontrollers such as the ESP32.
//!
//! # How the swarm thinks
//!
//! **Roles.** The base station is the *sink*. When the operator asks it to
//! monitor a fire, every drone derives the same formation from two points,
//! the sink and the fire (`plan.rs`): a two-rail *relay* ladder from base
//! to fire with rows about 160 apart (radio range is 260), *scouts* in
//! sectors around the fire, and parking spots near the middle of the
//! ladder for *reserves*, the hot spares. Slots are assigned by ranking the
//! drones each one can hear, so the initial allocation costs no messages;
//! duplicate claims resolve deterministically (in position beats en route,
//! clearly closer beats farther, then lowest id) and a loser never returns,
//! so roles don't thrash (`mission.rs`).
//!
//! **Messages.** Two kinds. The heartbeat (31 bytes) carries everything:
//! routing gradient, sink freshness, the sender's slot claim and flags, a
//! vacancy notice, a bitmap of slot holders it heard recently (so one
//! missed heartbeat is never mistaken for a death), its intent version, a
//! beacon counter (for loss estimation) and its beacon interval. Intents
//! are flooded once per sequence number with Trickle-style suppression,
//! and repaired by anti-entropy when a heartbeat shows a neighbour is
//! behind. There are no
//! requests, acks or per-repair messages.
//!
//! **Routing** (`routing.rs`). A distance-vector gradient to the sink with
//! the Babel feasibility condition (loop-free even when neighbours act on
//! stale information at the same instant, no count-to-infinity), sink
//! sequence numbers for freshness, poisoning of dead routes and a hop cap.
//! Parent choice weighs signal strength, so routes avoid edge-of-range
//! links that snap when anything moves, and avoids drones in transit.
//!
//! **Self-healing.** A relay that misses two beacons is suspected (more on a
//! lossy channel); its children switch to the other rail of the ladder,
//! which is always in range, and tell their own children with an immediate
//! heartbeat. Every route is back in about 1.5-2.5 s without anyone moving.
//! Meanwhile the dead relay's ladder neighbours notice its slot has gone
//! quiet and advertise it as needed; the closest parked reserve claims it
//! and flies in, restoring the redundancy. If no reserve is left, the
//! holder of the least important slot moves up after a while. Reserves
//! that are bridging traffic stay put.
//!
//! **Partitions and base loss.** A drone with no route climbs and flies
//! toward where its parent was last heard, then toward the sink, until it
//! re-merges. If the base falls silent, the drones that heard it directly
//! elect the lowest-id one among them as acting sink (about 4 s); the
//! mission carries on, and gradients from lower sink ids win, so groups
//! that elected separately merge when they meet. A group cut off for long
//! enough to elect its own sink while the base lives on keeps flying back
//! toward the base until it hears it.
//!
//! **Moving without breaking links.** Drones climb before they cruise (so
//! they don't skim up a slope into radio shadow), never descend while
//! others route through them or while their own parent link is long, and
//! won't fly further from a parent or child whose link is already
//! marginal. A relay on a weak link to a parent above it climbs toward it,
//! but never climbs away from a lower parent: signal follows distance, so
//! that would only stretch the link until it broke. Relays also keep every
//! link they carry (to the parent and to the drones routing through them)
//! shallower than a fixed slope by flying higher: with no terrain map, a
//! steep link is the one sign that a few units of motion will put a slope
//! in its way.
//!
//! **Efficiency.** Steady state is one heartbeat per second per drone with
//! a job, 0.8 for parked reserves and landed drones, 2 for the sink during
//! a mission: about one frame per drone per second overall (0.8 idle).
//! Periodic heartbeats are jittered (early only), so drones that change
//! interval together don't stay in lockstep. Repairs add only
//! rate-limited, jittered triggered heartbeats. A drone
//! tolerates more missed beacons when it measures more loss, so a lossy
//! channel doesn't trigger false repairs or role changes.
//!
//! **Telemetry airtime** (`airtime.rs`, Phase 5). The host also streams
//! its firmware telemetry, and on a shared 2.4 GHz channel that stream is
//! most of the traffic. Each brain picks its own telemetry interval
//! (`Actuation::telemetry_interval_ms`) by AIMD on local evidence: its
//! radio's refused sends (`Sensors::tx_dropped`) and its beacon-loss
//! estimate. Congestion levels ride in spare heartbeat bits (no extra bytes
//! or frames): reports travel up the gradient to the sink, and the sink's
//! verdict comes back down to everyone, so the whole funnel to the base
//! backs off and recovers together and converges to equal rates. It never
//! exceeds the operator's rate and never drops below 5 Hz. While telemetry
//! runs on a lossy channel, drones that carry traffic beacon twice as often
//! (repair stays within a couple of seconds) and nobody adopts a route
//! through a relay it already suspects dead. With telemetry off none of
//! this runs and the brain behaves exactly as before.
//!
//! Everything is bounded: fixed tables (48 neighbours, 4 needs, one slot
//! sighting per slot id), loops over those tables only, no allocation,
//! no floating-point library calls (`sqrt` is our own). The whole brain is
//! about 2.8 KB.
#![no_std]

mod airtime;
mod brain;
mod fixed;
mod math;
mod mission;
mod plan;
mod routing;
#[cfg(test)]
mod tests;
mod wire;

pub use airtime::MIN_HZ as TELEMETRY_MIN_HZ;
pub use brain::{Actuation, Brain, Config, FireReading, NO_ROUTE, Neighbor, Sensors};
pub use fixed::FixedVec;
pub use math::{Vec3, sqrt};
pub use plan::{NO_SLOT, Plan, Slot};
pub use routing::MAX_HOPS;
pub use wire::{
    BROADCAST, Beacon, FLAG_CARRYING, FLAG_CONGESTION_MASK, FLAG_CONGESTION_SHIFT, FLAG_CUT_EPOCH, FLAG_FILLING, FLAG_LANDED, FLAG_ORPHAN, FLAG_SETTLED, Frame, Intent,
    KIND_HEARTBEAT, KIND_INTENT, MAX_FRAME, Message, NodeId, OPERATOR, Payload, Role, Rx,
};

/// Most drones a brain tracks at once. Beyond this the stalest is evicted.
/// Large enough that a 50-drone swarm parked together can rank itself
/// consistently at mission start.
pub const MAX_NEIGHBORS: usize = 48;
/// Frames a drone can receive between two ticks; extra frames are dropped.
pub const MAX_INBOX: usize = 16;
/// Frames a drone can queue for sending in one tick.
pub const MAX_OUTBOX: usize = 4;

pub type Inbox = FixedVec<Rx, MAX_INBOX>;
pub type Outbox = FixedVec<Frame, MAX_OUTBOX>;
