//! Ground-truth checks for tests and the scenario suite.
//!
//! These look at the whole world at once, which no drone can do. They exist
//! to judge the swarm, never to steer it.

use swarm_brain::{NO_ROUTE, Role};

use crate::Sim;

/// Longest parent chain worth following; anything longer is a loop.
const MAX_CHAIN: usize = 64;

/// True if following `id`'s parent pointers reaches a live sink, with every
/// hop alive and actually within radio contact of the next (range and line
/// of sight, ignoring random loss).
pub fn route_ok(sim: &Sim, id: u16) -> bool {
    let mut at = id;
    for _ in 0..MAX_CHAIN {
        let Some(brain) = sim.brain(at) else { return false };
        if !sim.is_alive(at) {
            return false;
        }
        if brain.role() == Role::Sink {
            return true;
        }
        if brain.hops() == NO_ROUTE {
            return false;
        }
        let Some(parent) = brain.parent() else { return false };
        if !sim.is_alive(parent) || !sim.in_contact(at, parent) {
            return false;
        }
        at = parent;
    }
    false
}

/// Live drones currently in the given role.
pub fn with_role(sim: &Sim, role: Role) -> impl Iterator<Item = u16> + '_ {
    (0..sim.drone_count() as u16)
        .filter(move |&id| sim.is_alive(id) && sim.brain(id).is_some_and(|b| b.role() == role))
}

/// True when there is at least one live scout and every live scout has a
/// working route to a sink.
pub fn scouts_connected(sim: &Sim) -> bool {
    let mut any = false;
    for id in with_role(sim, Role::Scout) {
        any = true;
        if !route_ok(sim, id) {
            return false;
        }
    }
    any
}

/// Live drones (excluding sinks) whose route to a sink works.
pub fn routed_count(sim: &Sim) -> usize {
    (0..sim.drone_count() as u16)
        .filter(|&id| sim.is_alive(id))
        .filter(|&id| sim.brain(id).is_some_and(|b| b.role() != Role::Sink))
        .filter(|&id| route_ok(sim, id))
        .count()
}
