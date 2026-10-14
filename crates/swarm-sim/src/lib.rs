//! World simulation: terrain, radio and drone bodies.
//!
//! The simulator is the only thing that knows where everything is. Drones
//! never see this state; each brain gets only its own sensor readings and
//! whatever frames the radio model delivers to it.

pub mod airlog;
pub mod channel;
pub mod chatter;
pub mod fire;
pub mod linkq;
pub mod metrics;
pub mod painless;
pub mod preset;
#[cfg(test)]
mod preset_tests;
mod radio;
mod rng;
pub mod snapshot;
pub mod telemetry;
mod terrain;
mod window;
mod world;

pub use channel::ChannelConfig;
pub use fire::FireGrid;
pub use preset::Preset;
pub use radio::RadioConfig;
pub use telemetry::{RateMode, Routing, TelemetryConfig, Variant};
pub use terrain::Terrain;
pub use world::{BASE_ID, Sim, SimConfig, StepStats, base_position};
