//! C-ABI bridge so the browser can drive the simulator.
//!
//! This deliberately avoids wasm-bindgen: the web worker only needs a
//! handful of `extern "C"` functions and raw pointers into linear memory.
//! Every function taking a `*Sim` requires a live pointer from [`sim_new`]
//! that hasn't been passed to [`sim_free`]. A null pointer is tolerated (the
//! call does nothing and returns zero/null) so a worker that failed to start
//! can't trap; every other argument is sanitised, so no input panics.
//!
//! Exports: `sim_new`, `sim_free`, `sim_step`, `sim_snapshot_ptr`,
//! `sim_snapshot_len`, `sim_terrain_ptr`, `sim_terrain_size`,
//! `sim_terrain_extent`, `sim_kill`, `sim_command`, `sim_set_loss`,
//! `sim_fire_ptr`, `sim_fire_size`, and (added later, optional for callers)
//! `sim_set_wind`, `sim_ignite`, `sim_new_preset`, `sim_preset`, `sim_chatter_ptr`,
//! `sim_chatter_len`, and the telemetry / shared-channel model:
//! `sim_set_channel`, `sim_set_telemetry`, `sim_telemetry_ptr`,
//! `sim_telemetry_len`, `sim_channel_ptr`, `sim_channel_len`,
//! `sim_tlm_sample_ptr`, `sim_tlm_sample_len`.
//!
//! Layouts (see `swarm_sim::snapshot` and `swarm_sim::telemetry`):
//! the snapshot is v3 (`LINK_STRIDE` 4: links are `[a, b, kind,
//! quality]`, quality 0..1 = measured brain-frame delivery of the worse
//! direction over the last 3-4 s); the channel summary is `CHANNEL_LEN` 28
//! floats (16-26: bytes on air per second by kind, retries, frames per
//! second, link quality mean and degraded links, 27 the NodeSync /
//! TimeSync bytes per second over 60 s); per-drone stats stay
//! `TLM_STRIDE` 20 floats with 17 `tx_all_bytes_per_s`, 18
//! `ack_bytes_per_s` and 19 `parent_rate_mbps`. Byte figures are MPDU
//! bytes of every transmission (retries and ACKs included) and the
//! per-drone `tx_all_bytes_per_s` sum to `bytes_on_air`.

use swarm_brain::Intent;
use swarm_sim::{Preset, RateMode, Routing, Sim, SimConfig, TelemetryConfig, Variant};

const MAX_DRONES: u32 = 256;

/// Longest step accepted: a backgrounded tab can hand us a huge `dt`, and
/// teleporting the world forward helps nobody.
const MAX_STEP_MS: u32 = 1_000;

/// Shared access, or `default` for a null pointer.
///
/// # Safety
/// `sim` must be null or a live pointer from [`sim_new`].
unsafe fn with<R>(sim: *const Sim, default: R, f: impl FnOnce(&Sim) -> R) -> R {
    match unsafe { sim.as_ref() } {
        Some(sim) => f(sim),
        None => default,
    }
}

/// Exclusive access, or `default` for a null pointer.
///
/// # Safety
/// `sim` must be null or a live pointer from [`sim_new`], not aliased.
unsafe fn with_mut<R>(sim: *mut Sim, default: R, f: impl FnOnce(&mut Sim) -> R) -> R {
    match unsafe { sim.as_mut() } {
        Some(sim) => f(sim),
        None => default,
    }
}

/// A new world with `drones` drones (clamped to `1..=256`, base included).
#[unsafe(no_mangle)]
pub extern "C" fn sim_new(seed: u32, drones: u32) -> *mut Sim {
    let cfg = SimConfig {
        seed: u64::from(seed),
        drones: drones.clamp(1, MAX_DRONES) as usize,
        ..SimConfig::default()
    };
    Box::into_raw(Box::new(Sim::new(cfg)))
}

/// A new world on map `preset` (see `swarm_sim::Preset`), or null if the
/// preset is unknown. `drones` is clamped as in [`sim_new`].
/// `sim_new_preset(0, seed, n)` builds the same world as `sim_new(seed, n)`.
#[unsafe(no_mangle)]
pub extern "C" fn sim_new_preset(preset: u32, seed: u32, drones: u32) -> *mut Sim {
    let Some(preset) = Preset::from_u32(preset) else { return core::ptr::null_mut() };
    let drones = drones.clamp(1, MAX_DRONES) as usize;
    let cfg = SimConfig::for_preset(preset, u64::from(seed), drones);
    Box::into_raw(Box::new(Sim::new(cfg)))
}

/// The map the world was built from, as its `Preset` value.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_preset(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.preset() as u32) }
}

/// Decoded frames of the latest step, `swarm_sim::chatter::CHATTER_STRIDE`
/// floats per record (layout in `swarm_sim::chatter`).
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed. The
/// pointer is valid until the next call that mutates the simulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_chatter_ptr(sim: *const Sim) -> *const f32 {
    unsafe { with(sim, core::ptr::null(), |s| s.chatter().as_ptr()) }
}

/// Length of [`sim_chatter_ptr`] in floats (not records).
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_chatter_len(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.chatter().len() as u32) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_free(sim: *mut Sim) {
    if !sim.is_null() {
        drop(unsafe { Box::from_raw(sim) });
    }
}

/// Advances the world by `dt_ms`, capped at one second.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_step(sim: *mut Sim, dt_ms: u32) {
    unsafe { with_mut(sim, (), |s| s.step(dt_ms.min(MAX_STEP_MS))) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed. The
/// pointer is valid until the next call that mutates the simulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_snapshot_ptr(sim: *const Sim) -> *const f32 {
    unsafe { with(sim, core::ptr::null(), |s| s.snapshot().as_ptr()) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_snapshot_len(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.snapshot().len() as u32) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_terrain_ptr(sim: *const Sim) -> *const f32 {
    unsafe { with(sim, core::ptr::null(), |s| s.terrain().heights().as_ptr()) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_terrain_size(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.terrain().size() as u32) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_terrain_extent(sim: *const Sim) -> f32 {
    unsafe { with(sim, 0.0, |s| s.terrain().extent()) }
}

/// Sea surface height for maps with a sea, NaN for dry maps (or a null
/// `sim`). Ground below it is water: drones fly over the surface.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_water_level(sim: *const Sim) -> f32 {
    unsafe { with(sim, f32::NAN, |s| s.terrain().water_level().unwrap_or(f32::NAN)) }
}

/// Returns 1 if a live drone was switched off; unknown ids return 0.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_kill(sim: *mut Sim, id: u32) -> u32 {
    let Ok(id) = u16::try_from(id) else { return 0 };
    unsafe { with_mut(sim, 0, |s| u32::from(s.kill(id))) }
}

/// Returns 1 if the command reached the base station. `(x, z)` is the area
/// of interest, e.g. the reported fire location; non-finite coordinates are
/// rejected and off-map ones pulled onto the map edge.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_command(sim: *mut Sim, intent: u32, x: f32, z: f32) -> u32 {
    let Some(intent) = u8::try_from(intent).ok().and_then(Intent::from_u8) else { return 0 };
    unsafe { with_mut(sim, 0, |s| u32::from(s.command(intent, x, z))) }
}

/// Sets the probability, clamped to `[0, 1]` (NaN = 0), that any delivery
/// is lost.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_loss(sim: *mut Sim, loss: f32) {
    unsafe { with_mut(sim, (), |s| s.set_loss(loss)) }
}

/// Sets the wind as `(x, z)` in units per second; it only steers the fire.
/// Non-finite components count as calm. Default `(-3, 0)`: a gentle drift
/// toward -x that pushes the fire up toward the ridge. Additive export.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_wind(sim: *mut Sim, x: f32, z: f32) {
    unsafe { with_mut(sim, (), |s| s.set_wind(x, z)) }
}

/// Starts a spot fire at `(x, z)`; invalid or off-map points are ignored.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_ignite(sim: *mut Sim, x: f32, z: f32) {
    unsafe { with_mut(sim, (), |s| s.ignite(x, z)) }
}

/// Row-major fire cell states, `sim_fire_size()^2` bytes: 0 unburnt,
/// 1 burning, 2 burnt.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed. The
/// pointer is valid until the next call that mutates the simulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_fire_ptr(sim: *const Sim) -> *const u8 {
    unsafe { with(sim, core::ptr::null(), |s| s.fire().cells().as_ptr()) }
}

/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_fire_size(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.fire().size() as u32) }
}

/// Turns the shared-airtime model on (`enabled != 0`) or off and sets the
/// airtime used by other networks, clamped to `0..=0.8` (NaN = 0).
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_channel(sim: *mut Sim, enabled: u32, external_busy: f32) {
    unsafe {
        with_mut(sim, (), |s| {
            s.set_channel_enabled(enabled != 0);
            s.set_external_busy(external_busy);
        })
    }
}

/// Configures telemetry: `enabled != 0`; `variant` 0 XIAO S3, 1 C6 RTK,
/// 2 mixed; `rate_mode` 0 fixed at `hz`, 1 backend budget, 2 brain;
/// `hz` the operator's requested rate (clamped to `0.1..=50`, non-finite =
/// 25); `routing` 0 single to root, 1 broadcast. Returns 1 if applied, 0 if
/// a code is unknown (nothing changes).
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_telemetry(
    sim: *mut Sim,
    enabled: u32,
    variant: u32,
    rate_mode: u32,
    hz: f32,
    routing: u32,
) -> u32 {
    let (Some(variant), Some(rate), Some(routing)) =
        (Variant::from_u32(variant), RateMode::from_code(rate_mode, hz), Routing::from_u32(routing))
    else {
        return 0;
    };
    unsafe {
        with_mut(sim, 0, |s| {
            let cfg = TelemetryConfig {
                enabled: enabled != 0,
                variant,
                rate,
                routing,
                requested_hz: hz,
                ..s.config().telemetry
            };
            s.set_telemetry(cfg);
            1
        })
    }
}

/// Per-drone telemetry stats, `swarm_sim::telemetry::TLM_STRIDE` (20) floats
/// per drone (layout in `swarm_sim::telemetry`; offsets in `tlm`).
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed. The
/// pointer is valid until the next call that mutates the simulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_telemetry_ptr(sim: *const Sim) -> *const f32 {
    unsafe { with(sim, core::ptr::null(), |s| s.telemetry_stats().as_ptr()) }
}

/// Length of [`sim_telemetry_ptr`] in floats.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_telemetry_len(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.telemetry_stats().len() as u32) }
}

/// Fleet channel summary, `swarm_sim::telemetry::CHANNEL_LEN` (28) floats;
/// offsets in `swarm_sim::telemetry::chan`.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed. The
/// pointer is valid until the next call that mutates the simulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_channel_ptr(sim: *const Sim) -> *const f32 {
    unsafe { with(sim, core::ptr::null(), |s| s.channel_summary().as_ptr()) }
}

/// Length of [`sim_channel_ptr`] in floats.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_channel_len(sim: *const Sim) -> u32 {
    unsafe { with(sim, 0, |s| s.channel_summary().len() as u32) }
}

/// UTF-8 bytes of the last painlessMesh package drone `id` sent (no `'\0'`);
/// null for a null `sim`. Unknown ids give an empty sample.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed. The
/// pointer is valid until the next call that mutates the simulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_tlm_sample_ptr(sim: *const Sim, id: u32) -> *const u8 {
    let id = u16::try_from(id).unwrap_or(u16::MAX);
    unsafe { with(sim, core::ptr::null(), |s| s.tlm_sample(id).as_ptr()) }
}

/// Length of [`sim_tlm_sample_ptr`] in bytes.
///
/// # Safety
/// `sim` must be null or come from [`sim_new`] and not have been freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_tlm_sample_len(sim: *const Sim, id: u32) -> u32 {
    let id = u16::try_from(id).unwrap_or(u16::MAX);
    unsafe { with(sim, 0, |s| s.tlm_sample(id).len() as u32) }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drives every export the way a careless or hostile caller might.
    #[test]
    fn abi_survives_hostile_input() {
        let floats = [
            0.0,
            -0.0,
            1.0,
            -1.0,
            0.5,
            2.0,
            1e30,
            -1e30,
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
        ];
        for drones in [0, 1, 2, 20, 256, 257, u32::MAX] {
            let sim = sim_new(drones, drones);
            unsafe {
                assert!(sim_snapshot_len(sim) as usize >= swarm_sim::snapshot::HEADER_LEN);
                let count = *sim_snapshot_ptr(sim).add(3);
                assert_eq!(count as u32, drones.clamp(1, MAX_DRONES));
                assert!(sim_terrain_size(sim) >= 2 && sim_terrain_extent(sim) > 0.0);
                assert!(!sim_terrain_ptr(sim).is_null() && !sim_fire_ptr(sim).is_null());
                for id in [0, 1, 19, 255, 256, 65_535, 65_536, u32::MAX] {
                    sim_kill(sim, id);
                }
                for (k, &v) in floats.iter().enumerate() {
                    let w = floats[(k + 3) % floats.len()];
                    sim_set_loss(sim, v);
                    sim_set_channel(sim, k as u32 % 3, v);
                    for code in [0, 1, 2, 3, u32::MAX] {
                        let ok = sim_set_telemetry(sim, (k as u32).wrapping_add(code) % 2, code, code, w, code % 3);
                        assert_eq!(ok, u32::from(code <= 1), "codes {code}");
                    }
                    sim_set_telemetry(sim, 1, k as u32 % 3, k as u32 % 3, v, k as u32 % 2);
                    sim_set_wind(sim, v, w);
                    sim_ignite(sim, v, w);
                    for intent in [0, 1, 2, 3, 4, 255, 256, u32::MAX] {
                        sim_command(sim, intent, v, w);
                    }
                    for dt in [0, 1, 50, u32::MAX] {
                        sim_step(sim, dt);
                    }
                }
                let len = sim_snapshot_len(sim) as usize;
                let snap = core::slice::from_raw_parts(sim_snapshot_ptr(sim), len);
                assert!(snap.iter().all(|v| v.is_finite()));
                let tlm = core::slice::from_raw_parts(sim_telemetry_ptr(sim), sim_telemetry_len(sim) as usize);
                assert_eq!(tlm.len(), count as usize * swarm_sim::telemetry::TLM_STRIDE);
                assert!(tlm.iter().all(|v| v.is_finite()), "telemetry stats finite");
                let ch = core::slice::from_raw_parts(sim_channel_ptr(sim), sim_channel_len(sim) as usize);
                assert_eq!(ch.len(), swarm_sim::telemetry::CHANNEL_LEN);
                assert!(ch.iter().all(|v| v.is_finite()), "channel summary finite");
                for id in [0, 1, 19, 255, 256, 65_535, 65_536, u32::MAX] {
                    let len = sim_tlm_sample_len(sim, id) as usize;
                    let bytes = core::slice::from_raw_parts(sim_tlm_sample_ptr(sim, id), len);
                    assert!(core::str::from_utf8(bytes).is_ok());
                }
                let n = sim_fire_size(sim) as usize;
                let fire = core::slice::from_raw_parts(sim_fire_ptr(sim), n * n);
                assert!(fire.iter().all(|&c| c <= 2));
                sim_free(sim);
            }
        }
    }

    #[test]
    fn null_sim_is_harmless() {
        let null = core::ptr::null_mut();
        unsafe {
            sim_step(null, 50);
            sim_set_loss(null, 0.5);
            sim_set_wind(null, 1.0, 1.0);
            sim_ignite(null, 1.0, 1.0);
            assert_eq!(sim_kill(null, 1), 0);
            assert_eq!(sim_command(null, 1, 0.0, 0.0), 0);
            assert_eq!(sim_snapshot_len(null), 0);
            assert!(sim_snapshot_ptr(null).is_null() && sim_fire_ptr(null).is_null());
            assert_eq!((sim_terrain_size(null), sim_fire_size(null)), (0, 0));
            assert_eq!((sim_preset(null), sim_chatter_len(null)), (0, 0));
            assert!(sim_chatter_ptr(null).is_null());
            assert!(sim_water_level(null).is_nan());
            sim_set_channel(null, 1, 0.5);
            assert_eq!(sim_set_telemetry(null, 1, 0, 0, 25.0, 0), 0);
            assert!(sim_telemetry_ptr(null).is_null() && sim_channel_ptr(null).is_null());
            assert_eq!((sim_telemetry_len(null), sim_channel_len(null)), (0, 0));
            assert!(sim_tlm_sample_ptr(null, 1).is_null());
            assert_eq!(sim_tlm_sample_len(null, 1), 0);
            sim_free(null);
        }
    }

    #[test]
    fn presets_build_and_stream_chatter() {
        for p in [5, 6, u32::MAX] {
            assert!(sim_new_preset(p, 1, 20).is_null());
        }
        for p in 0..5 {
            let sim = sim_new_preset(p, 1, 0);
            assert!(!sim.is_null());
            unsafe {
                assert_eq!(sim_preset(sim), p);
                // Only the archipelago has a sea.
                assert_eq!(sim_water_level(sim).is_nan(), p != 3, "preset {p} water level");
                assert_eq!(*sim_snapshot_ptr(sim).add(3), 1.0, "drones clamped to 1");
                sim_free(sim);
            }
        }
        let (a, b) = (sim_new_preset(0, 3, 20), sim_new(3, 20));
        unsafe {
            for _ in 0..40 {
                sim_step(a, 50);
                sim_step(b, 50);
            }
            let snap = |s| core::slice::from_raw_parts(sim_snapshot_ptr(s), sim_snapshot_len(s) as usize);
            assert_eq!(snap(a), snap(b), "preset 0 is the classic world");
            let len = sim_chatter_len(a) as usize;
            assert_eq!(len % swarm_sim::chatter::CHATTER_STRIDE, 0);
            let mut heard = len > 0;
            for _ in 0..40 {
                sim_step(a, 50);
                heard |= sim_chatter_len(a) > 0;
            }
            assert!(heard, "drones beacon within a few seconds");
            sim_free(a);
            sim_free(b);
        }
    }

    #[test]
    fn telemetry_streams_packages_and_stats() {
        use swarm_sim::telemetry::{TLM_STRIDE, chan, tlm};
        let sim = sim_new(1, 6);
        unsafe {
            sim_set_channel(sim, 1, 0.2);
            assert_eq!(sim_set_telemetry(sim, 1, 1, 0, 25.0, 0), 1);
            for _ in 0..60 {
                sim_step(sim, 50);
            }
            let len = sim_tlm_sample_len(sim, 3) as usize;
            let text = core::str::from_utf8(core::slice::from_raw_parts(sim_tlm_sample_ptr(sim, 3), len)).unwrap();
            assert!(text.starts_with(r#"{"type":9,"dest":"#) && text.contains(r#"\"type\":\"drone\""#), "{text}");
            let st = core::slice::from_raw_parts(sim_telemetry_ptr(sim), sim_telemetry_len(sim) as usize);
            let row = &st[3 * TLM_STRIDE..4 * TLM_STRIDE];
            assert_eq!(row[tlm::ID], 3.0);
            assert_eq!(row[tlm::LAST_PACKAGE_BYTES] as usize, len + 1);
            assert!(row[tlm::DELIVERED_HZ] > 20.0, "{row:?}");
            let ch = core::slice::from_raw_parts(sim_channel_ptr(sim), sim_channel_len(sim) as usize);
            assert_eq!((ch[chan::CHANNEL_ON], ch[chan::TELEMETRY_ON]), (1.0, 1.0));
            assert!((ch[chan::EXTERNAL_BUSY] - 0.2).abs() < 1e-6);
            sim_free(sim);
        }
    }

    /// Variants, routing, rate modes and switching telemetry off again, all
    /// through the C ABI.
    #[test]
    fn telemetry_modes_through_the_abi() {
        use swarm_sim::telemetry::{TLM_STRIDE, chan, tlm};
        let sample = |sim: *const Sim, id: u32| unsafe {
            let len = sim_tlm_sample_len(sim, id) as usize;
            core::str::from_utf8(core::slice::from_raw_parts(sim_tlm_sample_ptr(sim, id), len)).unwrap().to_owned()
        };
        let stats = |sim: *const Sim| unsafe {
            core::slice::from_raw_parts(sim_telemetry_ptr(sim), sim_telemetry_len(sim) as usize).to_vec()
        };
        let summary = |sim: *const Sim| unsafe {
            core::slice::from_raw_parts(sim_channel_ptr(sim), sim_channel_len(sim) as usize).to_vec()
        };
        let sim = sim_new(2, 6);
        unsafe {
            // Mixed fleet, broadcast, fixed 10 Hz: odd ids carry GNSS keys.
            assert_eq!(sim_set_telemetry(sim, 1, 2, 0, 10.0, 1), 1);
            for _ in 0..60 {
                sim_step(sim, 50);
            }
            let (odd, even) = (sample(sim, 3), sample(sim, 4));
            assert!(odd.starts_with(r#"{"type":8,"dest":0,"#), "{odd}");
            assert!(odd.contains(r#"\"gq\""#) && !even.contains(r#"\"gq\""#), "{odd}\n{even}");
            let st = stats(sim);
            assert_eq!((st[3 * TLM_STRIDE + tlm::VARIANT], st[4 * TLM_STRIDE + tlm::VARIANT]), (1.0, 0.0));
            assert_eq!(st[3 * TLM_STRIDE + tlm::REQUESTED_HZ], 10.0);
            assert_eq!(st[tlm::TREE_HOPS], 0.0, "the base is the root");
            assert_eq!(sample(sim, 0), "", "the bridge sends no drone telemetry");
            // Backend budget, single to root: 5 senders share 60 pps.
            assert_eq!(sim_set_telemetry(sim, 1, 0, 1, 25.0, 0), 1);
            for _ in 0..200 {
                sim_step(sim, 50);
            }
            assert!(sample(sim, 3).starts_with(r#"{"type":9,"dest":2200000000,"#));
            let ch = summary(sim);
            assert_eq!((ch[chan::MODE], ch[chan::ONLINE]), (1.0, 5.0));
            assert_eq!(stats(sim)[TLM_STRIDE + tlm::REQUESTED_HZ], 12.0);
            // Brain mode: the brain's own interval.
            assert_eq!(sim_set_telemetry(sim, 1, 0, 2, 20.0, 0), 1);
            sim_step(sim, 50);
            assert_eq!(summary(sim)[chan::MODE], 2.0);
            assert!(stats(sim)[TLM_STRIDE + tlm::REQUESTED_HZ] > 0.0);
            // Off again: stats zero-filled, the summary says so.
            assert_eq!(sim_set_telemetry(sim, 0, 0, 0, 25.0, 0), 1);
            sim_step(sim, 50);
            let st = stats(sim);
            assert!(st.iter().enumerate().all(|(i, &v)| i % TLM_STRIDE == tlm::ID || v == 0.0 || v == -1.0), "{st:?}");
            assert!(summary(sim).iter().all(|&v| v == 0.0));
            // The slider is clamped like the sim's setter.
            sim_set_channel(sim, 1, 5.0);
            sim_step(sim, 50);
            assert_eq!(summary(sim)[chan::EXTERNAL_BUSY], swarm_sim::channel::MAX_EXTERNAL_BUSY);
            sim_set_channel(sim, 1, f32::NAN);
            sim_step(sim, 50);
            assert_eq!(summary(sim)[chan::EXTERNAL_BUSY], 0.0);
            sim_free(sim);
        }
    }

    #[test]
    fn command_and_kill_report_success() {
        let sim = sim_new(1, 20);
        unsafe {
            assert_eq!(sim_command(sim, Intent::MonitorFire as u32, 330.0, 40.0), 1);
            assert_eq!(sim_command(sim, 1, f32::NAN, 0.0), 0);
            assert_eq!(sim_kill(sim, 5), 1);
            assert_eq!(sim_kill(sim, 5), 0);
            assert_eq!(sim_kill(sim, 20), 0);
            sim_free(sim);
        }
    }
}
