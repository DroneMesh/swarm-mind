//! Per-map checks: every preset is deterministic, poses the radio problem
//! it advertises, and is fair, meaning a relay chain the swarm can actually
//! fly (at most `MAX_AGL` up, hops at most `HOP` long, line of sight on every
//! hop) links the base to the fire with spares left over.

use swarm_brain::Vec3;

use crate::fire::{BURNING, FireGrid, UNBURNT};
use crate::rng::Rng;
use crate::{Preset, SimConfig, Terrain, base_position};

/// Highest relay altitude the brain will climb to (its `CEILING_AGL`).
const MAX_AGL: f32 = 230.0;
/// Longest hop the chain may use: the radio reaches 260, minus margin.
const HOP: f32 = 200.0;
/// Scouts work this high over the fire.
const SCOUT_AGL: f32 = 40.0;
/// Drones held back from the chain: the base, four scouts and a spare.
const RESERVED: usize = 6;
const STEP_MS: u32 = 50;

fn setup(preset: Preset) -> (SimConfig, Terrain) {
    let cfg = SimConfig::for_preset(preset, 1, 20);
    let terrain = Terrain::for_preset(preset, cfg.seed, cfg.terrain_size, cfg.extent);
    (cfg, terrain)
}

fn at_agl(t: &Terrain, x: f32, z: f32, agl: f32) -> Vec3 {
    Vec3::new(x, t.surface_at(x, z) + agl, z)
}

fn horizontal(a: Vec3, b: Vec3) -> f32 {
    ((a.x - b.x).powi(2) + (a.z - b.z).powi(2)).sqrt()
}

/// Greedily strings relays from the base to a scout over the fire: from
/// the current hop, take the reachable point (any bearing, 25..=200 out,
/// 50..=230 AGL) that gets closest to the fire, preferring lower altitude
/// on near-ties. Returns the relays, or `None` if it gets stuck. Greedy is
/// pessimistic, so success proves the map solvable with this many relays.
fn greedy_chain(t: &Terrain, base: Vec3, fire: (f32, f32)) -> Option<Vec<Vec3>> {
    let goal = at_agl(t, fire.0, fire.1, SCOUT_AGL);
    let half = t.extent() * 0.5;
    let mut cur = base;
    let mut relays = Vec::new();
    while relays.len() < 40 {
        if (goal - cur).length_sq() <= HOP * HOP && t.line_of_sight(cur, goal) {
            return Some(relays);
        }
        let mut best: Option<(f32, Vec3)> = None;
        for ring in 1..=8 {
            let r = ring as f32 * 25.0;
            for k in 0..48 {
                let a = k as f32 * core::f32::consts::TAU / 48.0;
                let (x, z) = (cur.x + r * a.cos(), cur.z + r * a.sin());
                if x.abs() > half || z.abs() > half {
                    continue;
                }
                for agl in [50.0, 100.0, 150.0, 200.0, MAX_AGL] {
                    let p = at_agl(t, x, z, agl);
                    if (p - cur).length_sq() > HOP * HOP || !t.line_of_sight(cur, p) {
                        continue;
                    }
                    let score = horizontal(p, goal) + agl * 0.05;
                    if best.is_none_or(|(s, _)| score < s) {
                        best = Some((score, p));
                    }
                }
            }
        }
        let (_, next) = best?;
        if horizontal(next, goal) >= horizontal(cur, goal) - 1.0 {
            return None;
        }
        relays.push(next);
        cur = next;
    }
    None
}

/// Relays the brain's straight ladder would plan for this map (two rails
/// per row, rows spaced at most 165 apart), for the report.
fn ladder_relays(cfg: &SimConfig, t: &Terrain) -> usize {
    let base = base_position(cfg, t);
    let len = ((cfg.ignition.0 - base.x).powi(2) + (cfg.ignition.1 - base.z).powi(2)).sqrt();
    let rows = ((len / 165.0).ceil() as usize).saturating_sub(1).min(12);
    rows * 2
}

/// Burning or burnt cells, and the span of the box around them, after
/// `ms` of fire from the preset's ignition point and wind.
fn burn(cfg: &SimConfig, t: &Terrain, ms: u32) -> (FireGrid, usize, f32) {
    let mut fire = FireGrid::new(cfg.fire_size, cfg.extent);
    fire.set_wind(cfg.wind.0, cfg.wind.1);
    fire.ignite(cfg.ignition.0, cfg.ignition.1);
    let mut rng = Rng::new(cfg.seed);
    for _ in 0..ms / STEP_MS {
        fire.step(STEP_MS, t, &mut rng);
    }
    let n = fire.size();
    let cell = cfg.extent / n as f32;
    let (mut c0, mut c1, mut r0, mut r1, mut caught) = (n, 0, n, 0, 0);
    for (i, &s) in fire.cells().iter().enumerate() {
        if s != UNBURNT {
            caught += 1;
            let (r, c) = (i / n, i % n);
            (c0, c1, r0, r1) = (c0.min(c), c1.max(c), r0.min(r), r1.max(r));
        }
    }
    let span = if caught == 0 { 0.0 } else { (c1 - c0 + 1).max(r1 - r0 + 1) as f32 * cell };
    (fire, caught, span)
}

#[test]
fn every_preset_is_deterministic_and_seed_dependent() {
    for p in Preset::ALL {
        let (cfg, t) = setup(p);
        let again = Terrain::for_preset(p, cfg.seed, cfg.terrain_size, cfg.extent);
        assert_eq!(t.heights(), again.heights(), "{p:?}");
        let other = Terrain::for_preset(p, 2, cfg.terrain_size, cfg.extent);
        assert_ne!(t.heights(), other.heights(), "{p:?} ignores its seed");
        assert!(t.heights().iter().all(|h| h.is_finite()), "{p:?}");
    }
}

/// Golden checksums of the original map's terrain and fire, taken before
/// presets existed. Any change to Alpine Ridge's bits fails here.
#[test]
fn alpine_ridge_is_bit_identical() {
    fn fnv(bits: impl Iterator<Item = u32>) -> u64 {
        bits.fold(0xcbf2_9ce4_8422_2325, |h, b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
    }
    let (cfg, t) = setup(Preset::AlpineRidge);
    assert_eq!(t.heights(), Terrain::generate(1, 129, 1200.0).heights());
    assert_eq!(t.water_level(), None);
    assert_eq!(fnv(t.heights().iter().map(|h| h.to_bits())), 0x43c3_d78b_aa78_20ca);
    // The fire alone (the full sim trace also depends on the brain, which
    // is tuned separately).
    let (fire, _, _) = burn(&cfg, &t, 120_000);
    assert_eq!(fnv(fire.cells().iter().map(|&c| u32::from(c))), 0x0e46_7e3f_61f0_85f9, "golden fire");
}

#[test]
fn every_preset_is_solvable_with_spares() {
    for p in Preset::ALL {
        let (cfg, t) = setup(p);
        let base = base_position(&cfg, &t);
        let chain = greedy_chain(&t, base, cfg.ignition)
            .unwrap_or_else(|| panic!("{p:?}: no relay chain from {base:?} to {:?}", cfg.ignition));
        println!(
            "{:<12} base ({:.0}, {:.1}, {:.0})  fire ({:.0}, {:.0})  wind {:?}  water {:?}  \
             greedy relays {}  ladder relays {}",
            p.name(),
            base.x,
            base.y,
            base.z,
            cfg.ignition.0,
            cfg.ignition.1,
            cfg.wind,
            t.water_level(),
            chain.len(),
            ladder_relays(&cfg, &t),
        );
        assert!(chain.len() <= cfg.drones - RESERVED, "{p:?} needs {} relays", chain.len());
        assert!(
            ladder_relays(&cfg, &t) + 4 + 1 + 4 <= cfg.drones,
            "{p:?}: the brain's ladder plus scouts leaves fewer than 4 spares"
        );
    }
}

#[test]
fn bases_and_fires_sit_on_dry_burnable_ground() {
    for p in Preset::ALL {
        let (cfg, t) = setup(p);
        let base = base_position(&cfg, &t);
        assert!(!t.is_water(base.x, base.z), "{p:?} base in the sea");
        let (fx, fz) = cfg.ignition;
        assert!(!t.is_water(fx, fz), "{p:?} fire in the sea");
        assert!(t.height_at(fx, fz) < crate::fire::SNOW_LINE, "{p:?} fire on snow");
        // The whole spawn ring is dry land too.
        for r in [20.0, 35.0] {
            for k in 0..12 {
                let a = k as f32 * core::f32::consts::TAU / 12.0;
                let (x, z) = (base.x + r * a.cos(), base.z + r * a.sin());
                assert!(!t.is_water(x, z), "{p:?} spawn ring in the sea at ({x}, {z})");
            }
        }
    }
}

/// About three minutes of fire grows a visible front without leaving its
/// stage; none of it is on water or snow.
#[test]
fn every_preset_fire_spreads_on_land_only() {
    for p in Preset::ALL {
        let (cfg, t) = setup(p);
        let (fire, caught, span) = burn(&cfg, &t, 180_000);
        println!("{:<12} 3 min: {caught} cells, span {span:.0}, {} burning", p.name(), fire.burning_count());
        assert!(fire.burning_count() > 0, "{p:?}: the fire died");
        assert!((150.0..650.0).contains(&span), "{p:?}: front spans {span}");
        let (n, cell) = (fire.size(), cfg.extent / fire.size() as f32);
        for (i, &s) in fire.cells().iter().enumerate() {
            if s == UNBURNT || (s == BURNING && caught == 1) {
                continue;
            }
            let x = -cfg.extent * 0.5 + ((i % n) as f32 + 0.5) * cell;
            let z = -cfg.extent * 0.5 + ((i / n) as f32 + 0.5) * cell;
            let ignition = (x - cfg.ignition.0).abs() < cell && (z - cfg.ignition.1).abs() < cell;
            if !ignition {
                assert!(!t.is_water(x, z), "{p:?}: water burnt at ({x}, {z})");
                assert!(t.height_at(x, z) <= crate::fire::SNOW_LINE, "{p:?}: snow burnt");
            }
        }
    }
}

/// Canyon: from the gorge floor the walls hide the rim and the next bend;
/// above the rim they don't.
#[test]
fn canyon_walls_block_the_floor() {
    let (cfg, t) = setup(Preset::Canyon);
    let base = base_position(&cfg, &t);
    let (fx, fz) = cfg.ignition;
    assert!(t.height_at(fx, fz) - base.y > 100.0, "fire should be high above the floor");
    // Floor to rim: low flight inside the gorge can't see the fire.
    let low = at_agl(&t, base.x, base.z, 20.0);
    assert!(!t.line_of_sight(low, at_agl(&t, fx, fz, SCOUT_AGL)));
    // Floor to plateau: once a little back from the rim, nothing up on top
    // within radio range sees a low drone at the base. (Right at the lip a
    // drone can of course look down into the gorge.)
    let (mut plateau, mut seen) = (0, 0);
    for ring in 4..=10 {
        for k in 0..36 {
            let a = k as f32 * core::f32::consts::TAU / 36.0;
            let r = ring as f32 * 25.0;
            let (x, z) = (base.x + r * a.cos(), base.z + r * a.sin());
            let (centre, slope) = crate::terrain::canyon_centre(x / cfg.extent);
            let across = (z / cfg.extent - centre).abs() / (1.0 + slope * slope).sqrt();
            if across * cfg.extent > 140.0 && t.height_at(x, z) > 120.0 {
                plateau += 1;
                seen += usize::from(t.line_of_sight(low, at_agl(&t, x, z, 30.0)));
            }
        }
    }
    assert!(plateau > 60, "only {plateau} plateau samples near the base");
    assert!(seen * 20 <= plateau, "{seen} of {plateau} plateau samples see the floor");
    // Floor to floor: round the bends, walls hide drones well within range.
    let floor = |u: f32| {
        let x = u * cfg.extent;
        let z = crate::terrain::canyon_centre(u).0 * cfg.extent;
        at_agl(&t, x, z, 20.0)
    };
    let mut bends = 0;
    for i in 0..90 {
        let a = floor(-0.45 + i as f32 * 0.01);
        let b = (1..40).map(|j| floor(-0.45 + (i + j) as f32 * 0.01)).find(|b| horizontal(a, *b) > 200.0);
        if let Some(b) = b.filter(|b| horizontal(a, *b) < 260.0)
            && !t.line_of_sight(a, b)
        {
            bends += 1;
        }
    }
    assert!(bends >= 10, "only {bends} floor pairs hidden by a bend");
    // Both rims see each other over the gorge.
    let (ca, cb) = (floor(0.0), floor(0.0));
    let north = at_agl(&t, ca.x, ca.z + 150.0, 30.0);
    let south = at_agl(&t, cb.x, cb.z - 150.0, 30.0);
    assert!(t.line_of_sight(north, south), "rim to rim should be clear");
    assert!(t.height_at(ca.x, ca.z + 150.0) - ca.y > 80.0, "the gorge should be deep");
}

/// Twin Valleys: each ridge blocks low flight on its own, so the chain
/// needs a relay over both crests.
#[test]
fn twin_ridges_both_block_low_flight() {
    let (cfg, t) = setup(Preset::TwinValleys);
    let base = base_position(&cfg, &t);
    let low = |x: f32, z: f32| at_agl(&t, x, z, 30.0);
    let west = low(base.x, base.z);
    let middle = low(0.02 * cfg.extent, 0.0);
    let east = low(cfg.ignition.0, cfg.ignition.1);
    assert!(!t.line_of_sight(west, middle), "the west ridge should block");
    assert!(!t.line_of_sight(middle, east), "the east ridge should block");
    // Lowest point of the crest line near `x` (the ridges wobble).
    let crest = |x: f32| {
        let top = |z: f32| (-8..=8).map(|k| t.height_at(x + k as f32 * 10.0, z)).fold(f32::MIN, f32::max);
        (-300..=300).step_by(25).map(|z| top(z as f32)).fold(f32::MAX, f32::min)
    };
    assert!(crest(-0.14 * cfg.extent) > 120.0 && crest(0.17 * cfg.extent) > 120.0, "no low pass");
    assert!(middle.y < 90.0, "the middle valley should be low");
}

/// Archipelago: open sea separates the base's mainland from the fire's
/// island, every water gap on the way is shorter than a hop, and over the
/// water nothing blocks the radio.
#[test]
fn archipelago_has_sea_gaps_shorter_than_a_hop() {
    let (cfg, t) = setup(Preset::Archipelago);
    let level = t.water_level().expect("a sea");
    let base = base_position(&cfg, &t);
    let (fx, fz) = cfg.ignition;
    let (mut run, mut longest, mut water) = (0.0f32, 0.0f32, 0.0f32);
    let step = 5.0;
    let len = horizontal(base, Vec3::new(fx, 0.0, fz));
    for i in 0..=(len / step) as usize {
        let k = i as f32 * step / len;
        let (x, z) = (base.x + (fx - base.x) * k, base.z + (fz - base.z) * k);
        if t.height_at(x, z) <= level {
            run += step;
            water += step;
            longest = longest.max(run);
        } else {
            run = 0.0;
        }
    }
    assert!(water > 250.0, "only {water} units of sea between base and fire");
    assert!((60.0..HOP).contains(&longest), "longest water gap {longest}");
    // A flat sea: two drones low over the water see each other at range.
    let a = at_agl(&t, -0.3 * cfg.extent + 120.0, -0.42 * cfg.extent, 20.0);
    let b = at_agl(&t, a.x + 240.0, a.z, 20.0);
    assert!(t.is_water(a.x, a.z) && t.is_water(b.x, b.z));
    assert!(t.line_of_sight(a, b));
    // Drones fly over the surface, not the seabed.
    assert!(t.surface_at(b.x, b.z) == level && t.height_at(b.x, b.z) < level);
}

/// Foothills: low hills hide a low drone a long way off, but a modest
/// climb clears every one of them and nothing is high enough for snow.
#[test]
fn foothills_are_gentle() {
    let (cfg, t) = setup(Preset::Foothills);
    let max = t.heights().iter().copied().fold(f32::MIN, f32::max);
    assert!(max < 130.0 && max > 50.0, "highest hill {max}");
    let base = base_position(&cfg, &t);
    let (fx, fz) = cfg.ignition;
    let len = horizontal(base, Vec3::new(fx, 0.0, fz));
    assert!(len < 650.0, "fire should be close: {len}");
    let blocked = (0..8).any(|k| {
        let a = k as f32 * core::f32::consts::TAU / 8.0;
        let far = at_agl(&t, base.x + 250.0 * a.cos(), base.z + 250.0 * a.sin(), 10.0);
        !t.line_of_sight(at_agl(&t, base.x, base.z, 10.0), far)
    });
    assert!(blocked, "the hills should block some low flight");
}

/// Prints each map as ASCII (`~` water, `.`-`#` height bands, `^` snow,
/// `B` base, `F` fire) for tuning: `cargo test -p swarm-sim print_maps --
/// --ignored --nocapture`.
#[test]
#[ignore = "prints maps for tuning"]
fn print_maps() {
    for p in Preset::ALL {
        let (cfg, t) = setup(p);
        let base = base_position(&cfg, &t);
        let half = cfg.extent * 0.5;
        let max = t.heights().iter().copied().fold(f32::MIN, f32::max);
        let min = t.heights().iter().copied().fold(f32::MAX, f32::min);
        println!("{} (min {min:.0}, max {max:.0})", p.name());
        for r in 0..40 {
            let line: String = (0..80)
                .map(|c| {
                    let x = -half + (c as f32 + 0.5) * cfg.extent / 80.0;
                    let z = -half + (r as f32 + 0.5) * cfg.extent / 40.0;
                    let near = |px: f32, pz: f32| (x - px).abs() < 8.0 && (z - pz).abs() < 15.0;
                    if near(base.x, base.z) {
                        return 'B';
                    }
                    if near(cfg.ignition.0, cfg.ignition.1) {
                        return 'F';
                    }
                    let h = t.height_at(x, z);
                    if t.is_water(x, z) {
                        '~'
                    } else if h > crate::fire::SNOW_LINE {
                        '^'
                    } else {
                        b" .:-=+*#%@"[((h / 200.0) * 9.0).clamp(0.0, 9.0) as usize] as char
                    }
                })
                .collect();
            println!("{line}");
        }
    }
}
