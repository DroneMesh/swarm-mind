use swarm_brain::Vec3;

use crate::Preset;

/// Heightmap on a square grid centred on the origin. Row `r`, column `c`
/// sits at `x = -extent/2 + c * cell`, `z = -extent/2 + r * cell`.
pub struct Terrain {
    size: usize,
    cell: f32,
    heights: Vec<f32>,
    /// Highest sample: a line with both ends above it can't be blocked,
    /// which lets most high-altitude radio checks skip the ray march.
    max_height: f32,
    /// Sea surface height, or `-inf` on a map without a sea. Ground below
    /// it is seabed: drones fly over the surface and nothing there burns.
    water_level: f32,
}

impl Terrain {
    /// The heightmap for `preset`. Every preset is its own generator;
    /// [`Preset::AlpineRidge`] is [`Terrain::generate`], bit for bit.
    pub fn for_preset(preset: Preset, seed: u64, size: usize, extent: f32) -> Self {
        let terrain = match preset {
            Preset::AlpineRidge => Self::generate(seed, size, extent),
            Preset::Canyon => Self::sample(seed, size, extent, canyon),
            Preset::TwinValleys => Self::sample(seed, size, extent, twin_valleys),
            Preset::Archipelago => Self::sample(seed, size, extent, archipelago),
            Preset::Foothills => Self::sample(seed, size, extent, foothills),
        };
        match preset.water_level() {
            Some(level) => terrain.with_water_level(level),
            None => terrain,
        }
    }

    /// Samples `f(noise, u, v)` on the grid, with `u`, `v` the normalised
    /// coordinates in [-0.5, 0.5] (`u` along x, `v` along z).
    fn sample(seed: u64, size: usize, extent: f32, f: fn(&ValueNoise, f32, f32) -> f32) -> Self {
        assert!(size >= 2, "terrain needs at least 2x2 samples");
        let step = 1.0 / (size - 1) as f32;
        let noise = ValueNoise::new(seed);
        let mut heights = Vec::with_capacity(size * size);
        for r in 0..size {
            for c in 0..size {
                heights.push(f(&noise, c as f32 * step - 0.5, r as f32 * step - 0.5));
            }
        }
        Self::from_heights(size, extent, heights)
    }

    /// Declares ground at or below `level` to be under water.
    pub fn with_water_level(mut self, level: f32) -> Self {
        self.water_level = level;
        self
    }

    /// Sea surface height, if this map has a sea.
    pub fn water_level(&self) -> Option<f32> {
        self.water_level.is_finite().then_some(self.water_level)
    }

    /// True if `(x, z)` is open water.
    pub fn is_water(&self, x: f32, z: f32) -> bool {
        self.height_at(x, z) <= self.water_level
    }

    /// What a drone stands on or flies over at `(x, z)`: the ground, or the
    /// sea surface above the seabed. Without a sea this is [`Self::height_at`]
    /// exactly (`max` with `-inf`), so dry maps are unaffected.
    pub fn surface_at(&self, x: f32, z: f32) -> f32 {
        self.height_at(x, z).max(self.water_level)
    }

    /// Rolling foothills with a mountain ridge running north to south
    /// through the middle. The ridge is what makes relays necessary: drones
    /// low on either side can't see each other through it.
    pub fn generate(seed: u64, size: usize, extent: f32) -> Self {
        assert!(size >= 2, "terrain needs at least 2x2 samples");
        let cell = extent / (size - 1) as f32;
        let half = extent * 0.5;
        let noise = ValueNoise::new(seed);
        let mut heights = Vec::with_capacity(size * size);
        for r in 0..size {
            for c in 0..size {
                // Normalised coordinates in [-0.5, 0.5].
                let u = (c as f32 * cell - half) / extent;
                let v = (r as f32 * cell - half) / extent;
                let hills = (noise.fbm(u * 5.0, v * 5.0, 5) * 0.5 + 0.5) * 45.0;
                let ridge_x = 0.06 * (v * 7.0).sin();
                let d = (u - ridge_x) / 0.11;
                let peaks = 150.0 + noise.fbm(u * 9.0 + 17.0, v * 9.0, 4) * 90.0;
                heights.push((hills + (-d * d).exp() * peaks).max(0.0));
            }
        }
        Self::from_heights(size, extent, heights)
    }

    /// A terrain from explicit row-major samples, e.g. a plane for tests.
    pub fn from_heights(size: usize, extent: f32, heights: Vec<f32>) -> Self {
        assert!(size >= 2, "terrain needs at least 2x2 samples");
        assert_eq!(heights.len(), size * size, "need size * size heights");
        let cell = extent / (size - 1) as f32;
        let max_height = heights.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        Self { size, cell, heights, max_height, water_level: f32::NEG_INFINITY }
    }

    /// Samples per side.
    pub fn size(&self) -> usize {
        self.size
    }

    pub fn extent(&self) -> f32 {
        self.cell * (self.size - 1) as f32
    }

    /// Row-major heights, `size * size` long.
    pub fn heights(&self) -> &[f32] {
        &self.heights
    }

    /// Bilinear height at a world position, clamped to the map edge.
    pub fn height_at(&self, x: f32, z: f32) -> f32 {
        let half = self.extent() * 0.5;
        let max = (self.size - 1) as f32;
        let gx = ((x + half) / self.cell).clamp(0.0, max);
        let gz = ((z + half) / self.cell).clamp(0.0, max);
        let (c0, r0) = (gx as usize, gz as usize);
        let (c1, r1) = ((c0 + 1).min(self.size - 1), (r0 + 1).min(self.size - 1));
        let (tx, tz) = (gx - c0 as f32, gz - r0 as f32);
        let h = |r: usize, c: usize| self.heights[r * self.size + c];
        lerp(lerp(h(r0, c0), h(r0, c1), tx), lerp(h(r1, c0), h(r1, c1), tx), tz)
    }

    /// True if the straight line from `a` to `b` stays above the ground.
    /// Samples every half cell, so ridges thinner than that can slip
    /// through; at this map's scale none are.
    pub fn line_of_sight(&self, a: Vec3, b: Vec3) -> bool {
        if a.y > self.max_height && b.y > self.max_height {
            return true;
        }
        let d = b - a;
        if !(d.x.is_finite() && d.y.is_finite() && d.z.is_finite()) {
            return false;
        }
        let horizontal = (d.x * d.x + d.z * d.z).sqrt();
        // Capped at a map diagonal's worth of samples: beyond the map edge
        // the clamped heights don't change anyway.
        let max_steps = (self.size * 3) as f32;
        let steps = (horizontal / (self.cell * 0.5)).ceil().clamp(1.0, max_steps) as usize;
        (1..steps).all(|i| {
            let p = a + d * (i as f32 / steps as f32);
            self.height_at(p.x, p.z) < p.y
        })
    }
}

/// A high desert plateau (~150) cut by a deep (~135) winding gorge running
/// west to east. The base sits on the gorge floor, so walls hide it from
/// anything not directly above the canyon: relays must climb over the rim.
fn canyon(noise: &ValueNoise, u: f32, v: f32) -> f32 {
    let (centre, slope) = canyon_centre(u);
    // Distance across the gorge, measured perpendicular to its course so
    // the bends are no narrower than the straights.
    let d = (v - centre).abs() / (1.0 + slope * slope).sqrt();
    let wobble = noise.fbm(u * 14.0 + 3.0, v * 14.0, 3) * 0.008;
    let wall = smoothstep(CANYON_FLOOR + wobble, CANYON_RIM + wobble, d);
    // Sandstone benches: four layers with steep risers between them.
    let t = wall * 4.0;
    let terraced = (t.floor() + smooth(smooth(t.fract()))) / 4.0;
    let floor = 12.0 + noise.fbm(u * 8.0, v * 8.0 + 9.0, 3) * 4.0;
    let mesas = smoothstep(0.22, 0.34, noise.fbm(u * 3.2 + 40.0, v * 3.2 - 11.0, 3)) * 32.0;
    let plateau = 148.0 + noise.fbm(u * 4.0, v * 4.0, 4) * 12.0 + mesas;
    floor + (plateau - floor) * terraced
}

/// Gorge floor and rim half-widths, normalised (x1200: ~36 and ~90 units).
const CANYON_FLOOR: f32 = 0.03;
const CANYON_RIM: f32 = 0.075;

/// The gorge's centre line `v(u)` and its slope `dv/du`: two big bends
/// plus a smaller meander.
pub(crate) fn canyon_centre(u: f32) -> (f32, f32) {
    let (a, b) = (u * 7.5 + 0.6, u * 19.0 + 1.3);
    (0.17 * a.sin() + 0.04 * b.sin(), 0.17 * 7.5 * a.cos() + 0.04 * 19.0 * b.cos())
}

/// Three valleys split by two parallel north-south ridges. The base sits in
/// the west valley and the fire in the east one, so the relay chain has to
/// cross both crests: a longer ladder with two blind dips.
fn twin_valleys(noise: &ValueNoise, u: f32, v: f32) -> f32 {
    let hills = (noise.fbm(u * 5.0, v * 5.0, 5) * 0.5 + 0.5) * 30.0;
    let ridge = |centre: f32, phase: f32, k: f32| {
        let d = (u - centre - 0.03 * (v * 6.0 + phase).sin()) / 0.07;
        let peaks = 170.0 + noise.fbm(u * 9.0 + k, v * 9.0, 4) * 70.0;
        (-d * d).exp() * peaks
    };
    // The far east rises gently, giving the fire a slope to climb.
    let rim = smoothstep(0.38, 0.5, u) * 60.0;
    (hills + ridge(-0.14, 0.0, 17.0) + ridge(0.17, 2.0, 41.0) + rim).max(0.0)
}

/// A mainland coast in the west, a big wooded island with the fire in the
/// east, and a string of islets between. Open sea never blocks the radio,
/// but no water gap is short: hops are limited by range, not by terrain.
fn archipelago(noise: &ValueNoise, u: f32, v: f32) -> f32 {
    // `(u, v, radius, peak)`: stepping stones first, then the fire island
    // and decorative islets further off the route.
    const ISLANDS: [(f32, f32, f32, f32); 7] = [
        (-0.20, -0.04, 0.075, 55.0),
        (-0.03, 0.06, 0.085, 70.0),
        (0.29, 0.02, 0.19, 120.0),
        (0.04, -0.30, 0.07, 50.0),
        (-0.16, 0.31, 0.06, 40.0),
        (0.31, -0.37, 0.065, 45.0),
        (0.37, 0.39, 0.07, 60.0),
    ];
    let coast = -0.34 + 0.035 * (v * 11.0 + 1.0).sin() + 0.02 * noise.fbm(u * 6.0, v * 6.0, 3);
    let mainland = smoothstep(coast + 0.06, coast - 0.12, u);
    let mut land = mainland * 0.85;
    let mut peak = 80.0;
    for (cu, cv, r, p) in ISLANDS {
        let d2 = ((u - cu) * (u - cu) + (v - cv) * (v - cv)) / (r * r);
        let bump = (1.0 - d2).max(0.0);
        if bump > land {
            land = bump;
            peak = p;
        }
    }
    let rough = noise.fbm(u * 10.0, v * 10.0, 4);
    // Below `SHORE` the raw land value is seabed, above it ground; the
    // square root steepens the beach so islands read as islands.
    const SHORE: f32 = 0.25;
    if land < SHORE {
        -26.0 + land / SHORE * 24.0 + rough * 3.0
    } else {
        let t = ((land - SHORE) / (1.0 - SHORE)).sqrt();
        2.0 + t * peak * (0.8 + 0.35 * rough.max(-0.5))
    }
}

/// Low rolling hills with scattered knolls and no big obstacle: the easy
/// baseline. Each knoll blocks a little low flight, so the mesh keeps
/// re-routing, but a modest climb sees over all of them.
fn foothills(noise: &ValueNoise, u: f32, v: f32) -> f32 {
    let rolling = (noise.fbm(u * 3.5, v * 3.5, 5) * 0.5 + 0.5) * 70.0;
    let knolls = smoothstep(0.1, 0.45, noise.fbm(u * 7.0 + 23.0, v * 7.0 - 5.0, 3)) * 45.0;
    (rolling + knolls).max(0.0)
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    smooth(((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0))
}

struct ValueNoise {
    seed: u32,
}

impl ValueNoise {
    fn new(seed: u64) -> Self {
        Self { seed: (seed ^ (seed >> 32)) as u32 }
    }

    /// Pseudo-random value in [-1, 1] at an integer lattice point.
    fn lattice(&self, x: i32, y: i32) -> f32 {
        let mut h = (x as u32).wrapping_mul(0x27d4_eb2d)
            ^ (y as u32).wrapping_mul(0x1656_67b1)
            ^ self.seed;
        h ^= h >> 15;
        h = h.wrapping_mul(0x85eb_ca6b);
        h ^= h >> 13;
        h = h.wrapping_mul(0xc2b2_ae35);
        h ^= h >> 16;
        h as f32 / u32::MAX as f32 * 2.0 - 1.0
    }

    fn sample(&self, x: f32, y: f32) -> f32 {
        let (x0, y0) = (x.floor(), y.floor());
        let (ix, iy) = (x0 as i32, y0 as i32);
        let (tx, ty) = (smooth(x - x0), smooth(y - y0));
        let top = lerp(self.lattice(ix, iy), self.lattice(ix + 1, iy), tx);
        let bottom = lerp(self.lattice(ix, iy + 1), self.lattice(ix + 1, iy + 1), tx);
        lerp(top, bottom, ty)
    }

    /// Sum of octaves, roughly in [-1, 1].
    fn fbm(&self, x: f32, y: f32, octaves: u32) -> f32 {
        let (mut sum, mut amp, mut freq, mut norm) = (0.0, 1.0, 1.0, 0.0);
        for _ in 0..octaves {
            sum += self.sample(x * freq, y * freq) * amp;
            norm += amp;
            amp *= 0.5;
            freq *= 2.0;
        }
        sum / norm
    }
}

fn smooth(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;

    fn terrain() -> Terrain {
        Terrain::generate(1, 129, 1200.0)
    }

    #[test]
    fn same_seed_same_terrain() {
        assert_eq!(terrain().heights, terrain().heights);
    }

    #[test]
    fn height_at_matches_grid_samples() {
        let t = terrain();
        let (r, c) = (20, 10);
        let x = -600.0 + c as f32 * t.cell;
        let z = -600.0 + r as f32 * t.cell;
        assert!((t.height_at(x, z) - t.heights[r * t.size + c]).abs() < 1e-3);
    }

    #[test]
    fn ridge_blocks_low_flight_but_not_high() {
        let t = terrain();
        let low = |x: f32| Vec3::new(x, t.height_at(x, 0.0) + 20.0, 0.0);
        assert!(!t.line_of_sight(low(-400.0), low(400.0)));
        let high = |x: f32| Vec3::new(x, 600.0, 0.0);
        assert!(t.line_of_sight(high(-400.0), high(400.0)));
    }
}
