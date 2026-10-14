//! Wildfire on a coarse grid laid over the terrain.
//!
//! Same orientation as [`crate::Terrain`]: row `r` runs along +z, column `c`
//! along +x, and cell `(r, c)` is centred at
//! `(-extent/2 + (c + 0.5) * cell, -extent/2 + (r + 0.5) * cell)`.

use swarm_brain::{FireReading, Vec3};

use crate::Terrain;
use crate::rng::Rng;

pub const UNBURNT: u8 = 0;
pub const BURNING: u8 = 1;
pub const BURNT: u8 = 2;

/// Ground above this height is bare rock and snow: nothing to burn.
pub const SNOW_LINE: f32 = 200.0;

/// Ignition rate (per second) from a burning cell into a level, windless,
/// edge-adjacent neighbour with average fuel. Tuned with the burn time so
/// the demo front reaches a few hundred units across in about 3 minutes.
const BASE_RATE: f32 = 0.011;
/// Rate multiplier per unit of slope (rise over run), as `exp(k * slope)`:
/// flames lean into the hill above them and preheat it.
const SLOPE_GAIN: f32 = 3.0;
/// Rate multiplier per unit/s of wind along the spread direction, as
/// `exp(k * wind)`.
const WIND_GAIN: f32 = 0.22;
/// Bounds on the combined slope and wind multiplier, so a cliff or a gale
/// can't turn the automaton into a single-step flood (or stop it dead).
const FACTOR_MIN: f32 = 0.05;
const FACTOR_MAX: f32 = 6.0;
/// How long a cell burns before it is spent, in milliseconds.
const BURN_MS: (f32, f32) = (45_000.0, 80_000.0);

/// The 8 neighbours as `(dr, dc, distance in cells)`.
const NEIGHBORS: [(isize, isize, f32); 8] = [
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, core::f32::consts::SQRT_2),
    (-1, 1, core::f32::consts::SQRT_2),
    (1, -1, core::f32::consts::SQRT_2),
    (1, 1, core::f32::consts::SQRT_2),
];

pub struct FireGrid {
    size: usize,
    cell: f32,
    cells: Vec<u8>,
    /// Wind as `(x, z)` in units per second.
    wind: (f32, f32),
    /// Milliseconds of burning left, per cell (only meaningful while burning).
    burn_left: Vec<u32>,
    /// Indices of burning cells, in ignition order, so a step costs
    /// O(front) rather than O(map) and iteration order is deterministic.
    burning: Vec<u32>,
    /// Per-cell terrain height and fuel multiplier (0 = cannot burn),
    /// sampled lazily on the first step because [`FireGrid::new`] has no
    /// terrain.
    height: Vec<f32>,
    fuel: Vec<f32>,
    /// Scratch for cells lit this step; they only start spreading next step,
    /// so the result doesn't depend on scan order.
    lit: Vec<u32>,
}

impl FireGrid {
    pub fn new(size: usize, extent: f32) -> Self {
        assert!(size >= 1, "fire grid needs at least one cell");
        Self {
            size,
            cell: extent / size as f32,
            cells: vec![UNBURNT; size * size],
            wind: (0.0, 0.0),
            burn_left: vec![0; size * size],
            burning: Vec::new(),
            height: Vec::new(),
            fuel: Vec::new(),
            lit: Vec::new(),
        }
    }

    /// Cells per side.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Row-major cell states (`UNBURNT`, `BURNING`, `BURNT`).
    pub fn cells(&self) -> &[u8] {
        &self.cells
    }

    pub fn burning_count(&self) -> usize {
        self.burning.len()
    }

    /// Wind as `(x, z)` in units per second; fire runs faster downwind.
    /// Non-finite components are treated as calm.
    pub fn set_wind(&mut self, x: f32, z: f32) {
        let finite = |v: f32| if v.is_finite() { v } else { 0.0 };
        self.wind = (finite(x), finite(z));
    }

    pub fn wind(&self) -> (f32, f32) {
        self.wind
    }

    /// Sets the cell under `(x, z)` alight, if it hasn't burnt already.
    /// The burn time is fixed at the long end so a lone spark gets a fair
    /// chance to catch; cells lit by spread get a random one.
    pub fn ignite(&mut self, x: f32, z: f32) {
        if let Some(i) = self.index_at(x, z)
            && self.cells[i] == UNBURNT
        {
            self.light(i, BURN_MS.1 as u32);
        }
    }

    /// Advances the fire by `dt_ms`: burning cells may light their
    /// neighbours (faster uphill and downwind), then burn down towards
    /// `BURNT`. Deterministic for a given `rng` state.
    pub fn step(&mut self, dt_ms: u32, terrain: &Terrain, rng: &mut Rng) {
        if self.burning.is_empty() || dt_ms == 0 {
            return;
        }
        if self.fuel.is_empty() {
            self.sample_terrain(terrain, rng);
        }
        let dt = dt_ms as f32 / 1000.0;
        let n = self.size as isize;
        self.lit.clear();
        for &i in &self.burning {
            let i = i as usize;
            let (r, c) = ((i / self.size) as isize, (i % self.size) as isize);
            for &(dr, dc, dist) in &NEIGHBORS {
                let (nr, nc) = (r + dr, c + dc);
                if !(0..n).contains(&nr) || !(0..n).contains(&nc) {
                    continue;
                }
                let j = nr as usize * self.size + nc as usize;
                if self.cells[j] != UNBURNT || self.fuel[j] <= 0.0 {
                    continue;
                }
                let slope = (self.height[j] - self.height[i]) / (dist * self.cell);
                let wind = (self.wind.0 * dc as f32 + self.wind.1 * dr as f32) / dist;
                let factor =
                    (SLOPE_GAIN * slope + WIND_GAIN * wind).exp().clamp(FACTOR_MIN, FACTOR_MAX);
                let rate = BASE_RATE * self.fuel[j] * factor / dist;
                if rng.f32() < 1.0 - (-rate * dt).exp() {
                    // Mark now so a cell next to several fires is lit once.
                    self.cells[j] = BURNING;
                    self.lit.push(j as u32);
                }
            }
        }

        let (cells, burn_left) = (&mut self.cells, &mut self.burn_left);
        self.burning.retain(|&i| {
            let left = &mut burn_left[i as usize];
            *left = left.saturating_sub(dt_ms);
            let alive = *left > 0;
            if !alive {
                cells[i as usize] = BURNT;
            }
            alive
        });
        for k in 0..self.lit.len() {
            let j = self.lit[k] as usize;
            let burn = rng.range(BURN_MS.0, BURN_MS.1) as u32;
            self.light(j, burn);
        }
    }

    fn light(&mut self, i: usize, burn_ms: u32) {
        self.cells[i] = BURNING;
        self.burn_left[i] = burn_ms.max(1);
        self.burning.push(i as u32);
    }

    /// Caches each cell's ground height and fuel. Fuel varies a little per
    /// cell so the front comes out ragged rather than a perfect diamond;
    /// anything above the snow line or under water has none. The random
    /// draw happens for every cell regardless, so the stream (and with it
    /// the original map's fire) doesn't depend on where the water is.
    fn sample_terrain(&mut self, terrain: &Terrain, rng: &mut Rng) {
        let half = self.cell * self.size as f32 * 0.5;
        let count = self.size * self.size;
        self.height = Vec::with_capacity(count);
        self.fuel = Vec::with_capacity(count);
        for r in 0..self.size {
            for c in 0..self.size {
                let x = -half + (c as f32 + 0.5) * self.cell;
                let z = -half + (r as f32 + 0.5) * self.cell;
                let h = terrain.height_at(x, z);
                let fuel = rng.range(0.5, 1.5);
                self.height.push(h);
                let bare = h > SNOW_LINE || terrain.is_water(x, z);
                self.fuel.push(if bare { 0.0 } else { fuel });
            }
        }
    }

    /// What a downward thermal sensor at `pos` with ground radius `radius` sees.
    pub fn read(&self, pos: Vec3, radius: f32) -> FireReading {
        let half = self.cell * self.size as f32 * 0.5;
        let (mut n, mut sx, mut sz) = (0u32, 0.0f32, 0.0f32);
        let span = (radius / self.cell).ceil() as isize + 1;
        let (pc, pr) = (((pos.x + half) / self.cell) as isize, ((pos.z + half) / self.cell) as isize);
        for r in (pr - span).max(0)..(pr + span + 1).min(self.size as isize) {
            for c in (pc - span).max(0)..(pc + span + 1).min(self.size as isize) {
                if self.cells[r as usize * self.size + c as usize] != BURNING {
                    continue;
                }
                let x = -half + (c as f32 + 0.5) * self.cell;
                let z = -half + (r as f32 + 0.5) * self.cell;
                if (x - pos.x).powi(2) + (z - pos.z).powi(2) <= radius * radius {
                    n += 1;
                    sx += x;
                    sz += z;
                }
            }
        }
        if n == 0 {
            return FireReading::default();
        }
        let (cx, cz) = (sx / n as f32, sz / n as f32);
        FireReading { burning: n.min(u32::from(u16::MAX)) as u16, centroid: Vec3::new(cx, 0.0, cz) }
    }

    fn index_at(&self, x: f32, z: f32) -> Option<usize> {
        let half = self.cell * self.size as f32 * 0.5;
        let (c, r) = ((x + half) / self.cell, (z + half) / self.cell);
        let in_grid = (0.0..self.size as f32).contains(&c) && (0.0..self.size as f32).contains(&r);
        in_grid.then(|| r as usize * self.size + c as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXTENT: f32 = 1200.0;
    const SIZE: usize = 96;
    const STEP_MS: u32 = 50;

    /// A terrain whose height is `f(x, z)`.
    fn terrain_from(f: impl Fn(f32, f32) -> f32) -> Terrain {
        let n = 65;
        let cell = EXTENT / (n - 1) as f32;
        let heights = (0..n * n)
            .map(|i| {
                let (c, r) = ((i % n) as f32, (i / n) as f32);
                f(-EXTENT / 2.0 + c * cell, -EXTENT / 2.0 + r * cell)
            })
            .collect();
        Terrain::from_heights(n, EXTENT, heights)
    }

    fn run(fire: &mut FireGrid, terrain: &Terrain, rng: &mut Rng, ms: u32) {
        for _ in 0..ms / STEP_MS {
            fire.step(STEP_MS, terrain, rng);
        }
    }

    /// Cells that have caught (burning or burnt), split by which side of
    /// `x = 0` they sit on: `(west, east)`.
    fn caught_west_east(fire: &FireGrid) -> (usize, usize) {
        let mut sides = (0, 0);
        for (i, &c) in fire.cells().iter().enumerate() {
            if c == UNBURNT {
                continue;
            }
            if i % fire.size() < fire.size() / 2 {
                sides.0 += 1;
            } else {
                sides.1 += 1;
            }
        }
        sides
    }

    fn centre_fire(wind: (f32, f32)) -> FireGrid {
        let mut fire = FireGrid::new(SIZE, EXTENT);
        fire.set_wind(wind.0, wind.1);
        // Exactly on the boundary between two columns' worth of cells would
        // bias one side; light a 2-wide seed straddling x = 0.
        fire.ignite(-1.0, 1.0);
        fire.ignite(1.0, 1.0);
        fire
    }

    /// Sums `caught_west_east` over a few seeds to wash out luck.
    fn spread_sides(terrain: &Terrain, wind: (f32, f32), ms: u32) -> (usize, usize) {
        let mut total = (0, 0);
        for seed in 1..=4 {
            let mut fire = centre_fire(wind);
            run(&mut fire, terrain, &mut Rng::new(seed), ms);
            let (w, e) = caught_west_east(&fire);
            total = (total.0 + w, total.1 + e);
        }
        total
    }

    #[test]
    fn spreads_faster_uphill() {
        // Rising toward +x, staying under the snow line.
        let slope = terrain_from(|x, _| 100.0 + 0.15 * x);
        let (downhill, uphill) = spread_sides(&slope, (0.0, 0.0), 60_000);
        assert!(uphill > downhill * 3 / 2, "uphill {uphill} vs downhill {downhill}");
    }

    #[test]
    fn spreads_faster_downwind() {
        let flat = terrain_from(|_, _| 50.0);
        let (downwind, upwind) = spread_sides(&flat, (-3.0, 0.0), 60_000);
        assert!(downwind > upwind * 3 / 2, "downwind {downwind} vs upwind {upwind}");
    }

    #[test]
    fn cells_burn_out() {
        let flat = terrain_from(|_, _| 50.0);
        let mut fire = FireGrid::new(SIZE, EXTENT);
        fire.ignite(0.0, 0.0);
        let first = fire.index_at(0.0, 0.0).unwrap();
        run(&mut fire, &flat, &mut Rng::new(3), BURN_MS.1 as u32 + STEP_MS);
        assert_eq!(fire.cells()[first], BURNT);
    }

    #[test]
    fn same_seed_same_fire() {
        let terrain = Terrain::generate(1, 129, EXTENT);
        let burn = |seed| {
            let mut fire = centre_fire((-3.0, 0.0));
            run(&mut fire, &terrain, &mut Rng::new(seed), 60_000);
            fire.cells().to_vec()
        };
        assert_eq!(burn(7), burn(7));
        assert_ne!(burn(7), burn(8), "different seeds should give different fires");
    }

    #[test]
    fn never_relights_burnt_cells_and_respects_firebreaks() {
        let flat = terrain_from(|_, _| 50.0);
        let mut fire = FireGrid::new(SIZE, EXTENT);
        // A burnt ring two cells thick around the centre: a firebreak.
        let mid = SIZE as isize / 2;
        for r in mid - 4..=mid + 4 {
            for c in mid - 4..=mid + 4 {
                let ring = (r - mid).abs().max((c - mid).abs());
                if ring >= 3 {
                    fire.cells[r as usize * SIZE + c as usize] = BURNT;
                }
            }
        }
        fire.ignite(0.0, 0.0);
        let mut prev = fire.cells().to_vec();
        let mut rng = Rng::new(11);
        for _ in 0..300_000 / STEP_MS {
            fire.step(STEP_MS, &flat, &mut rng);
            for (i, (&before, &now)) in prev.iter().zip(fire.cells()).enumerate() {
                assert!(now >= before, "cell {i} went from {before} back to {now}");
            }
            prev.copy_from_slice(fire.cells());
        }
        let escaped = fire.cells().iter().enumerate().any(|(i, &c)| {
            let (r, c2) = ((i / SIZE) as isize, (i % SIZE) as isize);
            c != UNBURNT && (r - mid).abs().max((c2 - mid).abs()) > 4
        });
        assert!(!escaped, "fire crossed a burnt firebreak");
        assert_eq!(fire.burning_count(), 0, "fire inside the ring should have burnt out");
    }

    #[test]
    fn snow_does_not_burn() {
        let snowy = terrain_from(|_, _| SNOW_LINE + 50.0);
        let mut fire = FireGrid::new(SIZE, EXTENT);
        fire.ignite(0.0, 0.0);
        run(&mut fire, &snowy, &mut Rng::new(5), 60_000);
        let caught = fire.cells().iter().filter(|&&c| c != UNBURNT).count();
        assert_eq!(caught, 1, "only the spark itself should have burnt");
    }

    #[test]
    fn calm_wind_ignores_non_finite_input() {
        let mut fire = FireGrid::new(4, EXTENT);
        fire.set_wind(f32::NAN, f32::INFINITY);
        assert_eq!(fire.wind(), (0.0, 0.0));
        fire.ignite(f32::NAN, 0.0);
        fire.ignite(1e30, -1e30);
        assert_eq!(fire.burning_count(), 0);
    }

    /// The demo's tuning target: from the default ignition point, about 3
    /// minutes grows a front a few hundred units across without running
    /// away across the map.
    #[test]
    fn demo_fire_grows_to_a_few_hundred_units() {
        let terrain = Terrain::generate(1, 129, EXTENT);
        let mut fire = FireGrid::new(SIZE, EXTENT);
        fire.set_wind(-3.0, 0.0);
        fire.ignite(330.0, 40.0);
        let mut rng = Rng::new(1);
        run(&mut fire, &terrain, &mut rng, 180_000);
        let (w, h) = caught_span(&fire);
        let burning = fire.burning_count();
        assert!(burning > 0, "fire died out");
        assert!((150.0..600.0).contains(&w.max(h)), "front spans {w} x {h} units");
    }

    /// Width and depth, in units, of the box around every cell that caught.
    fn caught_span(fire: &FireGrid) -> (f32, f32) {
        let (mut c0, mut c1, mut r0, mut r1) = (usize::MAX, 0, usize::MAX, 0);
        for (i, &s) in fire.cells().iter().enumerate() {
            if s != UNBURNT {
                let (r, c) = (i / fire.size(), i % fire.size());
                (c0, c1, r0, r1) = (c0.min(c), c1.max(c), r0.min(r), r1.max(r));
            }
        }
        if c0 > c1 {
            return (0.0, 0.0);
        }
        (((c1 - c0 + 1) as f32) * fire.cell, ((r1 - r0 + 1) as f32) * fire.cell)
    }

    #[test]
    #[ignore = "prints the demo fire's growth for tuning"]
    fn print_demo_fire_growth() {
        let terrain = Terrain::generate(1, 129, EXTENT);
        for seed in 1..=3 {
            let mut fire = FireGrid::new(SIZE, EXTENT);
            fire.set_wind(-3.0, 0.0);
            fire.ignite(330.0, 40.0);
            let mut rng = Rng::new(seed);
            for minute in 1..=8 {
                run(&mut fire, &terrain, &mut rng, 60_000);
                let caught = fire.cells().iter().filter(|&&c| c != UNBURNT).count();
                println!(
                    "seed {seed} t={minute}min burning={} caught={caught} span={:?}",
                    fire.burning_count(),
                    caught_span(&fire)
                );
            }
        }
    }
}
