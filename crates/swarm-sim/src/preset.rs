//! Map presets: each one is a terrain generator plus where the base, the
//! fire and the wind start, chosen so it poses a different radio problem.
//!
//! The numeric values are part of the web contract (`web/src/presets.ts`
//! indexes its table by them), so never renumber them.

use crate::terrain::canyon_centre;

/// A named scenery. [`Preset::AlpineRidge`] is the original map and must
/// stay bit-identical; each other one has its own terrain generator (see
/// `Terrain::for_preset`) and layout, tested for fairness in `preset_tests`.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Preset {
    /// One north-south ridge between base and fire: needs a relay chain over it.
    #[default]
    AlpineRidge = 0,
    /// Base down in a winding gorge, fire up on the far rim: walls hide the
    /// floor, so relays must climb out of the canyon.
    Canyon = 1,
    /// Two parallel ridges between base and fire: a longer chain over two crests.
    TwinValleys = 2,
    /// Mainland base, fire on an island: open sea, hops limited only by range.
    Archipelago = 3,
    /// Gentle hills, fire close by: the easy baseline.
    Foothills = 4,
}

impl Preset {
    /// Every preset, in numeric order.
    pub const ALL: [Preset; 5] = [
        Preset::AlpineRidge,
        Preset::Canyon,
        Preset::TwinValleys,
        Preset::Archipelago,
        Preset::Foothills,
    ];

    pub fn from_u32(v: u32) -> Option<Preset> {
        Self::ALL.get(v as usize).copied()
    }

    pub fn name(self) -> &'static str {
        match self {
            Preset::AlpineRidge => "Alpine Ridge",
            Preset::Canyon => "Canyon",
            Preset::TwinValleys => "Twin Valleys",
            Preset::Archipelago => "Archipelago",
            Preset::Foothills => "Foothills",
        }
    }

    /// Where the fire starts, as `(x, z)`, on a map `extent` wide.
    pub fn ignition(self, extent: f32) -> (f32, f32) {
        match self {
            // Literal, not scaled: `SimConfig::default()` has always used it.
            Preset::AlpineRidge => (330.0, 40.0),
            // On the plateau just south of the gorge's eastern bend.
            Preset::Canyon => (0.275 * extent, (canyon_centre(0.275).0 - 0.12) * extent),
            // The east valley floor, at the foot of the far rim.
            Preset::TwinValleys => (0.36 * extent, -0.03 * extent),
            // The big island's western slope, so the fire climbs inland.
            Preset::Archipelago => (0.22 * extent, 0.04 * extent),
            // Closer than on the other maps: the easy baseline.
            Preset::Foothills => (0.15 * extent, -0.06 * extent),
        }
    }

    /// Initial wind `(x, z)` in units per second, chosen per map so the
    /// fire spreads visibly over a few minutes without leaving its stage.
    pub fn wind(self) -> (f32, f32) {
        match self {
            // A gentle westerly drift: the fire climbs toward the ridge.
            Preset::AlpineRidge => (-3.0, 0.0),
            // Runs the fire along the plateau, away from the rim (flames
            // creep only slowly down the benches into the gorge).
            Preset::Canyon => (-2.0, -2.5),
            // Drives it up the far rim, away from the crests.
            Preset::TwinValleys => (2.5, -1.0),
            // An onshore breeze pushing the fire up the island's hill.
            Preset::Archipelago => (3.0, 0.5),
            Preset::Foothills => (-2.5, 1.5),
        }
    }

    /// Ground position `(x, z)` of the base station on a map `extent` wide.
    /// Kept as a function of `extent` (not a config field) so tests that
    /// shrink the map still get a base on it, exactly as before presets.
    pub fn base_xz(self, extent: f32) -> (f32, f32) {
        match self {
            // The western foothills.
            Preset::AlpineRidge => (-0.38 * extent, 0.0),
            // Down on the gorge floor, near its western mouth.
            Preset::Canyon => (-0.38 * extent, canyon_centre(-0.38).0 * extent),
            // The west valley.
            Preset::TwinValleys => (-0.40 * extent, 0.02 * extent),
            // A little inland from the mainland coast.
            Preset::Archipelago => (-0.38 * extent, 0.0),
            Preset::Foothills => (-0.33 * extent, 0.05 * extent),
        }
    }

    /// Sea surface height, for maps with a sea. `web/src/presets.ts`
    /// mirrors it as `waterLevel`.
    pub fn water_level(self) -> Option<f32> {
        match self {
            Preset::Archipelago => Some(0.0),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn numbering_round_trips() {
        for (i, p) in Preset::ALL.iter().enumerate() {
            assert_eq!(*p as u32, i as u32);
            assert_eq!(Preset::from_u32(i as u32), Some(*p));
            assert!(!p.name().is_empty());
        }
        assert_eq!(Preset::from_u32(5), None);
        assert_eq!(Preset::from_u32(u32::MAX), None);
    }
}
