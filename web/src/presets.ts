// Map presets, indexed by swarm-sim's `Preset` value (crates/swarm-sim/src/preset.rs).
// Visual and narrative settings live here; terrain, base, fire and wind come from the sim.

export interface PresetInfo {
  id: number;
  key: string;
  name: string;
  blurb: string;
  /** The radio problem this map poses, in one line. */
  challenge: string;
  sky: number;
  fogNear: number;
  fogFar: number;
  palette: { water?: number; sand?: number; low: number; mid: number; rock: number; snow: number };
  /**
   * World height of the sea surface, for maps that have one. Must match swarm-sim's
   * `Preset::water_level` (the worker can also read it via `sim_water_level`).
   */
  waterLevel?: number;
  camera: { pos: [number, number, number]; target: [number, number, number] };
  /** Ground above this height (or steeper than a cliff) is bare rock. Default 130. */
  rockLine?: number;
  /** Flat ground above this height is snow. Default 195 (the sim's fire snow line is 200). */
  snowLine?: number;
  /** Paint cliffs in alternating sandstone layers (`rock` / `mid`) and low ground as `sand`. */
  strata?: boolean;
}

export const PRESETS: PresetInfo[] = [
  {
    id: 0,
    key: 'alpine',
    name: 'Alpine Ridge',
    blurb: 'Base in the western foothills, fire beyond a snow-capped north-south ridge.',
    challenge: 'The ridge blocks line of sight: the swarm must build a relay chain over it.',
    sky: 0x1d2433,
    fogNear: 1400,
    fogFar: 3400,
    palette: { low: 0x2c4a2a, mid: 0x4d6b3a, rock: 0x6b625a, snow: 0xe8ecf2 },
    // Framed from the south: base on the left, the ridge in the middle, the fire on the right.
    camera: { pos: [-90, 600, 900], target: [-40, 30, 30] },
  },
  {
    id: 1,
    key: 'canyon',
    name: 'Canyon',
    blurb: 'The base sits deep in a winding desert gorge; the fire is up on the plateau beyond the rim.',
    challenge: 'Sheer walls hide the canyon floor: relays have to climb out of the gorge and over the rim.',
    sky: 0x2e2228,
    fogNear: 1300,
    fogFar: 3300,
    palette: { sand: 0xd2b07a, low: 0xa8774a, mid: 0xc08a58, rock: 0x8a4e34, snow: 0xe6d3b3 },
    // High and steep, so the eye sees down into the gorge's bends.
    camera: { pos: [-120, 1000, 560], target: [-60, 40, -40] },
    rockLine: 1000,
    snowLine: 1000,
    strata: true,
  },
  {
    id: 2,
    key: 'twin-valleys',
    name: 'Twin Valleys',
    blurb: 'Two parallel snow-dusted ridges split the land into three valleys; the fire burns in the far one.',
    challenge: 'Both crests block low flight: a longer ladder with two blind dips, and the most relays of any map.',
    sky: 0x1b2a33,
    fogNear: 1400,
    fogFar: 3500,
    palette: { low: 0x2f5530, mid: 0x557a3c, rock: 0x6a6660, snow: 0xedf1f5 },
    camera: { pos: [-20, 680, 960], target: [-20, 40, 0] },
  },
  {
    id: 3,
    key: 'archipelago',
    name: 'Archipelago',
    blurb: 'Base on the mainland coast, fire on a wooded island across open sea, islets in between.',
    challenge: 'Nothing blocks the radio over flat water, but range does: relays hover over the sea to bridge it.',
    sky: 0x1a3047,
    fogNear: 1500,
    fogFar: 3800,
    palette: { water: 0x1f5a7a, sand: 0xd8c690, low: 0x2e5a34, mid: 0x4f7a3e, rock: 0x6e665c, snow: 0xe8ecf2 },
    waterLevel: 0,
    camera: { pos: [-100, 520, 820], target: [-60, 0, 20] },
  },
  {
    id: 4,
    key: 'foothills',
    name: 'Foothills',
    blurb: 'Gentle rolling hills and scattered knolls, the fire only a short hop away.',
    challenge: 'The easy baseline: small hills block a little low flight, so the mesh keeps re-routing.',
    sky: 0x23292a,
    fogNear: 1300,
    fogFar: 3200,
    palette: { low: 0x3d5f2d, mid: 0x7a8a40, rock: 0x7a6e5c, snow: 0xe8ecf2 },
    camera: { pos: [-130, 460, 700], target: [-110, 20, -10] },
  },
];
