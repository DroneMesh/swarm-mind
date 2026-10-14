use swarm_brain::Vec3;

use crate::Terrain;

#[derive(Clone, Copy, Debug)]
pub struct RadioConfig {
    /// Maximum distance a frame can travel.
    pub range: f32,
    /// Probability that any single delivery is lost.
    pub loss: f32,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self { range: 260.0, loss: 0.0 }
    }
}

impl RadioConfig {
    /// Signal strength at the receiver, or `None` if it can't hear the
    /// sender: out of range, or the terrain blocks the line between them.
    pub fn receive(&self, terrain: &Terrain, from: Vec3, to: Vec3) -> Option<i8> {
        let d2 = (to - from).length_sq();
        if d2 > self.range * self.range || !terrain.line_of_sight(from, to) {
            return None;
        }
        Some(self.rssi_dbm(d2.sqrt()))
    }

    /// Log-distance path loss, scaled so the edge of range reads -95 dBm.
    pub fn rssi_dbm(&self, distance: f32) -> i8 {
        self.mean_rx_dbm(distance).round().clamp(-127.0, -20.0) as i8
    }

    /// Mean received power at `distance`, dBm, unrounded: -40 dBm at 1 unit
    /// (the sim's metre) falling as `10 n log10(d)` with the path-loss
    /// exponent `n = 5.5 / log10(range)` (2.28 at the default 260), so the
    /// edge of range sits at -95 dBm, 2 dB above the ESP32's 1 Mbps
    /// sensitivity: "range" is where the most robust rate becomes marginal.
    /// The channel model adds shadowing and fading on top
    /// ([`crate::channel`]).
    pub fn mean_rx_dbm(&self, distance: f32) -> f32 {
        let exponent = 55.0 / self.range.max(2.0).log10();
        -40.0 - exponent * distance.max(1.0).log10()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rssi_falls_with_distance_to_minus_95_at_range() {
        let radio = RadioConfig::default();
        assert!(radio.rssi_dbm(10.0) > radio.rssi_dbm(100.0));
        assert_eq!(radio.rssi_dbm(radio.range), -95);
    }
}
