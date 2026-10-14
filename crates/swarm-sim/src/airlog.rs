//! Byte ledger: every frame that goes on air, by node and by kind.
//!
//! Counts MPDU bytes (MAC header + FCS + LLC/SNAP + IP/TCP/UDP + payload,
//! or the ESP-NOW / beacon / ACK frame as a whole) of every transmission:
//! first attempts, MAC retries, TCP retransmissions, 802.11 ACKs (charged to
//! the node that sends them), TCP ACKs, beacons, brain frames and the
//! root's uplink. Rates are per second over the same 2 s window as the
//! telemetry stats ([`crate::window::Short`]), and the fleet totals are kept
//! in step with the per-node rows, so the per-node sums equal the fleet
//! figures by construction.
//!
//! The ledger is owned by [`crate::channel::Channel`] and fed whether or not
//! the airtime model is on (bytes exist with the channel model off; only
//! their cost does not). It never draws random numbers and nothing the
//! brains see depends on it.

use crate::window::{Short, Window};

/// 60 s of completed buckets plus the current one.
type Minute = Window<241>;

/// What a counted frame is, for the fleet breakdown.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ByteKind {
    /// ESP-NOW brain frames.
    Brain = 0,
    /// painlessMesh TCP segments carrying telemetry packages (every hop).
    TlmData = 1,
    /// 802.11 ACKs and TCP ACKs.
    Ack = 2,
    /// painlessMesh NodeSync / TimeSync segments.
    MeshControl = 3,
    /// Root → WiFi station datagrams (both crossings with `gcs_on_wifi`).
    Uplink = 4,
    /// SoftAP beacons.
    Beacon = 5,
}

const KINDS: usize = 6;

#[derive(Clone, Copy, Debug)]
struct NodeBytes {
    /// Everything the node transmits.
    all: Short,
    /// ACKs it sends (802.11 + TCP).
    ack: Short,
    /// Its own telemetry packages' share of the segments it sends.
    own: Short,
    /// Others' packages' share of the segments it relays.
    relay: Short,
    /// Uplink datagrams (root only).
    uplink: Short,
    /// Brain frames.
    brain: Short,
}

impl NodeBytes {
    fn new() -> Self {
        Self {
            all: Short::new(),
            ack: Short::new(),
            own: Short::new(),
            relay: Short::new(),
            uplink: Short::new(),
            brain: Short::new(),
        }
    }
}

/// One node's rates, bytes (or frames) per second.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NodeRates {
    pub all: f32,
    pub ack: f32,
    pub own: f32,
    pub relay: f32,
    pub uplink: f32,
    pub brain: f32,
}

/// Fleet rates, per second.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FleetRates {
    /// Bytes by [`ByteKind`].
    pub kind: [f32; KINDS],
    /// Bytes that are retransmissions (MAC retries and TCP retransmissions),
    /// part of the per-kind figures.
    pub retry: f32,
    /// Frames of every kind (beacons, ACKs and retries included).
    pub frames: f32,
    /// Frames other than beacons.
    pub frames_no_beacon: f32,
    /// NodeSync / TimeSync bytes per second over the last 60 s (they run
    /// once a minute per connection, so the 2 s figure is mostly 0).
    pub mesh_control_60s: f32,
}

impl FleetRates {
    /// All bytes on air.
    pub fn total(&self) -> f32 {
        self.kind.iter().sum()
    }
}

#[derive(Clone, Debug)]
pub struct AirLedger {
    nodes: Vec<NodeBytes>,
    kind: [Short; KINDS],
    retry: Short,
    frames: Short,
    beacon_frames: Short,
    ctl60: Minute,
}

impl AirLedger {
    pub fn new(nodes: usize) -> Self {
        Self {
            nodes: vec![NodeBytes::new(); nodes],
            kind: [Short::new(); KINDS],
            retry: Short::new(),
            frames: Short::new(),
            beacon_frames: Short::new(),
            ctl60: Minute::new(),
        }
    }

    /// Counts `frames` frames of `bytes` bytes in all that `node` sent.
    /// `retry` bytes of them are retransmissions; for mesh segments,
    /// `own` / `relay` / `ctl` split the bytes (they sum to `bytes`).
    #[allow(clippy::too_many_arguments)]
    fn add(&mut self, now_ms: u32, node: u16, kind: ByteKind, bytes: f32, frames: f32, retry: f32, split: Split) {
        if !(bytes.is_finite() && bytes > 0.0) {
            return;
        }
        match kind {
            ByteKind::TlmData | ByteKind::MeshControl => {
                self.kind[ByteKind::TlmData as usize].add(now_ms, split.own + split.relay);
                self.kind[ByteKind::MeshControl as usize].add(now_ms, split.ctl);
                if split.ctl > 0.0 {
                    self.ctl60.add(now_ms, split.ctl);
                }
            }
            k => self.kind[k as usize].add(now_ms, bytes),
        }
        if retry > 0.0 {
            self.retry.add(now_ms, retry);
        }
        self.frames.add(now_ms, frames);
        if kind == ByteKind::Beacon {
            self.beacon_frames.add(now_ms, frames);
        }
        let Some(n) = self.nodes.get_mut(usize::from(node)) else { return };
        n.all.add(now_ms, bytes);
        match kind {
            ByteKind::Ack => n.ack.add(now_ms, bytes),
            ByteKind::Brain => n.brain.add(now_ms, bytes),
            ByteKind::Uplink => n.uplink.add(now_ms, bytes),
            ByteKind::TlmData | ByteKind::MeshControl => {
                n.own.add(now_ms, split.own);
                n.relay.add(now_ms, split.relay);
            }
            ByteKind::Beacon => {}
        }
    }

    /// A frame of `bytes` that is not a mesh segment.
    pub fn frame(&mut self, now_ms: u32, node: u16, kind: ByteKind, bytes: u32, retry: bool) {
        let b = bytes as f32;
        self.add(now_ms, node, kind, b, 1.0, if retry { b } else { 0.0 }, Split::default());
    }

    /// A mesh TCP segment of `bytes`: `own_frac` of it is the sender's own
    /// telemetry, `ctl_frac` NodeSync / TimeSync, the rest relayed telemetry.
    pub fn segment(&mut self, now_ms: u32, node: u16, bytes: u32, own_frac: f32, ctl_frac: f32, retry: bool) {
        let b = bytes as f32;
        let ctl = b * ctl_frac.clamp(0.0, 1.0);
        let own = (b * own_frac.clamp(0.0, 1.0)).min(b - ctl);
        let split = Split { own, relay: b - ctl - own, ctl };
        let kind = if ctl >= b { ByteKind::MeshControl } else { ByteKind::TlmData };
        self.add(now_ms, node, kind, b, 1.0, if retry { b } else { 0.0 }, split);
    }

    /// `count` beacons of `bytes` each from `node`.
    pub fn beacons(&mut self, now_ms: u32, node: u16, bytes: u32, count: f32) {
        self.add(now_ms, node, ByteKind::Beacon, bytes as f32 * count, count, 0.0, Split::default());
    }

    /// Per-second rates of `node` (zeros for an unknown id).
    pub fn node(&self, now_ms: u32, node: u16) -> NodeRates {
        let Some(n) = self.nodes.get(usize::from(node)) else { return NodeRates::default() };
        NodeRates {
            all: n.all.per_s_at(now_ms),
            ack: n.ack.per_s_at(now_ms),
            own: n.own.per_s_at(now_ms),
            relay: n.relay.per_s_at(now_ms),
            uplink: n.uplink.per_s_at(now_ms),
            brain: n.brain.per_s_at(now_ms),
        }
    }

    pub fn fleet(&self, now_ms: u32) -> FleetRates {
        let mut kind = [0.0; KINDS];
        for (k, w) in kind.iter_mut().zip(&self.kind) {
            *k = w.per_s_at(now_ms);
        }
        let frames = self.frames.per_s_at(now_ms);
        FleetRates {
            kind,
            retry: self.retry.per_s_at(now_ms),
            frames,
            frames_no_beacon: (frames - self.beacon_frames.per_s_at(now_ms)).max(0.0),
            mesh_control_60s: self.ctl60.per_s_at(now_ms),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Split {
    own: f32,
    relay: f32,
    ctl: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_rows_sum_to_the_fleet() {
        let mut l = AirLedger::new(3);
        for t in (0..3_000).step_by(50) {
            l.frame(t, 0, ByteKind::Brain, 74, false);
            l.segment(t, 1, 400, 0.5, 0.25, t % 200 == 0);
            l.frame(t, 2, ByteKind::Ack, 14, false);
            l.beacons(t, 2, 190, 0.5);
        }
        let now = 3_000;
        let f = l.fleet(now);
        let sum: f32 = (0..3).map(|i| l.node(now, i).all).sum();
        assert!((sum - f.total()).abs() < 1e-2 * f.total(), "{sum} vs {}", f.total());
        assert!((f.kind[ByteKind::MeshControl as usize] - 400.0 * 0.25 * 20.0).abs() < 1.0);
        assert!((l.node(now, 1).own - 400.0 * 0.5 * 20.0).abs() < 1.0);
        assert!((l.node(now, 1).relay - 400.0 * 0.25 * 20.0).abs() < 1.0);
        assert!(f.retry > 0.0 && f.frames > f.frames_no_beacon);
        assert_eq!(l.node(now, 9), NodeRates::default());
    }
}
