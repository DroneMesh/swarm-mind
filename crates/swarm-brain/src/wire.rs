//! The radio protocol: messages and their byte encoding.
//!
//! Frame layout (little-endian): `[kind u8][src u16][dst u16][payload..]`.
//! Every message fits in [`MAX_FRAME`] bytes, well under the 250-byte
//! ESP-NOW payload limit.
//!
//! There are only two kinds on the air. Everything the swarm needs to
//! coordinate (routing gradient, sink freshness, slot claims, vacancy
//! notices, intent version) rides on the periodic [`Payload::Heartbeat`], so
//! coordination costs no extra frames. The other kind carries operator
//! intents: [`Payload::Intent`] from the console to the base, and the
//! [`Payload::Mission`] the base floods from there. Both use
//! [`KIND_INTENT`] and are told apart by length, so the renderer can colour
//! every intent pulse the same.

use crate::Vec3;

pub type NodeId = u16;

/// Destination meaning "everyone in radio range".
pub const BROADCAST: NodeId = 0xFFFF;
/// Source id for commands typed by the human operator at the base station.
pub const OPERATOR: NodeId = 0xFFFE;
/// Largest encoded frame in bytes.
pub const MAX_FRAME: usize = 32;
const _: () = assert!(MAX_FRAME <= 250, "frames must fit in one ESP-NOW packet");

pub const KIND_HEARTBEAT: u8 = 0;
pub const KIND_INTENT: u8 = 1;

const HEADER_LEN: usize = 5;
const HEARTBEAT_LEN: usize = HEADER_LEN + 26;
const INTENT_LEN: usize = HEADER_LEN + 15;
const MISSION_LEN: usize = HEADER_LEN + 21;
const _: () = assert!(HEARTBEAT_LEN <= MAX_FRAME && MISSION_LEN <= MAX_FRAME);
const _: () = assert!(INTENT_LEN != MISSION_LEN, "intent kinds are told apart by length");

/// Heartbeat flag: the sender is in position at the slot it claims.
pub const FLAG_SETTLED: u8 = 1 << 0;
/// Heartbeat flag: the sender is parked on the ground.
pub const FLAG_LANDED: u8 = 1 << 1;
/// Heartbeat flag: the sender's claim came from a vacancy notice and it is
/// still on its way, so it does not count as holding the slot yet.
pub const FLAG_FILLING: u8 = 1 << 2;
/// Heartbeat flag: other drones route through the sender, so it should not
/// wander off (a reserve bridging a gap is not available as a filler).
pub const FLAG_CARRYING: u8 = 1 << 3;
/// Heartbeat flag: the sender's route leads to a sink that drones elected
/// after being cut off for a long time, not because the base fell silent in
/// front of them. The base may well be alive elsewhere, so that group keeps
/// heading back toward it until the gradients meet.
pub const FLAG_ORPHAN: u8 = 1 << 4;
/// Heartbeat bits 5-6: the telemetry congestion level (0-3) the sink
/// decided, passed down the gradient to everyone (`airtime.rs`). Always 0
/// while the host runs no telemetry.
pub const FLAG_CONGESTION_SHIFT: u8 = 5;
pub const FLAG_CONGESTION_MASK: u8 = 0b11 << FLAG_CONGESTION_SHIFT;
/// Heartbeat bit 7: the sink's cut epoch. The sink flips it each time it
/// decides that everyone cuts their telemetry rate, and every drone copies
/// it from its parent, so the fleet cuts once per decision however long the
/// verdict lingers on the way down (`airtime.rs`). Always 0 while the host
/// runs no telemetry.
pub const FLAG_CUT_EPOCH: u8 = 1 << 7;
/// Bits of the role byte below [`Beacon::congestion_up`].
const ROLE_MASK: u8 = 0x3F;
const CONGESTION_UP_SHIFT: u8 = 6;

#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Role {
    #[default]
    Reserve = 0,
    Scout = 1,
    Relay = 2,
    Sink = 3,
}

impl Role {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Reserve,
            1 => Self::Scout,
            2 => Self::Relay,
            3 => Self::Sink,
            _ => return None,
        })
    }
}

/// A high-level goal from the operator. Drones decide for themselves how to
/// achieve it.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Intent {
    #[default]
    Hold = 0,
    MonitorFire = 1,
    MapPerimeter = 2,
    ReturnToBase = 3,
}

impl Intent {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Hold,
            1 => Self::MonitorFire,
            2 => Self::MapPerimeter,
            3 => Self::ReturnToBase,
            _ => return None,
        })
    }
}

/// Everything a drone tells its neighbours, once per beacon interval.
/// Positions travel as whole units (`i16`), which is far finer than the
/// radio range and halves their size.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Beacon {
    /// Hops to the sink, or [`crate::NO_ROUTE`]. Advertising `NO_ROUTE` is
    /// the poison that makes children drop a broken route at once.
    pub hops: u8,
    /// Next hop toward the sink, or [`BROADCAST`] for none (split horizon).
    pub parent: NodeId,
    pub role: Role,
    /// Telemetry congestion (0-3) the sender or any drone beyond it (farther
    /// from the sink) reports, on its way up to the sink (`airtime.rs`).
    /// Rides in the top two bits of the role byte; always 0 while the host
    /// runs no telemetry.
    pub congestion_up: u8,
    pub pos: Vec3,
    /// Which sink the gradient leads to, and its freshness counter. The sink
    /// bumps `sink_seq` every beacon; a route whose counter stops advancing
    /// leads to a dead or unreachable sink.
    pub sink: NodeId,
    pub sink_seq: u16,
    /// Per-sender beacon counter. Gaps tell the receiver how lossy the
    /// channel is, which sets how many missed beacons it tolerates.
    pub counter: u8,
    /// Sequence number of the newest intent the sender knows. A neighbour
    /// that is behind gets the intent re-sent (anti-entropy).
    pub intent_seq: u16,
    /// Formation slot the sender claims (0 = none).
    pub slot: u8,
    /// A slot the sender believes is vacant (0 = none), and how many hops
    /// that notice has travelled.
    pub need: u8,
    pub need_hops: u8,
    /// `FLAG_*` bits.
    pub flags: u8,
    /// The sender's current beacon interval, in 50 ms units, so receivers
    /// know when to expect the next one.
    pub interval_50ms: u8,
    /// Relay and scout slots whose holders the sender heard directly in the
    /// last few seconds (see `plan::slot_bit`). Lets neighbours tell a
    /// vacancy from a heartbeat they merely missed.
    pub heard_slots: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Payload {
    /// Periodic "I'm alive" beacon carrying all coordination state.
    Heartbeat(Beacon),
    /// Operator intent typed at the console, delivered to the base.
    /// `target` is the area of interest, e.g. the reported fire location.
    Intent { seq: u16, intent: Intent, target: Vec3 },
    /// An intent as flooded through the mesh by the sink, once per sequence
    /// number. `origin` is where the sink stood, so every drone derives the
    /// same formation geometry from the same two points.
    Mission { seq: u16, intent: Intent, target: Vec3, origin: Vec3 },
}

impl Payload {
    pub fn kind(&self) -> u8 {
        match self {
            Self::Heartbeat(_) => KIND_HEARTBEAT,
            Self::Intent { .. } | Self::Mission { .. } => KIND_INTENT,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Message {
    pub src: NodeId,
    pub dst: NodeId,
    pub payload: Payload,
}

impl Message {
    pub fn encode(&self) -> Frame {
        let mut frame = Frame::default();
        let mut w = Writer { buf: &mut frame.bytes, at: 0 };
        w.u8(self.payload.kind());
        w.u16(self.src);
        w.u16(self.dst);
        match self.payload {
            Payload::Heartbeat(b) => {
                w.u8(b.hops);
                w.u16(b.parent);
                w.u8(b.role as u8 | (b.congestion_up.min(3) << CONGESTION_UP_SHIFT));
                w.pos16(b.pos);
                w.u16(b.sink);
                w.u16(b.sink_seq);
                w.u8(b.counter);
                w.u16(b.intent_seq);
                w.u8(b.slot);
                w.u8(b.need);
                w.u8(b.need_hops);
                w.u8(b.flags);
                w.u8(b.interval_50ms);
                w.u32(b.heard_slots);
            }
            Payload::Intent { seq, intent, target } => {
                w.u16(seq);
                w.u8(intent as u8);
                w.f32(target.x);
                w.f32(target.y);
                w.f32(target.z);
            }
            Payload::Mission { seq, intent, target, origin } => {
                w.u16(seq);
                w.u8(intent as u8);
                w.f32(target.x);
                w.f32(target.y);
                w.f32(target.z);
                w.pos16(origin);
            }
        }
        frame.len = w.at as u8;
        frame
    }

    /// Returns `None` for truncated, unknown or malformed frames.
    pub fn decode(frame: &Frame) -> Option<Self> {
        let bytes = frame.as_bytes();
        let mut r = Reader { buf: bytes, at: 0 };
        let kind = r.u8()?;
        let src = r.u16()?;
        let dst = r.u16()?;
        let payload = match (kind, bytes.len()) {
            (KIND_HEARTBEAT, HEARTBEAT_LEN) => {
                let hops = r.u8()?;
                let parent = r.u16()?;
                let role = r.u8()?;
                Payload::Heartbeat(Beacon {
                    hops,
                    parent,
                    role: Role::from_u8(role & ROLE_MASK)?,
                    congestion_up: role >> CONGESTION_UP_SHIFT,
                    pos: r.pos16()?,
                    sink: r.u16()?,
                    sink_seq: r.u16()?,
                    counter: r.u8()?,
                    intent_seq: r.u16()?,
                    slot: r.u8()?,
                    need: r.u8()?,
                    need_hops: r.u8()?,
                    flags: r.u8()?,
                    interval_50ms: r.u8()?,
                    heard_slots: r.u32()?,
                })
            }
            (KIND_INTENT, INTENT_LEN) => Payload::Intent {
                seq: r.u16()?,
                intent: Intent::from_u8(r.u8()?)?,
                target: Vec3::new(r.f32()?, r.f32()?, r.f32()?),
            },
            (KIND_INTENT, MISSION_LEN) => Payload::Mission {
                seq: r.u16()?,
                intent: Intent::from_u8(r.u8()?)?,
                target: Vec3::new(r.f32()?, r.f32()?, r.f32()?),
                origin: r.pos16()?,
            },
            _ => return None,
        };
        Some(Self { src, dst, payload })
    }
}

/// Raw bytes as they travel over the air.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    len: u8,
    bytes: [u8; MAX_FRAME],
}

impl Default for Frame {
    fn default() -> Self {
        Self { len: 0, bytes: [0; MAX_FRAME] }
    }
}

impl Frame {
    /// Wraps bytes received off the air. Returns `None` if they don't fit.
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        let mut frame = Self::default();
        frame.bytes.get_mut(..bytes.len())?.copy_from_slice(bytes);
        frame.len = bytes.len() as u8;
        Some(frame)
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }

    // Header fields, readable without decoding the payload.

    pub fn kind(&self) -> u8 {
        self.bytes[0]
    }

    pub fn src(&self) -> NodeId {
        u16::from_le_bytes([self.bytes[1], self.bytes[2]])
    }

    pub fn dst(&self) -> NodeId {
        u16::from_le_bytes([self.bytes[3], self.bytes[4]])
    }
}

/// A frame as received, with the signal strength the radio measured.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rx {
    pub frame: Frame,
    /// Received signal strength in dBm: about -40 right next to the sender,
    /// -95 at the edge of range.
    pub rssi_dbm: i8,
}

/// Whole-unit coordinate, saturating at the `i16` range (NaN becomes 0).
fn quantize(v: f32) -> i16 {
    // `as` saturates and maps NaN to 0, which is exactly what we want.
    (v + if v < 0.0 { -0.5 } else { 0.5 }) as i16
}

struct Writer<'a> {
    buf: &'a mut [u8],
    at: usize,
}

impl Writer<'_> {
    fn put(&mut self, bytes: &[u8]) {
        self.buf[self.at..self.at + bytes.len()].copy_from_slice(bytes);
        self.at += bytes.len();
    }

    fn u8(&mut self, v: u8) {
        self.put(&[v]);
    }

    fn u16(&mut self, v: u16) {
        self.put(&v.to_le_bytes());
    }

    fn u32(&mut self, v: u32) {
        self.put(&v.to_le_bytes());
    }

    fn f32(&mut self, v: f32) {
        self.put(&v.to_le_bytes());
    }

    fn pos16(&mut self, p: Vec3) {
        for v in [p.x, p.y, p.z] {
            self.put(&quantize(v).to_le_bytes());
        }
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    at: usize,
}

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> Option<[u8; N]> {
        let bytes = self.buf.get(self.at..self.at + N)?;
        self.at += N;
        bytes.try_into().ok()
    }

    fn u8(&mut self) -> Option<u8> {
        self.take::<1>().map(|b| b[0])
    }

    fn u16(&mut self) -> Option<u16> {
        self.take().map(u16::from_le_bytes)
    }

    fn u32(&mut self) -> Option<u32> {
        self.take().map(u32::from_le_bytes)
    }

    fn f32(&mut self) -> Option<f32> {
        self.take().map(f32::from_le_bytes)
    }

    fn i16(&mut self) -> Option<f32> {
        self.take().map(|b| f32::from(i16::from_le_bytes(b)))
    }

    fn pos16(&mut self) -> Option<Vec3> {
        Some(Vec3::new(self.i16()?, self.i16()?, self.i16()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn heartbeat() -> Message {
        Message {
            src: 7,
            dst: BROADCAST,
            payload: Payload::Heartbeat(Beacon {
                congestion_up: 0,
                hops: 3,
                parent: 2,
                role: Role::Relay,
                pos: Vec3::new(1.0, -2.0, 300.0),
                sink: 0,
                sink_seq: 65_000,
                counter: 200,
                intent_seq: 9,
                slot: 5,
                need: 3,
                need_hops: 1,
                flags: FLAG_SETTLED,
                interval_50ms: 20,
                heard_slots: 0x8000_0012,
            }),
        }
    }

    fn intent() -> Message {
        Message {
            src: OPERATOR,
            dst: 0,
            payload: Payload::Intent {
                seq: 42,
                intent: Intent::MonitorFire,
                target: Vec3::new(300.0, 0.0, -50.0),
            },
        }
    }

    fn mission() -> Message {
        Message {
            src: 0,
            dst: BROADCAST,
            payload: Payload::Mission {
                seq: 42,
                intent: Intent::MapPerimeter,
                target: Vec3::new(330.5, 12.25, 40.0),
                origin: Vec3::new(-456.0, 35.0, 0.0),
            },
        }
    }

    #[test]
    fn messages_round_trip() {
        for msg in [heartbeat(), intent(), mission()] {
            assert_eq!(Message::decode(&msg.encode()), Some(msg));
        }
    }

    #[test]
    fn positions_are_rounded_to_whole_units() {
        let decode_pos = |pos: Vec3| {
            let mut msg = heartbeat();
            if let Payload::Heartbeat(ref mut b) = msg.payload {
                b.pos = pos;
            }
            match Message::decode(&msg.encode()).map(|m| m.payload) {
                Some(Payload::Heartbeat(b)) => b.pos,
                other => panic!("decoded {other:?}"),
            }
        };
        assert_eq!(decode_pos(Vec3::new(1.4, -2.6, f32::NAN)), Vec3::new(1.0, -3.0, 0.0));
        assert_eq!(decode_pos(Vec3::new(1e9, -1e9, 0.0)), Vec3::new(32767.0, -32768.0, 0.0));
    }

    #[test]
    fn header_is_readable_without_decoding() {
        let frame = heartbeat().encode();
        assert_eq!((frame.kind(), frame.src(), frame.dst()), (KIND_HEARTBEAT, 7, BROADCAST));
        assert_eq!(mission().encode().kind(), KIND_INTENT);
    }

    #[test]
    fn messages_fit_in_a_frame() {
        for msg in [heartbeat(), intent(), mission()] {
            assert!(msg.encode().as_bytes().len() <= MAX_FRAME);
        }
    }

    #[test]
    fn truncated_frames_are_rejected() {
        for msg in [heartbeat(), intent(), mission()] {
            let full = msg.encode();
            let bytes = full.as_bytes();
            let cut = Frame::from_bytes(&bytes[..bytes.len() - 1]).unwrap();
            assert_eq!(Message::decode(&cut), None);
        }
    }

    #[test]
    fn unknown_kinds_and_bad_enums_are_rejected() {
        let mut bytes = [0u8; INTENT_LEN];
        bytes.copy_from_slice(intent().encode().as_bytes());
        bytes[0] = 99;
        assert_eq!(Message::decode(&Frame::from_bytes(&bytes).unwrap()), None);

        bytes[0] = KIND_INTENT;
        bytes[7] = 200; // not a valid Intent
        assert_eq!(Message::decode(&Frame::from_bytes(&bytes).unwrap()), None);

        let mut hb = [0u8; HEARTBEAT_LEN];
        hb.copy_from_slice(heartbeat().encode().as_bytes());
        hb[8] = 9; // not a valid Role
        assert_eq!(Message::decode(&Frame::from_bytes(&hb).unwrap()), None);
    }
}
