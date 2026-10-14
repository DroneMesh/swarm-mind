//! Decoded radio traffic for the operator console.
//!
//! [`crate::Sim::chatter`] holds one record per frame put on the air during
//! the latest step (cleared at the start of each step), decoded from the
//! wire bytes with `swarm_brain::Message::decode`, so the console shows
//! exactly what a sniffer in the field would. Operator intents typed at the
//! console are wired into the base rather than radioed, but they are logged
//! too, in the step in which the base reads them.
//!
//! Each record is [`CHATTER_STRIDE`] floats:
//!
//! | offset | field                                              |
//! |--------|----------------------------------------------------|
//! | 0      | `t_ms`: sim time of the step, milliseconds          |
//! | 1      | `src`: sender id (`65534` = the operator console)   |
//! | 2      | `dst`: receiver id, or `-1` for broadcast           |
//! | 3      | `kind`: one of the `CHATTER_*` kinds below          |
//! | 4..12  | `f0..f7`, depending on `kind`                       |
//!
//! Heartbeat (`CHATTER_HEARTBEAT`): `f = [hops, parent, role, sink, slot,
//! flags, need, intent_seq]`. `hops` is raw (`swarm_brain::NO_ROUTE` = no
//! route), `parent` is `-1` for none, `role` is the `Role` value, `slot` and
//! `need` are `-1` for none, `flags` are the `FLAG_*` bits.
//!
//! Intent (`CHATTER_INTENT`) and mission (`CHATTER_MISSION`): `f = [seq,
//! intent, target_x, target_z, origin_x, origin_z, 0, 0]`; an intent has no
//! origin, so both origin fields are NaN.
//!
//! Undecodable (`CHATTER_UNDECODABLE`): all `f` are 0; `src`/`dst` come from
//! the raw frame header.
//!
//! The kinds below are not brain frames: they come from the telemetry /
//! painlessMesh model ([`crate::telemetry`], [`crate::painless`]) and appear
//! only while telemetry is on. TLM and ROOT_UPLINK are sampled: at most one
//! record per drone per second (`TelemetryConfig::chatter_every_frame` logs
//! every package), so the stream stays readable at 25 Hz.
//!
//! Telemetry (`CHATTER_TLM`): a painlessMesh package with a telemetry
//! frame; `dst` is the root, or `-1` for a flooded broadcast. `f = [bytes
//! (JSON payload), package_bytes (mesh JSON incl. '\0'), hops_to_root (-1
//! none), seq, has_slow (0/1), 0, 0, 0]`.
//!
//! Mesh sync (`CHATTER_PM_SYNC`) and time sync (`CHATTER_PM_TIME`): `f =
//! [bytes, package_type (5/6, or 3/4), 0, ...]`.
//!
//! Root uplink (`CHATTER_ROOT_UPLINK`): the root's UDP datagram to the WiFi
//! station; `dst` is `telemetry::STATION_ID` (65533). `f = [bytes (UDP
//! payload), origin drone, seq, 0, ...]`.

use swarm_brain::{BROADCAST, Frame, Message, NO_SLOT, Payload};

pub const CHATTER_STRIDE: usize = 12;

pub const CHATTER_HEARTBEAT: f32 = 0.0;
pub const CHATTER_INTENT: f32 = 1.0;
pub const CHATTER_MISSION: f32 = 2.0;
pub const CHATTER_TLM: f32 = 3.0;
pub const CHATTER_PM_SYNC: f32 = 4.0;
pub const CHATTER_PM_TIME: f32 = 5.0;
pub const CHATTER_ROOT_UPLINK: f32 = 6.0;
pub const CHATTER_UNDECODABLE: f32 = 255.0;

/// Field offsets within a record.
pub const T_MS: usize = 0;
pub const SRC: usize = 1;
pub const DST: usize = 2;
pub const KIND: usize = 3;
pub const F0: usize = 4;

fn id_or_none(id: u16) -> f32 {
    if id == BROADCAST { -1.0 } else { f32::from(id) }
}

/// `0` means "none" for slot fields on the wire.
fn slot_or_none(slot: u8) -> f32 {
    if slot == NO_SLOT { -1.0 } else { f32::from(slot) }
}

/// Appends a record that is not a brain frame (see the telemetry kinds).
pub(crate) fn record_raw(out: &mut Vec<f32>, t_ms: u32, src: u16, dst: u16, kind: f32, f: [f32; 8]) {
    out.extend_from_slice(&[t_ms as f32, f32::from(src), id_or_none(dst), kind]);
    out.extend_from_slice(&f);
}

/// Appends one decoded record for `frame` to `out`.
pub(crate) fn record(out: &mut Vec<f32>, t_ms: u32, frame: &Frame) {
    let t = t_ms as f32;
    let Some(msg) = Message::decode(frame) else {
        out.extend_from_slice(&[t, frame.src().into(), id_or_none(frame.dst()), CHATTER_UNDECODABLE]);
        out.extend_from_slice(&[0.0; 8]);
        return;
    };
    let (src, dst) = (f32::from(msg.src), id_or_none(msg.dst));
    match msg.payload {
        Payload::Heartbeat(b) => out.extend_from_slice(&[
            t,
            src,
            dst,
            CHATTER_HEARTBEAT,
            f32::from(b.hops),
            id_or_none(b.parent),
            f32::from(b.role as u8),
            f32::from(b.sink),
            slot_or_none(b.slot),
            f32::from(b.flags),
            slot_or_none(b.need),
            f32::from(b.intent_seq),
        ]),
        Payload::Intent { seq, intent, target } => out.extend_from_slice(&[
            t,
            src,
            dst,
            CHATTER_INTENT,
            f32::from(seq),
            f32::from(intent as u8),
            target.x,
            target.z,
            f32::NAN,
            f32::NAN,
            0.0,
            0.0,
        ]),
        Payload::Mission { seq, intent, target, origin } => out.extend_from_slice(&[
            t,
            src,
            dst,
            CHATTER_MISSION,
            f32::from(seq),
            f32::from(intent as u8),
            target.x,
            target.z,
            origin.x,
            origin.z,
            0.0,
            0.0,
        ]),
    }
}
