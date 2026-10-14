//! What the real drones also send: JSON telemetry over painlessMesh to a
//! bridge (the base station), which republishes it to the ground station.
//!
//! Each drone builds the real firmware's frame (`drone_mavlink_wifi.ino`,
//! `maybeBroadcastTelem` / `appendSlowStatus`) every telemetry interval on
//! a phase-locked schedule: a FAST frame (~140 B) and, every
//! [`SLOW_INTERVAL_MS`]` + node_id % 97` ms, the SLOW status keys appended to
//! it (~280 B more). The frame becomes the `msg` of a painlessMesh package
//! (see [`crate::painless`]), crosses the tree hop by hop as TCP segments
//! (retransmitted when lost, so the channel's losses show up as stalls,
//! full send buffers and refused frames), and leaves the root as one UDP
//! datagram `"drone/<host>/data\n<json>"` to the WiFi station on the same
//! channel (not retransmitted: refused or lost, it is gone). Every hop
//! costs shared airtime (see [`crate::channel`]).
//!
//! The ground station sees only what arrives: `"s"` gaps are loss, `"t"`
//! gives latency (queueing and retransmission included), and the budget
//! rule runs on that measured loss.
//!
//! Frame fields come from the sim's truth: x/z metres around
//! [`ORIGIN_LAT`]/[`ORIGIN_LON`] (+x east, -z north), `amsl` = height +
//! [`AMSL_DATUM_M`], `a` above the take-off point, speed / heading / NED
//! velocity from motion, armed while airborne, PX4 modes from what it is
//! doing (LOITER hovering, MISSION moving, LAND descending low), current and
//! battery from a hover-plus-speed draw on a 4S pack, `wifiRssi`/`ap` from
//! its painlessMesh parent link, `tf` = its packages the mesh refused.
//! Values the sim has no model for (`cyc`, `dl`/`rb`, RTCM and GNSS link
//! counters) are plausible constants or clocks.
//!
//! Variants: [`Variant::XiaoS3`] (`GNSS_UART_ENABLED 0`) and
//! [`Variant::C6Rtk`] (UM982 GNSS keys on); [`Variant::Mixed`] gives odd ids
//! the C6. The base station is the bridge and sends no drone telemetry.
//!
//! Rate modes ([`RateMode`]): a fixed rate; the backend's budget rule
//! (`Hz = clamp(budget_pps / online, 5, 25)` with AIMD on the fleet-median
//! loss, see [`BackendBudget`]); or each brain picks its own interval
//! (`Actuation::telemetry_interval_ms`, Phase 5).
//!
//! # Stats layout (shared with `web/src/telemetry.ts`; keep in sync)
//!
//! [`crate::Sim::telemetry_stats`]: [`TLM_STRIDE`] floats per drone, in id
//! order. Rates are over a 2 s window unless noted.
//!
//! | off | field                | meaning                                                         |
//! |-----|----------------------|-----------------------------------------------------------------|
//! | 0   | `id`                 | drone id                                                        |
//! | 1   | `requested_hz`       | rate the drone is sending at now (fixed / budget share / brain) |
//! | 2   | `delivered_hz`       | its frames reaching the ground station, by `"t"`, 0.75 s late   |
//! | 3   | `loss_pct`           | from gaps in `"s"` seen at the ground station, 10 s window      |
//! | 4   | `latency_ms`         | mean `"t"` → ground-station latency (queues, retransmits)      |
//! | 5   | `tx_bytes_per_s`     | its own packages' share of the mesh segments it sends (MPDU: JSON + TCP/IP + LLC + MAC), MAC retries and TCP retransmissions included |
//! | 6   | `relay_bytes_per_s`  | others' packages' share of the segments it relays, same basis (root: its uplink datagram MPDUs, both crossings) |
//! | 7   | `airtime_frac`       | channel time it spends transmitting (everything it sends)       |
//! | 8   | `busy_frac`          | channel busy as it hears it, external included                 |
//! | 9   | `queue_drops_per_s`  | refused by its WiFi TX queue + its full mesh send buffers       |
//! | 10  | `tree_parent`        | painlessMesh parent, -1 none (root or disconnected)             |
//! | 11  | `tree_hops`          | hops to the root, -1 not connected                              |
//! | 12  | `last_frame_bytes`   | last telemetry JSON payload, bytes                              |
//! | 13  | `last_package_bytes` | last painlessMesh package text incl. `'\0'`, bytes              |
//! | 14  | `brain_bytes_per_s`  | brain (ESP-NOW) frame MPDUs incl. ESP-NOW + MAC overhead (+ MAC retries of unicasts) |
//! | 15  | `variant`            | 0 XIAO S3, 1 C6 RTK                                             |
//! | 16  | `operator_hz`        | rate the operator asked for (0 when telemetry is off)          |
//! | 17  | `tx_all_bytes_per_s` | everything it transmits: its packages, relayed packages, NodeSync / TimeSync, TCP ACKs, 802.11 ACKs it sends, beacons, brain frames, uplink, retries |
//! | 18  | `ack_bytes_per_s`    | ACKs it sends: 802.11 ACKs (14 B) + TCP ACK MPDUs (78 B) and their retries |
//! | 19  | `parent_rate_mbps`   | PHY rate of its painlessMesh link to its parent (Minstrel-like choice), 0 none / channel off |
//!
//! [`crate::Sim::channel_summary`]: [`CHANNEL_LEN`] floats, fleet-wide.
//!
//! | off | field              | meaning                                                  |
//! |-----|--------------------|----------------------------------------------------------|
//! | 0   | `pps_on_air`       | frames on air per second except beacons (= 24 − beacons) |
//! | 1   | `busy_frac_mean`   | mean busy fraction over live nodes                       |
//! | 2   | `busy_frac_max`    | worst node                                               |
//! | 3   | `external_busy`    | the slider                                               |
//! | 4   | `gcs_pps`          | telemetry datagrams reaching the ground station / s      |
//! | 5   | `gcs_bytes_per_s`  | their bytes / s (UDP payload)                            |
//! | 6   | `budget_pps`       | backend budget (AIMD), meaningful in `BackendBudget`     |
//! | 7   | `mode`             | 0 fixed, 1 backend budget, 2 brain                       |
//! | 8   | `channel_on`       | 1 if the channel model is on                             |
//! | 9   | `telemetry_on`     | 1 if telemetry is on                                     |
//! | 10  | `median_loss_pct`  | fleet-median `loss_pct` over online drones               |
//! | 11  | `bridge_up`        | 1 if the root (base) is alive                            |
//! | 12  | `air_brain`        | channel time fraction used by brain frames               |
//! | 13  | `air_mesh`         | ... by mesh data, ACKs and mesh control                  |
//! | 14  | `air_uplink`       | ... by the root's uplink                                 |
//! | 15  | `online`           | drones heard by the ground station in the last 10 s     |
//! | 16  | `bytes_on_air`     | all MPDU bytes on air / s (= 17 + 18 + 19 + 20 + 21 + 22) |
//! | 17  | `bytes_brain`      | ESP-NOW brain frames                                      |
//! | 18  | `bytes_tlm_data`   | mesh segments carrying telemetry, every hop               |
//! | 19  | `bytes_ack`        | 802.11 ACKs + TCP ACKs                                    |
//! | 20  | `bytes_mesh_control` | NodeSync + TimeSync segments                            |
//! | 21  | `bytes_uplink`     | root → station datagrams (both crossings with `gcs_on_wifi`) |
//! | 22  | `bytes_beacon`     | SoftAP beacons                                            |
//! | 23  | `bytes_retry`      | the part of 16 that is retransmissions (MAC + TCP)        |
//! | 24  | `frames_on_air`    | all frames / s: data, ACKs, beacons, retries, brain       |
//! | 25  | `link_quality_mean`| mean `quality` over the snapshot's links                  |
//! | 26  | `links_degraded`   | snapshot links with `quality` < 0.5                       |
//! | 27  | `bytes_mesh_control_60s` | 20 averaged over 60 s (NodeSync / TimeSync run once a minute per connection, so the 2 s figure is usually 0; display this one) |
//!
//! Bytes are MPDU bytes (MAC header + FCS + LLC/SNAP + IP/TCP/UDP + payload;
//! whole ESP-NOW / beacon / ACK frames), every transmission counted,
//! from the channel's byte ledger ([`crate::airlog`]). Every byte is booked
//! to the node that sends it, so the per-drone `tx_all_bytes_per_s` sum to
//! `bytes_on_air`; the WiFi router / ground station is not a drone, so its
//! uplink crossing and 802.11 ACKs are booked to the root (the base site).
//!
//! Both are zero-filled (except `id` and the flags) while their models are
//! off, and are rebuilt every step while the channel or telemetry is on.

use core::fmt::Write as _;

use swarm_brain::Vec3;

use crate::channel::{self, AirFrame, AirKind, Channel, Dst, NodeRadio};
use crate::airlog::ByteKind;
use crate::chatter;
use crate::painless::{self, MeshEvent, MeshTx, Package, PainlessMesh};
use crate::window::{BySent, Long, SENT_LAG, Short};

pub const TLM_STRIDE: usize = 20;
/// Field offsets within a [`TLM_STRIDE`] record.
pub mod tlm {
    pub const ID: usize = 0;
    pub const REQUESTED_HZ: usize = 1;
    pub const DELIVERED_HZ: usize = 2;
    pub const LOSS_PCT: usize = 3;
    pub const LATENCY_MS: usize = 4;
    pub const TX_BYTES_PER_S: usize = 5;
    pub const RELAY_BYTES_PER_S: usize = 6;
    pub const AIRTIME_FRAC: usize = 7;
    pub const BUSY_FRAC: usize = 8;
    pub const QUEUE_DROPS_PER_S: usize = 9;
    pub const TREE_PARENT: usize = 10;
    pub const TREE_HOPS: usize = 11;
    pub const LAST_FRAME_BYTES: usize = 12;
    pub const LAST_PACKAGE_BYTES: usize = 13;
    pub const BRAIN_BYTES_PER_S: usize = 14;
    pub const VARIANT: usize = 15;
    pub const OPERATOR_HZ: usize = 16;
    pub const TX_ALL_BYTES_PER_S: usize = 17;
    pub const ACK_BYTES_PER_S: usize = 18;
    pub const PARENT_RATE_MBPS: usize = 19;
}

pub const CHANNEL_LEN: usize = 28;
/// Field offsets within [`crate::Sim::channel_summary`].
pub mod chan {
    pub const PPS_ON_AIR: usize = 0;
    pub const BUSY_MEAN: usize = 1;
    pub const BUSY_MAX: usize = 2;
    pub const EXTERNAL_BUSY: usize = 3;
    pub const GCS_PPS: usize = 4;
    pub const GCS_BYTES_PER_S: usize = 5;
    pub const BUDGET_PPS: usize = 6;
    pub const MODE: usize = 7;
    pub const CHANNEL_ON: usize = 8;
    pub const TELEMETRY_ON: usize = 9;
    pub const MEDIAN_LOSS_PCT: usize = 10;
    pub const BRIDGE_UP: usize = 11;
    pub const AIR_BRAIN: usize = 12;
    pub const AIR_MESH: usize = 13;
    pub const AIR_UPLINK: usize = 14;
    pub const ONLINE: usize = 15;
    pub const BYTES_ON_AIR: usize = 16;
    pub const BYTES_BRAIN: usize = 17;
    pub const BYTES_TLM_DATA: usize = 18;
    pub const BYTES_ACK: usize = 19;
    pub const BYTES_MESH_CONTROL: usize = 20;
    pub const BYTES_UPLINK: usize = 21;
    pub const BYTES_BEACON: usize = 22;
    pub const BYTES_RETRY: usize = 23;
    pub const FRAMES_ON_AIR: usize = 24;
    pub const LINK_QUALITY_MEAN: usize = 25;
    pub const LINKS_DEGRADED: usize = 26;
    pub const BYTES_MESH_CONTROL_60S: usize = 27;
}

/// `HOSTNAME_SUFFIX` default of the drone firmware.
pub const HOSTNAME_SUFFIX: &str = "-d-21";
/// Pseudo node id of the WiFi station (ground-station side) in chatter.
pub const STATION_ID: u16 = 0xFFFD;

/// SLOW keys ride along every this many ms, plus `node_id % 97`
/// (`gNodeId % 97`, `drone_mavlink_wifi.ino`).
pub const SLOW_INTERVAL_MS: u32 = 500;
/// Backend budget rule (UDP_VS_MQTT.md).
pub const BUDGET_PPS: f32 = 60.0;
pub const BUDGET_FLOOR_PPS: f32 = 20.0;
pub const MIN_HZ: f32 = 5.0;
pub const MAX_HZ: f32 = 25.0;
/// Fixed / requested rates are clamped to this.
pub const HZ_LIMIT: f32 = 50.0;
/// lwIP TCP maximum segment size on the ESP32.
pub const TCP_MSS: u32 = painless::TCP_MSS;

/// Map origin for lat/lon (sim units are metres; +x east, -z north).
pub const ORIGIN_LAT: f64 = 47.4979;
pub const ORIGIN_LON: f64 = 19.0402;
const M_PER_DEG_LAT: f64 = 111_320.0;
/// Sim height 0 is this many metres above mean sea level.
pub const AMSL_DATUM_M: f32 = 105.0;
/// Pack capacity behind the voltage model (4S LiPo).
const PACK_MAH: f32 = 5_200.0;
/// Above this height over ground a drone counts as flying (and armed).
const AIRBORNE_AGL: f32 = 2.0;

/// PX4 `custom_mode` values (`main << 16 | sub << 24`) the firmware forwards
/// as `"mode"`, with their `"modeName"`.
const PX4_POSCTL: (u32, &str) = (3 << 16, "POSCTL");
const PX4_LOITER: (u32, &str) = ((4 << 16) | (3 << 24), "AUTO.LOITER");
const PX4_MISSION: (u32, &str) = ((4 << 16) | (4 << 24), "AUTO.MISSION");
const PX4_LAND: (u32, &str) = ((4 << 16) | (6 << 24), "AUTO.LAND");

/// Which node firmware a drone runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Variant {
    /// Seeed XIAO ESP32-S3, no direct GNSS UART.
    #[default]
    XiaoS3,
    /// ESP32-C6 with a UM982 RTK receiver on its UART.
    C6Rtk,
    /// Fleet mix: odd ids C6 RTK, even ids XIAO S3.
    Mixed,
}

impl Variant {
    /// 0 XiaoS3, 1 C6Rtk, 2 Mixed.
    pub fn from_u32(v: u32) -> Option<Self> {
        [Self::XiaoS3, Self::C6Rtk, Self::Mixed].get(v as usize).copied()
    }

    /// The concrete firmware drone `id` runs (never `Mixed`).
    pub fn for_drone(self, id: u16) -> Self {
        match self {
            Self::Mixed if id % 2 == 1 => Self::C6Rtk,
            Self::Mixed => Self::XiaoS3,
            v => v,
        }
    }

    fn has_gnss(self) -> bool {
        self == Self::C6Rtk
    }
}

/// How each drone's telemetry rate is chosen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RateMode {
    /// Every drone at this many Hz.
    Fixed(f32),
    /// The ground station's budget rule with AIMD on measured loss.
    BackendBudget,
    /// Each brain sets its own interval (`Actuation::telemetry_interval_ms`).
    Brain,
}

impl RateMode {
    /// 0 fixed, 1 backend budget, 2 brain.
    pub fn code(self) -> u32 {
        match self {
            Self::Fixed(_) => 0,
            Self::BackendBudget => 1,
            Self::Brain => 2,
        }
    }

    /// From a mode code; `hz` is the fixed rate for code 0.
    pub fn from_code(code: u32, hz: f32) -> Option<Self> {
        match code {
            0 => Some(Self::Fixed(clamp_hz(hz))),
            1 => Some(Self::BackendBudget),
            2 => Some(Self::Brain),
            _ => None,
        }
    }
}

/// How a drone addresses its telemetry package.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Routing {
    /// `mesh.sendSingle(root, ...)`: up the tree only.
    #[default]
    Single,
    /// `mesh.sendBroadcast(...)` (the old node): flooded over every link.
    Broadcast,
}

impl Routing {
    /// 0 single, 1 broadcast.
    pub fn from_u32(v: u32) -> Option<Self> {
        [Self::Single, Self::Broadcast].get(v as usize).copied()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TelemetryConfig {
    /// Model telemetry at all. Off by default.
    pub enabled: bool,
    pub variant: Variant,
    pub rate: RateMode,
    pub routing: Routing,
    /// What the operator asks for, Hz: the brain's `telemetry_requested_ms`
    /// in [`RateMode::Brain`] and the cap in [`RateMode::BackendBudget`].
    pub requested_hz: f32,
    /// Log every telemetry package to chatter instead of at most one per
    /// drone per second.
    pub chatter_every_frame: bool,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            variant: Variant::XiaoS3,
            rate: RateMode::Fixed(MAX_HZ),
            routing: Routing::Single,
            requested_hz: MAX_HZ,
            chatter_every_frame: false,
        }
    }
}

fn clamp_hz(hz: f32) -> f32 {
    if hz.is_finite() { hz.clamp(0.1, HZ_LIMIT) } else { MAX_HZ }
}

fn sanitize(mut cfg: TelemetryConfig) -> TelemetryConfig {
    cfg.requested_hz = clamp_hz(cfg.requested_hz);
    if let RateMode::Fixed(hz) = cfg.rate {
        cfg.rate = RateMode::Fixed(clamp_hz(hz));
    }
    cfg
}

/// Everything one frame reports: the firmware's view of its drone.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TelemetryState {
    /// `String(gNodeId)`: the 10-digit painlessMesh node id.
    pub n: u32,
    pub seq: u32,
    pub t_ms: u32,
    pub fix_type: u8,
    pub ekf: bool,
    pub fix: bool,
    pub sats: u8,
    pub lat: f64,
    pub lon: f64,
    pub rel_alt: f32,
    pub amsl: f32,
    pub speed_kmh: f32,
    pub heading: f32,
    pub vn: f32,
    pub ve: f32,
    pub vd: f32,
    // Direct GNSS (C6 RTK).
    pub gnss_fresh: bool,
    pub gnss_quality: u8,
    pub gnss_tod_ms: u32,
    pub gnss_lat: f64,
    pub gnss_lon: f64,
    pub gnss_alt: f32,
    // SLOW keys.
    pub wifi_rssi: i8,
    pub ap: [u8; 2],
    pub armed: bool,
    pub mode_flags: u32,
    /// arm, rth, poshold, wp, land, failsafe.
    pub modes: [bool; 6],
    pub mode_name: &'static str,
    pub cyc: u32,
    pub volts: f32,
    pub amps: f32,
    pub mah: u32,
    pub eph: f32,
    pub epv: f32,
    pub hdop: f32,
    pub flight_s: u32,
    pub dl: u32,
    pub rb: u32,
    pub tf: u32,
    pub rk: u32,
    /// Seconds since the last RTCM datagram, `None` before the first.
    pub ra: Option<u32>,
    pub ghz: f32,
    pub guhz: f32,
    pub gsa: u8,
    pub geph: f32,
    pub gag: u32,
    pub gok: bool,
    pub gby: u32,
    pub gsn: u32,
    pub gbc: u32,
    pub gbd: u32,
    pub grs: i32,
    pub rtp: u8,
}

/// Clears `out` and writes the firmware's telemetry JSON for `st`, with the
/// SLOW keys if `slow`. `variant` must be concrete (see [`Variant::for_drone`]).
/// Key order and decimals match `maybeBroadcastTelem` / `appendSlowStatus`.
pub fn build_frame(out: &mut String, st: &TelemetryState, variant: Variant, slow: bool) {
    out.clear();
    let b = |v: bool| u8::from(v);
    let _ = write!(
        out,
        "{{\"type\":\"drone\",\"n\":{},\"s\":{},\"t\":{},\"ft\":{},\"ekf\":{}",
        st.n,
        st.seq,
        st.t_ms,
        st.fix_type,
        b(st.ekf)
    );
    let gnss = variant.has_gnss();
    if gnss && st.gnss_fresh {
        let _ = write!(out, ",\"gq\":{},\"gtm\":{}", st.gnss_quality, st.gnss_tod_ms);
        if st.gnss_quality != 0 {
            let _ = write!(out, ",\"gla\":{:.7},\"glo\":{:.7},\"gal\":{:.2}", st.gnss_lat, st.gnss_lon, st.gnss_alt);
        }
    }
    if st.fix {
        let _ = write!(
            out,
            ",\"f\":1,\"sa\":{},\"la\":{:.6},\"lo\":{:.6},\"a\":{:.1},\"amsl\":{:.1},\"sp\":{:.1},\"h\":{:.1},\
             \"vn\":{:.2},\"ve\":{:.2},\"vd\":{:.2}",
            st.sats, st.lat, st.lon, st.rel_alt, st.amsl, st.speed_kmh, st.heading, st.vn, st.ve, st.vd
        );
    } else {
        let _ = write!(out, ",\"f\":0,\"sa\":{}", st.sats);
    }
    if slow {
        let m = st.modes;
        let _ = write!(
            out,
            ",\"wifiRssi\":{},\"ap\":\"{:02x}:{:02x}\",\"armed\":{},\"mode\":{},\"modes\":{{\"arm\":{},\"rth\":{},\
             \"poshold\":{},\"wp\":{},\"land\":{},\"failsafe\":{}}},\"modeName\":\"{}\",\"cyc\":{},\"v\":{:.2},\
             \"i\":{:.2},\"mah\":{}",
            st.wifi_rssi,
            st.ap[0],
            st.ap[1],
            b(st.armed),
            st.mode_flags,
            b(m[0]),
            b(m[1]),
            b(m[2]),
            b(m[3]),
            b(m[4]),
            b(m[5]),
            st.mode_name,
            st.cyc,
            st.volts,
            st.amps,
            st.mah
        );
        if st.fix {
            let _ = write!(out, ",\"eph\":{:.2},\"epv\":{:.2},\"hd\":{:.1}", st.eph, st.epv, st.hdop);
        }
        let _ = write!(out, ",\"flight_s\":{},\"dl\":{},\"rb\":{},\"tf\":{},\"rk\":{}", st.flight_s, st.dl, st.rb, st.tf, st.rk);
        if let Some(ra) = st.ra {
            let _ = write!(out, ",\"ra\":{ra}");
        }
        let _ = write!(out, ",\"ghz\":{:.1}", st.ghz);
        if gnss {
            let _ = write!(
                out,
                ",\"guhz\":{:.1},\"gsa\":{},\"geph\":{:.3},\"gag\":{},\"gok\":{},\"gby\":{},\"gsn\":{},\"gbc\":{},\
                 \"gbd\":{},\"grs\":{}",
                st.guhz,
                st.gsa,
                st.geph,
                st.gag,
                b(st.gok),
                st.gby,
                st.gsn,
                st.gbc,
                st.gbd,
                st.grs
            );
        }
        let _ = write!(out, ",\"rtp\":{}", st.rtp);
    }
    out.push('}');
}

/// The backend's telemetry budget: `Hz = clamp(budget / online, 5, 25)`,
/// with AIMD on the fleet-median loss: while it is above 10 %, cut the
/// budget ×0.8 every 10 s (floor 20 pps); after 30 s below 5 %, add 5 pps
/// every 30 s (up to the initial 60 pps).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BackendBudget {
    pub budget_pps: f32,
    last_cut_ms: Option<u32>,
    good_since_ms: Option<u32>,
    last_add_ms: Option<u32>,
}

impl Default for BackendBudget {
    fn default() -> Self {
        Self { budget_pps: BUDGET_PPS, last_cut_ms: None, good_since_ms: None, last_add_ms: None }
    }
}

impl BackendBudget {
    /// Feeds the current fleet-median loss (percent).
    pub fn update(&mut self, now_ms: u32, median_loss_pct: f32) {
        let since = |t: Option<u32>, ms: u32| t.is_none_or(|t| now_ms.wrapping_sub(t) >= ms);
        if median_loss_pct > 10.0 {
            self.good_since_ms = None;
            self.last_add_ms = None;
            if since(self.last_cut_ms, 10_000) {
                self.budget_pps = (self.budget_pps * 0.8).max(BUDGET_FLOOR_PPS);
                self.last_cut_ms = Some(now_ms);
            }
        } else if median_loss_pct < 5.0 {
            let good = *self.good_since_ms.get_or_insert(now_ms);
            if now_ms.wrapping_sub(good) >= 30_000 && since(self.last_add_ms, 30_000) {
                self.budget_pps = (self.budget_pps + 5.0).min(BUDGET_PPS);
                self.last_add_ms = Some(now_ms);
            }
        } else {
            self.good_since_ms = None;
        }
    }

    /// Per-node rate for `online` nodes, at most `cap_hz`.
    pub fn hz(&self, online: usize, cap_hz: f32) -> f32 {
        (self.budget_pps / online.max(1) as f32).clamp(MIN_HZ, MAX_HZ).min(cap_hz)
    }
}


/// What the world tells telemetry about a drone each step.
#[derive(Clone, Copy, Debug, Default)]
pub struct DroneView {
    pub pos: Vec3,
    /// Velocity, units/s.
    pub vel: Vec3,
    pub alive: bool,
    /// Height above ground.
    pub agl: f32,
    /// The brain's `Actuation::telemetry_interval_ms` (0 = none).
    pub brain_interval_ms: u16,
}

#[derive(Clone, Debug)]
struct TlmDrone {
    variant: Variant,
    next_ms: Option<u32>,
    last_slow_ms: Option<u32>,
    last_chatter_ms: Option<u32>,
    seq: u32,
    hz: f32,
    package: String,
    last_frame_bytes: u32,
    last_package_bytes: u32,
    armed_at_ms: Option<u32>,
    /// Ground height under the take-off point ("home"), for `"a"`.
    home_y: Option<f32>,
    mah: f32,
    amps: f32,
    tx_fails: u32,
    last_uplink_chatter_ms: Option<u32>,
    /// Ground-station side: last `s` seen from this drone.
    gcs_seq: Option<u32>,
    /// Frames that reached the ground station, by their `"t"`.
    delivered: BySent,
    received: Long,
    gaps: Long,
    lat_sum: Short,
    lat_n: Short,
    queue_drops: Short,
    /// Packages the mesh refused at this node since the brain last read it
    /// (`Sensors::tx_dropped`).
    refused: u16,
}

impl TlmDrone {
    fn new() -> Self {
        Self {
            variant: Variant::XiaoS3,
            next_ms: None,
            last_slow_ms: None,
            last_chatter_ms: None,
            seq: 0,
            hz: 0.0,
            package: String::new(),
            last_frame_bytes: 0,
            last_package_bytes: 0,
            armed_at_ms: None,
            home_y: None,
            mah: 0.0,
            amps: 0.0,
            tx_fails: 0,
            last_uplink_chatter_ms: None,
            gcs_seq: None,
            delivered: BySent::new(),
            received: Long::new(),
            gaps: Long::new(),
            lat_sum: Short::new(),
            lat_n: Short::new(),
            queue_drops: Short::new(),
            refused: 0,
        }
    }

    /// Arming, home, current draw and the battery, from how it flies.
    fn fly(&mut self, now_ms: u32, dt_ms: u32, v: &DroneView) {
        let ground = v.pos.y - v.agl;
        let armed = v.agl > AIRBORNE_AGL;
        if armed {
            self.armed_at_ms.get_or_insert(now_ms);
            self.home_y.get_or_insert(ground);
        } else {
            self.armed_at_ms = None;
            self.home_y = Some(ground);
        }
        let speed = (v.vel.x * v.vel.x + v.vel.z * v.vel.z).sqrt();
        // Hover ~9 A on a 4S 5" class quad, more to go fast or to climb.
        self.amps = if armed { 9.0 + 0.45 * speed + 2.5 * v.vel.y.max(0.0) } else { 0.35 };
        self.mah += self.amps * dt_ms as f32 / 3_600.0;
    }
}

/// Telemetry runtime: schedules, frames, delivery and stats.
#[derive(Clone, Debug)]
pub struct Telemetry {
    cfg: TelemetryConfig,
    root: u16,
    budget: BackendBudget,
    drones: Vec<TlmDrone>,
    /// Scratch: the frame being built.
    frame: String,
    alive: Vec<bool>,
    mesh_tx: Vec<MeshTx>,
    events: Vec<MeshEvent>,
    median_scratch: Vec<f32>,
    gcs_pps: Short,
    gcs_bytes: Short,
    median_loss_pct: f32,
    online: usize,
    link_summary: (f32, f32),
    stats: Vec<f32>,
    summary: Vec<f32>,
}

impl Telemetry {
    /// Packages the mesh refused at node `i` since the last call (send
    /// returned false, or a queue drop there); feeds `Sensors::tx_dropped`.
    pub fn take_refused(&mut self, i: usize) -> u16 {
        self.drones.get_mut(i).map_or(0, |d| core::mem::take(&mut d.refused))
    }

    pub fn new(cfg: TelemetryConfig, nodes: usize, root: u16, seed: u64) -> Self {
        let _ = seed; // telemetry draws nothing; the channel owns the loss draws
        let mut t = Self {
            cfg: sanitize(cfg),
            root,
            budget: BackendBudget::default(),
            drones: (0..nodes).map(|_| TlmDrone::new()).collect(),
            frame: String::with_capacity(768),
            alive: Vec::with_capacity(nodes),
            mesh_tx: Vec::new(),
            events: Vec::with_capacity(256),
            median_scratch: Vec::with_capacity(nodes),
            gcs_pps: Short::new(),
            gcs_bytes: Short::new(),
            median_loss_pct: 0.0,
            online: 0,
            link_summary: (0.0, 0.0),
            stats: vec![0.0; nodes * TLM_STRIDE],
            summary: vec![0.0; CHANNEL_LEN],
        };
        t.apply_variants();
        for (i, row) in t.stats.as_chunks_mut::<TLM_STRIDE>().0.iter_mut().enumerate() {
            row[tlm::ID] = i as f32;
        }
        t
    }

    pub fn config(&self) -> &TelemetryConfig {
        &self.cfg
    }

    pub fn enabled(&self) -> bool {
        self.cfg.enabled
    }

    /// Replaces the configuration. Turning telemetry on restarts every
    /// drone's schedule; turning it off forgets every drone's state and
    /// the ground station's measurements, and zero-fills the stats.
    pub fn set_config(&mut self, cfg: TelemetryConfig) {
        let was = self.cfg.enabled;
        self.cfg = sanitize(cfg);
        if was && !self.cfg.enabled {
            for d in &mut self.drones {
                *d = TlmDrone::new();
            }
            self.budget = BackendBudget::default();
            self.gcs_pps = Short::new();
            self.gcs_bytes = Short::new();
            self.median_loss_pct = 0.0;
            self.online = 0;
            for (i, row) in self.stats.as_chunks_mut::<TLM_STRIDE>().0.iter_mut().enumerate() {
                row.fill(0.0);
                row[tlm::ID] = i as f32;
                row[tlm::TREE_PARENT] = -1.0;
                row[tlm::TREE_HOPS] = -1.0;
            }
            self.summary.fill(0.0);
        }
        self.apply_variants();
        if self.cfg.enabled && !was {
            for d in &mut self.drones {
                d.next_ms = None;
            }
        }
    }

    fn apply_variants(&mut self) {
        for (i, d) in self.drones.iter_mut().enumerate() {
            d.variant = self.cfg.variant.for_drone(i as u16);
        }
    }

    /// The interval the operator asks brains for, ms (0 while off).
    pub fn requested_ms(&self) -> u16 {
        if !self.cfg.enabled {
            return 0;
        }
        (1000.0 / self.cfg.requested_hz).round().clamp(1.0, f32::from(u16::MAX)) as u16
    }

    /// Last painlessMesh package text drone `id` sent ("" if none).
    pub fn sample(&self, id: u16) -> &str {
        self.drones.get(usize::from(id)).map_or("", |d| d.package.as_str())
    }

    pub fn stats(&self) -> &[f32] {
        &self.stats
    }

    pub fn summary(&self) -> &[f32] {
        &self.summary
    }

    pub fn budget(&self) -> &BackendBudget {
        &self.budget
    }

    /// One step of telemetry: tree upkeep, every frame due, transport over
    /// the mesh, the root's uplink and the ground station's measurements.
    /// Called only while telemetry is on; airtime goes to `channel` only
    /// while it is on. `rssi(a, b)` is the signal of `b` heard at `a`, for
    /// the mesh's parent choice and link checks.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        now_ms: u32,
        dt_ms: u32,
        views: &[DroneView],
        mesh: &mut PainlessMesh,
        channel: &mut Channel,
        rssi: &mut dyn FnMut(usize, usize) -> Option<i8>,
        chatter_out: &mut Vec<f32>,
    ) {
        let n = self.drones.len().min(views.len());

        // Tree upkeep; its node sync / time sync packages are queued on the
        // connections and spend airtime in `transport` below.
        self.alive.clear();
        self.alive.extend(views.iter().map(|v| v.alive));
        self.mesh_tx.clear();
        mesh.step(now_ms, dt_ms, &self.alive, rssi, &mut self.mesh_tx);
        record_mesh_tx(chatter_out, now_ms, &self.mesh_tx);

        // Every frame due, handed to the mesh.
        let budget_hz = self.budget.hz(self.online, self.cfg.requested_hz);
        let root = usize::from(self.root);
        for (i, &v) in views.iter().enumerate().take(n) {
            if i == root || !v.alive {
                self.drones[i].hz = 0.0;
                self.drones[i].next_ms = None;
                continue;
            }
            let hz = match self.cfg.rate {
                RateMode::Fixed(hz) => hz,
                RateMode::BackendBudget => budget_hz,
                RateMode::Brain if v.brain_interval_ms > 0 => 1000.0 / f32::from(v.brain_interval_ms),
                RateMode::Brain => self.cfg.requested_hz,
            };
            let hz = clamp_hz(hz);
            let interval = (1000.0 / hz).round().max(1.0) as u32;
            let d = &mut self.drones[i];
            d.hz = hz;
            d.fly(now_ms, dt_ms, &v);

            // Phase-locked like the firmware: each frame is due one period
            // after the previous one was due, so a 50 ms step carries one or
            // two 40 ms frames and the average rate is exact. Falling a full
            // period behind (a huge dt, a rate change) resyncs instead of
            // bursting.
            let mut next = *d.next_ms.get_or_insert(now_ms.wrapping_add((i as u32 * 7) % interval));
            if reached(now_ms, next) && now_ms.wrapping_sub(next) >= interval + dt_ms {
                next = now_ms;
            }
            while reached(now_ms, next) {
                self.send(i, next, now_ms, &v, mesh, chatter_out);
                next = next.wrapping_add(interval);
            }
            self.drones[i].next_ms = Some(next);
        }

        // Transport, then what reached the root goes up to the station.
        self.events.clear();
        self.mesh_tx.clear();
        mesh.transport(now_ms, dt_ms, channel, &mut self.events, &mut self.mesh_tx);
        record_mesh_tx(chatter_out, now_ms, &self.mesh_tx);
        let events = core::mem::take(&mut self.events);
        for e in &events {
            match *e {
                // Bytes on air are counted by the channel's ledger.
                MeshEvent::Hop { .. } => {}
                MeshEvent::Dropped { at, origin } => {
                    if let Some(d) = self.drones.get_mut(usize::from(at)) {
                        d.queue_drops.add(now_ms, 1.0);
                        d.refused = d.refused.saturating_add(1);
                    }
                    if let Some(d) = self.drones.get_mut(usize::from(origin))
                        && origin == at
                    {
                        d.tx_fails += 1;
                    }
                }
                MeshEvent::Arrived(pkg) => self.uplink(now_ms, pkg, channel, chatter_out),
            }
        }
        self.events = events;

        // Ground-station loss estimate and the budget rule.
        self.median_scratch.clear();
        self.online = 0;
        for d in &mut self.drones {
            let rx = d.received.sum(now_ms);
            if rx > 0.0 {
                self.online += 1;
                let gaps = d.gaps.sum(now_ms);
                self.median_scratch.push(100.0 * gaps / (gaps + rx));
            }
        }
        self.median_loss_pct = median(&mut self.median_scratch);
        if self.cfg.rate == RateMode::BackendBudget {
            self.budget.update(now_ms, self.median_loss_pct);
        }
    }

    /// Builds frame `i` stamped `at` and hands its package to the mesh.
    fn send(&mut self, i: usize, at: u32, now_ms: u32, v: &DroneView, mesh: &mut PainlessMesh, chatter_out: &mut Vec<f32>) {
        let id = i as u16;
        let (slow, variant, seq, st) = {
            let d = &mut self.drones[i];
            let slow_every = SLOW_INTERVAL_MS + painless::node_id(id) % 97;
            let slow = d.last_slow_ms.is_none_or(|t| at.wrapping_sub(t) >= slow_every);
            if slow {
                d.last_slow_ms = Some(at);
            }
            let seq = d.seq;
            d.seq = d.seq.wrapping_add(1);
            let st = state_for(id, seq, at, v, d, mesh);
            (slow, d.variant, seq, st)
        };
        build_frame(&mut self.frame, &st, variant, slow);
        let broadcast = self.cfg.routing == Routing::Broadcast;
        let (kind, dest) = if broadcast {
            (painless::BROADCAST, 0)
        } else {
            (painless::SINGLE, painless::node_id(self.root))
        };
        let d = &mut self.drones[i];
        painless::encode_package(&mut d.package, kind, dest, painless::node_id(id), &self.frame);
        d.last_frame_bytes = self.frame.len() as u32;
        d.last_package_bytes = d.package.len() as u32 + 1;
        let pkg = Package {
            kind,
            origin: id,
            seq,
            born_ms: at,
            bytes: d.last_package_bytes,
            datagram: uplink_len(id, variant, d.last_frame_bytes),
            slow,
            ..Package::default()
        };
        if !mesh.send(id, pkg, broadcast) {
            // sendSingle/sendBroadcast returned false: no route or a full buffer.
            d.tx_fails += 1;
            d.queue_drops.add(now_ms, 1.0);
            d.refused = d.refused.saturating_add(1);
        }

        let due = self.cfg.chatter_every_frame || d.last_chatter_ms.is_none_or(|t| now_ms.wrapping_sub(t) >= 1_000);
        if due {
            d.last_chatter_ms = Some(now_ms);
            let dst = if broadcast { swarm_brain::BROADCAST } else { self.root };
            let hops_f = mesh.hops(id).map_or(-1.0, f32::from);
            let f = [d.last_frame_bytes as f32, pkg.bytes as f32, hops_f, seq as f32, f32::from(u8::from(slow)), 0.0, 0.0, 0.0];
            chatter::record_raw(chatter_out, now_ms, id, dst, chatter::CHATTER_TLM, f);
        }
    }

    /// The root republishes a telemetry package that reached it as one UDP
    /// datagram `"drone/<host>/data\n<json>"` to the WiFi station on the same
    /// channel; what gets there is what the ground station measures.
    fn uplink(&mut self, now_ms: u32, pkg: Package, channel: &mut Channel, chatter_out: &mut Vec<f32>) {
        let root = self.root;
        let mut lat_us = pkg.air_us;
        let on = channel.enabled();
        // UDP: refused by a full driver queue or lost after its MAC
        // retries, it is gone.
        if on && !channel.admit(root) {
            return;
        }
        let floor = channel.config().min_rate_mbps;
        let rates = if on { channel.station_rates(root) } else { (floor, floor) };
        let mpdu = channel::QOS_MAC_OVERHEAD + channel::LLC_SNAP + channel::UDP_IP_OVERHEAD + pkg.datagram;
        let f = AirFrame::new(root, AirKind::Uplink, mpdu, rates.0).with_rates(rates);
        // Root → router, then (laptop on the same WiFi) router → laptop.
        let crossings = if channel.config().gcs_on_wifi { 2 } else { 1 };
        for _ in 0..crossings {
            let r = channel.unicast(now_ms, &f, Dst::Station, true);
            lat_us += r.us;
            if !r.delivered {
                return;
            }
        }
        let Some(d) = self.drones.get_mut(usize::from(pkg.origin)) else { return };
        // The ground station: gaps in "s" are loss; latency from "t".
        let seq = pkg.seq;
        match d.gcs_seq {
            Some(last) if seq > last => d.gaps.add(now_ms, (seq - last - 1) as f32),
            Some(last) if seq <= last => return, // duplicate / stale
            _ => {}
        }
        d.gcs_seq = Some(seq);
        d.delivered.add_at(now_ms, pkg.born_ms, 1.0);
        d.received.add(now_ms, 1.0);
        let latency_ms = now_ms.wrapping_sub(pkg.born_ms) as f32 + lat_us / 1_000.0;
        d.lat_sum.add(now_ms, latency_ms);
        d.lat_n.add(now_ms, 1.0);
        self.gcs_pps.add(now_ms, 1.0);
        self.gcs_bytes.add(now_ms, pkg.datagram as f32);
        let due = self.cfg.chatter_every_frame || d.last_uplink_chatter_ms.is_none_or(|t| now_ms.wrapping_sub(t) >= 1_000);
        if due {
            d.last_uplink_chatter_ms = Some(now_ms);
            let f = [pkg.datagram as f32, f32::from(pkg.origin), seq as f32, 0.0, 0.0, 0.0, 0.0, 0.0];
            chatter::record_raw(chatter_out, now_ms, root, STATION_ID, chatter::CHATTER_ROOT_UPLINK, f);
        }
    }

    /// Rebuilds [`Telemetry::stats`] and [`Telemetry::summary`]. Called every
    /// step while the channel or telemetry is on.
    pub fn build_stats(&mut self, now_ms: u32, channel: &Channel, mesh: &PainlessMesh, nodes: &[NodeRadio]) {
        let on = self.cfg.enabled;
        let op_hz = if on { self.cfg.requested_hz } else { 0.0 };
        for (i, (d, row)) in self.drones.iter_mut().zip(self.stats.as_chunks_mut::<TLM_STRIDE>().0).enumerate() {
            let id = i as u16;
            let lat_n = d.lat_n.sum(now_ms);
            let rx = d.received.sum(now_ms);
            let gaps = d.gaps.sum(now_ms);
            row.fill(0.0);
            row[tlm::ID] = i as f32;
            row[tlm::REQUESTED_HZ] = if on { d.hz } else { 0.0 };
            row[tlm::DELIVERED_HZ] = d.delivered.per_s_lagged(now_ms, SENT_LAG);
            row[tlm::LOSS_PCT] = if rx + gaps > 0.0 { 100.0 * gaps / (rx + gaps) } else { 0.0 };
            row[tlm::LATENCY_MS] = if lat_n > 0.0 { d.lat_sum.sum(now_ms) / lat_n } else { 0.0 };
            let air = channel.ledger().node(now_ms, id);
            row[tlm::TX_BYTES_PER_S] = air.own;
            row[tlm::RELAY_BYTES_PER_S] = air.relay + air.uplink;
            row[tlm::AIRTIME_FRAC] = channel.airtime_frac(id);
            row[tlm::BUSY_FRAC] = if channel.enabled() { channel.busy(id) } else { 0.0 };
            row[tlm::QUEUE_DROPS_PER_S] = channel.queue_drops_per_s(id) + d.queue_drops.per_s(now_ms);
            row[tlm::TREE_PARENT] = if on { mesh.parent(id).map_or(-1.0, f32::from) } else { -1.0 };
            row[tlm::TREE_HOPS] = if on { mesh.hops(id).map_or(-1.0, f32::from) } else { -1.0 };
            row[tlm::LAST_FRAME_BYTES] = d.last_frame_bytes as f32;
            row[tlm::LAST_PACKAGE_BYTES] = d.last_package_bytes as f32;
            row[tlm::BRAIN_BYTES_PER_S] = air.brain;
            row[tlm::VARIANT] = f32::from(u8::from(d.variant == Variant::C6Rtk));
            row[tlm::OPERATOR_HZ] = op_hz;
            row[tlm::TX_ALL_BYTES_PER_S] = air.all;
            row[tlm::ACK_BYTES_PER_S] = air.ack;
            row[tlm::PARENT_RATE_MBPS] =
                if on && channel.enabled() { mesh.parent_rate_mbps(id).unwrap_or(0.0) } else { 0.0 };
        }
        let (mean, max) = if channel.enabled() { channel.busy_mean_max(nodes) } else { (0.0, 0.0) };
        let s = &mut self.summary;
        s.fill(0.0);
        s[chan::BUSY_MEAN] = mean;
        s[chan::BUSY_MAX] = max;
        s[chan::EXTERNAL_BUSY] = channel.config().external_busy;
        s[chan::GCS_PPS] = self.gcs_pps.per_s(now_ms);
        s[chan::GCS_BYTES_PER_S] = self.gcs_bytes.per_s(now_ms);
        s[chan::BUDGET_PPS] = self.budget.budget_pps;
        s[chan::MODE] = self.cfg.rate.code() as f32;
        s[chan::CHANNEL_ON] = f32::from(u8::from(channel.enabled()));
        s[chan::TELEMETRY_ON] = f32::from(u8::from(on));
        s[chan::MEDIAN_LOSS_PCT] = self.median_loss_pct;
        s[chan::BRIDGE_UP] = f32::from(u8::from(nodes.get(usize::from(self.root)).is_some_and(|n| n.alive)));
        s[chan::AIR_BRAIN] = channel.kind_frac(AirKind::Brain);
        s[chan::AIR_MESH] =
            channel.kind_frac(AirKind::MeshData) + channel.kind_frac(AirKind::Ack) + channel.kind_frac(AirKind::MeshControl);
        s[chan::AIR_UPLINK] = channel.kind_frac(AirKind::Uplink);
        s[chan::ONLINE] = self.online as f32;
        let air = channel.ledger().fleet(now_ms);
        s[chan::PPS_ON_AIR] = air.frames_no_beacon;
        s[chan::BYTES_ON_AIR] = air.total();
        s[chan::BYTES_BRAIN] = air.kind[ByteKind::Brain as usize];
        s[chan::BYTES_TLM_DATA] = air.kind[ByteKind::TlmData as usize];
        s[chan::BYTES_ACK] = air.kind[ByteKind::Ack as usize];
        s[chan::BYTES_MESH_CONTROL] = air.kind[ByteKind::MeshControl as usize];
        s[chan::BYTES_UPLINK] = air.kind[ByteKind::Uplink as usize];
        s[chan::BYTES_BEACON] = air.kind[ByteKind::Beacon as usize];
        s[chan::BYTES_RETRY] = air.retry;
        s[chan::FRAMES_ON_AIR] = air.frames;
        s[chan::BYTES_MESH_CONTROL_60S] = air.mesh_control_60s;
        s[chan::LINK_QUALITY_MEAN] = self.link_summary.0;
        s[chan::LINKS_DEGRADED] = self.link_summary.1;
    }

    /// Link quality over the snapshot's links (mean, count below 0.5), for
    /// the summary; the world sets it after each snapshot.
    pub fn set_link_summary(&mut self, mean: f32, degraded: usize) {
        self.link_summary = (mean, degraded as f32);
        self.summary[chan::LINK_QUALITY_MEAN] = mean;
        self.summary[chan::LINKS_DEGRADED] = degraded as f32;
    }
}

/// True once `now` is at or past `at` (wrap-safe).
fn reached(now: u32, at: u32) -> bool {
    (now.wrapping_sub(at) as i32) >= 0
}

fn digits(id: u16) -> usize {
    match id {
        0..=9 => 1,
        10..=99 => 2,
        100..=999 => 3,
        1_000..=9_999 => 4,
        _ => 5,
    }
}

fn median(v: &mut [f32]) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_unstable_by(f32::total_cmp);
    let m = v.len() / 2;
    if v.len() % 2 == 1 { v[m] } else { 0.5 * (v[m - 1] + v[m]) }
}

/// Chatter for queued node sync / time sync packages.
fn record_mesh_tx(out: &mut Vec<f32>, now_ms: u32, txs: &[MeshTx]) {
    for tx in txs {
        let kind = if matches!(tx.kind, painless::TIME_SYNC | painless::TIME_DELAY) {
            chatter::CHATTER_PM_TIME
        } else {
            chatter::CHATTER_PM_SYNC
        };
        chatter::record_raw(out, now_ms, tx.src, tx.dst, kind, [tx.bytes as f32, f32::from(tx.kind), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    }
}

/// Appends drone `id`'s hostname (`OTA_HOSTNAME` = `NODE_NUM` +
/// `HOSTNAME_SUFFIX`, default `"-d-21"`, `drone_mavlink_wifi.ino`), e.g.
/// `"07-d-21"`; both builds use the default suffix.
pub fn write_host(out: &mut String, id: u16, variant: Variant) {
    let _ = variant;
    let _ = write!(out, "{id:02}{HOSTNAME_SUFFIX}");
}

/// Writes the root's republished datagram for drone `id`'s `frame`:
/// `"drone/<host>/data\n<json>"`.
pub fn write_uplink(out: &mut String, id: u16, variant: Variant, frame: &str) {
    out.clear();
    out.push_str("drone/");
    write_host(out, id, variant);
    out.push_str("/data\n");
    out.push_str(frame);
}

/// Length of [`write_uplink`]'s datagram without building it.
fn uplink_len(id: u16, variant: Variant, frame_len: u32) -> u32 {
    let id_len = digits(id).max(2) as u32;
    let _ = variant;
    "drone/".len() as u32 + id_len + HOSTNAME_SUFFIX.len() as u32 + "/data\n".len() as u32 + frame_len
}

/// The firmware's view of drone `id` at `t_ms`, from the sim's truth.
fn state_for(id: u16, seq: u32, t_ms: u32, v: &DroneView, d: &TlmDrone, mesh: &PainlessMesh) -> TelemetryState {
    let lat = ORIGIN_LAT + f64::from(-v.pos.z) / M_PER_DEG_LAT;
    let lon = ORIGIN_LON + f64::from(v.pos.x) / (M_PER_DEG_LAT * ORIGIN_LAT.to_radians().cos());
    // `+ 0.0` turns -0.0 into 0.0: Arduino's `String(float)` never prints "-0.00".
    let (vn, ve, vd) = (-v.vel.z + 0.0, v.vel.x + 0.0, -v.vel.y + 0.0);
    let ground_speed = (vn * vn + ve * ve).sqrt();
    let heading = if ground_speed > 0.05 { ve.atan2(vn).to_degrees().rem_euclid(360.0) } else { 0.0 };
    let armed = d.armed_at_ms.is_some();
    let flight_s = d.armed_at_ms.map_or(0, |t| t_ms.wrapping_sub(t) / 1000);
    let rtk = d.variant == Variant::C6Rtk;
    // STA side: the parent's SoftAP (BSSID low octets = low bytes of its
    // node id); 0 dBm and "00:00" while not associated, like `WiFi.RSSI()`.
    let parent = mesh.parent(id).map_or(0, painless::node_id);
    let wifi_rssi = mesh.parent_rssi(id).unwrap_or(0);
    let (mode_flags, mode_name) = if !armed {
        PX4_POSCTL
    } else if vd > 0.5 && v.agl < 8.0 {
        PX4_LAND
    } else if ground_speed > 1.0 {
        PX4_MISSION
    } else {
        PX4_LOITER
    };
    let used = (d.mah / PACK_MAH).min(1.0);
    let volts = (16.7 - 2.9 * used - 0.018 * d.amps).max(13.2);
    let amsl = v.pos.y + AMSL_DATUM_M;
    TelemetryState {
        n: painless::node_id(id),
        seq,
        t_ms,
        fix_type: if rtk { 6 } else { 3 },
        ekf: true,
        fix: true,
        sats: if rtk { 31 } else { 17 },
        lat,
        lon,
        rel_alt: d.home_y.map_or(v.agl, |h| v.pos.y - h),
        amsl,
        speed_kmh: ground_speed * 3.6,
        heading,
        vn,
        ve,
        vd,
        gnss_fresh: rtk,
        gnss_quality: 4,
        gnss_tod_ms: (36_000_000 + t_ms) % 86_400_000,
        gnss_lat: lat,
        gnss_lon: lon,
        gnss_alt: amsl,
        wifi_rssi,
        ap: [(parent >> 8) as u8, parent as u8],
        armed,
        mode_flags,
        modes: [armed, false, mode_flags == PX4_LOITER.0, mode_flags == PX4_MISSION.0, mode_flags == PX4_LAND.0, false],
        mode_name,
        cyc: 4_000 + u32::from(id % 7) * 13,
        volts,
        amps: d.amps,
        mah: d.mah as u32,
        eph: if rtk { 0.02 } else { 0.9 },
        epv: if rtk { 0.03 } else { 1.4 },
        hdop: if rtk { 0.6 } else { 0.8 },
        flight_s,
        dl: 0,
        rb: 0,
        tf: d.tx_fails,
        // RTCM from the base at 1 Hz.
        rk: t_ms / 1_000,
        ra: Some(0),
        ghz: if rtk { 10.0 } else { 5.0 },
        guhz: 10.0,
        gsa: 31,
        geph: 0.014,
        gag: 0,
        gok: true,
        gby: t_ms.wrapping_mul(9) / 10,
        gsn: t_ms / 50,
        gbc: 0,
        gbd: 460_800,
        grs: 1,
        rtp: if rtk { 2 } else { 1 },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A hovering-then-cruising drone ten minutes into a flight, as the
    /// firmware would see it.
    fn state(variant: Variant) -> TelemetryState {
        let d = TlmDrone { variant, armed_at_ms: Some(120_000), home_y: Some(31.4), mah: 812.0, amps: 11.3, ..TlmDrone::new() };
        let v = DroneView {
            pos: Vec3::new(-312.4, 72.2, 118.9),
            vel: Vec3::new(3.0, 0.1, -9.0),
            alive: true,
            agl: 25.3,
            brain_interval_ms: 0,
        };
        let mut mesh = PainlessMesh::new(13, 0);
        mesh.step(0, 50, &[true; 13], &mut |_, _| Some(-61), &mut Vec::new());
        state_for(12, 9_876, 734_567, &v, &d, &mesh)
    }

    /// Top-level keys of a frame, in order (nested `modes` skipped).
    fn keys(s: &str) -> Vec<&str> {
        let mut out = Vec::new();
        let mut depth = 0;
        let b = s.as_bytes();
        let mut i = 0;
        while i < b.len() {
            match b[i] {
                b'{' => depth += 1,
                b'}' => depth -= 1,
                b'"' => {
                    let end = i + 1 + s[i + 1..].find('"').unwrap();
                    if depth == 1 && b.get(end + 1) == Some(&b':') {
                        out.push(&s[i + 1..end]);
                    }
                    i = end;
                }
                _ => {}
            }
            i += 1;
        }
        out
    }

    #[test]
    fn frame_matches_the_firmware_byte_for_byte() {
        let mut s = String::new();
        let st = state(Variant::XiaoS3);
        build_frame(&mut s, &st, Variant::XiaoS3, false);
        assert_eq!(
            s,
            r#"{"type":"drone","n":2200095028,"s":9876,"t":734567,"ft":3,"ekf":1,"f":1,"sa":17,"la":47.496832,"lo":19.036046,"a":40.8,"amsl":177.2,"sp":34.2,"h":18.4,"vn":9.00,"ve":3.00,"vd":-0.10}"#
        );
        build_frame(&mut s, &st, Variant::XiaoS3, true);
        let slow = &s[s.find(",\"wifiRssi\"").unwrap()..];
        assert_eq!(
            slow,
            r#","wifiRssi":-61,"ap":"74:ef","armed":1,"mode":67371008,"modes":{"arm":1,"rth":0,"poshold":0,"wp":1,"land":0,"failsafe":0},"modeName":"AUTO.MISSION","cyc":4065,"v":16.04,"i":11.30,"mah":812,"eph":0.90,"epv":1.40,"hd":0.8,"flight_s":614,"dl":0,"rb":0,"tf":0,"rk":734,"ra":0,"ghz":5.0,"rtp":1}"#
        );
        // Key order of maybeBroadcastTelem + appendSlowStatus (UDP + RTCM build).
        let fast_keys = ["type", "n", "s", "t", "ft", "ekf", "f", "sa", "la", "lo", "a", "amsl", "sp", "h", "vn", "ve", "vd"];
        let slow_keys = [
            "wifiRssi", "ap", "armed", "mode", "modes", "modeName", "cyc", "v", "i", "mah", "eph", "epv", "hd", "flight_s",
            "dl", "rb", "tf", "rk", "ra", "ghz",
        ];
        let mut want: Vec<&str> = fast_keys.into();
        want.extend(slow_keys);
        want.push("rtp");
        assert_eq!(keys(&s), want);

        // C6 RTK: the GNSS keys after "ekf" (gla/glo 7 dp, gal 2 dp) and the
        // UART link keys before "rtp".
        let st = state(Variant::C6Rtk);
        build_frame(&mut s, &st, Variant::C6Rtk, true);
        assert!(s.contains(r#""ekf":1,"gq":4,"gtm":"#), "{s}");
        assert!(s.contains(r#","gla":47.4968319,"glo":19.0360463,"gal":177.20,"f":1,"#), "{s}");
        assert!(s.contains(r#","ghz":10.0,"guhz":10.0,"gsa":31,"geph":0.014,"gag":0,"gok":1,"#), "{s}");
        assert!(s.ends_with(r#","gbc":0,"gbd":460800,"grs":1,"rtp":2}"#), "{s}");
        // No fix: "f":0 and "sa" only, no eph/epv/hd.
        let st = TelemetryState { fix: false, ..state(Variant::XiaoS3) };
        build_frame(&mut s, &st, Variant::XiaoS3, true);
        assert!(s.contains(r#""ekf":1,"f":0,"sa":17,"wifiRssi""#) && !s.contains("\"eph\""), "{s}");
        // Never "-0.00".
        let st = TelemetryState { vn: -0.0 + 0.0, ..state(Variant::XiaoS3) };
        build_frame(&mut s, &st, Variant::XiaoS3, false);
        assert!(!s.contains("-0.00"), "{s}");
    }

    #[test]
    fn frame_sizes_match_the_field() {
        let mut s = String::new();
        // Cruising at 34 km/h (every NED velocity non-zero) the fast frame is
        // a few bytes longer than hovering in place.
        for (variant, fast_range, slow_range) in [(Variant::XiaoS3, 158..=188, 428..=488), (Variant::C6Rtk, 228..=268, 608..=708)] {
            let st = state(variant);
            let hover = TelemetryState { speed_kmh: 0.0, heading: 0.0, vn: 0.0, ve: 0.0, vd: 0.0, ..st };
            build_frame(&mut s, &hover, variant, false);
            let fast = s.len();
            if variant == Variant::XiaoS3 {
                assert!((128..=183).contains(&fast), "hover fast {fast}: {s}");
                build_frame(&mut s, &hover, variant, true);
                assert!((388..=488).contains(&s.len()), "hover with slow keys {}: {s}", s.len());
            }
            build_frame(&mut s, &st, variant, false);
            let fast = s.len();
            assert!(fast_range.contains(&fast), "{variant:?} fast {fast}: {s}");
            build_frame(&mut s, &st, variant, true);
            assert!(slow_range.contains(&s.len()), "{variant:?} with slow keys {}: {s}", s.len());
            assert_eq!(s.contains("\"gla\""), variant == Variant::C6Rtk);
            assert_eq!(s.contains("\"guhz\""), variant == Variant::C6Rtk);
        }
    }

    #[test]
    fn uplink_datagram_shape() {
        let mut s = String::new();
        write_uplink(&mut s, 7, Variant::XiaoS3, "{}");
        assert_eq!(s, "drone/07-d-21/data\n{}");
        assert_eq!(uplink_len(7, Variant::XiaoS3, 2) as usize, s.len());
        write_uplink(&mut s, 123, Variant::C6Rtk, "{\"n\":123}");
        assert_eq!(s, "drone/123-d-21/data\n{\"n\":123}");
        assert_eq!(uplink_len(123, Variant::C6Rtk, 9) as usize, s.len());
    }

    /// A tiny world for the telemetry engine alone: drones `0..n` on a line
    /// 50 m apart (everyone within `range` hears everyone), channel off.
    struct Bench {
        tlm: Telemetry,
        mesh: PainlessMesh,
        channel: Channel,
        views: Vec<DroneView>,
        range: f32,
        now: u32,
        chatter: Vec<f32>,
    }

    impl Bench {
        fn new(n: usize, cfg: TelemetryConfig) -> Self {
            let views = (0..n)
                .map(|i| DroneView {
                    pos: Vec3::new(i as f32 * 50.0, 30.0, 0.0),
                    vel: Vec3::ZERO,
                    alive: true,
                    agl: 20.0,
                    brain_interval_ms: 0,
                })
                .collect();
            Self {
                tlm: Telemetry::new(cfg, n, 0, 1),
                mesh: PainlessMesh::new(n, 0),
                channel: Channel::new(channel::ChannelConfig::default(), n, 1),
                views,
                range: 400.0,
                now: 0,
                chatter: Vec::new(),
            }
        }

        fn step(&mut self) {
            self.now += 50;
            self.chatter.clear();
            let (views, range) = (&self.views, self.range);
            let mut rssi = |a: usize, b: usize| {
                let d = (views[a].pos - views[b].pos).length_sq().sqrt();
                (d <= range).then(|| (-40.0 - d / 10.0) as i8)
            };
            self.tlm.step(self.now, 50, &self.views, &mut self.mesh, &mut self.channel, &mut rssi, &mut self.chatter);
            let nodes: Vec<NodeRadio> = self.views.iter().map(|v| NodeRadio { pos: v.pos, alive: v.alive }).collect();
            self.tlm.build_stats(self.now, &self.channel, &self.mesh, &nodes);
        }

        fn records(&self, kind: f32) -> impl Iterator<Item = &[f32; chatter::CHATTER_STRIDE]> {
            self.chatter.as_chunks::<{ chatter::CHATTER_STRIDE }>().0.iter().filter(move |r| r[chatter::KIND] == kind)
        }

        fn row(&self, id: usize) -> &[f32] {
            &self.tlm.stats()[id * TLM_STRIDE..(id + 1) * TLM_STRIDE]
        }
    }

    fn on(rate: RateMode) -> TelemetryConfig {
        TelemetryConfig { enabled: true, rate, chatter_every_frame: true, ..TelemetryConfig::default() }
    }

    #[test]
    fn phase_locked_25hz_with_two_slow_frames_per_second() {
        let mut b = Bench::new(2, on(RateMode::Fixed(25.0)));
        b.step();
        // Drone 1: slow every 500 + 1 ms, i.e. every 13th 40 ms frame.
        let (mut frames, mut slow, mut t_prev) = (0u32, Vec::new(), None::<f32>);
        for _ in 0..(10_400 / 50) {
            b.step();
            for r in b.records(chatter::CHATTER_TLM) {
                assert_eq!(r[chatter::SRC], 1.0, "the root sends no telemetry");
                frames += 1;
                if r[chatter::F0 + 4] > 0.5 {
                    slow.push(r[chatter::F0 + 3]);
                }
                t_prev = Some(r[chatter::F0 + 3]);
            }
        }
        assert_eq!(frames, 260, "25 Hz over 10.4 s");
        assert_eq!(t_prev, Some(259.0), "seq counts every frame");
        assert_eq!(slow.len(), 20, "two slow frames per 1.04 s");
        assert!(slow.windows(2).all(|w| w[1] - w[0] == 13.0), "{slow:?}");
        let r = b.row(1);
        assert!((r[tlm::DELIVERED_HZ] - 25.0).abs() < 0.5, "{r:?}");
        assert_eq!(r[tlm::LOSS_PCT], 0.0);
        // Upward TCP runs Nagle: frames wait for the parent's delayed ACK
        // (0-250 ms, ~125 ms mean) on top of the 50 ms step.
        assert!((60.0..250.0).contains(&r[tlm::LATENCY_MS]), "{r:?}");
        assert_eq!(r[tlm::TREE_PARENT], 0.0);
        assert_eq!(r[tlm::TREE_HOPS], 1.0);
        // Frames carry the true schedule time: 40 ms apart in "t".
        let s = b.tlm.sample(1);
        assert!(s.contains("\\\"s\\\":259,\\\"t\\\":10417,"), "{s}");
    }

    #[test]
    fn rate_modes() {
        // Fixed.
        let mut b = Bench::new(2, on(RateMode::Fixed(10.0)));
        for _ in 0..60 {
            b.step();
        }
        assert_eq!(b.row(1)[tlm::REQUESTED_HZ], 10.0);
        assert!((b.row(1)[tlm::DELIVERED_HZ] - 10.0).abs() < 0.6);
        // Brain: each drone's own interval; 0 falls back to the request.
        let mut b = Bench::new(3, TelemetryConfig { requested_hz: 20.0, ..on(RateMode::Brain) });
        b.views[1].brain_interval_ms = 125;
        assert_eq!(b.tlm.requested_ms(), 50);
        for _ in 0..60 {
            b.step();
        }
        assert_eq!(b.row(1)[tlm::REQUESTED_HZ], 8.0);
        assert_eq!(b.row(2)[tlm::REQUESTED_HZ], 20.0);
        assert!((b.row(1)[tlm::DELIVERED_HZ] - 8.0).abs() < 0.6);
        // Backend budget: 60 pps over 3 online → 20 Hz each, once the
        // ground station has heard them.
        let mut b = Bench::new(4, on(RateMode::BackendBudget));
        for _ in 0..100 {
            b.step();
        }
        assert_eq!(b.tlm.summary()[chan::ONLINE], 3.0);
        assert!((1..4).all(|i| b.row(i)[tlm::REQUESTED_HZ] == 20.0), "{:?}", b.row(1));
    }

    #[test]
    fn ground_station_measures_loss_from_seq_gaps() {
        let mut b = Bench::new(2, on(RateMode::Fixed(25.0)));
        for _ in 0..40 {
            b.step();
        }
        // Out of range for 4 s (< NODE_TIMEOUT): TCP stalls, the 50-package
        // buffer holds the first 2 s and the rest is refused at the source.
        b.range = 10.0;
        for _ in 0..80 {
            b.step();
        }
        assert_eq!(b.row(1)[tlm::DELIVERED_HZ], 0.0);
        assert!(b.row(1)[tlm::QUEUE_DROPS_PER_S] > 20.0, "{:?}", b.row(1));
        b.range = 400.0;
        let (mut late, mut back) = (0.0f32, None);
        for k in 0..100 {
            b.step();
            late = late.max(b.row(1)[tlm::LATENCY_MS]);
            if back.is_none() && b.row(1)[tlm::DELIVERED_HZ] > 0.0 {
                back = Some(k * 50);
            }
        }
        // The retransmission timer had backed off to RTO_MAX: the stream
        // resumes within that, then the buffer flushes.
        let back = back.expect("delivery resumed");
        assert!(back <= painless::RTO_MAX_MS + 300, "resumed after {back} ms");
        let r = b.row(1);
        // Cut off for 4 s plus up to RTO_MAX of backed-off retransmission:
        // the buffered 50 frames arrive late, the rest of that time's frames
        // are missing "s" in the 10 s window.
        assert!((20.0..80.0).contains(&r[tlm::LOSS_PCT]), "{r:?}");
        assert!(late > 1_000.0, "flushed frames are seconds late: {late}");
        assert!(b.tlm.summary()[chan::MEDIAN_LOSS_PCT] > 10.0);
        assert_eq!(b.mesh.parent(1), Some(0), "4 s is shorter than NODE_TIMEOUT");
    }

    #[test]
    fn broadcast_floods_and_relays_count_bytes() {
        // A chain: 0 - 1 - 2 (range 60 m): 2's telemetry is relayed by 1.
        let mut b = Bench::new(3, on(RateMode::Fixed(10.0)));
        b.range = 60.0;
        for _ in 0..60 {
            b.step();
        }
        let single_relay = b.row(1)[tlm::RELAY_BYTES_PER_S];
        assert_eq!(b.row(2)[tlm::TREE_HOPS], 2.0);
        assert!(single_relay > 10.0 * b.row(2)[tlm::LAST_PACKAGE_BYTES], "{:?}", b.row(1));
        assert!(b.row(0)[tlm::RELAY_BYTES_PER_S] > 0.0, "the root's uplink");
        let mut b = Bench::new(3, TelemetryConfig { routing: Routing::Broadcast, ..on(RateMode::Fixed(10.0)) });
        b.range = 60.0;
        for _ in 0..60 {
            b.step();
        }
        assert!(b.tlm.sample(1).starts_with("{\"type\":8,\"dest\":0,"), "{}", b.tlm.sample(1));
        // 1's broadcast also goes down to 2; 2's telemetry is relayed up.
        assert!(b.row(1)[tlm::TX_BYTES_PER_S] > 1.9 * 10.0 * b.row(1)[tlm::LAST_PACKAGE_BYTES], "{:?}", b.row(1));
        assert!((b.row(2)[tlm::DELIVERED_HZ] - 10.0).abs() < 0.6);
        let dst: Vec<f32> = b.records(chatter::CHATTER_TLM).map(|r| r[chatter::DST]).collect();
        assert!(dst.iter().all(|&d| d == -1.0), "{dst:?}");
    }

    #[test]
    fn budget_rule_and_aimd() {
        let mut b = BackendBudget::default();
        let hz: Vec<f32> = [1, 2, 3, 4, 6, 20].iter().map(|&n| b.hz(n, 25.0)).collect();
        assert_eq!(hz, [25.0, 25.0, 20.0, 15.0, 10.0, 5.0]);
        assert_eq!(b.hz(1, 10.0), 10.0);
        b.update(0, 30.0);
        assert_eq!(b.budget_pps, 48.0);
        b.update(5_000, 30.0);
        assert_eq!(b.budget_pps, 48.0, "one cut per 10 s");
        for t in 1..20 {
            b.update(t * 10_000, 30.0);
        }
        assert_eq!(b.budget_pps, BUDGET_FLOOR_PPS);
        b.update(200_000, 1.0);
        b.update(229_000, 1.0);
        assert_eq!(b.budget_pps, BUDGET_FLOOR_PPS, "needs 30 s of good");
        b.update(230_000, 1.0);
        assert_eq!(b.budget_pps, 25.0);
        b.update(250_000, 1.0);
        assert_eq!(b.budget_pps, 25.0);
        b.update(260_000, 1.0);
        assert_eq!(b.budget_pps, 30.0);
    }

    #[test]
    fn windows_slide() {
        let mut w = Short::new();
        w.add(0, 4.0);
        w.add(1_900, 4.0);
        assert_eq!(w.sum(1_999), 8.0);
        assert_eq!(w.per_s(1_999), 4.0 / 2.0, "the current bucket is not counted");
        assert_eq!(w.per_s(2_000), 8.0 / 2.0, "2 s of completed buckets");
        assert_eq!(w.per_s(2_250), 4.0 / 2.0, "the t=0 bucket rolled out");
        assert_eq!(w.sum(2_300), 4.0);
        // By send time, read late: an arrival burst lands in its frames' buckets.
        let mut d = BySent::new();
        for born in (0..3_000).step_by(40) {
            let at = born + 300 - born % 250;
            if at <= 3_000 {
                d.add_at(at, born, 1.0);
            }
        }
        assert_eq!(d.per_s_lagged(3_000, SENT_LAG), 25.0);
        d.add_at(3_000, 0, 1.0);
        assert_eq!(d.per_s_lagged(3_000, SENT_LAG), 25.0, "older than the window: ignored");
        assert_eq!(w.sum(100_000), 0.0);
    }

    #[test]
    fn modes_and_variants_decode() {
        assert_eq!(RateMode::from_code(0, f32::NAN), Some(RateMode::Fixed(MAX_HZ)));
        assert_eq!(RateMode::from_code(0, 1e9), Some(RateMode::Fixed(HZ_LIMIT)));
        assert_eq!(RateMode::from_code(2, 0.0).map(RateMode::code), Some(2));
        assert_eq!(RateMode::from_code(3, 1.0), None);
        assert_eq!(Variant::from_u32(3), None);
        assert_eq!(Variant::Mixed.for_drone(3), Variant::C6Rtk);
        assert_eq!(Variant::Mixed.for_drone(4), Variant::XiaoS3);
        assert_eq!(Routing::from_u32(1), Some(Routing::Broadcast));
        assert_eq!(median(&mut [3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(&mut []), 0.0);
    }
}
