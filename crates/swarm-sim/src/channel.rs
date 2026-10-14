//! Shared 2.4 GHz channel: everything on air costs airtime, and every
//! frame is received (or not) from its own signal, noise and interference.
//!
//! Brain frames (ESP-NOW at [`ChannelConfig::espnow_rate_mbps`]),
//! painlessMesh TCP segments with their 802.11 ACK and the TCP ACKs, mesh
//! NodeSync / TimeSync traffic, the root's uplink to the WiFi station and
//! every painlessMesh SoftAP's beacons go through [`Channel::broadcast`],
//! [`Channel::unicast`] and [`Channel::beacons`]. Those also feed the byte
//! ledger ([`crate::airlog`]), whether or not the airtime model is on.
//!
//! # Airtime (IEEE 802.11-2016, ESP-IDF)
//!
//! One attempt costs AIFS + mean backoff + PPDU (+ SIFS + 802.11 ACK for a
//! unicast). painlessMesh and the uplink are HT QoS stations, so data rides
//! best effort: AIFS\[BE\] = SIFS + 3 slots = 37 µs, CW 15…1023 (§10.22.2,
//! Table 9-155). ESP-NOW action frames go out on AC_VO (AIFS 28 µs, CW 3…7),
//! beacons at TBTT with no backoff. The slot is 9 µs: an all-ESP32 ERP BSS
//! uses the short slot even for DSSS frames. Each retry doubles the
//! contention window (binary exponential backoff, §10.3.3), so a retry
//! costs more air than the first attempt. PPDUs: DSSS long preamble
//! (192 µs) at 1 Mbps, short preamble (96 µs) at 2 / 5.5 / 11 Mbps; ERP-OFDM
//! 20 µs + 4 µs symbols + 6 µs signal extension; HT-mixed 36 µs + 4 µs
//! symbols + 6 µs. ACKs go at the highest basic rate not above the data
//! rate (§10.6.6.5.2): the data rate itself for DSSS, 6 / 12 / 24 Mbps for
//! OFDM and HT. QoS data frames carry a 26 B MAC header, management and
//! action frames 24 B, both + 4 B FCS. A unicast gets at most
//! [`MAX_ATTEMPTS`] (dot11ShortRetryLimit = 7) attempts.
//!
//! # Reception (SINR)
//!
//! For a frame `s → r` at rate `R` and length `L`:
//!
//! * **Signal.** `S = P(d) + LINK_BUDGET_DB + X_sr + F` dBm: the
//!   log-distance mean of [`RadioConfig::mean_rx_dbm`] (-95 dBm at the edge
//!   of range on the brains' RSSI scale) plus [`LINK_BUDGET_DB`] (the edge
//!   at -85 dBm, 12 dB over the 1 Mbps sensitivity), a static
//!   log-normal shadowing term per pair `X ~ N(0, SHADOW_SIGMA_DB)`
//!   (seeded from the channel's seed, symmetric) and per-frame Rician
//!   small-scale fading `F` (K = [`RICIAN_K_DB`]).
//! * **Noise.** `N = -174 dBm/Hz + 10 log10(20 MHz) + NF` with NF = 5 dB →
//!   [`NOISE_FLOOR_DBM`] = -96 dBm, raised by up to
//!   [`EXTERNAL_NOISE_RISE_DB`] in proportion to the external slider
//!   (adjacent / partially overlapping channels below CCA).
//! * **Interference.** Overlapping transmissions add power, they do not
//!   kill a frame outright. Per attempt the chance that a given node `k`
//!   overlaps is, for a sender that defers to `s` (contention: both
//!   backoffs expire in the same slot) `p_coll(b) · f_k / b` with
//!   `p_coll(b) = 1 − exp(−K_COLL · b)` and `b` the contending busy
//!   fraction at `r` (the airtime `f_k` of those senders plus the external
//!   share); for a *hidden* sender `r` hears but `s` cannot sense
//!   `1 − exp(−f_k (1 + T / T_k))` for our frame of duration `T` and its
//!   mean transmission time `T_k` (unslotted ALOHA: any of its frame starts
//!   within `(−T_k, T)` overlaps, so a slow, long frame is hit far more
//!   often); external traffic (the slider) is carrier-sensed time share
//!   and collides like a contender, at a power drawn from
//!   `N(EXT_INTERFERER_DBM, EXT_INTERFERER_SIGMA_DB)`. The receiver
//!   itself transmitting at that moment always loses the frame
//!   (half-duplex). One overlapping frame is drawn (the dominant
//!   interferer).
//! * **Capture.** A frame survives an overlap only if `S − I` clears the
//!   capture threshold of its modulation ([`CAPTURE_DSSS_DB`],
//!   [`CAPTURE_OFDM_DB`]).
//! * **Errors.** `PER = 1 − (1 − PER₁₀₀₀(SINR))^(L/1000)` with a logistic
//!   waterfall `PER₁₀₀₀ = 1 / (1 + 9 e^(a(SINR − γ_R)))`: 10 % at the
//!   rate's threshold `γ_R` = ESP32 datasheet sensitivity − noise floor
//!   (the sensitivity criterion is ~10 % PER at ~1000 B), slope
//!   [`PER_SLOPE_PER_DB`].
//!
//! # Rates (Minstrel-like)
//!
//! Each mesh link takes the rate with the best expected throughput
//! `(1 − p_R) · L / airtime_R`, where `p_R` is the mean per-attempt failure
//! of the model above at that rate (fading and the current interference
//! included), recomputed on every geometry refresh: what Minstrel converges
//! to. Rates run from DSSS 1 Mbps (the mesh firmware leaves 802.11b on) to
//! HT20 MCS7 ([`MESH_MIN_RATE_MBPS`]); the router's 12 Mbps minimum
//! ([`ChannelConfig::min_rate_mbps`]) applies to the root's uplink only. A
//! weak or interfered link falls back, and a slower frame costs more air.
//! ESP-NOW broadcasts stay at [`ChannelConfig::espnow_rate_mbps`].
//!
//! # Delivery, retries and queues
//!
//! A broadcast is one attempt. A unicast is attempted until it is ACKed or
//! [`MAX_ATTEMPTS`] are spent; every attempt is drawn from the model above
//! and charged its airtime as it happens (so retries feed the busy
//! fraction, which feeds collisions and retries). A frame to a receiver
//! that is gone spends all its attempts.
//!
//! Each sender holds at most [`ChannelConfig::tx_queue_frames`] frames
//! waiting for the medium, seen as one server shared by its carrier-sense
//! domain: utilisation ρ = the domain's offered airtime / what external
//! traffic leaves. A new frame is refused with the M/M/1/K blocking
//! probability `(1 − ρ) ρ^K / (1 − ρ^{K+1})`; refused frames never air and
//! are counted for `Sensors::tx_dropped`.
//!
//! # Calibration (field, `swarm_control/UDP_VS_MQTT.md`)
//!
//! One 2.4 GHz AP, ~400 B UDP telemetry frames crossing the air twice:
//! ~70 pps → ~1 % loss, 6 × 25 Hz ≈ 150 pps → 26-32 %, at the pre-hygiene
//! DSSS rates ([`FIELD_PRE_HYGIENE_RATE_MBPS`]). [`K_COLL`] is the one
//! fitted constant; the rest are 802.11 / ESP-IDF / datasheet figures or
//! the documented assumptions above.
//!
//! The world calls, per step: [`Channel::begin_step`], then
//! [`Channel::admit`] and a transmit call per frame, and
//! [`Channel::end_step`]. With [`ChannelConfig::enabled`] false only the
//! byte ledger is fed (no airtime, no draws, nothing a brain can see), so a
//! default world is bit-identical to one without a channel model. The
//! channel draws from its own RNG, never the world's.

use swarm_brain::Vec3;

use crate::airlog::{AirLedger, ByteKind};
use crate::rng::Rng;
use crate::{RadioConfig, Terrain};

/// Most external busy fraction the slider can add.
pub const MAX_EXTERNAL_BUSY: f32 = 0.8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChannelConfig {
    /// Model the shared channel at all. Off by default.
    pub enabled: bool,
    /// Airtime fraction used by other networks on the channel,
    /// `0..=MAX_EXTERNAL_BUSY`.
    pub external_busy: f32,
    /// PHY rate of brain (ESP-NOW) frames, Mbps. 1 = DSSS long preamble
    /// (ESP-NOW's default `WIFI_PHY_RATE_1M_L`).
    pub espnow_rate_mbps: f32,
    /// Minimum data rate of the router SSID, i.e. the root → WiFi station
    /// uplink, Mbps (802.11b disabled, UniFi "Minimum Data Rate" 12 Mbps per
    /// `UDP_VS_MQTT.md`). painlessMesh links between the ESP32 SoftAPs do
    /// not get this setting: they adapt down to [`MESH_MIN_RATE_MBPS`].
    pub min_rate_mbps: f32,
    /// Carrier-sense range as a multiple of the radio range (≥ 1). CCA
    /// preamble detection reaches ~-82 dBm and energy detect further than
    /// a decodable frame, so a node defers to senders it can't decode.
    pub cs_range_factor: f32,
    /// Frames a sender can hold waiting for the medium; more are dropped.
    /// 8 ≈ the ESP32 WiFi driver's static TX buffers for a busy socket.
    pub tx_queue_frames: u16,
    /// The ground-station laptop is a WiFi client of the same router, so
    /// each uplink datagram crosses the air twice (root → router →
    /// laptop), as in the field calibration. On by default.
    pub gcs_on_wifi: bool,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            external_busy: 0.0,
            espnow_rate_mbps: 1.0,
            min_rate_mbps: 12.0,
            cs_range_factor: 1.5,
            tx_queue_frames: 8,
            gcs_on_wifi: true,
        }
    }
}

/// What a transmission is, for the per-kind airtime split.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AirKind {
    /// A brain frame (ESP-NOW; broadcasts are never retried).
    Brain = 0,
    /// A painlessMesh TCP segment carrying a package.
    MeshData = 1,
    /// A TCP acknowledgement.
    Ack = 2,
    /// Node sync / time sync packages.
    MeshControl = 3,
    /// Root → WiFi station UDP datagram.
    Uplink = 4,
}

/// Number of [`AirKind`]s.
const KINDS: usize = 5;

/// Receiver of a unicast.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dst {
    /// Another node.
    Node(u16),
    /// The WiFi router / ground station at the sender's site (the root's
    /// uplink); [`STATION_DISTANCE`] away, line of sight.
    Station,
}

/// One frame handed to the channel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirFrame {
    pub src: u16,
    pub kind: AirKind,
    /// MPDU bytes (MAC header + FCS + body).
    pub mpdu: u32,
    pub rate_mbps: f32,
    /// Rate of the attempts after the first [`MRR_PRIMARY_ATTEMPTS`]
    /// (multi-rate retry); the same as `rate_mbps` for a fixed-rate frame.
    pub fallback_mbps: f32,
    /// Mesh segments: share of the bytes that are the sender's own
    /// telemetry, and share that is NodeSync / TimeSync.
    pub own_frac: f32,
    pub ctl_frac: f32,
    /// A TCP retransmission: all its bytes count as retransmitted.
    pub retx: bool,
}

impl AirFrame {
    pub fn new(src: u16, kind: AirKind, mpdu: u32, rate_mbps: f32) -> Self {
        Self { src, kind, mpdu, rate_mbps, fallback_mbps: rate_mbps, own_frac: 0.0, ctl_frac: 0.0, retx: false }
    }

    /// The same frame with a multi-rate retry chain `(primary, fallback)`.
    pub fn with_rates(mut self, rates: (f32, f32)) -> Self {
        self.rate_mbps = rates.0;
        self.fallback_mbps = rates.1;
        self
    }
}

/// What a unicast came to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TxResult {
    /// ACKed by the receiver.
    pub delivered: bool,
    /// Channel time of all its attempts, µs.
    pub us: f32,
    pub attempts: u32,
}

// ---- 802.11 / ESP-IDF timing -------------------------------------------------

/// Short slot (802.11-2016 §18.4.4, ERP; an all-ESP32 BSS uses it for DSSS
/// frames too).
pub const SLOT_US: f32 = 9.0;
/// SIFS, 2.4 GHz.
pub const SIFS_US: f32 = 10.0;
/// DIFS = SIFS + 2 slots (= AIFS\[VO\], AIFSN 2).
pub const DIFS_US: f32 = SIFS_US + 2.0 * SLOT_US;
/// AIFS\[BE\] = SIFS + 3 slots (EDCA default AIFSN 3, Table 9-155).
pub const AIFS_BE_US: f32 = SIFS_US + 3.0 * SLOT_US;
/// Mean initial backoff of best-effort data: CWmin 15 → 7.5 slots.
pub const MEAN_BACKOFF_US: f32 = 7.5 * SLOT_US;
/// EDCA contention windows (Table 9-155): BE 15…1023, VO 3…7.
const CW_BE: (u32, u32) = (15, 1023);
const CW_VO: (u32, u32) = (3, 7);
/// Management / action frame MAC header (24) + FCS (4): ESP-NOW, beacons.
pub const MAC_OVERHEAD: u32 = 24 + 4;
/// QoS data MAC header (26, HT/WMM stations) + FCS (4): mesh segments, TCP
/// ACKs, the uplink.
pub const QOS_MAC_OVERHEAD: u32 = 26 + 4;
/// LLC/SNAP header in front of an IP packet.
pub const LLC_SNAP: u32 = 8;
/// IPv4 + TCP headers (lwIP, no options).
pub const TCP_IP_OVERHEAD: u32 = 40;
/// IPv4 + UDP headers.
pub const UDP_IP_OVERHEAD: u32 = 28;
/// ESP-NOW vendor action frame overhead around the payload (category 1,
/// OUI 3, random 4, vendor element header 7; ESP-IDF ESP-NOW docs).
pub const ESPNOW_OVERHEAD: u32 = 1 + 3 + 4 + 7;
/// 802.11 ACK frame (FC 2 + duration 2 + RA 6 + FCS 4).
pub const ACK_BYTES: u32 = 14;
/// MPDU of a TCP ACK: QoS header + FCS, LLC/SNAP, IPv4 + TCP.
pub const TCP_ACK_MPDU: u32 = QOS_MAC_OVERHEAD + LLC_SNAP + TCP_IP_OVERHEAD;
/// DSSS long PLCP preamble + header (1 Mbps), and short (2-11 Mbps).
pub const DSSS_PLCP_US: f32 = 192.0;
pub const DSSS_SHORT_PLCP_US: f32 = 96.0;
/// ERP-OFDM PLCP preamble + SIGNAL (16 + 4 µs).
pub const OFDM_PLCP_US: f32 = 20.0;
/// HT-mixed PLCP: L-STF 8 + L-LTF 8 + L-SIG 4 + HT-SIG 8 + HT-STF 4 +
/// HT-LTF 4 µs (one stream).
pub const HT_PLCP_US: f32 = 36.0;
/// 2.4 GHz OFDM signal extension.
pub const OFDM_SIGNAL_EXT_US: f32 = 6.0;
/// dot11ShortRetryLimit 7: a unicast gets at most 7 attempts (ESP-IDF
/// default).
pub const MAX_ATTEMPTS: u32 = 7;
/// Multi-rate retry: the first attempts go at the best-throughput rate,
/// the rest at the most robust one (Minstrel's retry chain; ESP-IDF's rate
/// control also steps down within a frame's retries).
pub const MRR_PRIMARY_ATTEMPTS: u32 = 2;
/// Residual loss (after every attempt) the retry chain aims below: a lost
/// TCP segment costs a 1-3 s retransmission timeout on the mesh.
pub const MRR_RESIDUAL_LOSS: f32 = 1e-3;

// ---- radio model -------------------------------------------------------------

/// Link budget of the channel model over radio.rs's RSSI scale, dB.
/// `RadioConfig::rssi_dbm` (what the brains read, kept as it was) puts the
/// edge of range at -95 dBm, a hair above the noise floor; the channel puts
/// it at -85 dBm, a 12 dB margin over the ESP32's 1 Mbps sensitivity:
/// `range` is where the swarm's most robust links are still planned to
/// work (a 10-15 dB fade margin is the usual link-planning figure; ESP32
/// output 19.5 dBm at 11b, 18 dBm at HT20 MCS7). Same slope, so every
/// relative quantity (SIR, capture, rate steps per distance) is unchanged.
pub const LINK_BUDGET_DB: f32 = 10.0;
/// Thermal noise in 20 MHz (-174 dBm/Hz + 73 dB) plus a 5 dB receiver noise
/// figure. With it, the ESP32's -97 dBm at 1 Mbps means DSSS needs about
/// -1 dB SNR (Barker spreading gain 10.4 dB over DBPSK's ~9 dB Eb/N0).
pub const NOISE_FLOOR_DBM: f32 = -96.0;
/// Noise-floor rise at external busy 1.0 (scaled linearly in power): some
/// of the "other networks" sit on partially overlapping channels whose
/// energy stays below CCA but adds to the floor. A modest assumption; 0
/// switches it off.
pub const EXTERNAL_NOISE_RISE_DB: f32 = 3.0;
/// Log-normal shadowing per pair (static), dB. Air-to-air / air-to-ground
/// links with line of sight measure ~2-6 dB; 5 dB.
pub const SHADOW_SIGMA_DB: f32 = 5.0;
/// Shadowing is clamped to this many sigmas.
const SHADOW_CLAMP: f32 = 2.5;
/// Rician K factor of the per-frame fading, dB (UAV links with a line of
/// sight measure K ≈ 2-15 dB; 6 dB is a cautious middle).
pub const RICIAN_K_DB: f32 = 6.0;
/// Logistic waterfall slope: PER falls from 90 % to 10 % over ~2.2 dB, as
/// the AWGN PER curves of the 802.11 PHYs do.
pub const PER_SLOPE_PER_DB: f32 = 2.0;
/// Reference frame length of the sensitivity figures (PER 10 % at 1000 B
/// OFDM, 8 % at 1024 B DSSS).
pub const PER_REF_BYTES: f32 = 1_000.0;
/// Capture thresholds: a frame survives an overlapping one when it is this
/// much stronger (DSSS 4-6 dB, OFDM / HT ≥ 10 dB).
pub const CAPTURE_DSSS_DB: f32 = 5.0;
pub const CAPTURE_OFDM_DB: f32 = 10.0;
/// Power of an external (other-network) frame at the receiver, on the
/// sim's own power scale (-95 dBm at the edge of range): nodes defer to it,
/// so it is above what they carrier-sense (a node at
/// `cs_range_factor × range` ≈ -99 dBm), and on average as strong as a
/// node at the edge of range. Drawn per overlap from N(mean, sigma),
/// clamped to the range below. Strong links capture over it, marginal
/// ones do not.
pub const EXT_INTERFERER_DBM: f32 = -95.0 + LINK_BUDGET_DB;
pub const EXT_INTERFERER_SIGMA_DB: f32 = 6.0;
const EXT_INTERFERER_RANGE: (f32, f32) = (-99.0 + LINK_BUDGET_DB, -70.0 + LINK_BUDGET_DB);
/// The WiFi router / ground station is this far from the root, in units.
pub const STATION_DISTANCE: f32 = 15.0;

/// Collision steepness, the one fitted constant: the contenders around a
/// receiver overlap a frame with probability `1 − exp(−K_COLL · b)` at
/// their busy share `b` (and other networks with `1 − exp(−K_COLL · ext)`).
/// Fitted on the field cell (`t5_field_calibration`): 70 pps → 0.1 %,
/// 150 pps → 24.6 % (field ~1 %, 26-32 %), and the external slider still
/// adding fully to the busy fraction (`t5_external_busy_costs_delivery`);
/// 1.1-1.3 all pass, 1.5 reaches 30 %. (0.8 under the old all-or-nothing
/// collisions: capture now saves part of the overlaps.)
pub const K_COLL: f32 = 1.2;
/// Busy is capped here inside the collision formula.
pub const BUSY_CAP: f32 = 0.97;
/// Exponential window of the busy / airtime / utilisation estimates.
pub const WINDOW_MS: f32 = 1_000.0;
/// Carrier-sense / audibility geometry (and the rate choice) is recomputed
/// this often.
pub const GEOMETRY_REFRESH_MS: u32 = 250;
/// The field calibration's PHY rate: the ESP32s fell back to DSSS
/// (802.11b on, retries at 1-11 Mbps); 2 Mbps is the middle of that.
pub const FIELD_PRE_HYGIENE_RATE_MBPS: f32 = 2.0;
/// External traffic never leaves less than this share of the medium in
/// the queue model, so ρ stays finite.
const MIN_SHARE: f32 = 0.03;
/// Lowest rate of the painlessMesh links (ESP32 SoftAP ↔ STA), Mbps.
/// painlessMesh firmware (`drone_mavlink_node.ino`, `gps_oled_master_v2.ino`)
/// never disables 802.11b, so weak mesh links fall back to DSSS.
pub const MESH_MIN_RATE_MBPS: f32 = 1.0;
/// Beacon interval of every painlessMesh SoftAP (100 TU = 102.4 ms).
pub const BEACON_INTERVAL_MS: f32 = 102.4;
/// MPDU length of a mesh SoftAP beacon: MAC header + FCS, timestamp,
/// interval, capabilities, SSID, supported / extended rates, DS parameter,
/// TIM, ERP, HT capabilities / information, RSN and WMM elements.
pub const BEACON_BYTES: u32 = 190;
/// Frame length the rate choice optimises for (a typical mesh segment).
const RATE_REF_MPDU: u32 = 500;
/// Senders below this transmit fraction are ignored as interferers.
const ACTIVE_EPS: f32 = 1e-4;
/// Fading quantiles for the expected-value computations.
const FADE_Q: usize = 8;
/// Geometry flags.
const HEAR: u8 = 1;
const SENSE: u8 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phy {
    DsssLong,
    DsssShort,
    Ofdm,
    Ht,
}

#[derive(Clone, Copy, Debug)]
struct RateInfo {
    mbps: f32,
    /// ESP32 receive sensitivity, dBm (ESP32 datasheet, "Wi-Fi Radio
    /// Characteristics"; HT MCS1-6 interpolated between the published MCS0
    /// / MCS7 figures along the legacy rate with the same modulation).
    sens_dbm: f32,
    phy: Phy,
}

const fn rate(mbps: f32, sens_dbm: f32, phy: Phy) -> RateInfo {
    RateInfo { mbps, sens_dbm, phy }
}

const RATES: [RateInfo; 20] = [
    rate(1.0, -97.0, Phy::DsssLong),
    rate(2.0, -94.0, Phy::DsssShort),
    rate(5.5, -92.0, Phy::DsssShort),
    rate(11.0, -88.0, Phy::DsssShort),
    rate(6.0, -92.0, Phy::Ofdm),
    rate(9.0, -91.0, Phy::Ofdm),
    rate(12.0, -89.0, Phy::Ofdm),
    rate(18.0, -86.0, Phy::Ofdm),
    rate(24.0, -84.0, Phy::Ofdm),
    rate(36.0, -80.0, Phy::Ofdm),
    rate(48.0, -76.0, Phy::Ofdm),
    rate(54.0, -74.0, Phy::Ofdm),
    rate(6.5, -92.0, Phy::Ht),
    rate(13.0, -89.0, Phy::Ht),
    rate(19.5, -86.0, Phy::Ht),
    rate(26.0, -84.0, Phy::Ht),
    rate(39.0, -80.0, Phy::Ht),
    rate(52.0, -76.0, Phy::Ht),
    rate(58.5, -74.0, Phy::Ht),
    rate(65.0, -72.0, Phy::Ht),
];

/// Rates the mesh (and the uplink) choose from: DSSS and HT20 (an ESP32 in
/// 11bgn mode uses HT MCS for unicast data, legacy OFDM only for control).
const AUTO_RATES: [usize; 12] = [0, 1, 2, 3, 12, 13, 14, 15, 16, 17, 18, 19];

fn rate_index(mbps: f32) -> Option<usize> {
    RATES.iter().position(|r| (r.mbps - mbps).abs() < 1e-3)
}

fn phy_of(mbps: f32) -> Phy {
    match rate_index(mbps) {
        Some(i) => RATES[i].phy,
        None if mbps <= 1.0 => Phy::DsssLong,
        None if mbps < 6.0 => Phy::DsssShort,
        None => Phy::Ofdm,
    }
}

fn sane_rate(rate_mbps: f32) -> f32 {
    if rate_mbps.is_finite() { rate_mbps.clamp(1.0, 65.0) } else { 1.0 }
}

/// PHY duration of an `mpdu`-byte frame at `rate_mbps`: DSSS (long preamble
/// at 1 Mbps, short above), ERP-OFDM or HT-mixed (both with the 2.4 GHz
/// signal extension).
pub fn phy_us(mpdu: u32, rate_mbps: f32) -> f32 {
    let rate = sane_rate(rate_mbps);
    let symbols = |plcp: f32| {
        // SERVICE 16 bits + data + tail 6 bits, in 4 µs symbols.
        let bits_per_symbol = 4.0 * rate;
        plcp + 4.0 * ((16 + 8 * mpdu + 6) as f32 / bits_per_symbol).ceil() + OFDM_SIGNAL_EXT_US
    };
    match phy_of(rate) {
        Phy::DsssLong => DSSS_PLCP_US + 8.0 * mpdu as f32 / rate,
        Phy::DsssShort => DSSS_SHORT_PLCP_US + 8.0 * mpdu as f32 / rate,
        Phy::Ofdm => symbols(OFDM_PLCP_US),
        Phy::Ht => symbols(HT_PLCP_US),
    }
}

/// Rate of the 802.11 ACK to a frame at `rate_mbps`: the highest basic rate
/// not above it (§10.6.6.5.2): the DSSS rate itself, else 6 / 12 / 24 Mbps.
pub fn ack_rate_mbps(rate_mbps: f32) -> f32 {
    let rate = sane_rate(rate_mbps);
    match phy_of(rate) {
        Phy::DsssLong | Phy::DsssShort => rate,
        _ if rate >= 24.0 => 24.0,
        _ if rate >= 12.0 => 12.0,
        _ => 6.0,
    }
}

/// Mean backoff of `attempt` (0 = first) with contention window `cw`:
/// CW doubles per retry up to its maximum.
fn backoff_us(cw: (u32, u32), attempt: u32) -> f32 {
    let w = ((cw.0 + 1) << attempt.min(10)).saturating_sub(1).min(cw.1);
    w as f32 / 2.0 * SLOT_US
}

/// Channel time of an ESP-NOW broadcast with `payload` bytes: AIFS\[VO\] +
/// mean VO backoff + the frame.
pub fn espnow_airtime_us(payload: u32, rate_mbps: f32) -> f32 {
    DIFS_US + backoff_us(CW_VO, 0) + phy_us(MAC_OVERHEAD + ESPNOW_OVERHEAD + payload, rate_mbps)
}

/// Channel time of attempt `attempt` (0 = first) of a unicast of `mpdu`
/// bytes: AIFS + backoff (doubling per retry) + the frame + SIFS + ACK.
/// `voice` = AC_VO (ESP-NOW), else best effort.
pub fn unicast_attempt_us(mpdu: u32, rate_mbps: f32, attempt: u32, voice: bool) -> f32 {
    let (aifs, cw) = if voice { (DIFS_US, CW_VO) } else { (AIFS_BE_US, CW_BE) };
    aifs + backoff_us(cw, attempt) + phy_us(mpdu, rate_mbps) + SIFS_US + phy_us(ACK_BYTES, ack_rate_mbps(rate_mbps))
}

/// Channel time of the first attempt of a best-effort QoS data frame whose
/// IP packet is `ip_bytes` long.
pub fn wifi_airtime_us(ip_bytes: u32, rate_mbps: f32) -> f32 {
    unicast_attempt_us(QOS_MAC_OVERHEAD + LLC_SNAP + ip_bytes, rate_mbps, 0, false)
}

/// Channel time of one SoftAP beacon at `rate_mbps`: sent at TBTT after a
/// DIFS-long idle, no backoff, never acknowledged.
pub fn beacon_airtime_us(rate_mbps: f32) -> f32 {
    DIFS_US + phy_us(BEACON_BYTES, rate_mbps)
}

/// Expected attempts of a unicast with per-attempt failure `p`:
/// `1 + p + … + p^(MAX_ATTEMPTS − 1)`.
pub fn expected_attempts(p: f32) -> f32 {
    let p = p.clamp(0.0, 1.0);
    if p >= 1.0 - 1e-6 {
        return MAX_ATTEMPTS as f32;
    }
    (1.0 - p.powi(MAX_ATTEMPTS as i32)) / (1.0 - p)
}

/// Probability that a frame overlaps a contending one at contending busy
/// fraction `busy`.
pub fn collision_prob(busy: f32) -> f32 {
    let b = if busy.is_finite() { busy.clamp(0.0, BUSY_CAP) } else { 0.0 };
    1.0 - (-K_COLL * b).exp()
}

/// M/M/1/K blocking probability: a queue of `k` frames at utilisation
/// `rho` refuses this share of arrivals.
pub fn queue_block_prob(rho: f32, k: u16) -> f32 {
    if rho.is_nan() || rho <= 0.0 {
        return 0.0;
    }
    let k = i32::from(k.max(1));
    if (rho - 1.0).abs() < 1e-4 {
        return 1.0 / (k + 1) as f32;
    }
    // For ρ > 1 use the equivalent form in 1/ρ, which stays finite.
    if rho > 1.0 {
        let r = 1.0 / rho;
        return (1.0 - r) / (1.0 - r.powi(k + 1));
    }
    (1.0 - rho) * rho.powi(k) / (1.0 - rho.powi(k + 1))
}

/// SNR threshold of a rate: where its PER is 10 % at 1000 B.
fn threshold_db(r: &RateInfo) -> f32 {
    r.sens_dbm - NOISE_FLOOR_DBM
}

fn capture_db(r: &RateInfo) -> f32 {
    match r.phy {
        Phy::DsssLong | Phy::DsssShort => CAPTURE_DSSS_DB,
        _ => CAPTURE_OFDM_DB,
    }
}

/// Packet error rate of an `mpdu`-byte frame at `sinr_db` and rate `r`.
fn per(sinr_db: f32, r: &RateInfo, mpdu: u32) -> f32 {
    let x = PER_SLOPE_PER_DB * (sinr_db - threshold_db(r)) + 9.0f32.ln();
    if x > 40.0 {
        return 0.0;
    }
    let p_ref = 1.0 / (1.0 + x.exp());
    if p_ref >= 1.0 - 1e-7 {
        return 1.0;
    }
    -((mpdu as f32 / PER_REF_BYTES) * (-p_ref).ln_1p()).exp_m1()
}

/// Packet error rate of a frame at `sinr_db` and `rate_mbps` (public for
/// tests and reports).
pub fn packet_error_rate(sinr_db: f32, rate_mbps: f32, mpdu: u32) -> f32 {
    let r = rate_index(sane_rate(rate_mbps)).map_or(RATES[0], |i| RATES[i]);
    per(sinr_db, &r, mpdu)
}

fn dbm_to_mw(dbm: f32) -> f32 {
    10f32.powf(dbm / 10.0)
}

fn mw_to_dbm(mw: f32) -> f32 {
    10.0 * mw.max(1e-30).log10()
}

/// Standard normal from a pair of uniforms (Box-Muller), both outputs.
fn normals(rng: &mut Rng) -> (f32, f32) {
    let u1 = 1.0 - rng.f32(); // (0, 1]
    let u2 = rng.f32();
    let r = (-2.0 * u1.ln()).sqrt();
    let t = core::f32::consts::TAU * u2;
    (r * t.cos(), r * t.sin())
}

/// Rician fading power gain in dB from two standard normals.
fn rician_db(g: (f32, f32)) -> f32 {
    let k = dbm_to_mw(RICIAN_K_DB);
    let los = (k / (k + 1.0)).sqrt();
    let s = (1.0 / (2.0 * (k + 1.0))).sqrt();
    let (re, im) = (los + s * g.0, s * g.1);
    mw_to_dbm(re * re + im * im)
}

/// The static shadowing field of a channel (see [`Channel::shadowing`]).
#[derive(Clone, Copy, Debug)]
pub struct Shadowing(u64);

impl Shadowing {
    /// Shadowing of the pair `(a, b)` (order-free), dB.
    pub fn db(&self, a: u16, b: u16) -> f32 {
        shadow_db(self.0, usize::from(a), usize::from(b))
    }
}

/// Static shadowing of the pair `(a, b)` (order-free), dB.
fn shadow_db(seed: u64, a: usize, b: usize) -> f32 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    let mut r = Rng::new(seed ^ ((lo as u64) << 32 | hi as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
    let (g, _) = normals(&mut r);
    SHADOW_SIGMA_DB * g.clamp(-SHADOW_CLAMP, SHADOW_CLAMP)
}

/// Per-node position and liveness the channel needs for carrier sense.
#[derive(Clone, Copy, Debug)]
pub struct NodeRadio {
    pub pos: Vec3,
    pub alive: bool,
}

/// One possible overlapping frame at a receiver.
#[derive(Clone, Copy, Debug)]
struct Interferer {
    /// Chance it overlaps an attempt (contention and external); for a
    /// hidden sender see `hidden`.
    p: f32,
    /// Hidden sender: (its airtime fraction, its mean frame time µs). It
    /// overlaps a frame of duration `T` with `1 − exp(−h (1 + T / T_k))`:
    /// its frames start at rate `h / T_k` and any start within `(−T_k, T)`
    /// overlaps (unslotted ALOHA vulnerable window).
    hidden: Option<(f32, f32)>,
    /// Its power at the receiver, dBm (`INFINITY`: the receiver itself).
    dbm: f32,
    external: bool,
}

impl Interferer {
    /// Overlap probability for an attempt lasting `t_us`.
    fn p_for(&self, t_us: f32) -> f32 {
        match self.hidden {
            Some((h, tk)) => 1.0 - (-h * (1.0 + t_us / tk.max(1.0))).exp(),
            None => self.p,
        }
        .clamp(0.0, 1.0)
    }
}

/// Context of the last broadcast, for [`Channel::lost_broadcast`].
#[derive(Clone, Copy, Debug)]
struct LastBroadcast {
    src: u16,
    mpdu: u32,
    rate: usize,
}

#[derive(Clone, Debug)]
pub struct Channel {
    cfg: ChannelConfig,
    rng: Rng,
    now_ms: u32,
    dt_ms: u32,
    n: usize,
    /// Per node: airtime this step, µs (retries included).
    own_us: Vec<f32>,
    /// Per node: airtime refused frames would have needed this step, µs.
    refused_us: Vec<f32>,
    /// Per node: extra backoff of its retries this step (CW doubling), µs:
    /// time its queue is held, but idle medium others may use, so it counts
    /// towards queue utilisation and not towards anyone's busy fraction.
    wait_us: Vec<f32>,
    /// Per node: smoothed own-airtime fraction.
    own_frac: Vec<f32>,
    /// Per node: airtime this step of frames that contend for the medium
    /// (everything but beacons), and its smoothed fraction.
    cont_us: Vec<f32>,
    cont_frac: Vec<f32>,
    /// Per node: smoothed busy fraction heard from others (external excluded).
    heard_frac: Vec<f32>,
    /// Per node: smoothed offered airtime fraction (carried + refused).
    offered_frac: Vec<f32>,
    /// Per node: smoothed offered demand of its carrier-sense domain.
    demand_frac: Vec<f32>,
    /// Per node: smoothed airtime of one of its frames (all attempts), µs.
    frame_us: Vec<f32>,
    /// Per node: smoothed channel time of one of its transmissions (one
    /// attempt), µs: the hidden-terminal window.
    attempt_us: Vec<f32>,
    /// Per node: the queue's blocking probability.
    block_p: Vec<f32>,
    /// Per node: frames refused by [`Channel::admit`] since the last take.
    tx_dropped: Vec<u16>,
    /// Per node: frames refused, smoothed per second.
    drops_per_s: Vec<f32>,
    /// Per node: refused this step.
    step_drops: Vec<u16>,
    /// `n × n` flags: `[r * n + k]` HEAR = r can decode k, SENSE = r
    /// carrier-senses k.
    geo: Vec<u8>,
    geo_at_ms: Option<u32>,
    /// Geometry refreshes so far (rate-cache epoch).
    epoch: u32,
    /// `n × n` mean received power `[r * n + k]` of k at r with shadowing,
    /// dBm, for pairs within carrier-sense range (NaN otherwise).
    pw: Vec<f32>,
    /// Positions at the last refresh, for pairs outside `pw`.
    pos: Vec<Vec3>,
    radio: RadioConfig,
    /// `n × n` mesh rate cache `[tx * n + rx]`: (epoch + 1, primary,
    /// fallback Mbps); the diagonal holds the node's station link.
    rate_cache: Vec<(u32, f32, f32)>,
    shadow_seed: u64,
    /// Fading gain quantiles, dB (ascending).
    fade_q: [f32; FADE_Q],
    /// Noise floor with the external rise, mW.
    noise_mw: f32,
    /// Nodes transmitting more than [`ACTIVE_EPS`] of the time.
    active: Vec<u16>,
    /// Scratch for the interferers of one attempt.
    scratch: Vec<Interferer>,
    /// Airtime by [`AirKind`] this step, µs, and smoothed fractions.
    kind_us: [f32; KINDS],
    kind_frac: [f32; KINDS],
    /// SoftAP beacon airtime this step, µs, and its smoothed fraction
    /// (also counted under [`AirKind::MeshControl`]).
    beacon_us: f32,
    beacon_frac: f32,
    /// Frames on air this step (attempts and 802.11 ACKs, no beacons), and
    /// smoothed per second.
    step_tx: u32,
    pps: f32,
    last_bcast: Option<LastBroadcast>,
    ledger: AirLedger,
}

impl Channel {
    pub fn new(cfg: ChannelConfig, nodes: usize, seed: u64) -> Self {
        let mut fade = Rng::new(0xFADE_0000_0000_0006);
        let mut samples: Vec<f32> = (0..4096).map(|_| rician_db(normals(&mut fade))).collect();
        samples.sort_unstable_by(f32::total_cmp);
        let mut fade_q = [0.0; FADE_Q];
        for (i, q) in fade_q.iter_mut().enumerate() {
            let (a, b) = (i * samples.len() / FADE_Q, (i + 1) * samples.len() / FADE_Q);
            // Mean power of the bin, in dB.
            *q = mw_to_dbm(samples[a..b].iter().map(|&d| dbm_to_mw(d)).sum::<f32>() / (b - a) as f32);
        }
        let mut ch = Self {
            cfg: sanitize(cfg),
            rng: Rng::new(seed ^ 0xC4A7_7E1D_0000_0001),
            now_ms: 0,
            dt_ms: 0,
            n: nodes,
            own_us: vec![0.0; nodes],
            refused_us: vec![0.0; nodes],
            wait_us: vec![0.0; nodes],
            own_frac: vec![0.0; nodes],
            cont_us: vec![0.0; nodes],
            cont_frac: vec![0.0; nodes],
            heard_frac: vec![0.0; nodes],
            offered_frac: vec![0.0; nodes],
            demand_frac: vec![0.0; nodes],
            frame_us: vec![0.0; nodes],
            attempt_us: vec![0.0; nodes],
            block_p: vec![0.0; nodes],
            tx_dropped: vec![0; nodes],
            drops_per_s: vec![0.0; nodes],
            step_drops: vec![0; nodes],
            geo: Vec::new(),
            geo_at_ms: None,
            epoch: 0,
            pw: Vec::new(),
            pos: Vec::new(),
            radio: RadioConfig::default(),
            rate_cache: Vec::new(),
            shadow_seed: Rng::new(seed ^ 0x5AAD_0000_0000_0003).next_u64(),
            fade_q,
            noise_mw: dbm_to_mw(NOISE_FLOOR_DBM),
            active: Vec::with_capacity(nodes),
            scratch: Vec::with_capacity(nodes + 2),
            kind_us: [0.0; KINDS],
            kind_frac: [0.0; KINDS],
            beacon_us: 0.0,
            beacon_frac: 0.0,
            step_tx: 0,
            pps: 0.0,
            last_bcast: None,
            ledger: AirLedger::new(nodes),
        };
        ch.refresh_derived();
        ch
    }

    pub fn config(&self) -> &ChannelConfig {
        &self.cfg
    }

    pub fn enabled(&self) -> bool {
        self.cfg.enabled
    }

    pub fn set_enabled(&mut self, on: bool) {
        self.cfg.enabled = on;
    }

    /// Clamped to `0..=MAX_EXTERNAL_BUSY`; NaN means 0.
    pub fn set_external_busy(&mut self, busy: f32) {
        self.cfg.external_busy = clamp_busy(busy);
        self.refresh_derived();
    }

    pub fn set_config(&mut self, cfg: ChannelConfig) {
        self.cfg = sanitize(cfg);
        self.geo_at_ms = None; // cs range may have changed
        self.refresh_derived();
    }

    /// The byte ledger: every frame on air, by node and kind.
    pub fn ledger(&self) -> &AirLedger {
        &self.ledger
    }

    /// Starts a step of `dt_ms`: clears the per-step accumulators.
    pub fn begin_step(&mut self, now_ms: u32, dt_ms: u32) {
        self.now_ms = now_ms;
        self.dt_ms = dt_ms;
        self.own_us.fill(0.0);
        self.cont_us.fill(0.0);
        self.refused_us.fill(0.0);
        self.wait_us.fill(0.0);
        self.step_drops.fill(0);
        self.kind_us = [0.0; KINDS];
        self.beacon_us = 0.0;
        self.step_tx = 0;
    }

    /// Whether `src` may put a frame on air now. `false` means its queue
    /// is full and the frame is dropped (counted for `Sensors::tx_dropped`
    /// and the stats): drawn with the M/M/1/K blocking probability of its
    /// transmit queue.
    pub fn admit(&mut self, src: u16) -> bool {
        let i = usize::from(src);
        let Some(&p) = self.block_p.get(i) else { return true };
        if p > 0.0 && self.rng.f32() < p {
            self.tx_dropped[i] = self.tx_dropped[i].saturating_add(1);
            self.step_drops[i] = self.step_drops[i].saturating_add(1);
            self.refused_us[i] += self.frame_us[i];
            return false;
        }
        true
    }

    fn charge(&mut self, src: u16, kind: AirKind, us: f32, frames: u32) {
        let i = usize::from(src);
        if let Some(own) = self.own_us.get_mut(i) {
            *own += us;
            self.cont_us[i] += us;
            let a = &mut self.attempt_us[i];
            *a = if *a == 0.0 { us } else { *a + 0.1 * (us - *a) };
        }
        self.kind_us[kind as usize] += us;
        self.step_tx += frames;
    }

    fn note_frame_us(&mut self, src: u16, us: f32) {
        if let Some(f) = self.frame_us.get_mut(usize::from(src)) {
            *f = if *f == 0.0 { us } else { *f + 0.1 * (us - *f) };
        }
    }

    /// Puts a broadcast on air (one attempt, never ACKed): charges its
    /// airtime (channel on) and counts its bytes. Receivers then draw with
    /// [`Channel::lost_broadcast`]. Returns its channel time, µs.
    pub fn broadcast(&mut self, now_ms: u32, f: &AirFrame) -> f32 {
        let us = DIFS_US + backoff_us(CW_VO, 0) + phy_us(f.mpdu, f.rate_mbps);
        self.ledger.frame(now_ms, f.src, byte_kind(f.kind), f.mpdu, f.retx);
        if self.cfg.enabled {
            self.charge(f.src, f.kind, us, 1);
            self.note_frame_us(f.src, us);
            let rate = rate_index(sane_rate(f.rate_mbps)).unwrap_or(0);
            self.last_bcast = Some(LastBroadcast { src: f.src, mpdu: f.mpdu, rate });
        }
        us
    }

    /// Draws whether `rx` misses the last [`Channel::broadcast`] (channel
    /// on; the caller has already checked range and line of sight).
    pub fn lost_broadcast(&mut self, rx: u16) -> bool {
        let Some(b) = self.last_bcast else { return false };
        if !self.cfg.enabled || usize::from(rx) >= self.n {
            return false;
        }
        let (s, r) = (usize::from(b.src), usize::from(rx));
        let Some(sig) = self.signal_dbm(s, r) else { return false };
        !self.attempt_ok(s, r, false, sig, &RATES[b.rate], b.mpdu)
    }

    /// Puts a unicast on air: attempts it until the receiver ACKs it or
    /// [`MAX_ATTEMPTS`] are spent, charging every attempt's airtime (channel
    /// on) and counting every attempt's bytes and the receiver's 802.11 ACK.
    /// `reachable` is the caller's range / line-of-sight / liveness check:
    /// an unreachable receiver costs all the attempts. With the channel off
    /// a reachable frame is delivered at the first attempt.
    pub fn unicast(&mut self, now_ms: u32, f: &AirFrame, dst: Dst, reachable: bool) -> TxResult {
        let voice = f.kind == AirKind::Brain;
        let (s, station) = (usize::from(f.src), dst == Dst::Station);
        let r = match dst {
            Dst::Node(r) => usize::from(r),
            Dst::Station => s,
        };
        let on = self.cfg.enabled;
        let primary = rate_index(sane_rate(f.rate_mbps)).map_or(RATES[0], |i| RATES[i]);
        let fallback = rate_index(sane_rate(f.fallback_mbps)).map_or(primary, |i| RATES[i]);
        let sig = if !on || !reachable {
            None
        } else if station {
            Some(self.station_dbm())
        } else {
            self.signal_dbm(s, r)
        };
        let (mut us, mut delivered, mut attempts) = (0.0, false, 0);
        while attempts < MAX_ATTEMPTS {
            let rate = if attempts < MRR_PRIMARY_ATTEMPTS { primary } else { fallback };
            let a_us = unicast_attempt_us(f.mpdu, rate.mbps, attempts, voice);
            let retry = attempts > 0 || f.retx;
            us += a_us;
            attempts += 1;
            self.count_bytes(now_ms, f, retry);
            let ok = if !reachable {
                false
            } else if !on {
                true
            } else {
                match sig {
                    Some(sig) => self.attempt_ok(s, r, station, sig, &rate, f.mpdu),
                    None => true,
                }
            };
            if on {
                // The attempt and, when it got through, the ACK. The backoff
                // beyond the first attempt's is idle medium: queue time only.
                let extra = a_us - unicast_attempt_us(f.mpdu, rate.mbps, 0, voice);
                self.charge(f.src, f.kind, a_us - extra, 1 + u32::from(ok));
                if let Some(w) = self.wait_us.get_mut(s) {
                    *w += extra;
                }
            }
            if ok {
                delivered = true;
                // The receiver sends the 802.11 ACK; a station's ACK is
                // booked to the root it answers (it is not a drone).
                let acker = if station { f.src } else { r as u16 };
                self.ledger.frame(now_ms, acker, ByteKind::Ack, ACK_BYTES, false);
                break;
            }
        }
        if on {
            self.note_frame_us(f.src, us);
        }
        TxResult { delivered, us, attempts }
    }

    fn count_bytes(&mut self, now_ms: u32, f: &AirFrame, retry: bool) {
        match f.kind {
            AirKind::MeshData | AirKind::MeshControl => {
                self.ledger.segment(now_ms, f.src, f.mpdu, f.own_frac, f.ctl_frac, retry);
            }
            k => self.ledger.frame(now_ms, f.src, byte_kind(k), f.mpdu, retry),
        }
    }

    /// Books `dt_ms` of SoftAP beacons from every alive painlessMesh node in
    /// `nodes`: one [`BEACON_BYTES`] frame every [`BEACON_INTERVAL_MS`] at
    /// [`MESH_MIN_RATE_MBPS`], arithmetically (never queued or retried: the
    /// AP sends them at TBTT). Bytes always; airtime (under
    /// [`AirKind::MeshControl`], see [`Channel::beacon_frac`]) with the
    /// channel on. Beacons are not counted in [`Channel::pps`].
    pub fn beacons(&mut self, now_ms: u32, dt_ms: u32, nodes: &[NodeRadio]) {
        let count = dt_ms as f32 / BEACON_INTERVAL_MS;
        let us = count * beacon_airtime_us(MESH_MIN_RATE_MBPS);
        for (i, node) in nodes.iter().enumerate().take(self.n) {
            if !node.alive {
                continue;
            }
            self.ledger.beacons(now_ms, i as u16, BEACON_BYTES, count);
            if self.cfg.enabled {
                self.own_us[i] += us;
                self.kind_us[AirKind::MeshControl as usize] += us;
                self.beacon_us += us;
            }
        }
    }

    /// Mean signal of `s` at `r` with shadowing, dBm: from the geometry
    /// cache, or from the last known positions for a pair outside
    /// carrier-sense range (`None` before the first refresh).
    fn signal_dbm(&self, s: usize, r: usize) -> Option<f32> {
        if s >= self.n || r >= self.n || s == r {
            return None;
        }
        if let Some(&p) = self.pw.get(r * self.n + s)
            && p.is_finite()
        {
            return Some(p);
        }
        let (a, b) = (self.pos.get(s)?, self.pos.get(r)?);
        let d = (*a - *b).length_sq().sqrt();
        d.is_finite().then(|| self.radio.mean_rx_dbm(d) + LINK_BUDGET_DB + shadow_db(self.shadow_seed, s, r))
    }

    /// Static shadowing of the pair `(a, b)`, dB (what a node's RSSI reading
    /// of the other includes on top of the mean path loss).
    pub fn shadowing_db(&self, a: u16, b: u16) -> f32 {
        shadow_db(self.shadow_seed, usize::from(a), usize::from(b))
    }

    /// A copy of the shadowing field, usable while the channel is borrowed
    /// mutably.
    pub fn shadowing(&self) -> Shadowing {
        Shadowing(self.shadow_seed)
    }

    /// Mean signal of a node's WiFi station / router, dBm.
    fn station_dbm(&self) -> f32 {
        self.radio.mean_rx_dbm(STATION_DISTANCE) + LINK_BUDGET_DB
    }

    /// Collects the frames that may overlap an attempt `s → r` into `out`
    /// (`station`: the receiver is `s`'s own station, which hears what `s`
    /// hears).
    fn gather_interferers(&self, s: usize, r: usize, station: bool, out: &mut Vec<Interferer>) {
        out.clear();
        let n = self.n;
        if self.geo.len() == n * n && r < n {
            let ext = self.cfg.external_busy;
            let mut contend = 0.0;
            for &k in &self.active {
                let k = usize::from(k);
                if k == s || (k == r && station) {
                    continue;
                }
                // Hidden senders overlap whatever they send (beacons too);
                // contenders collide only through backoff, which beacons,
                // sent at TBTT, do not take part in.
                let f = self.cont_frac[k];
                if k == r {
                    // r itself: senses s, so it contends; it cannot receive
                    // while it transmits.
                    contend += f;
                    continue;
                }
                if self.geo[r * n + k] & SENSE == 0 {
                    continue;
                }
                if station || self.geo[s * n + k] & SENSE != 0 {
                    contend += f;
                } else {
                    let dbm = self.pw[r * n + k];
                    let h = self.own_frac[k];
                    let tk = self.attempt_us[k].max(1.0);
                    out.push(Interferer { p: 0.0, hidden: Some((h, tk)), dbm, external: false });
                }
            }
            // The contenders as a group overlap with `p_coll(b)` at their
            // busy share `b`, split between them by airtime; external
            // traffic is a group of its own (it neither adds to nor
            // dilutes the cell's collisions).
            if contend > 0.0 {
                let pc = collision_prob(contend) / contend;
                for &k in &self.active {
                    let k = usize::from(k);
                    if k == s || (k == r && station) {
                        continue;
                    }
                    let p = pc * self.cont_frac[k];
                    if k == r {
                        out.push(Interferer { p, hidden: None, dbm: f32::INFINITY, external: false });
                    } else if self.geo[r * n + k] & SENSE != 0 && (station || self.geo[s * n + k] & SENSE != 0) {
                        out.push(Interferer { p, hidden: None, dbm: self.pw[r * n + k], external: false });
                    }
                }
            }
            if ext > 0.0 {
                out.push(Interferer { p: collision_prob(ext), hidden: None, dbm: EXT_INTERFERER_DBM, external: true });
            }
        }
    }

    /// Draws one attempt `s → r` with mean signal `sig` dBm: fading, an
    /// overlapping frame (if any), capture and the PER at the SINR.
    fn attempt_ok(&mut self, s: usize, r: usize, station: bool, sig: f32, rate: &RateInfo, mpdu: u32) -> bool {
        let fade = rician_db(normals(&mut self.rng));
        let sig = sig + fade;
        let mut sc = core::mem::take(&mut self.scratch);
        self.gather_interferers(s, r, station, &mut sc);
        let ok = self.draw_attempt(sig, rate, mpdu, &sc);
        self.scratch = sc;
        ok
    }

    fn draw_attempt(&mut self, sig: f32, rate: &RateInfo, mpdu: u32, sc: &[Interferer]) -> bool {
        let t = phy_us(mpdu, rate.mbps);
        let (mut keep, mut sum_p) = (1.0f32, 0.0f32);
        for i in sc {
            let p = i.p_for(t);
            keep *= 1.0 - p;
            sum_p += p;
        }
        let p_any = 1.0 - keep;
        let mut i_mw = 0.0;
        if p_any > 0.0 {
            let u = self.rng.f32();
            if u < p_any {
                // Which one: weighted by its chance (u / p_any is uniform).
                let mut v = u / p_any * sum_p;
                let mut hit = sc[sc.len() - 1];
                for i in sc {
                    let p = i.p_for(t);
                    if v < p {
                        hit = *i;
                        break;
                    }
                    v -= p;
                }
                let dbm = if hit.external {
                    let (g, _) = normals(&mut self.rng);
                    (hit.dbm + EXT_INTERFERER_SIGMA_DB * g).clamp(EXT_INTERFERER_RANGE.0, EXT_INTERFERER_RANGE.1)
                } else {
                    hit.dbm
                };
                if !dbm.is_finite() || sig - dbm < capture_db(rate) {
                    return false;
                }
                i_mw = dbm_to_mw(dbm);
            }
        }
        let sinr = sig - mw_to_dbm(self.noise_mw + i_mw);
        let p = per(sinr, rate, mpdu);
        if p <= 1e-7 {
            return true;
        }
        p < 1.0 && self.rng.f32() >= p
    }

    /// Mean per-attempt failure of `s → r` at mean signal `sig`, given its
    /// possible overlaps `sc`: fading quantiles × (no overlap, or each
    /// overlapping frame at its mean power).
    fn mean_failure(&self, sig: f32, rate: &RateInfo, mpdu: u32, sc: &[Interferer]) -> f32 {
        let t = phy_us(mpdu, rate.mbps);
        let (mut keep, mut sum_p) = (1.0f32, 0.0f32);
        for i in sc {
            let p = i.p_for(t);
            keep *= 1.0 - p;
            sum_p += p;
        }
        let p_any = 1.0 - keep;
        let mut clean = 0.0;
        for q in &self.fade_q {
            clean += per(sig + q - mw_to_dbm(self.noise_mw), rate, mpdu);
        }
        clean /= FADE_Q as f32;
        if p_any <= 0.0 || sum_p <= 0.0 {
            return clean;
        }
        let cap = capture_db(rate);
        let fail_at = |i_dbm: f32| {
            let i_mw = dbm_to_mw(i_dbm);
            let mut f = 0.0;
            for q in &self.fade_q {
                let s = sig + q;
                f += if s - i_dbm < cap { 1.0 } else { per(s - mw_to_dbm(self.noise_mw + i_mw), rate, mpdu) };
            }
            f / FADE_Q as f32
        };
        let mut hit = 0.0;
        for i in sc {
            let f = if !i.dbm.is_finite() {
                1.0
            } else if i.external {
                // Three-point Gauss-Hermite over the external power's spread.
                let d = 3f32.sqrt() * EXT_INTERFERER_SIGMA_DB;
                let at = |x: f32| fail_at(x.clamp(EXT_INTERFERER_RANGE.0, EXT_INTERFERER_RANGE.1));
                (at(i.dbm - d) + 4.0 * at(i.dbm) + at(i.dbm + d)) / 6.0
            } else {
                fail_at(i.dbm)
            };
            hit += i.p_for(t) / sum_p * f;
        }
        (1.0 - p_any) * clean + p_any * hit
    }

    /// Rate of the mesh link `tx → rx` (Minstrel-like, see the module
    /// docs), at least [`MESH_MIN_RATE_MBPS`]; cached per geometry refresh.
    /// Before the channel has seen the geometry: the weakest rate.
    pub fn link_rate_mbps(&mut self, tx: u16, rx: u16) -> f32 {
        self.link_rates(tx, rx).0
    }

    /// The link's multi-rate retry chain: (best-throughput rate, most
    /// robust rate), see [`MRR_PRIMARY_ATTEMPTS`].
    pub fn link_rates(&mut self, tx: u16, rx: u16) -> (f32, f32) {
        self.cached_rates(usize::from(tx), usize::from(rx), MESH_MIN_RATE_MBPS)
    }

    /// Rate chain of `node`'s uplink to its station / router: the same
    /// choice, at least [`ChannelConfig::min_rate_mbps`].
    pub fn station_rates(&mut self, node: u16) -> (f32, f32) {
        let i = usize::from(node);
        self.cached_rates(i, i, self.cfg.min_rate_mbps)
    }

    fn cached_rates(&mut self, tx: usize, rx: usize, floor: f32) -> (f32, f32) {
        let n = self.n;
        if tx >= n || rx >= n {
            return (floor.max(1.0), floor.max(1.0));
        }
        if self.rate_cache.len() != n * n {
            self.rate_cache = vec![(0, 0.0, 0.0); n * n];
        }
        let slot = tx * n + rx;
        let (epoch, a, b) = self.rate_cache[slot];
        if epoch == self.epoch + 1 {
            return (a, b);
        }
        let (a, b) = self.choose_rates(tx, rx, floor);
        self.rate_cache[slot] = (self.epoch + 1, a, b);
        (a, b)
    }

    /// Minstrel's choice from the model's success probabilities: the rate
    /// with the best expected throughput for the first attempts, and as the
    /// retry fallback the rate that delivers the frame in the least
    /// expected channel time while keeping the loss after all
    /// [`MAX_ATTEMPTS`] at most [`MRR_RESIDUAL_LOSS`] (or, if none can, the
    /// one with the lowest residual loss). A link limited by its SNR falls
    /// back to a robust rate; a link limited by collisions, where a slow
    /// rate is barely more reliable and far longer on air, does not.
    fn choose_rates(&mut self, tx: usize, rx: usize, floor: f32) -> (f32, f32) {
        let station = tx == rx;
        let low = floor.max(MESH_MIN_RATE_MBPS);
        let sig = if station { Some(self.station_dbm()) } else { self.signal_dbm(tx, rx) };
        let Some(sig) = sig.filter(|_| !self.pos.is_empty()) else { return (low, low) };
        let mut sc = core::mem::take(&mut self.scratch);
        self.gather_interferers(tx, rx, station, &mut sc);
        let mut cand: [(f32, f32); AUTO_RATES.len()] = [(0.0, 1.0); AUTO_RATES.len()];
        let mut n = 0;
        let mut best = (f32::NEG_INFINITY, 0usize);
        for &k in &AUTO_RATES {
            let r = &RATES[k];
            // Without 802.11b (floor ≥ 6) no DSSS rate.
            if r.mbps < floor - 1e-3 || (floor >= 6.0 && r.phy != Phy::Ht) {
                continue;
            }
            let p = self.mean_failure(sig, r, RATE_REF_MPDU, &sc);
            let us = unicast_attempt_us(RATE_REF_MPDU, r.mbps, 0, false);
            if (1.0 - p) / us > best.0 {
                best = ((1.0 - p) / us, n);
            }
            cand[n] = (r.mbps, p);
            n += 1;
        }
        self.scratch = sc;
        if n == 0 {
            return (low, low);
        }
        let (primary, pa) = cand[best.1];
        let cost = |rate: f32, pf: f32| {
            // Expected channel time of the chain and its residual loss.
            let (mut reach, mut us) = (1.0f32, 0.0f32);
            for k in 0..MAX_ATTEMPTS {
                let (r, p) = if k < MRR_PRIMARY_ATTEMPTS { (primary, pa) } else { (rate, pf) };
                us += reach * unicast_attempt_us(RATE_REF_MPDU, r, k, false);
                reach *= p;
            }
            (us, reach)
        };
        let mut pick: Option<(f32, f32, f32)> = None; // (cost, loss, rate)
        for &(rate, pf) in &cand[..n] {
            if rate > primary {
                continue;
            }
            let (us, loss) = cost(rate, pf);
            let better = match pick {
                None => true,
                Some((pu, pl, _)) => {
                    let ok = loss <= MRR_RESIDUAL_LOSS;
                    let was_ok = pl <= MRR_RESIDUAL_LOSS;
                    (ok && (!was_ok || us < pu)) || (!ok && !was_ok && loss < pl)
                }
            };
            if better {
                pick = Some((us, loss, rate));
            }
        }
        (primary, pick.map_or(primary, |p| p.2))
    }

    /// Mean per-attempt failure of a brain frame `src → rx` (ESP-NOW rate,
    /// a 31 B heartbeat) under the current load; 1 when `rx` cannot decode
    /// `src` at all (out of range or blocked).
    pub fn attempt_failure(&self, src: u16, rx: u16) -> f32 {
        let (s, r) = (usize::from(src), usize::from(rx));
        if s >= self.n || r >= self.n || s == r || self.geo.len() != self.n * self.n {
            return 0.0;
        }
        if self.geo[r * self.n + s] & HEAR == 0 {
            return 1.0;
        }
        let Some(sig) = self.signal_dbm(s, r) else { return 0.0 };
        let rate = rate_index(self.cfg.espnow_rate_mbps).map_or(RATES[0], |i| RATES[i]);
        let mut sc = Vec::new();
        self.gather_interferers(s, r, false, &mut sc);
        self.mean_failure(sig, &rate, MAC_OVERHEAD + ESPNOW_OVERHEAD + 31, &sc)
    }

    /// Mean per-attempt failure of a unicast `tx → rx` of `mpdu` bytes at
    /// `rate_mbps` under the current load, and the link's mean signal
    /// (dBm, with shadowing); `None` before the geometry is known.
    pub fn unicast_failure(&self, tx: u16, rx: u16, rate_mbps: f32, mpdu: u32) -> Option<(f32, f32)> {
        let (s, r) = (usize::from(tx), usize::from(rx));
        let sig = if s == r { Some(self.station_dbm()) } else { self.signal_dbm(s, r) }?;
        let rate = rate_index(sane_rate(rate_mbps)).map_or(RATES[0], |i| RATES[i]);
        let mut sc = Vec::new();
        self.gather_interferers(s, r, s == r, &mut sc);
        Some((self.mean_failure(sig, &rate, mpdu, &sc), sig))
    }

    /// Mean loss of a brain frame `src → rx`: one attempt for a broadcast,
    /// all [`MAX_ATTEMPTS`] failing for an ESP-NOW unicast.
    pub fn link_loss(&self, src: u16, rx: u16, unicast: bool) -> f32 {
        let p = self.attempt_failure(src, rx);
        if unicast { p.powi(MAX_ATTEMPTS as i32) } else { p }
    }

    /// Closes the step: works out who hears what (carrier sense over
    /// `nodes`, using `radio` and `terrain`) and updates the smoothed
    /// fractions and the queues' blocking probability.
    pub fn end_step(&mut self, nodes: &[NodeRadio], terrain: &Terrain, radio: &RadioConfig) {
        let n = self.n;
        let stale = self.geo_at_ms.is_none_or(|t| self.now_ms.wrapping_sub(t) >= GEOMETRY_REFRESH_MS);
        if stale || self.geo.len() != n * n {
            self.refresh_geometry(nodes, terrain, radio);
        }
        let dt = self.dt_ms as f32;
        let dt_us = dt * 1_000.0;
        let a = if self.dt_ms == 0 { 0.0 } else { (dt / WINDOW_MS).min(1.0) };
        let frac = |us: f32| if dt_us > 0.0 { (us / dt_us).min(1.0) } else { 0.0 };
        for i in 0..n {
            let alive = nodes.get(i).is_some_and(|nd| nd.alive);
            let own = if alive { frac(self.own_us[i]) } else { 0.0 };
            let offered = if alive { frac(self.own_us[i] + self.refused_us[i] + self.wait_us[i]) } else { 0.0 };
            self.own_frac[i] += a * (own - self.own_frac[i]);
            let cont = if alive { frac(self.cont_us[i]) } else { 0.0 };
            self.cont_frac[i] += a * (cont - self.cont_frac[i]);
            self.offered_frac[i] += a * (offered - self.offered_frac[i]);
            let per_s = if self.dt_ms > 0 { f32::from(self.step_drops[i]) * 1000.0 / dt } else { 0.0 };
            self.drops_per_s[i] += a * (per_s - self.drops_per_s[i]);
        }
        let demand = |us: f32| if dt_us > 0.0 { us / dt_us } else { 0.0 };
        for r in 0..n {
            let alive = nodes.get(r).is_some_and(|nd| nd.alive);
            let (mut us, mut offered) = (0.0, 0.0);
            if alive {
                offered = self.own_us[r] + self.refused_us[r] + self.wait_us[r];
                let row = &self.geo[r * n..(r + 1) * n];
                for (k, &f) in row.iter().enumerate() {
                    if f & SENSE != 0 {
                        us += self.own_us[k];
                        offered += self.own_us[k] + self.refused_us[k] + self.wait_us[k];
                    }
                }
            }
            self.heard_frac[r] += a * (frac(us) - self.heard_frac[r]);
            self.demand_frac[r] += a * (demand(offered) - self.demand_frac[r]);
        }
        for k in 0..KINDS {
            self.kind_frac[k] += a * (frac(self.kind_us[k]) - self.kind_frac[k]);
        }
        self.beacon_frac += a * (frac(self.beacon_us) - self.beacon_frac);
        let pps = if self.dt_ms > 0 { self.step_tx as f32 * 1000.0 / dt } else { 0.0 };
        self.pps += a * (pps - self.pps);
        self.active.clear();
        self.active.extend((0..n).filter(|&k| self.own_frac[k] > ACTIVE_EPS).map(|k| k as u16));
        self.refresh_derived();
    }

    /// Recomputes the queues' blocking probability and the noise floor.
    fn refresh_derived(&mut self) {
        let k = self.cfg.tx_queue_frames;
        for i in 0..self.n {
            self.block_p[i] = if self.offered_frac[i] > 0.0 { queue_block_prob(self.utilisation(i as u16), k) } else { 0.0 };
        }
        let rise = dbm_to_mw(EXTERNAL_NOISE_RISE_DB) - 1.0;
        self.noise_mw = dbm_to_mw(NOISE_FLOOR_DBM) * (1.0 + rise * self.cfg.external_busy);
    }

    fn refresh_geometry(&mut self, nodes: &[NodeRadio], terrain: &Terrain, radio: &RadioConfig) {
        let n = self.n;
        self.geo.clear();
        self.geo.resize(n * n, 0);
        self.pw.clear();
        self.pw.resize(n * n, f32::NAN);
        self.pos.clear();
        self.pos.extend(nodes.iter().take(n).map(|nd| nd.pos));
        self.radio = *radio;
        self.geo_at_ms = Some(self.now_ms);
        self.epoch = self.epoch.wrapping_add(1);
        let range = radio.range.max(0.0);
        let cs = range * self.cfg.cs_range_factor;
        let (r2, cs2) = (range * range, cs * cs);
        for a in 0..n.min(nodes.len()) {
            if !nodes[a].alive {
                continue;
            }
            for b in a + 1..n.min(nodes.len()) {
                if !nodes[b].alive {
                    continue;
                }
                let d2 = (nodes[a].pos - nodes[b].pos).length_sq();
                if d2.is_nan() || d2 > cs2 || !terrain.line_of_sight(nodes[a].pos, nodes[b].pos) {
                    continue;
                }
                let f = if d2 <= r2 { HEAR | SENSE } else { SENSE };
                self.geo[a * n + b] = f;
                self.geo[b * n + a] = f;
                let p = radio.mean_rx_dbm(d2.sqrt()) + LINK_BUDGET_DB + shadow_db(self.shadow_seed, a, b);
                self.pw[a * n + b] = p;
                self.pw[b * n + a] = p;
            }
        }
    }

    /// Busy fraction at `node`: heard airtime plus external, at most 1.
    pub fn busy(&self, node: u16) -> f32 {
        let heard = self.heard_frac.get(usize::from(node)).copied().unwrap_or(0.0);
        (heard + self.cfg.external_busy).min(1.0)
    }

    /// Smoothed fraction of airtime `node` spends transmitting.
    pub fn airtime_frac(&self, node: u16) -> f32 {
        self.own_frac.get(usize::from(node)).copied().unwrap_or(0.0)
    }

    /// Utilisation of the medium in front of `node`'s transmit queue: the
    /// offered airtime of its carrier-sense domain (its own and everything
    /// it senses, retries and refused frames included) over what external
    /// traffic leaves (> 1 = saturated).
    pub fn utilisation(&self, node: u16) -> f32 {
        let Some(&d) = self.demand_frac.get(usize::from(node)) else { return 0.0 };
        d / (1.0 - self.cfg.external_busy).max(MIN_SHARE)
    }

    /// Mean wait for the medium plus queueing of a frame at `node`, µs
    /// (M/M/1 sojourn of a frame of its typical length, capped at the queue).
    pub fn access_delay_us(&self, node: u16) -> f32 {
        let i = usize::from(node);
        let Some(&f) = self.frame_us.get(i) else { return 0.0 };
        let share = (1.0 - self.busy(node)).max(MIN_SHARE);
        let rho = self.utilisation(node).min(0.99);
        let q = (rho / (1.0 - rho)).min(f32::from(self.cfg.tx_queue_frames));
        f / share * (1.0 + q)
    }

    /// Frames `node` had refused since the last call (for
    /// `Sensors::tx_dropped`); resets the count.
    pub fn take_tx_dropped(&mut self, node: u16) -> u16 {
        self.tx_dropped.get_mut(usize::from(node)).map_or(0, core::mem::take)
    }

    /// Smoothed refused frames per second at `node`.
    pub fn queue_drops_per_s(&self, node: u16) -> f32 {
        self.drops_per_s.get(usize::from(node)).copied().unwrap_or(0.0)
    }

    /// Smoothed fraction of channel time used by `kind`, network-wide.
    pub fn kind_frac(&self, kind: AirKind) -> f32 {
        self.kind_frac[kind as usize]
    }

    /// Smoothed fraction of channel time used by SoftAP beacons,
    /// network-wide (part of [`AirKind::MeshControl`]).
    pub fn beacon_frac(&self) -> f32 {
        self.beacon_frac
    }

    /// Smoothed frames on air per second, network-wide: every attempt
    /// (retries included) and every 802.11 ACK, beacons excluded.
    pub fn pps(&self) -> f32 {
        self.pps
    }

    /// Mean and max of [`Channel::busy`] over the nodes that are alive.
    pub fn busy_mean_max(&self, nodes: &[NodeRadio]) -> (f32, f32) {
        let (mut sum, mut max, mut n) = (0.0f32, 0.0f32, 0u32);
        for (i, node) in nodes.iter().enumerate() {
            if node.alive {
                let b = self.busy(i as u16);
                sum += b;
                max = max.max(b);
                n += 1;
            }
        }
        if n == 0 { (self.cfg.external_busy, self.cfg.external_busy) } else { (sum / n as f32, max) }
    }
}

fn byte_kind(k: AirKind) -> ByteKind {
    match k {
        AirKind::Brain => ByteKind::Brain,
        AirKind::MeshData => ByteKind::TlmData,
        AirKind::Ack => ByteKind::Ack,
        AirKind::MeshControl => ByteKind::MeshControl,
        AirKind::Uplink => ByteKind::Uplink,
    }
}

fn clamp_busy(busy: f32) -> f32 {
    if busy.is_nan() { 0.0 } else { busy.clamp(0.0, MAX_EXTERNAL_BUSY) }
}

fn sanitize(mut cfg: ChannelConfig) -> ChannelConfig {
    let d = ChannelConfig::default();
    cfg.external_busy = clamp_busy(cfg.external_busy);
    let rate = |r: f32, def: f32| if r.is_finite() && r >= 1.0 { r.min(54.0) } else { def };
    cfg.espnow_rate_mbps = rate(cfg.espnow_rate_mbps, d.espnow_rate_mbps);
    cfg.min_rate_mbps = rate(cfg.min_rate_mbps, d.min_rate_mbps);
    cfg.cs_range_factor =
        if cfg.cs_range_factor.is_finite() { cfg.cs_range_factor.clamp(1.0, 4.0) } else { d.cs_range_factor };
    cfg.tx_queue_frames = cfg.tx_queue_frames.max(1);
    cfg
}

#[cfg(test)]
mod tests;
