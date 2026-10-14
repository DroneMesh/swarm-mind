use super::*;

const STEP: u32 = 50;

fn terrain() -> Terrain {
    Terrain::for_preset(crate::Preset::Foothills, 1, 9, 100.0)
}

fn on(external: f32) -> ChannelConfig {
    ChannelConfig { enabled: true, external_busy: external, ..Default::default() }
}

fn at(x: f32) -> NodeRadio {
    NodeRadio { pos: Vec3::new(x, 2_000.0, 0.0), alive: true }
}

/// The field cell (the geometry of `tests/telemetry.rs` T5): node 0 the
/// AP, node 1 the ground-station laptop next to it, nodes 2.. the drones a
/// few metres away on a spiral, all in range of each other.
fn cell(stations: usize) -> Vec<NodeRadio> {
    (0..stations + 2)
        .map(|i| {
            let a = i as f32 * 1.1;
            let r = if i == 0 { 0.0 } else { 3.0 + i as f32 };
            NodeRadio { pos: Vec3::new(r * a.cos(), 2_000.0, r * a.sin()), alive: true }
        })
        .collect()
}

fn flat() -> Terrain {
    Terrain::from_heights(9, 400.0, vec![0.0; 81])
}

fn udp_mpdu(bytes: u32) -> u32 {
    QOS_MAC_OVERHEAD + LLC_SNAP + UDP_IP_OVERHEAD + bytes
}

/// `pps` total of `bytes`-byte UDP frames from `stations` nodes to the AP,
/// which forwards each to the laptop (two crossings, as in the field), at
/// `rate` Mbps, for `secs` seconds; the loss over the last two thirds
/// (refused anywhere or lost after all attempts).
fn field_loss(pps: f32, stations: usize, bytes: u32, rate: f32, external: f32, secs: u32) -> f32 {
    let nodes = cell(stations);
    let terrain = flat();
    let radio = RadioConfig::default();
    let mut ch = Channel::new(on(external), nodes.len(), 1);
    let per_station = pps / stations as f32;
    let mut due = vec![0.0f32; stations];
    let (mut sent, mut lost) = (0u32, 0u32);
    let steps = secs * 1000 / STEP;
    for step in 0..steps {
        let now = step * STEP;
        let measure = step >= steps / 3;
        ch.begin_step(now, STEP);
        for (s, d) in due.iter_mut().enumerate() {
            *d += per_station * STEP as f32 / 1000.0;
            while *d >= 1.0 {
                *d -= 1.0;
                let src = (s + 2) as u16;
                let hop = |ch: &mut Channel, from: u16, to: u16| {
                    ch.admit(from)
                        && ch
                            .unicast(now, &AirFrame::new(from, AirKind::Uplink, udp_mpdu(bytes), rate), Dst::Node(to), true)
                            .delivered
                };
                let ok = hop(&mut ch, src, 0) && hop(&mut ch, 0, 1);
                if measure {
                    sent += 1;
                    lost += u32::from(!ok);
                }
            }
        }
        ch.end_step(&nodes, &terrain, &radio);
    }
    lost as f32 / sent.max(1) as f32
}

#[test]
fn airtime_formulas() {
    // 31-byte heartbeat at 1 Mbps DSSS on AC_VO: 192 + 8 · (28 + 15 + 31) µs + AIFS/backoff.
    let hb = espnow_airtime_us(31, 1.0);
    assert!((hb - (DIFS_US + 1.5 * SLOT_US + 192.0 + 8.0 * 74.0)).abs() < 1e-3, "{hb}");
    // Short preamble from 2 Mbps up.
    assert_eq!(phy_us(100, 2.0), 96.0 + 400.0);
    assert_eq!(phy_us(100, 11.0), 96.0 + 800.0 / 11.0);
    // A ~400 B telemetry frame at 13 Mbps HT is a few hundred µs.
    let t = wifi_airtime_us(400 + TCP_IP_OVERHEAD, 13.0);
    assert!((300.0..500.0).contains(&t), "{t}");
    assert!(wifi_airtime_us(400, 26.0) < wifi_airtime_us(400, 13.0));
    assert!(phy_us(100, f32::NAN).is_finite());
    // 14 B ACK at 24 Mbps: 20 + 4 · ceil(134 / 96) + 6 = 34 µs; HT-mixed preamble 36 µs.
    assert_eq!(phy_us(ACK_BYTES, 24.0), 34.0);
    assert_eq!(phy_us(ACK_BYTES, 26.0), 36.0 + 4.0 * (134.0f32 / 104.0).ceil() + 6.0);
    // ACK rates: DSSS itself, OFDM / HT the basic rate at or below.
    assert_eq!((ack_rate_mbps(2.0), ack_rate_mbps(9.0), ack_rate_mbps(19.5), ack_rate_mbps(65.0)), (2.0, 6.0, 12.0, 24.0));
    // Best effort waits AIFS[BE] = 37 µs, and each retry doubles CW.
    assert_eq!(AIFS_BE_US, 37.0);
    let first = unicast_attempt_us(500, 2.0, 0, false);
    let third = unicast_attempt_us(500, 2.0, 2, false);
    assert!((third - first - (63.0 - 15.0) / 2.0 * SLOT_US).abs() < 1e-3);
    assert_eq!(unicast_attempt_us(500, 2.0, 9, false), unicast_attempt_us(500, 2.0, 12, false), "CWmax");
    // Beacons: no backoff.
    assert_eq!(beacon_airtime_us(1.0), DIFS_US + 192.0 + 8.0 * BEACON_BYTES as f32);
}

#[test]
fn retries_queue_and_per_formulas() {
    assert_eq!(expected_attempts(0.0), 1.0);
    assert!((expected_attempts(0.5) - 1.984375).abs() < 1e-5, "7 attempts at most");
    assert_eq!(expected_attempts(1.0), 7.0);
    assert_eq!(collision_prob(0.0), 0.0);
    assert!(collision_prob(0.5) < collision_prob(0.8));
    assert!(collision_prob(f32::NAN) == 0.0 && collision_prob(5.0) < 1.0);
    assert_eq!(queue_block_prob(0.0, 8), 0.0);
    assert!(queue_block_prob(0.5, 8) < 0.01);
    assert!((queue_block_prob(1.0, 8) - 1.0 / 9.0).abs() < 1e-3);
    assert!((queue_block_prob(2.0, 8) - 0.5).abs() < 0.01);
    assert_eq!(queue_block_prob(f32::NAN, 8), 0.0);
    // PER: 10 % at the sensitivity for 1000 B, monotone, shorter is safer.
    for r in &RATES {
        let th = threshold_db(r);
        assert!((per(th, r, 1000) - 0.1).abs() < 1e-3, "{}", r.mbps);
        assert!(per(th + 3.0, r, 1000) < 0.01 && per(th - 3.0, r, 1000) > 0.9);
        assert!(per(th, r, 100) < per(th, r, 1000));
    }
    assert!(packet_error_rate(5.0, 1.0, 100) < 1e-4);
    assert!(packet_error_rate(5.0, 65.0, 100) > 0.999);
    // Fading quantiles have unit mean power.
    let ch = Channel::new(on(0.0), 2, 1);
    let mean = ch.fade_q.iter().map(|&q| dbm_to_mw(q)).sum::<f32>() / FADE_Q as f32;
    assert!((mean - 1.0).abs() < 0.02, "{mean}");
    assert!(ch.fade_q[0] < -3.0 && ch.fade_q[FADE_Q - 1] > 1.0, "{:?}", ch.fade_q);
}

#[test]
fn shadowing_is_static_symmetric_and_about_5_db() {
    let seed = 42;
    assert_eq!(shadow_db(seed, 3, 9), shadow_db(seed, 9, 3));
    let v: Vec<f32> = (0..2000).map(|i| shadow_db(seed, i, i + 1)).collect();
    let mean = v.iter().sum::<f32>() / v.len() as f32;
    let sd = (v.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / v.len() as f32).sqrt();
    assert!(mean.abs() < 0.4 && (4.3..5.5).contains(&sd), "{mean} {sd}");
}

#[test]
fn config_is_sanitized_and_external_busy_counts() {
    let mut ch = Channel::new(on(0.0), 3, 1);
    for v in [f32::NAN, -1.0, 5.0, f32::INFINITY] {
        ch.set_external_busy(v);
        assert!((0.0..=MAX_EXTERNAL_BUSY).contains(&ch.config().external_busy));
    }
    ch.set_external_busy(0.3);
    // Nodes far apart: nobody hears anybody, busy is the external part.
    let nodes: Vec<NodeRadio> = (0..3).map(|i| at(i as f32 * 5_000.0)).collect();
    let terrain = terrain();
    for step in 0..40 {
        ch.begin_step(step * STEP, STEP);
        if ch.admit(0) {
            let f = AirFrame::new(0, AirKind::Brain, 4_000, 1.0);
            ch.broadcast(step * STEP, &f);
        }
        ch.end_step(&nodes, &terrain, &RadioConfig::default());
    }
    assert!((ch.busy(1) - 0.3).abs() < 1e-6);
    assert!(ch.airtime_frac(0) > 0.3, "{}", ch.airtime_frac(0));
    assert_eq!(ch.airtime_frac(99), 0.0);
    assert_eq!(ch.busy(99), 0.3);
    assert!(ch.pps() > 15.0);
    assert_eq!(ch.link_loss(0, 999, false), 0.0);
    assert_eq!(ch.link_loss(0, 1, false), 1.0, "5 km apart: undecodable");
    assert!(ch.access_delay_us(0) > 0.0 && ch.access_delay_us(999) == 0.0);
}

#[test]
fn quiet_channel_never_draws_before_it_knows_the_geometry() {
    let mut ch = Channel::new(on(0.0), 4, 1);
    let before = ch.rng.clone().next_u64();
    for _ in 0..100 {
        assert!(ch.admit(1));
        ch.broadcast(0, &AirFrame::new(1, AirKind::Brain, 74, 1.0));
        assert!(!ch.lost_broadcast(2));
    }
    assert_eq!(ch.rng.next_u64(), before, "no draws while nothing can be lost");
}

#[test]
fn disabled_channel_only_counts_bytes() {
    let mut ch = Channel::new(ChannelConfig::default(), 3, 1);
    let before = ch.rng.clone().next_u64();
    let f = AirFrame::new(1, AirKind::MeshData, 500, 1.0);
    for t in (0..3_000).step_by(50) {
        let r = ch.unicast(t, &f, Dst::Node(2), true);
        assert!(r.delivered && r.attempts == 1);
        let r = ch.unicast(t, &f, Dst::Node(0), false);
        assert!(!r.delivered && r.attempts == MAX_ATTEMPTS, "gone: every attempt");
    }
    assert_eq!(ch.rng.next_u64(), before);
    assert_eq!((ch.airtime_frac(1), ch.pps()), (0.0, 0.0));
    let l = ch.ledger().node(3_000, 1);
    // 20 / s reachable + 7 × 20 / s unreachable frames of 500 B.
    assert!((l.all - 500.0 * 20.0 * 8.0).abs() < 50.0, "{l:?}");
    assert!((ch.ledger().node(3_000, 2).ack - 14.0 * 20.0).abs() < 1.0);
    assert!((ch.ledger().fleet(3_000).retry - 500.0 * 20.0 * 6.0).abs() < 50.0);
}

#[test]
fn carrier_sense_hears_neighbours_within_cs_range_only() {
    let radio = RadioConfig::default(); // range 260, cs 390
    let nodes = [at(0.0), at(350.0), at(1_000.0), NodeRadio { pos: Vec3::new(100.0, 2_000.0, 0.0), alive: false }];
    let terrain = terrain();
    let mut ch = Channel::new(on(0.0), 4, 1);
    let mut us = 0.0;
    for step in 0..60 {
        ch.begin_step(step * STEP, STEP);
        us = ch.broadcast(step * STEP, &AirFrame::new(0, AirKind::Brain, 1_200, 1.0));
        ch.end_step(&nodes, &terrain, &radio);
    }
    let f = us / (STEP as f32 * 1_000.0);
    assert!((ch.busy(1) - f).abs() < 0.01, "{} vs {f}", ch.busy(1));
    assert_eq!(ch.busy(2), 0.0);
    assert_eq!(ch.busy(3), 0.0, "dead nodes hear nothing");
    assert_eq!(ch.busy(0), 0.0, "own airtime is not busy");
}

/// Runs a busy hidden sender H next to C, with A and C both sending to R.
fn hidden_setup() -> Channel {
    // A -- R -- H in a line: A and H both reach R (250 each) but are 500
    // apart, beyond carrier sense (390). C sits next to H.
    let radio = RadioConfig::default();
    let nodes = [at(0.0), at(250.0), at(500.0), at(480.0)];
    let terrain = terrain();
    let mut ch = Channel::new(on(0.0), 4, 1);
    for step in 0..300 {
        ch.begin_step(step * STEP, STEP);
        ch.broadcast(step * STEP, &AirFrame::new(2, AirKind::Brain, 1_200, 1.0)); // H busy ~20 %
        ch.end_step(&nodes, &terrain, &radio);
    }
    ch
}

#[test]
fn hidden_terminals_hurt_only_senders_that_cannot_sense_them() {
    let ch = hidden_setup();
    // Frames as long as H's: the hidden sender overlaps over two frame
    // lengths, the sensed one only through a backoff collision.
    let long = MAC_OVERHEAD + ESPNOW_OVERHEAD + 1_200;
    let from_a = ch.unicast_failure(0, 1, 1.0, long).unwrap().0;
    let from_c = ch.unicast_failure(3, 1, 1.0, long).unwrap().0;
    assert!(from_a > from_c + 0.05, "hidden {from_a} vs sensing {from_c}");
    // A short frame is far less exposed to the hidden sender than a long one.
    let short = ch.unicast_failure(0, 1, 1.0, 100).unwrap().0;
    assert!(short < from_a - 0.1, "short {short} vs long {from_a}");
    assert!(ch.link_loss(0, 1, true) < ch.link_loss(0, 1, false), "retries help");
}

#[test]
fn strong_links_capture_and_weak_links_lose_under_load() {
    // R at 0 hears S1 at 30 (strong) and S2 at 210 (marginal); a busy
    // contender B at 120 transmits a third of the time.
    let radio = RadioConfig::default();
    let nodes = [at(0.0), at(30.0), at(210.0), at(120.0)];
    let terrain = terrain();
    let run = |busy_us: f32, external: f32| {
        let mut ch = Channel::new(on(external), 4, 3);
        for step in 0..200 {
            ch.begin_step(step * STEP, STEP);
            if busy_us > 0.0 {
                ch.broadcast(step * STEP, &AirFrame::new(3, AirKind::Brain, (busy_us / 8.0) as u32, 1.0));
            }
            ch.end_step(&nodes, &terrain, &radio);
        }
        let mut lost = [0u32; 2];
        for k in 0..4_000 {
            for (j, src) in [1u16, 2].into_iter().enumerate() {
                ch.broadcast(k, &AirFrame::new(src, AirKind::Brain, 74, 1.0));
                lost[j] += u32::from(ch.lost_broadcast(0));
            }
        }
        (lost[0] as f32 / 4_000.0, lost[1] as f32 / 4_000.0)
    };
    let (strong_q, weak_q) = run(0.0, 0.0);
    let (strong_b, weak_b) = run(16_000.0, 0.0);
    let (strong_x, weak_x) = run(0.0, 0.5);
    println!("quiet {strong_q:.3}/{weak_q:.3}, busy {strong_b:.3}/{weak_b:.3}, external {strong_x:.3}/{weak_x:.3}");
    assert!(strong_q < 0.01, "a 30-unit link is clean on a quiet channel: {strong_q}");
    assert!(weak_b > weak_q + 0.15, "load costs the marginal link: {weak_q} → {weak_b}");
    assert!(weak_b > 2.0 * strong_b + 0.1, "the weak link loses first and most: {strong_b} vs {weak_b}");
    assert!(strong_b < 0.15, "the strong link captures over the contender: {strong_b}");
    assert!(weak_x > weak_q + 0.15 && strong_x < 0.05, "external busy: {strong_x} / {weak_x}");
}

#[test]
fn mesh_rate_follows_the_link_and_feeds_the_airtime() {
    let radio = RadioConfig::default();
    let nodes = [at(0.0), at(10.0), at(120.0), at(255.0)];
    let terrain = terrain();
    let mut ch = Channel::new(on(0.0), 4, 1);
    // Before the first geometry: the weakest rate.
    assert_eq!(ch.link_rate_mbps(0, 1), MESH_MIN_RATE_MBPS);
    for step in 0..10 {
        ch.begin_step(step * STEP, STEP);
        ch.end_step(&nodes, &terrain, &radio);
    }
    let (near, mid, far) = (ch.link_rate_mbps(0, 1), ch.link_rate_mbps(0, 2), ch.link_rate_mbps(0, 3));
    println!("rates near {near} mid {mid} far {far}; station {}", ch.station_rates(0).0);
    assert!(near >= 52.0, "{near}");
    assert!(near > mid && mid > far, "{near} {mid} {far}");
    assert!(far <= 6.5, "the edge of range rides the most robust rates: {far}");
    assert!(ch.station_rates(0).0 >= 13.0, "router minimum, HT only");
    assert_eq!(ch.link_rate_mbps(0, 99), MESH_MIN_RATE_MBPS);
    // A slow link costs far more air for the same segment.
    let ip = 400 + TCP_IP_OVERHEAD;
    assert!(wifi_airtime_us(ip, far) > 3.0 * wifi_airtime_us(ip, near));
    // Busy air pushes a middling link down, not up.
    let mut busy = Channel::new(on(0.6), 4, 1);
    for step in 0..10 {
        busy.begin_step(step * STEP, STEP);
        busy.end_step(&nodes, &terrain, &radio);
    }
    assert!(busy.link_rate_mbps(0, 2) <= mid);
}

/// Every alive mesh node's SoftAP beacons ~1.6 % of the time at 1 Mbps.
#[test]
fn softap_beacons_cost_airtime_per_alive_node() {
    let us = beacon_airtime_us(MESH_MIN_RATE_MBPS);
    let per_node = us / (BEACON_INTERVAL_MS * 1_000.0);
    assert!((0.015..0.017).contains(&per_node), "{per_node}");
    let mut nodes = cell(18); // 20 nodes, all in range
    nodes[5].alive = false;
    let terrain = terrain();
    let mut ch = Channel::new(on(0.0), nodes.len(), 1);
    for step in 0..300 {
        ch.begin_step(step * STEP, STEP);
        ch.beacons(step * STEP, STEP, &nodes);
        ch.end_step(&nodes, &terrain, &RadioConfig::default());
    }
    assert!((ch.airtime_frac(0) - per_node).abs() < 1e-4, "{}", ch.airtime_frac(0));
    assert_eq!(ch.airtime_frac(5), 0.0, "dead nodes do not beacon");
    assert!((ch.busy(0) - 18.0 * per_node).abs() < 1e-3);
    assert!((ch.beacon_frac() - 19.0 * per_node).abs() < 1e-3);
    assert!((ch.kind_frac(AirKind::MeshControl) - ch.beacon_frac()).abs() < 1e-6);
    assert_eq!(ch.pps(), 0.0, "beacons are not counted in pps");
    let fleet = ch.ledger().fleet(300 * STEP);
    assert!((fleet.frames - 19.0 * 1000.0 / BEACON_INTERVAL_MS).abs() < 1.0, "{fleet:?}");
}

#[test]
fn queue_saturation_refuses_frames_and_reports_them() {
    let nodes = cell(1);
    let terrain = terrain();
    let mut ch = Channel::new(on(0.6), nodes.len(), 3);
    let mut refused = 0u32;
    for step in 0..200 {
        ch.begin_step(step * STEP, STEP);
        for _ in 0..20 {
            if ch.admit(2) {
                ch.unicast(step * STEP, &AirFrame::new(2, AirKind::MeshData, 400, 2.0), Dst::Node(0), true);
            } else {
                refused += 1;
            }
        }
        ch.end_step(&nodes, &terrain, &RadioConfig::default());
    }
    assert!(ch.utilisation(2) > 1.0, "{}", ch.utilisation(2));
    assert!(refused > 500, "{refused}");
    assert!(ch.queue_drops_per_s(2) > 50.0);
    let taken = ch.take_tx_dropped(2);
    assert!(taken > 0 && ch.take_tx_dropped(2) == 0);
    assert!(ch.busy(0) > 0.6);
}

/// Field: ~70 pps of ~400 B frames → ~1 % (≤ 3 %); 6 × 25 Hz ≈ 150 pps
/// → 26-32 % (20-40 %), at the PHY rates the field actually used.
#[test]
fn field_calibration_reproduces_the_measured_ceiling() {
    let rate = FIELD_PRE_HYGIENE_RATE_MBPS;
    let clean = field_loss(70.0, 4, 400, rate, 0.0, 60);
    let collapse = field_loss(150.0, 6, 400, rate, 0.0, 60);
    assert!(clean <= 0.03, "70 pps: {:.1} %", clean * 100.0);
    assert!((0.20..=0.40).contains(&collapse), "150 pps: {:.1} %", collapse * 100.0);
}

/// After the radio hygiene (802.11b off, 12 Mbps minimum) the same loads
/// fit easily, as `UDP_VS_MQTT.md` expects.
#[test]
fn field_calibration_at_12_mbps_is_clean() {
    let loss = field_loss(150.0, 6, 400, 12.0, 0.0, 30);
    assert!(loss < 0.02, "{:.1} %", loss * 100.0);
    assert!(field_loss(60.0, 6, 400, 12.0, 0.0, 30) < 0.01);
}

#[test]
fn loss_is_monotone_in_load_and_external_busy() {
    let rate = FIELD_PRE_HYGIENE_RATE_MBPS;
    let mut last = -1.0;
    for pps in [20.0, 50.0, 80.0, 110.0, 140.0, 170.0, 200.0] {
        let l = field_loss(pps, 6, 400, rate, 0.0, 30);
        assert!(l >= last - 0.02, "{pps} pps: {l} < {last}");
        last = l;
    }
    assert!(last > 0.3, "200 pps saturates: {last}");
    let mut last = -1.0;
    for ext in [0.0, 0.2, 0.4, 0.6, 0.8] {
        let l = field_loss(300.0, 6, 400, 12.0, ext, 30);
        assert!(l >= last - 0.02, "external {ext}: {l} < {last}");
        last = l;
    }
    assert!(field_loss(300.0, 6, 400, 12.0, 0.0, 30) < 0.02);
    assert!(last > 0.5, "an 80 %-busy channel hurts: {last}");
}

/// `cargo test -p swarm-sim channel_sweep -- --ignored --nocapture`
#[test]
#[ignore]
fn channel_sweep() {
    for rate in [FIELD_PRE_HYGIENE_RATE_MBPS, 12.0] {
        for pps in [40.0, 70.0, 75.0, 90.0, 100.0, 120.0, 150.0, 200.0, 300.0] {
            println!("rate {rate:>4} pps {pps:>5}: {:5.1} %", field_loss(pps, 6, 400, rate, 0.0, 60) * 100.0);
        }
    }
    for ext in [0.0, 0.1, 0.2, 0.25, 0.3] {
        println!("2 Mbps 75 pps ext {ext}: {:5.1} %", field_loss(75.0, 5, 400, 2.0, ext, 60) * 100.0);
    }
    for pps in [60.0, 300.0, 600.0] {
        for ext in [0.0, 0.2, 0.4, 0.6, 0.8] {
            println!("12 Mbps {pps} pps ext {ext}: {:5.1} %", field_loss(pps, 6, 400, 12.0, ext, 30) * 100.0);
        }
    }
}

#[test]
fn channel_is_deterministic() {
    let run = || {
        let nodes = cell(6);
        let terrain = terrain();
        let mut ch = Channel::new(on(0.2), nodes.len(), 11);
        let mut trace = Vec::new();
        for step in 0..100 {
            ch.begin_step(step * STEP, STEP);
            for s in 2..8u16 {
                let ok = ch.admit(s)
                    && ch.unicast(step * STEP, &AirFrame::new(s, AirKind::MeshData, 1_000, 2.0), Dst::Node(0), true).delivered;
                trace.push(ok);
            }
            ch.end_step(&nodes, &terrain, &RadioConfig::default());
        }
        (trace, ch.busy(0).to_bits(), ch.pps().to_bits())
    };
    assert_eq!(run(), run());
}
