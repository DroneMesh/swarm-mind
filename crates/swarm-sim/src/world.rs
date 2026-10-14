use swarm_brain::{
    BROADCAST, Brain, Config, Frame, Inbox, Intent, Message, OPERATOR, Outbox, Payload, Rx,
    Sensors, Vec3,
};

use crate::channel::{self, AirFrame, AirKind, Channel, Dst, NodeRadio};
use crate::linkq::LinkMeter;
use crate::painless::PainlessMesh;
use crate::rng::Rng;
use crate::telemetry::{DroneView, Telemetry};
use crate::{ChannelConfig, FireGrid, Preset, RadioConfig, TelemetryConfig, Terrain, chatter, snapshot};

/// The base station is always drone 0 and acts as the network sink.
pub const BASE_ID: u16 = 0;

/// Drones never fly closer to the ground than this.
const MIN_CLEARANCE: f32 = 5.0;

/// Drones never fly higher than this, whatever their brain asks for.
const CEILING: f32 = 1_000.0;

/// A dead drone falls with this acceleration (units/s²) ...
const GRAVITY: f32 = 9.81;
/// ... up to this speed (units/s), so even a high relay takes a few seconds
/// to come down: long enough for the viewer to watch it drop out of the sky.
const TERMINAL_SPEED: f32 = 30.0;
/// A wreck rests this far above the ground, so its mesh is not buried.
const WRECK_CLEARANCE: f32 = 1.0;

/// Radio cache markers for [`Sim::deliver`]; real RSSI fits in an `i8`.
const HEAR_UNKNOWN: i16 = i16::MAX;
const HEAR_NONE: i16 = i16::MIN;

#[derive(Clone, Copy, Debug)]
pub struct SimConfig {
    pub seed: u64,
    /// Total drones, including the base station.
    pub drones: usize,
    /// Width of the square map.
    pub extent: f32,
    /// Terrain samples per side.
    pub terrain_size: usize,
    pub radio: RadioConfig,
    pub brain: Config,
    /// Cruise height above ground.
    pub cruise_agl: f32,
    /// Top speed in units per second.
    pub max_speed: f32,
    /// Fire grid cells per side (covers the whole map).
    pub fire_size: usize,
    /// Where the fire starts, as `(x, z)`.
    pub ignition: (f32, f32),
    /// Ground radius the downward thermal sensor covers.
    pub sensor_radius: f32,
    /// Wind as `(x, z)` in units per second. It only steers the fire; the
    /// drones are assumed to hold position against it.
    pub wind: (f32, f32),
    /// Which map: picks the terrain generator and the base location.
    /// `ignition` and `wind` are plain fields, seeded by [`SimConfig::for_preset`].
    pub preset: Preset,
    /// Shared-airtime model (off by default; see [`crate::channel`]).
    pub channel: ChannelConfig,
    /// Real-system telemetry over painlessMesh (off by default; see
    /// [`crate::telemetry`]).
    pub telemetry: TelemetryConfig,
}

impl SimConfig {
    /// The standard setup for `preset`: its terrain, base, ignition point
    /// and wind. `for_preset(Preset::AlpineRidge, 1, 20)` is the default.
    pub fn for_preset(preset: Preset, seed: u64, drones: usize) -> Self {
        let base = Self::default();
        Self {
            seed,
            drones,
            ignition: preset.ignition(base.extent),
            wind: preset.wind(),
            preset,
            ..base
        }
    }
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            seed: 1,
            drones: 20,
            extent: 1200.0,
            terrain_size: 129,
            radio: RadioConfig::default(),
            brain: Config::default(),
            cruise_agl: 25.0,
            max_speed: 12.0,
            fire_size: 96,
            ignition: (330.0, 40.0),
            sensor_radius: 60.0,
            // A gentle westerly drift: the fire climbs toward the ridge.
            wind: (-3.0, 0.0),
            preset: Preset::AlpineRidge,
            channel: ChannelConfig::default(),
            telemetry: TelemetryConfig::default(),
        }
    }
}

/// Radio traffic during the last step.
#[derive(Clone, Copy, Debug, Default)]
pub struct StepStats {
    pub frames: u32,
    pub bytes: u32,
    /// Deliveries lost because a receiver's inbox was full.
    pub dropped: u32,
}

struct Drone {
    brain: Brain,
    pos: Vec3,
    alive: bool,
    /// Downward speed while a dead drone falls; zero while alive.
    fall_speed: f32,
    inbox: Inbox,
    /// Velocity over the last step (units/s), for telemetry.
    vel: Vec3,
    /// The brain's latest `Actuation::telemetry_interval_ms`.
    tlm_interval_ms: u16,
}

/// A frame put on the air during the last step.
#[derive(Clone, Copy, Debug)]
struct Transmission {
    src: u16,
    dst: u16,
    kind: u8,
}

pub struct Sim {
    cfg: SimConfig,
    terrain: Terrain,
    fire: FireGrid,
    rng: Rng,
    now_ms: u32,
    tick: u32,
    drones: Vec<Drone>,
    command_seq: u16,
    stats: StepStats,
    // Scratch buffers reused every step to avoid per-step allocation.
    outgoing: Vec<(usize, Frame)>,
    sent: Vec<Transmission>,
    links: Vec<(u16, u16, u8)>,
    snapshot: Vec<f32>,
    /// Per receiver: RSSI from the current sender, or a `HEAR_*` marker.
    hear: Vec<i16>,
    /// Decoded frames of the latest step; see [`chatter`] for the layout.
    chatter: Vec<f32>,
    /// Operator frames handed to the base since the last step, logged into
    /// the chatter of the step that reads them.
    console: Vec<Frame>,
    channel: Channel,
    /// Measured brain-frame delivery per pair, for the snapshot's links.
    linkq: LinkMeter,
    mesh: PainlessMesh,
    tlm: Telemetry,
    /// Scratch: per-drone radio view and telemetry view, rebuilt each step
    /// while the channel or telemetry is on.
    nodes: Vec<NodeRadio>,
    views: Vec<DroneView>,
}

impl Sim {
    pub fn new(cfg: SimConfig) -> Self {
        assert!(
            (1..usize::from(OPERATOR)).contains(&cfg.drones),
            "drone count must fit below the reserved ids"
        );
        let terrain = Terrain::for_preset(cfg.preset, cfg.seed, cfg.terrain_size, cfg.extent);
        let mut rng = Rng::new(cfg.seed);
        let drones = (0..cfg.drones)
            .map(|i| {
                let pos = spawn_point(i, &cfg, &terrain, &mut rng);
                let is_sink = i == usize::from(BASE_ID);
                Drone {
                    brain: Brain::new(i as u16, is_sink, pos, cfg.brain),
                    pos,
                    alive: true,
                    fall_speed: 0.0,
                    inbox: Inbox::new(),
                    vel: Vec3::ZERO,
                    tlm_interval_ms: 0,
                }
            })
            .collect();
        let mut fire = FireGrid::new(cfg.fire_size, cfg.extent);
        fire.set_wind(cfg.wind.0, cfg.wind.1);
        fire.ignite(cfg.ignition.0, cfg.ignition.1);
        let mut sim = Self {
            cfg,
            terrain,
            fire,
            rng,
            now_ms: 0,
            tick: 0,
            drones,
            command_seq: 0,
            stats: StepStats::default(),
            outgoing: Vec::new(),
            sent: Vec::new(),
            links: Vec::new(),
            snapshot: Vec::new(),
            hear: Vec::new(),
            chatter: Vec::new(),
            console: Vec::new(),
            channel: Channel::new(cfg.channel, cfg.drones, cfg.seed),
            linkq: LinkMeter::new(cfg.drones),
            mesh: PainlessMesh::new(cfg.drones, BASE_ID),
            tlm: Telemetry::new(cfg.telemetry, cfg.drones, BASE_ID, cfg.seed),
            nodes: Vec::with_capacity(cfg.drones),
            views: Vec::with_capacity(cfg.drones),
        };
        sim.cfg.channel = *sim.channel.config();
        sim.cfg.telemetry = *sim.tlm.config();
        sim.build_snapshot();
        sim
    }

    /// Advance the world by `dt_ms`: every live brain thinks once, bodies
    /// move, and frames sent this step are delivered for the next one.
    pub fn step(&mut self, dt_ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(dt_ms);
        self.tick = self.tick.wrapping_add(1);
        self.stats = StepStats::default();
        self.outgoing.clear();
        self.chatter.clear();
        for frame in &self.console {
            chatter::record(&mut self.chatter, self.now_ms, frame);
        }
        self.console.clear();
        self.fire.step(dt_ms, &self.terrain, &mut self.rng);
        let ch_on = self.channel.enabled();
        let tlm_on = self.tlm.enabled();
        if ch_on {
            self.channel.begin_step(self.now_ms, dt_ms);
        }
        let telemetry_requested_ms = self.tlm.requested_ms();

        let max_step = self.cfg.max_speed * dt_ms as f32 / 1000.0;
        let half = self.cfg.extent * 0.5;
        let dt_s = dt_ms as f32 / 1000.0;
        for (i, d) in self.drones.iter_mut().enumerate() {
            let before = d.pos;
            if !d.alive {
                fall(d, dt_s, &self.terrain);
                d.vel = Vec3::new(0.0, -d.fall_speed, 0.0);
                continue;
            }
            let sensors = Sensors {
                now_ms: self.now_ms,
                pos: d.pos,
                ground_agl: d.pos.y - self.terrain.surface_at(d.pos.x, d.pos.z),
                fire: self.fire.read(d.pos, self.cfg.sensor_radius),
                tx_dropped: (if ch_on { self.channel.take_tx_dropped(i as u16) } else { 0 })
                    .saturating_add(if tlm_on { self.tlm.take_refused(i) } else { 0 }),
                telemetry_requested_ms,
            };
            let mut outbox = Outbox::new();
            let act = d.brain.tick(&sensors, &d.inbox, &mut outbox);
            d.inbox.clear();
            d.tlm_interval_ms = act.telemetry_interval_ms;
            if !d.brain.is_sink() {
                d.pos = fly(d.pos, act.target, max_step, half, &self.terrain);
            }
            d.vel = if dt_s > 0.0 { (d.pos - before) * (1.0 / dt_s) } else { Vec3::ZERO };
            self.outgoing.extend(outbox.iter().map(|f| (i, *f)));
        }

        self.deliver();
        if ch_on || tlm_on {
            self.step_airtime(dt_ms);
        }
        self.build_snapshot();
    }

    /// Telemetry, mesh upkeep and the channel's end of step; only runs
    /// while the channel or telemetry model is on.
    fn step_airtime(&mut self, dt_ms: u32) {
        self.nodes.clear();
        self.nodes.extend(self.drones.iter().map(|d| NodeRadio { pos: d.pos, alive: d.alive }));
        if self.tlm.enabled() {
            self.views.clear();
            let terrain = &self.terrain;
            self.views.extend(self.drones.iter().map(|d| DroneView {
                pos: d.pos,
                vel: d.vel,
                alive: d.alive,
                agl: d.pos.y - terrain.surface_at(d.pos.x, d.pos.z),
                brain_interval_ms: d.tlm_interval_ms,
            }));
            let (radio, nodes) = (self.cfg.radio, &self.nodes);
            let shadowing = self.channel.shadowing();
            // With the channel model on, a node's RSSI reading includes the
            // pair's shadowing, so painlessMesh picks parents by what it
            // really receives.
            let shadow = self.channel.enabled();
            let mut rssi = |a: usize, b: usize| match (nodes.get(a), nodes.get(b)) {
                (Some(na), Some(nb)) => radio.receive(terrain, nb.pos, na.pos).map(|r| {
                    if shadow {
                        (f32::from(r) + shadowing.db(a as u16, b as u16)).round().clamp(-127.0, -20.0) as i8
                    } else {
                        r
                    }
                }),
                _ => None,
            };
            self.tlm.step(
                self.now_ms,
                dt_ms,
                &self.views,
                &mut self.mesh,
                &mut self.channel,
                &mut rssi,
                &mut self.chatter,
            );
        }
        if self.tlm.enabled() {
            // painlessMesh runs while telemetry does: every node's SoftAP
            // beacons (bytes always, airtime with the channel on).
            self.channel.beacons(self.now_ms, dt_ms, &self.nodes);
        }
        if self.channel.enabled() {
            self.channel.end_step(&self.nodes, &self.terrain, &self.cfg.radio);
        }
        self.tlm.build_stats(self.now_ms, &self.channel, &self.mesh, &self.nodes);
    }

    /// Switch a drone off. It stops thinking and transmitting at once; the
    /// rest of the swarm only finds out by noticing its silence.
    pub fn kill(&mut self, id: u16) -> bool {
        match self.drones.get_mut(usize::from(id)) {
            Some(d) if d.alive => {
                d.alive = false;
                d.inbox.clear();
                true
            }
            _ => false,
        }
    }

    /// Issue an operator command about the area around `(x, z)`. It enters
    /// the network at the base station, like a radio console plugged into
    /// the base.
    ///
    /// Non-finite coordinates are rejected; points off the map are pulled
    /// onto its edge.
    pub fn command(&mut self, intent: Intent, x: f32, z: f32) -> bool {
        if !(x.is_finite() && z.is_finite()) {
            return false;
        }
        let half = self.cfg.extent * 0.5;
        let (x, z) = (x.clamp(-half, half), z.clamp(-half, half));
        let target = Vec3::new(x, self.terrain.surface_at(x, z), z);
        let base = &mut self.drones[usize::from(BASE_ID)];
        if !base.alive {
            return false;
        }
        self.command_seq = self.command_seq.wrapping_add(1);
        let payload = Payload::Intent { seq: self.command_seq, intent, target };
        let frame = Message { src: OPERATOR, dst: BASE_ID, payload }.encode();
        // The console is wired to the base, so signal strength is maximal.
        let ok = base.inbox.push(Rx { frame, rssi_dbm: -20 }).is_ok();
        if ok {
            self.console.push(frame);
        }
        ok
    }

    /// Probability, clamped to `[0, 1]`, that any single delivery is lost.
    /// NaN means no loss.
    pub fn set_loss(&mut self, loss: f32) {
        self.cfg.radio.loss = if loss.is_nan() { 0.0 } else { loss.clamp(0.0, 1.0) };
    }

    /// Turns the shared-airtime model on or off (see [`crate::channel`]).
    pub fn set_channel_enabled(&mut self, on: bool) {
        self.channel.set_enabled(on);
        self.cfg.channel = *self.channel.config();
    }

    /// Airtime used by other networks, clamped to
    /// `0..=channel::MAX_EXTERNAL_BUSY`; NaN means 0.
    pub fn set_external_busy(&mut self, busy: f32) {
        self.channel.set_external_busy(busy);
        self.cfg.channel = *self.channel.config();
    }

    /// Replaces the whole channel configuration (sanitised).
    pub fn set_channel(&mut self, cfg: ChannelConfig) {
        self.channel.set_config(cfg);
        self.cfg.channel = *self.channel.config();
    }

    /// Replaces the telemetry configuration (sanitised; rates are clamped).
    pub fn set_telemetry(&mut self, cfg: TelemetryConfig) {
        self.tlm.set_config(cfg);
        self.cfg.telemetry = *self.tlm.config();
    }

    /// Per-drone telemetry / airtime stats,
    /// [`crate::telemetry::TLM_STRIDE`] floats per drone (layout in
    /// [`crate::telemetry`]). Zero-filled while both models are off.
    pub fn telemetry_stats(&self) -> &[f32] {
        self.tlm.stats()
    }

    /// Fleet-wide channel / telemetry summary,
    /// [`crate::telemetry::CHANNEL_LEN`] floats (layout in [`crate::telemetry`]).
    pub fn channel_summary(&self) -> &[f32] {
        self.tlm.summary()
    }

    /// The last painlessMesh package text (exact bytes, without the
    /// trailing `'\0'`) drone `id` sent; "" if none or unknown.
    pub fn tlm_sample(&self, id: u16) -> &str {
        self.tlm.sample(id)
    }

    pub fn channel(&self) -> &Channel {
        &self.channel
    }

    pub fn painless(&self) -> &PainlessMesh {
        &self.mesh
    }

    pub fn telemetry(&self) -> &Telemetry {
        &self.tlm
    }

    /// Wind as `(x, z)` in units per second; only the fire feels it.
    /// Non-finite components are treated as calm.
    pub fn set_wind(&mut self, x: f32, z: f32) {
        self.fire.set_wind(x, z);
        self.cfg.wind = self.fire.wind();
    }

    /// Starts a spot fire at `(x, z)`, e.g. embers landing beyond the front.
    /// Off-map, non-finite or already-burnt points are ignored.
    pub fn ignite(&mut self, x: f32, z: f32) {
        if x.is_finite() && z.is_finite() {
            self.fire.ignite(x, z);
        }
    }

    pub fn terrain(&self) -> &Terrain {
        &self.terrain
    }

    pub fn fire(&self) -> &FireGrid {
        &self.fire
    }

    pub fn now_ms(&self) -> u32 {
        self.now_ms
    }

    pub fn stats(&self) -> StepStats {
        self.stats
    }

    pub fn drone_count(&self) -> usize {
        self.drones.len()
    }

    pub fn brain(&self, id: u16) -> Option<&Brain> {
        self.drones.get(usize::from(id)).map(|d| &d.brain)
    }

    pub fn is_alive(&self, id: u16) -> bool {
        self.drones.get(usize::from(id)).is_some_and(|d| d.alive)
    }

    pub fn position(&self, id: u16) -> Option<Vec3> {
        self.drones.get(usize::from(id)).map(|d| d.pos)
    }

    /// True if `a` and `b` are within radio range with line of sight
    /// (ignores random loss and liveness).
    pub fn in_contact(&self, a: u16, b: u16) -> bool {
        match (self.position(a), self.position(b)) {
            (Some(pa), Some(pb)) => self.cfg.radio.receive(&self.terrain, pa, pb).is_some(),
            _ => false,
        }
    }

    pub fn config(&self) -> &SimConfig {
        &self.cfg
    }

    /// The latest snapshot; see [`snapshot`] for the layout.
    pub fn snapshot(&self) -> &[f32] {
        &self.snapshot
    }

    /// Every frame on the air during the latest step, decoded; one
    /// [`chatter::CHATTER_STRIDE`]-float record each (see [`chatter`]).
    pub fn chatter(&self) -> &[f32] {
        &self.chatter
    }

    /// The map this world was built from.
    pub fn preset(&self) -> Preset {
        self.cfg.preset
    }

    fn deliver(&mut self) {
        self.sent.clear();
        let radio = self.cfg.radio;
        let ch_on = self.channel.enabled();
        let note_bytes = ch_on || self.tlm.enabled();
        let espnow_rate = self.channel.config().espnow_rate_mbps;
        let now = self.now_ms;
        let drones = &self.drones;
        self.linkq.roll(now, &|i| drones[i].alive);
        // A drone's frames for this step sit together in `outgoing`, and
        // nothing moves during delivery, so who can hear a sender is worked
        // out once per sender (and only for receivers it addresses).
        let mut cached_src = usize::MAX;
        for &(src, frame) in &self.outgoing {
            let dst = frame.dst();
            let len = frame.as_bytes().len() as u32;
            let air = AirFrame::new(src as u16, AirKind::Brain, channel::MAC_OVERHEAD + channel::ESPNOW_OVERHEAD + len, espnow_rate);
            // An addressed brain frame is an ESP-NOW unicast to a registered
            // peer: ACKed and retried by the MAC. Only the channel model
            // changes what that means for delivery.
            let unicast = dst != BROADCAST;
            if ch_on {
                // A full transmit queue refuses the frame: it never airs.
                if !self.channel.admit(src as u16) {
                    continue;
                }
            }
            if note_bytes && !unicast {
                self.channel.broadcast(now, &air);
            }
            self.stats.frames += 1;
            self.stats.bytes += frame.as_bytes().len() as u32;
            self.sent.push(Transmission { src: src as u16, dst, kind: frame.kind() });
            chatter::record(&mut self.chatter, now, &frame);
            if src != cached_src {
                cached_src = src;
                self.hear.clear();
                self.hear.resize(self.drones.len(), HEAR_UNKNOWN);
            }

            let from = self.drones[src].pos;
            let receivers: core::ops::Range<usize> = if !unicast {
                0..self.drones.len()
            } else {
                let j = usize::from(dst).min(self.drones.len());
                j..(j + 1).min(self.drones.len())
            };
            for j in receivers {
                let rx = &mut self.drones[j];
                if j == src {
                    continue;
                }
                if !rx.alive {
                    if unicast && note_bytes {
                        // Nobody ACKs: the MAC spends every attempt.
                        self.channel.unicast(now, &air, Dst::Node(j as u16), false);
                    }
                    continue;
                }
                let heard = &mut self.hear[j];
                if *heard == HEAR_UNKNOWN {
                    *heard = radio
                        .receive(&self.terrain, from, rx.pos)
                        .map_or(HEAR_NONE, i16::from);
                }
                let in_range = *heard != HEAR_NONE;
                let mut ok = in_range && !(radio.loss > 0.0 && self.rng.f32() < radio.loss);
                if unicast && note_bytes {
                    let r = self.channel.unicast(now, &air, Dst::Node(j as u16), in_range);
                    ok &= !ch_on || r.delivered;
                } else if ok && ch_on && self.channel.lost_broadcast(j as u16) {
                    ok = false;
                }
                self.linkq.record(j, src, ok);
                if !ok {
                    continue;
                }
                let rssi_dbm = *heard as i8;
                if rx.inbox.push(Rx { frame, rssi_dbm }).is_err() {
                    self.stats.dropped += 1;
                }
            }
        }
    }

    fn build_snapshot(&mut self) {
        self.links.clear();
        for (i, d) in self.drones.iter().enumerate() {
            if !d.alive {
                continue;
            }
            let me = i as u16;
            let parent = d.brain.parent();
            for n in d.brain.neighbors() {
                let (a, b) = if me < n.id { (me, n.id) } else { (n.id, me) };
                self.links.push((a, b, u8::from(parent == Some(n.id))));
            }
        }
        // Both ends usually list each other; keep one entry, marked as a
        // route if either end routes through the other.
        self.links.sort_unstable();
        self.links.dedup_by(|later, kept| {
            let same = (later.0, later.1) == (kept.0, kept.1);
            if same {
                kept.2 = kept.2.max(later.2);
            }
            same
        });

        let s = &mut self.snapshot;
        s.clear();
        s.extend_from_slice(&[
            snapshot::VERSION,
            self.tick as f32,
            self.now_ms as f32 / 1000.0,
            self.drones.len() as f32,
            self.links.len() as f32,
            self.sent.len() as f32,
            self.stats.frames as f32,
            self.stats.bytes as f32,
            self.stats.dropped as f32,
            self.fire.burning_count() as f32,
            f32::from(self.drones[usize::from(BASE_ID)].brain.intent() as u8),
            0.0,
        ]);
        for (i, d) in self.drones.iter().enumerate() {
            let b = &d.brain;
            s.extend_from_slice(&[
                i as f32,
                d.pos.x,
                d.pos.y,
                d.pos.z,
                f32::from(u8::from(d.alive)),
                f32::from(b.role() as u8),
                f32::from(b.hops()),
                b.parent().map_or(-1.0, f32::from),
            ]);
        }
        let (mut q_sum, mut degraded) = (0.0f32, 0usize);
        for &(a, b, kind) in &self.links {
            let q = self.linkq.quality(usize::from(a), usize::from(b));
            q_sum += q;
            degraded += usize::from(q < 0.5);
            s.extend_from_slice(&[a.into(), b.into(), kind.into(), q]);
        }
        for t in &self.sent {
            let dst = if t.dst == BROADCAST { -1.0 } else { f32::from(t.dst) };
            s.extend_from_slice(&[t.src.into(), dst, t.kind.into()]);
        }
        if self.channel.enabled() || self.tlm.enabled() {
            let mean = if self.links.is_empty() { 0.0 } else { q_sum / self.links.len() as f32 };
            self.tlm.set_link_summary(mean, degraded);
        }
    }
}

/// Where the base station sits for `cfg.preset` (Alpine Ridge: the western
/// foothills), on the ground of `terrain`, which must be that preset's.
/// Heights here and in flight use `Terrain::surface_at`, so on a sea map
/// drones float over the water rather than the seabed; on dry maps it is
/// the plain ground height.
pub fn base_position(cfg: &SimConfig, terrain: &Terrain) -> Vec3 {
    let (x, z) = cfg.preset.base_xz(cfg.extent);
    Vec3::new(x, terrain.surface_at(x, z) + 3.0, z)
}

/// The base station, with every drone parked on the ground in a ring
/// around it, waiting for a mission.
fn spawn_point(i: usize, cfg: &SimConfig, terrain: &Terrain, rng: &mut Rng) -> Vec3 {
    let base = base_position(cfg, terrain);
    if i == usize::from(BASE_ID) {
        return base;
    }
    let angle = i as f32 * 2.399_963; // golden angle: even spacing for any count
    let radius = 15.0 + 4.0 * (i as f32).sqrt() + rng.range(-1.0, 1.0);
    let (x, z) = (base.x + radius * angle.cos(), base.z + radius * angle.sin());
    Vec3::new(x, terrain.surface_at(x, z) + MIN_CLEARANCE, z)
}

/// A dead drone has no thrust: it drops under gravity (capped at terminal
/// speed) until it rests on the ground. The base station, already on the
/// ground, simply stays put.
fn fall(d: &mut Drone, dt_s: f32, terrain: &Terrain) {
    if d.brain.is_sink() {
        return;
    }
    let ground = terrain.surface_at(d.pos.x, d.pos.z) + WRECK_CLEARANCE;
    if d.pos.y <= ground {
        d.fall_speed = 0.0;
        return;
    }
    d.fall_speed = (d.fall_speed + GRAVITY * dt_s).min(TERMINAL_SPEED);
    d.pos.y = (d.pos.y - d.fall_speed * dt_s).max(ground);
}

/// Moves at most `max_step` toward `target`, staying over the map, under
/// the ceiling and clear of the ground. A non-finite target (a confused
/// brain) means hover in place rather than poisoning the position.
fn fly(pos: Vec3, target: Vec3, max_step: f32, half: f32, terrain: &Terrain) -> Vec3 {
    let to = target - pos;
    let dist = to.length_sq().sqrt();
    let mut next = if !dist.is_finite() {
        pos
    } else if dist <= max_step {
        target
    } else {
        pos + to * (max_step / dist)
    };
    next.x = next.x.clamp(-half, half);
    next.z = next.z.clamp(-half, half);
    next.y = next.y.min(CEILING).max(terrain.surface_at(next.x, next.z) + MIN_CLEARANCE);
    next
}

#[cfg(test)]
mod tests {
    use swarm_brain::NO_ROUTE;

    use super::*;
    use crate::snapshot::{DRONE_STRIDE, HEADER_LEN, LINK_STRIDE, PULSE_STRIDE};

    const STEP_MS: u32 = 50;

    fn run(sim: &mut Sim, ms: u32) {
        for _ in 0..ms / STEP_MS {
            sim.step(STEP_MS);
        }
    }

    fn ids(sim: &Sim) -> impl Iterator<Item = u16> {
        0..sim.drone_count() as u16
    }

    #[test]
    fn alpine_preset_is_the_default_config() {
        let a = SimConfig::for_preset(Preset::AlpineRidge, 1, 20);
        assert_eq!(format!("{a:?}"), format!("{:?}", SimConfig::default()));
        for p in Preset::ALL {
            let c = SimConfig::for_preset(p, 7, 12);
            assert_eq!((c.preset, c.seed, c.drones), (p, 7, 12));
            let sim = Sim::new(c);
            assert_eq!(sim.preset(), p);
            assert_eq!(sim.position(BASE_ID), Some(base_position(sim.config(), sim.terrain())));
        }
    }

    #[test]
    fn chatter_decodes_heartbeats_intents_and_missions() {
        use crate::chatter::*;
        let mut sim = Sim::new(SimConfig::default());
        let records = |sim: &Sim| -> Vec<Vec<f32>> {
            sim.chatter().as_chunks::<CHATTER_STRIDE>().0.iter().map(|r| r.to_vec()).collect()
        };
        let (mut heartbeats, mut base_beacons) = (0, 0);
        for _ in 0..4_000 / STEP_MS {
            sim.step(STEP_MS);
            assert_eq!(sim.chatter().len() % CHATTER_STRIDE, 0);
            assert_eq!(sim.chatter().len() / CHATTER_STRIDE, sim.stats().frames as usize);
            for r in records(&sim) {
                assert_eq!(r[T_MS], sim.now_ms() as f32);
                assert_ne!(r[KIND], CHATTER_UNDECODABLE);
                if r[KIND] != CHATTER_HEARTBEAT {
                    continue;
                }
                heartbeats += 1;
                let f = &r[F0..];
                assert_eq!(r[DST], -1.0, "heartbeats are broadcast");
                assert!((r[SRC] as usize) < sim.drone_count());
                assert!(f[0] == f32::from(NO_ROUTE) || f[0] < 32.0, "hops {}", f[0]);
                assert!(f[1] >= -1.0 && f[1] < sim.drone_count() as f32, "parent {}", f[1]);
                assert!((0.0..=3.0).contains(&f[2]), "role {}", f[2]);
                assert!(f[4] >= -1.0 && f[6] >= -1.0 && (0.0..256.0).contains(&f[5]));
                if r[SRC] == f32::from(BASE_ID) {
                    assert_eq!((f[0], f[3]), (0.0, f32::from(BASE_ID)), "base beacon");
                    base_beacons += 1;
                }
            }
        }
        assert!(heartbeats > sim.drone_count(), "only {heartbeats} heartbeats");
        assert!(base_beacons > 0, "the base should beacon as sink at hops 0");

        assert!(sim.command(Intent::MonitorFire, 330.0, 40.0));
        let (mut intents, mut missions) = (0, 0);
        for _ in 0..3_000 / STEP_MS {
            sim.step(STEP_MS);
            for r in records(&sim) {
                let f = &r[F0..];
                if r[KIND] == CHATTER_INTENT {
                    intents += 1;
                    assert_eq!((r[SRC], r[DST]), (f32::from(OPERATOR), 0.0));
                    assert_eq!((f[0], f[1], f[2], f[3]), (1.0, 1.0, 330.0, 40.0));
                    assert!(f[4].is_nan() && f[5].is_nan());
                } else if r[KIND] == CHATTER_MISSION {
                    missions += 1;
                    assert_eq!((f[0], f[1], f[2], f[3]), (1.0, 1.0, 330.0, 40.0));
                    assert!(f[4].is_finite() && f[5].is_finite());
                }
            }
        }
        assert_eq!(intents, 1, "the console intent is logged once");
        assert!(missions >= 1, "the base floods a mission");
    }

    /// Every heartbeat record must say exactly what its sender's brain
    /// believed when it transmitted. `transmit` is the last thing a tick
    /// does, so right after the step the brain still holds that state.
    #[test]
    fn chatter_heartbeats_match_the_senders_brain() {
        use crate::chatter::*;
        use swarm_brain::{FLAG_LANDED, NO_SLOT};
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 3_000);
        assert!(sim.command(Intent::MonitorFire, 330.0, 40.0));
        let none_or = |v: u8| if v == NO_SLOT { -1.0 } else { f32::from(v) };
        let (mut checked, mut slotted, mut routed_via_parent, mut airborne) = (0, 0, 0, 0);
        for _ in 0..40_000 / STEP_MS {
            sim.step(STEP_MS);
            for r in sim.chatter().as_chunks::<CHATTER_STRIDE>().0 {
                if r[KIND] != CHATTER_HEARTBEAT {
                    continue;
                }
                let id = r[SRC] as u16;
                let b = sim.brain(id).unwrap();
                let f = &r[F0..];
                assert_eq!(f[0], f32::from(b.hops()), "drone {id} hops");
                assert_eq!(f[1], b.parent().map_or(-1.0, f32::from), "drone {id} parent");
                assert_eq!(f[2], f32::from(b.role() as u8), "drone {id} role");
                if let Some(sink) = b.sink() {
                    assert_eq!(f[3], f32::from(sink), "drone {id} sink");
                }
                assert_eq!(f[4], none_or(b.slot()), "drone {id} slot");
                let p = sim.position(id).unwrap();
                // The brain sensed its height before this step's move (at
                // most 0.6 units), so skip drones crossing the 8-unit mark.
                let agl = p.y - sim.terrain().surface_at(p.x, p.z);
                let landed = agl < 8.0;
                if (agl - 8.0).abs() > 1.0 {
                    assert_eq!(f[5] as u8 & FLAG_LANDED != 0, landed, "drone {id} landed flag");
                }
                assert!(f[6] == -1.0 || (1.0..128.0).contains(&f[6]), "drone {id} need {}", f[6]);
                assert!(f[7] == 0.0 || f[7] == 1.0, "drone {id} intent_seq {}", f[7]);
                checked += 1;
                slotted += usize::from(f[4] > 0.0);
                routed_via_parent += usize::from(f[1] >= 0.0);
                airborne += usize::from(!landed);
            }
        }
        // Make sure the comparison exercised real mission state.
        assert!(
            checked > 500 && slotted > 100 && routed_via_parent > 100 && airborne > 100,
            "checked {checked}, slotted {slotted}, parented {routed_via_parent}, airborne {airborne}"
        );
    }

    #[test]
    fn gradient_reaches_every_drone() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 10_000);
        for id in ids(&sim) {
            assert_ne!(sim.brain(id).unwrap().hops(), NO_ROUTE, "drone {id} has no route");
        }
    }

    #[test]
    fn neighbors_forget_a_killed_drone() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 5_000);
        let victim = 10;
        assert!(sim.kill(victim));
        assert!(!sim.kill(victim), "already dead");
        run(&mut sim, Config::default().neighbor_timeout_ms + 1_000);
        for id in ids(&sim).filter(|&id| sim.is_alive(id)) {
            let knows = sim.brain(id).unwrap().neighbors().iter().any(|n| n.id == victim);
            assert!(!knows, "drone {id} still lists dead drone {victim}");
        }
    }

    #[test]
    fn operator_command_floods_the_mesh() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 5_000);
        assert!(sim.command(Intent::MonitorFire, 330.0, 40.0));
        run(&mut sim, 2_000);
        for id in ids(&sim) {
            assert_eq!(sim.brain(id).unwrap().intent(), Intent::MonitorFire, "drone {id}");
        }
    }

    #[test]
    fn dead_base_station_rejects_commands() {
        let mut sim = Sim::new(SimConfig::default());
        sim.kill(BASE_ID);
        assert!(!sim.command(Intent::Hold, 0.0, 0.0));
    }

    #[test]
    fn steady_state_traffic_is_about_one_frame_per_drone_per_second() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 5_000);
        let mut frames = 0;
        for _ in 0..10_000 / STEP_MS {
            sim.step(STEP_MS);
            frames += sim.stats().frames;
        }
        let per_drone_per_s = frames as f32 / sim.drone_count() as f32 / 10.0;
        assert!(per_drone_per_s <= 1.1, "{per_drone_per_s} frames/drone/s");
    }

    #[test]
    fn snapshot_length_matches_its_header() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 3_000);
        let s = sim.snapshot();
        assert_eq!(s[0], snapshot::VERSION);
        let (drones, links, pulses) = (s[3] as usize, s[4] as usize, s[5] as usize);
        assert!(links > 0);
        assert_eq!(
            s.len(),
            HEADER_LEN + drones * DRONE_STRIDE + links * LINK_STRIDE + pulses * PULSE_STRIDE
        );
    }

    /// Quality of every snapshot link, keyed `(a, b)`.
    fn link_qualities(sim: &Sim) -> Vec<((u16, u16), f32)> {
        let s = sim.snapshot();
        let (drones, links) = (s[3] as usize, s[4] as usize);
        let at = HEADER_LEN + drones * DRONE_STRIDE;
        (0..links)
            .map(|k| {
                let r = &s[at + k * LINK_STRIDE..][..LINK_STRIDE];
                ((r[0] as u16, r[1] as u16), r[3])
            })
            .collect()
    }

    #[test]
    fn link_quality_is_one_without_loss_and_falls_with_it() {
        // Parked around the base, everyone hears everyone: exactly 1.
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 6_000);
        let q = link_qualities(&sim);
        assert!(q.len() > 50);
        assert!(q.iter().all(|&(_, v)| v == 1.0), "{q:?}");
        // Random loss shows in every link, and averages out near 1 − loss.
        sim.set_loss(0.4);
        run(&mut sim, 6_000);
        let q = link_qualities(&sim);
        let mean = q.iter().map(|&(_, v)| v).sum::<f32>() / q.len() as f32;
        assert!(q.iter().all(|&(_, v)| (0.0..=1.0).contains(&v)));
        // The worse of two directions over a few frames each reads a little below 0.6.
        assert!((0.3..0.7).contains(&mean), "mean quality {mean} at 40 % loss");
    }

    #[test]
    fn a_dead_neighbours_link_decays_to_zero_before_it_is_forgotten() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 6_000);
        let victim = 10;
        sim.kill(victim);
        let mut seen = Vec::new();
        for _ in 0..(Config::default().neighbor_timeout_ms + 2_000) / STEP_MS {
            sim.step(STEP_MS);
            let q: Vec<f32> = link_qualities(&sim).into_iter().filter(|((a, b), _)| *a == victim || *b == victim).map(|(_, v)| v).collect();
            if let Some(&max) = q.iter().max_by(|a, b| a.total_cmp(b)) {
                seen.push(max);
            }
        }
        assert!(!seen.is_empty(), "the tables listed the dead drone for a while");
        assert!(seen.windows(2).all(|w| w[1] <= w[0] + 1e-6), "monotone decay: {seen:?}");
        assert_eq!(*seen.last().unwrap(), 0.0, "{seen:?}");
    }

    #[test]
    fn fire_spreads_in_the_world() {
        let mut sim = Sim::new(SimConfig::default());
        assert_eq!(sim.fire().burning_count(), 1);
        run(&mut sim, 120_000);
        assert!(sim.fire().burning_count() > 1, "the fire should have spread");
        assert_eq!(sim.snapshot()[9], sim.fire().burning_count() as f32);
    }

    #[test]
    fn drones_stay_on_the_map_and_above_ground() {
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 2_000);
        // Ask for somewhere far off the map; the target is pulled onto it.
        assert!(sim.command(Intent::MonitorFire, 1e9, -1e9));
        run(&mut sim, 20_000);
        let half = sim.config().extent * 0.5;
        // The base sits on its mast rather than flying, so skip it.
        for id in ids(&sim).filter(|&id| id != BASE_ID) {
            let p = sim.position(id).unwrap();
            assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite(), "drone {id} at {p:?}");
            assert!(p.x.abs() <= half && p.z.abs() <= half, "drone {id} left the map: {p:?}");
            let agl = p.y - sim.terrain().height_at(p.x, p.z);
            assert!(agl >= MIN_CLEARANCE - 1e-3, "drone {id} below clearance: {agl}");
        }
    }

    #[test]
    fn ignite_starts_a_spot_fire_and_ignores_bad_input() {
        let mut sim = Sim::new(SimConfig::default());
        let before = sim.fire().burning_count();
        sim.ignite(f32::NAN, 0.0);
        sim.ignite(1e9, 1e9);
        assert_eq!(sim.fire().burning_count(), before);
        sim.ignite(330.0, 300.0);
        assert_eq!(sim.fire().burning_count(), before + 1);
    }

    #[test]
    fn killed_drone_falls_to_the_ground_and_stays() {
        let mut sim = Sim::new(SimConfig::default());
        let victim = 5;
        assert!(sim.command(Intent::MonitorFire, 330.0, 40.0));
        run(&mut sim, 30_000);
        let p = sim.position(victim).unwrap();
        let agl = |sim: &Sim, p: Vec3| p.y - sim.terrain().height_at(p.x, p.z);
        assert!(agl(&sim, p) > 15.0, "victim should be airborne first: {p:?}");
        assert!(sim.kill(victim));
        run(&mut sim, 500);
        let mid = sim.position(victim).unwrap();
        assert!(mid.y < p.y, "a dead drone must start falling at once");
        assert_eq!((mid.x, mid.z), (p.x, p.z), "it falls straight down");
        run(&mut sim, 30_000);
        let rest = sim.position(victim).unwrap();
        assert!((agl(&sim, rest) - WRECK_CLEARANCE).abs() < 1e-3, "wreck at {rest:?}");
        run(&mut sim, 1_000);
        assert_eq!(sim.position(victim), Some(rest), "the wreck stays put");

        let base = sim.position(BASE_ID).unwrap();
        sim.kill(BASE_ID);
        run(&mut sim, 2_000);
        assert_eq!(sim.position(BASE_ID), Some(base), "the base is already grounded");
    }

    #[test]
    fn hostile_inputs_never_panic() {
        for drones in [1, 2, 256] {
            let mut sim = Sim::new(SimConfig { drones, ..SimConfig::default() });
            for id in [0, 1, 255, 256, 999, u16::MAX - 1, u16::MAX] {
                sim.kill(id);
                let _ = (sim.brain(id), sim.position(id), sim.is_alive(id), sim.in_contact(id, 0));
            }
            for loss in [f32::NAN, -1.0, 0.5, 2.0, f32::INFINITY] {
                sim.set_loss(loss);
                assert!((0.0..=1.0).contains(&sim.config().radio.loss), "loss {loss}");
            }
            sim.set_loss(0.1);
            let odd = [(f32::NAN, 0.0), (0.0, f32::INFINITY), (f32::MAX, f32::MIN), (1e-30, 0.0)];
            for (x, z) in odd {
                sim.set_wind(x, z);
                assert!(sim.config().wind.0.is_finite() && sim.config().wind.1.is_finite());
                sim.command(Intent::MonitorFire, x, z);
                sim.step(STEP_MS);
            }
            assert!(!sim.command(Intent::Hold, f32::NAN, f32::NAN));
            sim.set_wind(-3.0, 0.0);
            for dt in [0, 1, STEP_MS, 10_000] {
                sim.step(dt);
            }
            let s = sim.snapshot();
            assert!(s.iter().all(|v| v.is_finite()), "snapshot with {drones} drones has NaN");
        }
    }

    #[test]
    fn lossy_radio_still_builds_a_mesh() {
        let mut sim = Sim::new(SimConfig::default());
        sim.set_loss(0.3);
        run(&mut sim, 15_000);
        let routed = ids(&sim).filter(|&id| sim.brain(id).unwrap().hops() != NO_ROUTE).count();
        assert!(routed > sim.drone_count() / 2, "only {routed} drones routed at 30% loss");
    }

    /// (mean quality, links below 0.5) straight from the snapshot.
    fn snapshot_link_quality(sim: &Sim) -> (f32, f32) {
        let s = sim.snapshot();
        let (drones, links) = (s[3] as usize, s[4] as usize);
        let at = HEADER_LEN + drones * DRONE_STRIDE;
        let q: Vec<f32> = (0..links).map(|k| s[at + k * LINK_STRIDE + 3]).collect();
        let mean = if q.is_empty() { 0.0 } else { q.iter().sum::<f32>() / q.len() as f32 };
        (mean, q.iter().filter(|&&v| v < 0.5).count() as f32)
    }

    /// Mean analytic loss of a brain broadcast over every pair of live
    /// drones in radio contact (both directions).
    fn brain_loss(sim: &Sim) -> f32 {
        let (mut sum, mut n) = (0.0, 0);
        for a in ids(sim) {
            for b in ids(sim) {
                if a != b && sim.is_alive(a) && sim.is_alive(b) && sim.in_contact(a, b) {
                    sum += sim.channel().link_loss(a, b, false);
                    n += 1;
                }
            }
        }
        if n == 0 { 0.0 } else { sum / n as f32 }
    }

    fn telemetry_on() -> TelemetryConfig {
        TelemetryConfig { enabled: true, ..TelemetryConfig::default() }
    }

    #[test]
    fn telemetry_and_channel_are_off_by_default() {
        use crate::telemetry::{CHANNEL_LEN, TLM_STRIDE, chan, tlm};
        let mut sim = Sim::new(SimConfig::default());
        run(&mut sim, 2_000);
        assert!(!sim.config().channel.enabled && !sim.config().telemetry.enabled);
        assert_eq!(sim.telemetry_stats().len(), sim.drone_count() * TLM_STRIDE);
        assert_eq!(sim.channel_summary().len(), CHANNEL_LEN);
        assert!(sim.channel_summary().iter().all(|&v| v == 0.0));
        let ids: Vec<f32> = sim.telemetry_stats().iter().step_by(TLM_STRIDE).copied().collect();
        assert_eq!(ids, (0..sim.drone_count()).map(|i| i as f32).collect::<Vec<_>>());
        assert_eq!(sim.tlm_sample(3), "");
        assert_eq!(sim.tlm_sample(u16::MAX), "");
        let _ = (chan::MODE, tlm::ID);
    }

    /// Telemetry draws from its own RNG and, with the channel off, costs
    /// nothing on air, so turning it on alone must not change the swarm.
    #[test]
    fn telemetry_alone_does_not_perturb_the_swarm() {
        let trace = |on: bool| {
            let mut sim = Sim::new(SimConfig { seed: 4, ..SimConfig::default() });
            sim.set_loss(0.1);
            if on {
                sim.set_external_busy(0.3); // ignored while the channel is off
                sim.set_telemetry(telemetry_on());
            }
            run(&mut sim, 2_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 8_000);
            (sim.snapshot().to_vec(), sim.fire().cells().to_vec())
        };
        assert_eq!(trace(false), trace(true));
    }

    /// With the channel on, brain frames cost airtime and the busy channel
    /// costs them deliveries: the swarm runs differently, and worse the
    /// busier the air.
    #[test]
    fn channel_costs_brain_frames_airtime_and_loss() {
        let run_with = |channel: Option<f32>| {
            let mut sim = Sim::new(SimConfig { seed: 4, ..SimConfig::default() });
            if let Some(ext) = channel {
                sim.set_channel_enabled(true);
                sim.set_external_busy(ext);
            }
            run(&mut sim, 2_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 8_000);
            sim
        };
        let off = run_with(None);
        let quiet = run_with(Some(0.0));
        let busy = run_with(Some(0.8));
        assert!(quiet.channel().airtime_frac(1) > 0.0, "brain frames cost airtime");
        assert!(quiet.channel().kind_frac(AirKind::Brain) > 0.0);
        assert!(busy.channel().busy(1) >= 0.8);
        let (lq, lb) = (brain_loss(&quiet), brain_loss(&busy));
        // Strong (short) links capture over other networks' frames; the
        // mean still rises clearly.
        assert!(lb > 2.0 * lq + 0.02, "busy air loses broadcasts: {lq} → {lb}");
        assert_ne!(off.snapshot(), busy.snapshot(), "loss changes what the swarm does");
        let heard = |sim: &Sim| ids(sim).map(|id| sim.brain(id).unwrap().neighbors().len()).sum::<usize>();
        assert!(heard(&busy) <= heard(&quiet), "{} > {}", heard(&busy), heard(&quiet));
    }

    /// A saturated sender's full queue refuses frames, and the world hands
    /// the count to its brain (`Sensors::tx_dropped`) every tick.
    #[test]
    fn saturated_queue_drops_reach_the_brain_sensor() {
        let mut sim = Sim::new(SimConfig { seed: 4, ..SimConfig::default() });
        sim.set_channel(ChannelConfig {
            enabled: true,
            external_busy: channel::MAX_EXTERNAL_BUSY,
            tx_queue_frames: 1,
            ..ChannelConfig::default()
        });
        sim.set_telemetry(TelemetryConfig { requested_hz: 50.0, rate: crate::RateMode::Fixed(50.0), ..telemetry_on() });
        run(&mut sim, 6_000);
        let drops: f32 = (0..sim.drone_count() as u16).map(|i| sim.channel().queue_drops_per_s(i)).sum();
        assert!(drops > 1.0, "{drops}");
        // Refusals are handed to each live brain at its next tick: after a
        // step, only those from that step's own deliveries remain pending.
        let mut ch = sim.channel.clone();
        let pending: u32 = (0..sim.drone_count() as u16).map(|i| u32::from(ch.take_tx_dropped(i))).sum();
        let per_step = drops * STEP_MS as f32 / 1000.0;
        assert!((pending as f32) < per_step * 4.0 + 20.0, "{pending} pending vs {per_step}/step: not drained");
    }

    #[test]
    fn telemetry_streams_sampled_chatter_and_stats() {
        use crate::chatter::*;
        use crate::telemetry::{TLM_STRIDE, chan, tlm};
        // 6 drones (5 × 25 Hz ≈ 125 pps) fit a clean 12 Mbps channel.
        let mut sim = Sim::new(SimConfig { drones: 6, ..SimConfig::default() });
        sim.set_channel_enabled(true);
        sim.set_telemetry(telemetry_on());
        let mut per_drone = vec![0u32; sim.drone_count()];
        for _ in 0..5_000 / STEP_MS {
            sim.step(STEP_MS);
            for r in sim.chatter().as_chunks::<CHATTER_STRIDE>().0 {
                if r[KIND] == CHATTER_TLM {
                    per_drone[r[SRC] as usize] += 1;
                    assert_eq!(r[DST], f32::from(BASE_ID));
                    assert!(r[F0 + 1] > r[F0], "package is larger than its frame");
                }
            }
        }
        assert_eq!(per_drone[usize::from(BASE_ID)], 0, "the bridge sends no drone telemetry");
        assert!(per_drone[1..].iter().all(|&n| (4..=6).contains(&n)), "≤ 1 TLM record / drone / s: {per_drone:?}");
        let st = sim.telemetry_stats();
        for row in st.as_chunks::<TLM_STRIDE>().0.iter().skip(1) {
            assert!((row[tlm::DELIVERED_HZ] - 25.0).abs() < 1.0, "{row:?}");
            assert_eq!(row[tlm::REQUESTED_HZ], 25.0);
            assert!(row[tlm::TX_BYTES_PER_S] > 25.0 * row[tlm::LAST_PACKAGE_BYTES] * 0.5);
            assert!(row[tlm::BRAIN_BYTES_PER_S] > 0.0);
        }
        let s = sim.tlm_sample(5);
        assert!(s.starts_with(r#"{"type":9,"dest":"#) && s.ends_with("\"}"), "{s}");
        let ch = sim.channel_summary();
        assert!(ch[chan::GCS_PPS] > 5.0 * 25.0 * 0.9, "{ch:?}");
        assert!(ch[chan::PPS_ON_AIR] > ch[chan::GCS_PPS]);
        assert_eq!(ch[chan::BRIDGE_UP], 1.0);
        sim.kill(BASE_ID);
        run(&mut sim, 3_000);
        let ch = sim.channel_summary();
        assert_eq!((ch[chan::BRIDGE_UP], ch[chan::GCS_PPS]), (0.0, 0.0), "no root, no bridge");
    }

    /// 19 drones × 25 Hz (475 pps, ~3× the field's collapse point) do not
    /// fit even a clean 12 Mbps channel: telemetry and brain frames both
    /// pay, and external airtime makes it worse.
    #[test]
    fn full_fleet_at_25_hz_congests_the_channel() {
        use crate::telemetry::{TLM_STRIDE, chan, tlm};
        let measure = |ext: f32| {
            let mut sim = Sim::new(SimConfig::default());
            sim.set_channel_enabled(true);
            sim.set_external_busy(ext);
            sim.set_telemetry(telemetry_on());
            run(&mut sim, 15_000);
            let st = sim.telemetry_stats();
            let rows = &st.as_chunks::<TLM_STRIDE>().0[1..];
            let delivered = rows.iter().map(|r| r[tlm::DELIVERED_HZ]).sum::<f32>() / rows.len() as f32;
            (delivered, brain_loss(&sim), sim.channel_summary()[chan::BUSY_MEAN], snapshot_link_quality(&sim).0)
        };
        let (clean_hz, clean_brain, clean_busy, clean_q) = measure(0.0);
        let (busy_hz, busy_brain, busy_busy, busy_q) = measure(0.6);
        println!("clean {clean_hz} Hz brain {clean_brain} busy {clean_busy} q {clean_q}; ext 0.6: {busy_hz} Hz brain {busy_brain} busy {busy_busy} q {busy_q}");
        assert!(clean_busy > 0.4, "{clean_busy}");
        assert!(clean_brain > 0.1, "brain broadcasts lose to telemetry airtime: {clean_brain}");
        assert!(busy_hz < clean_hz - 3.0, "{busy_hz} vs {clean_hz} Hz");
        // With 60 % external the telemetry collapses, so the swarm's own
        // load falls and its link quality need not (it reports both).
        assert!(busy_busy > clean_busy, "{clean_busy} → {busy_busy}");
        assert!(busy_brain > 0.05 && (0.0..=1.0).contains(&busy_q) && (0.0..=1.0).contains(&clean_q));
    }

    #[test]
    fn telemetry_world_is_deterministic() {
        let trace = || {
            let mut sim = Sim::new(SimConfig { seed: 9, ..SimConfig::default() });
            sim.set_channel_enabled(true);
            sim.set_telemetry(telemetry_on());
            run(&mut sim, 5_000);
            (sim.snapshot().to_vec(), sim.telemetry_stats().to_vec(), sim.tlm_sample(7).to_owned())
        };
        assert_eq!(trace(), trace());
    }

    /// `cargo test --release -p swarm-sim step_timing_telemetry -- --ignored --nocapture`
    #[test]
    #[ignore = "timing; run in release"]
    fn step_timing_telemetry() {
        for (drones, budget_ms) in [(20, 2.0), (100, 12.0)] {
            let mut sim = Sim::new(SimConfig { drones, ..SimConfig::default() });
            sim.set_channel_enabled(true);
            sim.set_external_busy(0.2);
            sim.set_telemetry(telemetry_on());
            run(&mut sim, 2_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 30_000);
            let steps = 1_200;
            let start = std::time::Instant::now();
            run(&mut sim, steps * STEP_MS);
            let avg = start.elapsed().as_secs_f64() * 1e3 / f64::from(steps);
            let (mean, max) = sim.channel().busy_mean_max(&sim.nodes);
            println!(
                "{drones:>3} drones, telemetry 25 Hz + channel: {avg:.3} ms/step \
                 (busy mean {mean:.2} max {max:.2}, {:.0} pps on air)",
                sim.channel().pps()
            );
            if !cfg!(debug_assertions) {
                assert!(avg < budget_ms, "{drones} drones: {avg:.3} ms/step over the {budget_ms} ms budget");
            }
        }
    }

    /// What the web demo shows: 20 drones at 25 Hz over the slider.
    /// `cargo test --release -p swarm-sim demo_channel_report -- --ignored --nocapture`
    #[test]
    #[ignore = "report"]
    fn demo_channel_report() {
        use crate::telemetry::{TLM_STRIDE, chan, tlm};
        // hz < 0: telemetry off; ext < 0: channel off too.
        for (hz, ext) in [(-1.0f32, -1.0f32), (-1.0, 0.0), (-1.0, 0.5), (25.0, 0.0), (25.0, 0.1), (25.0, 0.2), (25.0, 0.3), (25.0, 0.4), (25.0, 0.5), (5.0, 0.0), (5.0, 0.5)] {
            let mut sim = Sim::new(SimConfig::default());
            sim.set_channel_enabled(ext >= 0.0);
            sim.set_external_busy(ext.max(0.0));
            if hz > 0.0 {
                sim.set_telemetry(TelemetryConfig { rate: crate::RateMode::Fixed(hz), ..telemetry_on() });
            }
            run(&mut sim, 2_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 40_000);
            // Averages over the last 10 s.
            let mut acc = [0.0f32; 8];
            let mut nb = 0.0;
            for _ in 0..10_000 / STEP_MS {
                sim.step(STEP_MS);
                let ch = sim.channel_summary();
                let st = sim.telemetry_stats();
                let rows = &st.as_chunks::<TLM_STRIDE>().0[1..];
                let mean = |f: usize| rows.iter().map(|r| r[f]).sum::<f32>() / rows.len() as f32;
                for (k, v) in [
                    ch[chan::BUSY_MEAN],
                    ch[chan::BUSY_MAX],
                    mean(tlm::DELIVERED_HZ),
                    mean(tlm::LOSS_PCT),
                    snapshot_link_quality(&sim).0,
                    snapshot_link_quality(&sim).1,
                    sim.snapshot()[4],
                    brain_loss(&sim),
                ]
                .into_iter()
                .enumerate()
                {
                    acc[k] += v;
                }
                nb += ids(&sim).map(|id| sim.brain(id).unwrap().neighbors().len() as f32).sum::<f32>();
            }
            let n = (10_000 / STEP_MS) as f32;
            let a: Vec<f32> = acc.iter().map(|v| v / n).collect();
            println!(
                "{hz:>4} Hz ext {ext:.1}: busy {:.2}/{:.2}  delivered {:.1} Hz  tlm loss {:.1} %  link quality {:.2}  degraded {:.1} of {:.1} links  \
                 brain bcast loss {:.0} %  neighbours {:.1}",
                a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7] * 100.0, nb / n
            );
            if hz == 25.0 && ext == 0.0 {
                let ch = sim.channel_summary();
                let k = |i: usize| ch[i];
                println!(
                    "  bytes/s: on air {:.0} = brain {:.0} + tlm data {:.0} + acks {:.0} + mesh ctl {:.0} + uplink {:.0} + beacons {:.0}; \
                     retries {:.0}; frames/s {:.0} (pps_on_air {:.0}); sum tx_all {:.0}",
                    k(chan::BYTES_ON_AIR), k(chan::BYTES_BRAIN), k(chan::BYTES_TLM_DATA), k(chan::BYTES_ACK),
                    k(chan::BYTES_MESH_CONTROL), k(chan::BYTES_UPLINK), k(chan::BYTES_BEACON), k(chan::BYTES_RETRY),
                    k(chan::FRAMES_ON_AIR), k(chan::PPS_ON_AIR),
                    sim.telemetry_stats().as_chunks::<TLM_STRIDE>().0.iter().map(|r| r[tlm::TX_ALL_BYTES_PER_S]).sum::<f32>(),
                );
            }
        }
    }

    /// The web UI's default run, headless: Alpine, 20 drones, Fixed 25 Hz,
    /// S3, Single, channel on, MonitorFire; one line every 10 s for 300 s.
    /// `cargo test --release -p swarm-sim collapse_timeline -- --ignored --nocapture`
    #[test]
    #[ignore = "report"]
    fn collapse_timeline() {
        use crate::telemetry::{TLM_STRIDE, chan, tlm};
        let mut sim = Sim::new(SimConfig::default());
        sim.set_channel_enabled(true);
        sim.set_telemetry(telemetry_on());
        sim.command(Intent::MonitorFire, 330.0, 40.0);
        println!(
            "  t | Hz med/mean | loss% | busy mean/max | q    degr/links | hops max/mean | bufs | stalled conns (retx/worst/blocked) \
             | RTOs unreach/air | closed | drops/s buf+queue | parent rate | retry fallback <=2 Mbps | p_fail"
        );
        for k in 1..=30 {
            run(&mut sim, 10_000);
            let st = sim.telemetry_stats();
            let rows: Vec<&[f32; TLM_STRIDE]> = st.as_chunks::<TLM_STRIDE>().0.iter().skip(1).collect();
            let mut hz: Vec<f32> = rows.iter().map(|r| r[tlm::DELIVERED_HZ]).collect();
            hz.sort_by(f32::total_cmp);
            let mean = |f: usize| rows.iter().map(|r| r[f]).sum::<f32>() / rows.len() as f32;
            let m = sim.painless();
            let hops: Vec<u8> = (1..20u16).filter_map(|i| m.hops(i)).collect();
            let bufs: usize = (0..20u16).map(|i| m.queued(i).0 + m.queued(i).1).sum();
            let (retx, worst, blocked) = m.tcp_stall_stats(sim.now_ms());
            let mut chc = sim.channel().clone();
            let chains: Vec<(f32, f32, f32)> = (1..20u16)
                .filter_map(|i| m.parent(i).map(|p| (i, p)))
                .map(|(i, p)| {
                    let (a, b) = chc.link_rates(i, p);
                    (a, b, sim.channel().unicast_failure(i, p, a, 500).map_or(0.0, |x| x.0))
                })
                .collect();
            let nc = chains.len().max(1) as f32;
            let ch = sim.channel_summary();
            let (q, degr) = snapshot_link_quality(&sim);
            println!(
                "{:>3} | {:4.1}/{:4.1} | {:5.1} | {:.2}/{:.2} | {:.2} {:>3}/{:<3} | {}/{:.1} | {:>4} | {}/{}/{} | {:?} | {:?} | {:.0} | {:.1} | {} | {:.2}",
                k * 10, hz[hz.len() / 2], mean(tlm::DELIVERED_HZ), mean(tlm::LOSS_PCT),
                ch[chan::BUSY_MEAN], ch[chan::BUSY_MAX], q, degr, sim.snapshot()[4],
                hops.iter().max().copied().unwrap_or(0), hops.iter().map(|&h| f32::from(h)).sum::<f32>() / hops.len().max(1) as f32,
                bufs, retx, worst, blocked, m.rto_counts(), m.closed_counts(),
                rows.iter().map(|r| r[tlm::QUEUE_DROPS_PER_S]).sum::<f32>(),
                chains.iter().map(|c| c.0).sum::<f32>() / nc,
                chains.iter().filter(|c| c.1 <= 2.0).count(),
                chains.iter().map(|c| c.2).sum::<f32>() / nc,
            );
        }
    }

    /// `cargo test --release -p swarm-sim mesh_link_report -- --ignored --nocapture`
    #[test]
    #[ignore = "report"]
    fn mesh_link_report() {
        use crate::telemetry::{TLM_STRIDE, tlm};
        for (ext, rate) in [(0.0, crate::RateMode::BackendBudget), (0.0, crate::RateMode::Fixed(25.0))] {
            let mut sim = Sim::new(SimConfig::default());
            sim.set_channel_enabled(true);
            sim.set_external_busy(ext);
            sim.set_telemetry(TelemetryConfig { rate, ..telemetry_on() });
            run(&mut sim, 2_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 60_000);
            println!("--- ext {ext} {rate:?}: busy mean {:.2}", sim.channel_summary()[crate::telemetry::chan::BUSY_MEAN]);
            let st = sim.telemetry_stats().to_vec();
            for id in 1..sim.drone_count() as u16 {
                let r = &st[usize::from(id) * TLM_STRIDE..][..TLM_STRIDE];
                let Some(p) = sim.painless().parent(id) else { println!("#{id} no parent"); continue };
                let d = (sim.position(id).unwrap() - sim.position(p).unwrap()).length_sq().sqrt();
                let rate = r[tlm::PARENT_RATE_MBPS];
                let (fail, sig) = sim.channel().unicast_failure(id, p, rate.max(1.0), 500).unwrap_or((-1.0, 0.0));
                println!(
                    "#{id:>2} -> {p:>2} d {d:5.0} sig {sig:6.1} rate {rate:4.1} fail/attempt {fail:.2} busy {:.2} hz {:.1} loss {:.0}% qdrop {:.1} brain_loss_from_parent {:.2}",
                    r[tlm::BUSY_FRAC], r[tlm::DELIVERED_HZ], r[tlm::LOSS_PCT], r[tlm::QUEUE_DROPS_PER_S],
                    sim.channel().link_loss(p, id, false)
                );
            }
            let (f, s) = sim.channel().unicast_failure(0, 0, sim.channel().config().min_rate_mbps, 300).unwrap();
            println!("root uplink fail/attempt {f:.2} sig {s:.1} root util {:.2}", sim.channel().utilisation(0));
        }
    }

    #[test]
    fn same_seed_same_world() {
        let trace = || {
            let mut sim = Sim::new(SimConfig { seed: 9, ..SimConfig::default() });
            sim.set_loss(0.2);
            run(&mut sim, 3_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 10_000);
            (sim.snapshot().to_vec(), sim.fire().cells().to_vec())
        };
        assert_eq!(trace(), trace());
    }

    /// Average wall time per step over a mission-like run: drones take off,
    /// fly toward the fire and keep a mesh up. Debug builds are ~20x slower,
    /// so the real budgets are only asserted in release:
    /// `cargo test --release -p swarm-sim step_timing -- --ignored --nocapture`.
    #[test]
    #[ignore = "timing; run in release"]
    fn step_timing() {
        for (drones, budget_ms) in [(20, 1.0), (100, 8.0), (256, 40.0)] {
            let mut sim = Sim::new(SimConfig { drones, ..SimConfig::default() });
            run(&mut sim, 2_000);
            sim.command(Intent::MonitorFire, 330.0, 40.0);
            run(&mut sim, 30_000);
            let steps = 1_200;
            let start = std::time::Instant::now();
            let mut worst = 0.0f64;
            for _ in 0..steps {
                let t = std::time::Instant::now();
                sim.step(STEP_MS);
                worst = worst.max(t.elapsed().as_secs_f64() * 1e3);
            }
            let avg = start.elapsed().as_secs_f64() * 1e3 / f64::from(steps);
            let frames = sim.stats().frames;
            println!(
                "{drones:>3} drones: {avg:.3} ms/step avg, {worst:.3} ms worst, \
                 {frames} frames last step"
            );
            if !cfg!(debug_assertions) {
                assert!(avg < budget_ms, "{drones} drones: {avg:.3} ms/step over {budget_ms} ms");
            }
        }
    }
}
