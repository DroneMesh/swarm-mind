//! painlessMesh 1.5.7 model: the tree the real telemetry rides on.
//!
//! This is the simulator's own network, *not* the brain's routing. Every
//! node's STA associates to one parent's SoftAP (the mesh SSID), so the
//! mesh is a tree rooted at the bridge; packages are JSON objects sent over
//! TCP (port 5555), each terminated by `'\0'`.
//!
//! Package shapes (key order is ArduinoJson insertion order, as on air):
//!
//! ```text
//! SINGLE    (9) {"type":9,"dest":<u32>,"from":<u32>,"msg":"<escaped>"}
//! BROADCAST (8) {"type":8,"dest":<u32>,"from":<u32>,"msg":"<escaped>"}  flooded
//! NODE_SYNC_REQUEST (5) / REPLY (6)
//!           {"nodeId":<u32>[,"root":true][,"subs":[<subtree>...]],"type":5,"dest":..,"from":..}
//! TIME_SYNC (4) / TIME_DELAY (3)
//!           {"type":4,"dest":..,"from":..,"msg":{"type":0|1|2[,"t0":..[,"t1":..,"t2":..]]}}
//! ```
//!
//! `msg` of a single/broadcast is a JSON *string*, so the telemetry JSON's
//! own quotes are escaped (`\"`) and every one costs a byte.
//!
//! # Tree ([`PainlessMesh::step`])
//!
//! * **Joining.** A node without a parent scans ([`SCAN_MS`]) and
//!   associates ([`CONNECT_MS`]) to the strongest-RSSI node with a free
//!   SoftAP slot ([`MAX_CONN`]) that reaches the root; if none does, to the
//!   strongest other mesh node (painlessMesh joins any node it doesn't
//!   know). Nodes of its own subtree are never candidates (`filterAPs`),
//!   so no loop can form. A scan that finds nobody retries after
//!   [`RESCAN_MS`] (`0.5 * SCAN_INTERVAL`, painlessMesh's "fast" rate).
//! * **Cut off.** A connected node whose tree has no root (an upstream
//!   connection closed) scans too and moves, with its subtree, to the
//!   strongest AP that reaches the root (painlessMesh reconfigures when
//!   `!isRooted`; its `stability` dice are not modelled: the move is
//!   immediate).
//! * **Staying.** A rooted node keeps its parent (painlessMesh only
//!   reconfigures an *unrooted* node), until the connection dies: the parent
//!   unreachable (dead, out of range, terrain) for [`NODE_TIMEOUT_MS`], or a
//!   node sync request unanswered for [`NODE_TIMEOUT_MS`] (`timeOutTask`).
//!   Closing a connection drops both TCP send buffers. A node's own
//!   children stay with it while it rejoins.
//! * **No root election.** The root is the base station (drone
//!   [`crate::BASE_ID`]), configured like the real bridge (`setRoot(true)`,
//!   `setContainsRoot(true)`, `stationManual(router)`). If it dies, nobody
//!   reaches a root and telemetry has no bridge.
//! * **Boot.** The first step (or the first after a gap, e.g. telemetry was
//!   off) forms the tree at once, as if the swarm had been powered up on
//!   the ground before the scenario began; periodic syncs are staggered.
//!
//! # Transport ([`PainlessMesh::transport`])
//!
//! Each tree edge is one TCP connection with a send buffer in each
//! direction, bounded at [`MAX_MESSAGE_QUEUE`] packages (painlessMesh's
//! intended limit; 1.5.7 only bounds it by free heap). A full buffer
//! refuses new telemetry (the newest is discarded). Per step, every buffer
//! sends what the channel lets its node send: each TCP segment is a QoS
//! unicast at the link's adapted rate ([`Channel::link_rate_mbps`]) with
//! its MAC retries and 802.11 ACK, and the receiver's TCP ACKs are
//! unicasts back ([`Channel::unicast`]).
//!
//! * **Upward (station side) runs Nagle.** painlessMesh only calls
//!   `setNoDelay(true)` on its `AsyncServer` (`tcp.hpp`), which AsyncTCP
//!   copies onto the connections it *accepts*; the STA-side `AsyncClient`
//!   opened by `tcp::connect` keeps lwIP's default. So while a segment is
//!   unacknowledged, later writes are appended to one unsent segment (up to
//!   [`TCP_MSS`]); full-size segments still go. The parent ACKs every
//!   second segment at once and a lone one on its next delayed-ACK tick
//!   ([`TCP_FAST_TMR_MS`], `CONFIG_LWIP_TCP_TMR_INTERVAL`), which is when
//!   the held segment is released. A 25 Hz stream thus crosses each hop as
//!   about four coalesced segments a second, each with its own TCP ACK,
//!   and waits 0-250 ms per hop for it.
//! * **Downward (accepted side) is `NODELAY`.** Each package is a segment
//!   at once (coalesced only when backlogged); one TCP ACK per two segments,
//!   and a lone segment is ACKed on the receiver's next fast tick (lwIP
//!   delays ACKs the same way in both directions; only the holding of small
//!   segments is Nagle's, upward).
//! * **Loss.** A segment lost on air or to an unreachable receiver blocks
//!   the stream until lwIP's retransmission timeout: counted in
//!   [`TCP_SLOW_TMR_MS`] ticks, [`RTO_INITIAL_TICKS`] on a fresh connection,
//!   settling to [`RTO_MIN_TICKS`] (1-1.5 s) as sub-tick RTTs are sampled,
//!   doubling per retry ([`RTO_MAX_MS`]). One segment in flight gets no
//!   duplicate ACKs, so fast retransmit never helps. TCP does not lose
//!   data, it stalls, and stalls fill buffers. A frame the WiFi driver
//!   refuses (TX queue full, `ERR_MEM`) is not an RTO: it is retried on the
//!   next fast tick, without backoff.
//! * While a buffer is backlogged, pending packages are coalesced into
//!   segments of up to [`TCP_MSS`] bytes: lwIP's unsent queue (and Nagle,
//!   upward) appends writes the stack could not send yet to the last unsent
//!   segment. Packages are not split across segments (a simplification:
//!   lwIP fills to the MSS). Control packages jump the queue (`priority`).
//!
//! * SINGLE packages go up the tree hop by hop to the root.
//! * BROADCAST packages are flooded: each node forwards on all its
//!   connections except the one it came in on, so each package crosses
//!   every edge once.
//! * NODE_SYNC: on a new connection the station side requests at once; then
//!   each end runs its own exchange every [`NODE_SYNC_INTERVAL_MS`], the AP
//!   side [`AP_SYNC_LAG_MS`] after the station side; a topology change
//!   triggers a request on every edge of the affected tree (`syncLayout`),
//!   each answered with a reply. Each carries the sender's view of the mesh
//!   minus the receiver's side (`excludeRoute`), so it grows with the mesh.
//! * TIME_SYNC: the station side asks (`TIME_REQUEST`, `t0`), the AP side
//!   answers (`TIME_REPLY`, `t0 t1 t2`). A new connection needs two rounds
//!   ([`TIME_RETRY_MS`] apart: the first offset is far off); unsynced it
//!   retries every [`TIME_SYNC_FAST_MS`], synced every
//!   [`TIME_SYNC_INTERVAL_MS`].

use core::fmt::Write as _;
use std::collections::VecDeque;

use crate::channel::{self, AirFrame, AirKind, Channel, Dst};

/// Package types (`painlessmesh::protocol::Type`).
pub const TIME_DELAY: u8 = 3;
pub const TIME_SYNC: u8 = 4;
pub const NODE_SYNC_REQUEST: u8 = 5;
pub const NODE_SYNC_REPLY: u8 = 6;
pub const BROADCAST: u8 = 8;
pub const SINGLE: u8 = 9;

/// TCP port the mesh listens on.
pub const MESH_PORT: u16 = 5555;
/// A connection that has been silent this long is dropped.
pub const NODE_TIMEOUT_MS: u32 = 10_000;
/// Node sync per connection (`TASK_MINUTE`).
pub const NODE_SYNC_INTERVAL_MS: u32 = 60_000;
/// The AP side's periodic node sync runs this long after the station
/// side's (`mesh.hpp`: both ends schedule one per connection).
pub const AP_SYNC_LAG_MS: u32 = 10_000;
/// Time sync while not yet synced.
pub const TIME_SYNC_FAST_MS: u32 = 10_000;
/// Time sync once synced (`TIME_SYNC_INTERVAL`).
pub const TIME_SYNC_INTERVAL_MS: u32 = 60_000;
/// Retry of a time sync whose offset was not yet within accuracy.
pub const TIME_RETRY_MS: u32 = 200;
/// SoftAP stations per node (`MAX_CONN`, ESP32).
pub const MAX_CONN: usize = 10;
/// Packages a connection's send buffer holds (`MAX_MESSAGE_QUEUE`).
pub const MAX_MESSAGE_QUEUE: usize = 50;
/// Scanning the mesh channel for SoftAPs.
pub const SCAN_MS: u32 = 2_000;
/// Association + DHCP + TCP connect after picking an AP.
pub const CONNECT_MS: u32 = 1_500;
/// Rescan after a scan that found no usable AP (`0.5 * SCAN_INTERVAL`).
pub const RESCAN_MS: u32 = 15_000;
/// lwIP's fast TCP timer (`CONFIG_LWIP_TCP_TMR_INTERVAL`): sends delayed
/// ACKs and retries output the driver refused (`ERR_MEM`).
pub const TCP_FAST_TMR_MS: u32 = 250;
/// lwIP's slow TCP timer (`TCP_SLOW_INTERVAL`, 2 fast ticks): the unit of
/// the retransmission timeout.
pub const TCP_SLOW_TMR_MS: u32 = 2 * TCP_FAST_TMR_MS;
/// RTO of a fresh connection, in slow ticks (`CONFIG_LWIP_TCP_RTO_TIME`
/// 3000 ms: `sv` starts at 6 ticks).
pub const RTO_INITIAL_TICKS: u32 = 6;
/// RTO once RTTs are sampled, in slow ticks. Mesh RTTs are far below a
/// tick, so `sa` stays 0 and `sv` decays 6, 5, 4, 3 and stays at 3.
pub const RTO_MIN_TICKS: u32 = 3;
/// Settled first retransmission timeout. lwIP's timer fires on the
/// `RTO_MIN_TICKS`-th slow tick after the send, so the wait is
/// 1.0-1.5 s. Telemetry streams with one segment in flight (Nagle), so no
/// duplicate ACKs arrive and fast retransmit never fires.
pub const RTO_MIN_MS: u32 = RTO_MIN_TICKS * TCP_SLOW_TMR_MS;
/// Largest backed-off RTO: lwIP shifts by at most 7 (`tcp_backoff`).
/// In practice painlessMesh's [`NODE_TIMEOUT_MS`] closes a stalled
/// connection long before.
pub const RTO_MAX_MS: u32 = RTO_MIN_MS << 7;
/// lwIP TCP maximum segment size on the ESP32.
pub const TCP_MSS: u32 = 1_436;

/// A step longer than this (or a gap since the last step) re-forms the tree.
const FORM_GAP_MS: u32 = 2_000;
/// `,"root":true`
const ROOT_FLAG_LEN: u32 = 12;
/// `{"nodeId":` + `}`
const TREE_NODE_LEN: u32 = 11;
/// `,"subs":[` + `]`
const SUBS_LEN: u32 = 10;

/// painlessMesh node id (derived from the MAC on hardware) for sim drone
/// `id`: unique and ten digits, like real ones.
pub fn node_id(id: u16) -> u32 {
    2_200_000_000 + u32::from(id) * 7_919
}

/// Appends `s` as the body of a JSON string (no surrounding quotes),
/// escaping like ArduinoJson: `"` and `\` get a backslash, control
/// characters become `\b \f \n \r \t` or `\u00XX`.
pub fn escape_into(out: &mut String, s: &str) {
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\u{8}' => out.push_str("\\b"),
            '\u{c}' => out.push_str("\\f"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
}

/// Bytes [`escape_into`] would append for `s`, without building it.
pub fn escaped_len(s: &str) -> usize {
    s.chars()
        .map(|c| match c {
            '"' | '\\' | '\u{8}' | '\u{c}' | '\n' | '\r' | '\t' => 2,
            c if (c as u32) < 0x20 => 6,
            c => c.len_utf8(),
        })
        .sum()
}

/// Clears `out` and writes a SINGLE (`kind = SINGLE`) or BROADCAST package
/// carrying `msg`. The terminating `'\0'` is not included in the string;
/// add 1 for the bytes on the TCP stream.
pub fn encode_package(out: &mut String, kind: u8, dest: u32, from: u32, msg: &str) {
    out.clear();
    let _ = write!(out, "{{\"type\":{kind},\"dest\":{dest},\"from\":{from},\"msg\":\"");
    escape_into(out, msg);
    out.push_str("\"}");
}

/// Clears `out` and writes a TIME_SYNC / TIME_DELAY package; `msg_type`
/// 0 asks, 1 carries `t0`, 2 carries `t0, t1, t2`.
pub fn encode_time_sync(out: &mut String, kind: u8, dest: u32, from: u32, msg_type: u8, t: [u32; 3]) {
    out.clear();
    let _ = write!(out, "{{\"type\":{kind},\"dest\":{dest},\"from\":{from},\"msg\":{{\"type\":{msg_type}");
    if msg_type >= 1 {
        let _ = write!(out, ",\"t0\":{}", t[0]);
    }
    if msg_type >= 2 {
        let _ = write!(out, ",\"t1\":{},\"t2\":{}", t[1], t[2]);
    }
    out.push_str("}}");
}

fn digits(mut v: u32) -> u32 {
    let mut n = 1;
    while v >= 10 {
        v /= 10;
        n += 1;
    }
    n
}

/// The first tick of `node`'s lwIP TCP timer with `period` strictly after
/// `now`. Each node's timers run at their own (fixed, deterministic) phase.
fn tick_after(node: usize, now: u32, period: u32) -> u32 {
    let phase = (node as u32).wrapping_mul(89).wrapping_add(17) % TCP_FAST_TMR_MS;
    now.wrapping_add(period - now.wrapping_add(phase) % period)
}

/// True once `now` is at or past `at` (wrap-safe).
fn reached(now: u32, at: u32) -> bool {
    (now.wrapping_sub(at) as i32) >= 0
}

/// A package in a TCP send buffer.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Package {
    /// Package type ([`SINGLE`], [`BROADCAST`], [`NODE_SYNC_REQUEST`], ...).
    pub kind: u8,
    /// Node that created it.
    pub origin: u16,
    /// Telemetry `"s"`.
    pub seq: u32,
    /// When it was created (the frame's `"t"`).
    pub born_ms: u32,
    /// Bytes on the TCP stream, `'\0'` included.
    pub bytes: u32,
    /// Telemetry: UDP payload bytes of the root's republished datagram.
    pub datagram: u32,
    /// Telemetry: the frame carries the SLOW keys.
    pub slow: bool,
    /// TIME_SYNC: `msg.type` (1 request, 2 reply).
    pub sub: u8,
    /// Airtime spent on it so far, µs (latency accounting).
    pub air_us: f32,
}

impl Package {
    fn is_control(&self) -> bool {
        !matches!(self.kind, SINGLE | BROADCAST)
    }
}

/// One mesh transmission that is not telemetry payload (node sync, time
/// sync), reported when it is queued, for chatter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshTx {
    pub src: u16,
    pub dst: u16,
    /// Package type ([`NODE_SYNC_REQUEST`], [`TIME_SYNC`], ...).
    pub kind: u8,
    /// JSON bytes on the TCP stream, `'\0'` included.
    pub bytes: u32,
}

/// What [`PainlessMesh::transport`] reports about telemetry packages.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MeshEvent {
    /// `src` put (a share of) a segment carrying `origin`'s package on air:
    /// package bytes plus its share of TCP/IP, LLC/SNAP and MAC headers.
    /// Retransmissions count again.
    Hop { src: u16, origin: u16, air_bytes: u32 },
    /// A telemetry package reached the root.
    Arrived(Package),
    /// `at`'s send buffer was full and discarded `origin`'s package.
    Dropped { at: u16, origin: u16 },
}

/// Station-side connection state of a node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkState {
    /// No parent; the running scan completes at `scan_done_ms`.
    Down { scan_done_ms: u32 },
    /// Associating to `parent`; connected at `up_ms` if it is still usable.
    Joining { parent: u16, up_ms: u32 },
    /// Connected to its parent (the root is always `Up`, without one).
    Up,
}

/// One direction of a TCP connection.
#[derive(Clone, Debug)]
struct Conn {
    q: VecDeque<Package>,
    /// Waiting for a retransmission timeout until then.
    blocked_until: Option<u32>,
    retries: u8,
    /// A segment is waiting for its (delayed) TCP ACK.
    ack_odd: bool,
    /// Upward (Nagle): the receiver's delayed ACK for the outstanding
    /// segment fires then; until it does, small segments are held.
    ack_at: Option<u32>,
    /// RTT samples taken on this connection (the RTO settles after 3).
    rtt_samples: u8,
    /// Couldn't drain last time: coalesce.
    backlog: bool,
}

impl Conn {
    fn new() -> Self {
        Self {
            q: VecDeque::with_capacity(MAX_MESSAGE_QUEUE + 4),
            blocked_until: None,
            retries: 0,
            ack_odd: false,
            ack_at: None,
            rtt_samples: 0,
            backlog: false,
        }
    }

    /// Retransmission timeout for the `retries`-th consecutive loss of the
    /// segment sent at `now_ms` by `tx`: lwIP's estimate in slow ticks,
    /// doubled per earlier retry, firing on `tx`'s slow timer.
    fn rto_deadline(&self, tx: usize, now_ms: u32) -> u32 {
        let base = RTO_INITIAL_TICKS.saturating_sub(u32::from(self.rtt_samples)).max(RTO_MIN_TICKS);
        let ticks = base << u32::from(self.retries.saturating_sub(1)).min(7);
        tick_after(tx, now_ms, TCP_SLOW_TMR_MS).wrapping_add((ticks - 1) * TCP_SLOW_TMR_MS)
    }

    fn clear(&mut self) {
        self.q.clear();
        self.blocked_until = None;
        self.retries = 0;
        self.ack_odd = false;
        self.ack_at = None;
        self.rtt_samples = 0;
        self.backlog = false;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dir {
    /// Child → parent (the child's STA connection).
    Up,
    /// Parent → child.
    Down,
}

#[derive(Clone, Debug)]
struct Node {
    alive: bool,
    /// Parent while `Up`, `-1` otherwise.
    parent: i16,
    state: LinkState,
    /// Hops to the root, `u8::MAX` if there is no live path.
    hops: u8,
    /// Structural depth below the top of its (possibly floating) tree.
    depth: u16,
    /// Top of its tree (the root, or the head of a floating subtree).
    top: u16,
    /// Signal of the parent heard here, `None` when unreachable.
    rssi: Option<i8>,
    /// Parent unreachable since.
    bad_since: Option<u32>,
    node_sync_at: u32,
    /// The AP side's own periodic node sync on this connection (painlessMesh
    /// runs one per end, the AP side 10 s after the station side).
    ap_sync_at: u32,
    /// Rate of the connection upward (station → AP), Mbps, as last used.
    rate_mbps: f32,
    /// An unanswered node sync request closes the connection then.
    sync_deadline: Option<u32>,
    /// Connected but cut off from the root: its next scan for a rooted AP.
    member_scan_at: Option<u32>,
    time_sync_at: u32,
    time_synced: bool,
    time_rounds: u8,
    /// The edge to the parent needs a sync request this step (wave).
    wave_up: bool,
    wave_down: bool,
    /// Ancestor-or-self of a changed node (wave scratch).
    mark: bool,
    up: Conn,
    down: Conn,
}

impl Node {
    fn new() -> Self {
        Self {
            alive: true,
            parent: -1,
            state: LinkState::Down { scan_done_ms: 0 },
            hops: u8::MAX,
            depth: 0,
            top: 0,
            rssi: None,
            bad_since: None,
            node_sync_at: 0,
            ap_sync_at: 0,
            rate_mbps: 0.0,
            sync_deadline: None,
            member_scan_at: None,
            time_sync_at: 0,
            time_synced: false,
            time_rounds: 0,
            wave_up: false,
            wave_down: false,
            mark: false,
            up: Conn::new(),
            down: Conn::new(),
        }
    }
}

/// The painlessMesh tree, its TCP connections and their traffic.
#[derive(Clone, Debug)]
pub struct PainlessMesh {
    root: u16,
    nodes: Vec<Node>,
    started: bool,
    last_ms: u32,
    /// Children of each node (Up edges), CSR: `kids[kid_off[i]..kid_off[i+1]]`.
    kids: Vec<u16>,
    kid_off: Vec<u32>,
    /// Nodes by depth, deepest first.
    order: Vec<u16>,
    /// Topology changed at these nodes this step / new edges (child ids).
    changed: Vec<u16>,
    new_edges: Vec<u16>,
    allowance: Vec<f32>,
    cursor: Vec<u32>,
    scratch: Vec<Package>,
    text: String,
    /// TCP data segments and TCP ACKs put on air so far (transmissions,
    /// retransmissions included).
    segments: u64,
    tcp_acks: u64,
    /// Connections closed so far: parent unreachable for NODE_TIMEOUT, and
    /// node sync request unanswered for NODE_TIMEOUT.
    closed_silent: u32,
    closed_unanswered: u32,
    /// Segments that went to an RTO: receiver unreachable, lost on air.
    rto_unreachable: u32,
    rto_on_air: u32,
}

impl PainlessMesh {
    /// A mesh of `nodes` nodes rooted at `root`, nobody connected yet.
    pub fn new(nodes: usize, root: u16) -> Self {
        Self {
            root,
            nodes: (0..nodes).map(|_| Node::new()).collect(),
            started: false,
            last_ms: 0,
            kids: Vec::with_capacity(nodes),
            kid_off: vec![0; nodes + 1],
            order: Vec::with_capacity(nodes),
            changed: Vec::new(),
            new_edges: Vec::new(),
            allowance: vec![0.0; nodes],
            cursor: Vec::with_capacity(nodes),
            scratch: Vec::with_capacity(64),
            text: String::with_capacity(96),
            segments: 0,
            tcp_acks: 0,
            closed_silent: 0,
            closed_unanswered: 0,
            rto_unreachable: 0,
            rto_on_air: 0,
        }
    }

    /// TCP data segments and TCP ACKs put on air since the mesh was made.
    /// Connections closed so far: (parent silent for `NODE_TIMEOUT_MS`,
    /// node sync unanswered for `NODE_TIMEOUT_MS`).
    pub fn closed_counts(&self) -> (u32, u32) {
        (self.closed_silent, self.closed_unanswered)
    }

    /// Segments that went to a retransmission timeout so far: (receiver
    /// unreachable, lost on air after every MAC attempt).
    pub fn rto_counts(&self) -> (u32, u32) {
        (self.rto_unreachable, self.rto_on_air)
    }

    pub fn tcp_counts(&self) -> (u64, u64) {
        (self.segments, self.tcp_acks)
    }

    pub fn root(&self) -> u16 {
        self.root
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Parent of `id` in the tree, `None` for the root or a node without a
    /// connection to a parent. A node can have a parent and still not be
    /// [`connected`](Self::connected) (its subtree is cut off from the root).
    pub fn parent(&self, id: u16) -> Option<u16> {
        self.nodes.get(usize::from(id)).and_then(|n| u16::try_from(n.parent).ok())
    }

    /// Hops from `id` to the root, `None` if it has no live path there:
    /// every node on its parent chain alive and every link in radio contact.
    pub fn hops(&self, id: u16) -> Option<u8> {
        self.nodes.get(usize::from(id)).map(|n| n.hops).filter(|&h| h != u8::MAX)
    }

    /// True if `id` currently has a path to a live root.
    pub fn connected(&self, id: u16) -> bool {
        self.hops(id).is_some()
    }

    /// Signal of its parent's SoftAP as `id` hears it (what `WiFi.RSSI()`
    /// reports), `None` without a reachable parent.
    /// Rate of `id`'s connection to its parent, Mbps, as the channel last
    /// chose it (`None` without a parent or before it first sent).
    pub fn parent_rate_mbps(&self, id: u16) -> Option<f32> {
        let n = self.nodes.get(usize::from(id))?;
        (n.parent >= 0 && n.rate_mbps > 0.0).then_some(n.rate_mbps)
    }

    /// TCP state of the connections (diagnostics): (connections with a
    /// pending retransmission, most consecutive retransmissions of one
    /// segment, connections blocked waiting for an RTO or a driver retry
    /// at `now_ms`).
    pub fn tcp_stall_stats(&self, now_ms: u32) -> (usize, u8, usize) {
        let (mut retrying, mut worst, mut blocked) = (0, 0u8, 0);
        for n in self.nodes.iter().filter(|n| n.parent >= 0) {
            for c in [&n.up, &n.down] {
                if c.retries > 0 {
                    retrying += 1;
                    worst = worst.max(c.retries);
                }
                if c.blocked_until.is_some_and(|t| !reached(now_ms, t)) {
                    blocked += 1;
                }
            }
        }
        (retrying, worst, blocked)
    }

    pub fn parent_rssi(&self, id: u16) -> Option<i8> {
        self.nodes.get(usize::from(id)).and_then(|n| if n.parent >= 0 { n.rssi } else { None })
    }

    /// Station-side state of `id`.
    pub fn state(&self, id: u16) -> Option<LinkState> {
        self.nodes.get(usize::from(id)).map(|n| n.state)
    }

    /// Children of `id` (connected to its SoftAP).
    pub fn children(&self, id: u16) -> &[u16] {
        let i = usize::from(id);
        if i >= self.nodes.len() {
            return &[];
        }
        &self.kids[self.kid_off[i] as usize..self.kid_off[i + 1] as usize]
    }

    /// Packages waiting in `id`'s send buffer to its parent, and in its
    /// parent's buffer toward it.
    pub fn queued(&self, id: u16) -> (usize, usize) {
        self.nodes.get(usize::from(id)).map_or((0, 0), |n| (n.up.q.len(), n.down.q.len()))
    }

    /// Nodes a package from `id` to the root crosses, in order, starting
    /// with `id`'s parent and ending at the root (empty if not connected).
    pub fn path_to_root(&self, id: u16, out: &mut Vec<u16>) {
        out.clear();
        if !self.connected(id) {
            return;
        }
        let mut at = id;
        while let Some(p) = self.parent(at) {
            out.push(p);
            at = p;
            if out.len() > self.nodes.len() {
                break; // defensive: never loop on a corrupt tree
            }
        }
    }

    // ---- tree --------------------------------------------------------------

    /// Tree maintenance for one step: association by RSSI, NODE_TIMEOUT
    /// detection, reconnect delays, and the node sync / time sync packages
    /// that go with them. Those are queued on the connections (their
    /// airtime is spent by [`PainlessMesh::transport`]) and reported to `out`.
    ///
    /// `alive[i]` is the sim's liveness and `rssi(a, b)` the signal of `b`
    /// heard at `a` (`None` = out of range / blocked).
    pub fn step(
        &mut self,
        now_ms: u32,
        dt_ms: u32,
        alive: &[bool],
        rssi: &mut dyn FnMut(usize, usize) -> Option<i8>,
        out: &mut Vec<MeshTx>,
    ) {
        let n = self.nodes.len();
        for (i, node) in self.nodes.iter_mut().enumerate() {
            node.alive = alive.get(i).copied().unwrap_or(false);
        }
        let gap = now_ms.wrapping_sub(self.last_ms);
        let fresh = !self.started || dt_ms > FORM_GAP_MS || gap > FORM_GAP_MS;
        self.last_ms = now_ms;
        if fresh {
            self.started = true;
            self.form(now_ms, rssi);
            return;
        }
        self.changed.clear();
        self.new_edges.clear();

        // The dead lose what was in their RAM at once; the connections
        // themselves stay (silent) until the other end times out.
        for i in 0..n {
            if !self.nodes[i].alive {
                self.dead(i, now_ms);
            }
        }
        self.rebuild();

        let root = usize::from(self.root);
        for i in 0..n {
            if i == root || !self.nodes[i].alive {
                continue;
            }
            let state = self.nodes[i].state;
            match state {
                LinkState::Up => {
                    let p = self.nodes[i].parent as usize;
                    let r = if self.nodes[p].alive { rssi(i, p) } else { None };
                    let node = &mut self.nodes[i];
                    node.rssi = r;
                    let timed_out = match r {
                        None => now_ms.wrapping_sub(*node.bad_since.get_or_insert(now_ms)) >= NODE_TIMEOUT_MS,
                        Some(_) => {
                            node.bad_since = None;
                            false
                        }
                    };
                    let unanswered = node.sync_deadline.is_some_and(|t| reached(now_ms, t));
                    if timed_out || unanswered {
                        if timed_out {
                            self.closed_silent += 1;
                        } else {
                            self.closed_unanswered += 1;
                        }
                        self.close(i, now_ms);
                        continue;
                    }
                    // Cut off from the root (its layout has no root since an
                    // upstream connection closed): look for a rooted AP and
                    // move there, subtree and all (`!isRooted` reconfigure).
                    if self.rooted(i) || !self.root_alive() {
                        self.nodes[i].member_scan_at = None;
                        continue;
                    }
                    let jitter = i as u32 * 373 % 1_000;
                    let at = *self.nodes[i].member_scan_at.get_or_insert(now_ms.wrapping_add(SCAN_MS + jitter));
                    if reached(now_ms, at) {
                        match self.best_ap(i, rssi, true) {
                            Some(p) => {
                                self.close(i, now_ms);
                                self.nodes[i].state = LinkState::Joining { parent: p, up_ms: now_ms.wrapping_add(CONNECT_MS) };
                            }
                            None => self.nodes[i].member_scan_at = Some(now_ms.wrapping_add(RESCAN_MS)),
                        }
                    }
                }
                LinkState::Joining { parent, up_ms } => {
                    if reached(now_ms, up_ms) {
                        let p = usize::from(parent);
                        // Still there, a slot free, and not (by now) inside
                        // our own tree.
                        let usable = p < n
                            && p != i
                            && self.nodes[p].alive
                            && self.nodes[p].top != self.nodes[i].top
                            && self.kid_count(p) < MAX_CONN;
                        match if usable { rssi(i, p) } else { None } {
                            Some(r) => self.connect(i, p, r, now_ms),
                            // That AP is gone: try the next scan.
                            None => self.nodes[i].state = LinkState::Down { scan_done_ms: now_ms.wrapping_add(SCAN_MS) },
                        }
                    }
                }
                LinkState::Down { scan_done_ms } => {
                    if reached(now_ms, scan_done_ms) {
                        self.nodes[i].state = match self.best_ap(i, rssi, false) {
                            Some(p) => LinkState::Joining { parent: p, up_ms: now_ms.wrapping_add(CONNECT_MS) },
                            None => LinkState::Down { scan_done_ms: now_ms.wrapping_add(RESCAN_MS) },
                        };
                    }
                }
            }
        }
        self.rebuild();
        self.sync_wave(now_ms, out);
        self.periodic(now_ms, out);
    }

    fn kid_count(&self, i: usize) -> usize {
        (self.kid_off[i + 1] - self.kid_off[i]) as usize
    }

    fn root_alive(&self) -> bool {
        self.nodes.get(usize::from(self.root)).is_some_and(|r| r.alive)
    }

    /// Alive and, as the mesh's layout stands, in the root's tree. A dead
    /// node's own uplink goes at once; its children keep their (silent)
    /// connections to it until NODE_TIMEOUT.
    fn rooted(&self, i: usize) -> bool {
        let node = &self.nodes[i];
        node.alive && node.top == self.root && self.root_alive()
    }

    /// Strongest usable SoftAP for `i`: alive, a free slot, and not in
    /// `i`'s own tree (`filterAPs`, which also rules out loops). One that
    /// reaches the root wins over any that doesn't; `rooted_only` refuses
    /// the latter.
    fn best_ap(&self, i: usize, rssi: &mut dyn FnMut(usize, usize) -> Option<i8>, rooted_only: bool) -> Option<u16> {
        let mut best: Option<((bool, i8), u16)> = None;
        for c in 0..self.nodes.len() {
            if c == i || !self.nodes[c].alive || self.nodes[c].top == self.nodes[i].top || self.kid_count(c) >= MAX_CONN {
                continue;
            }
            let rooted = self.rooted(c);
            if rooted_only && !rooted {
                continue;
            }
            if let Some(r) = rssi(i, c)
                && best.is_none_or(|(b, _)| (rooted, r) > b)
            {
                best = Some(((rooted, r), c as u16));
            }
        }
        best.map(|(_, c)| c)
    }

    /// Forms the tree at once (boot, or after a gap): breadth-first, each
    /// node joining the strongest AP among those that joined before it.
    fn form(&mut self, now_ms: u32, rssi: &mut dyn FnMut(usize, usize) -> Option<i8>) {
        let n = self.nodes.len();
        let root = usize::from(self.root);
        for (i, node) in self.nodes.iter_mut().enumerate() {
            let alive = node.alive;
            *node = Node::new();
            node.alive = alive;
            node.state = if i == root {
                LinkState::Up
            } else {
                LinkState::Down { scan_done_ms: now_ms.wrapping_add(i as u32 * 373 % 1_000) }
            };
        }
        self.rebuild();
        if root >= n || !self.nodes[root].alive {
            return;
        }
        let mut level: Vec<bool> = vec![false; n];
        let mut joins: Vec<usize> = vec![0; n];
        loop {
            joins.fill(0);
            // Candidates: nodes connected before this round.
            for (c, l) in level.iter_mut().enumerate() {
                *l = self.nodes[c].hops != u8::MAX;
            }
            let mut joined = false;
            for i in 0..n {
                if i == root || !self.nodes[i].alive || self.nodes[i].state == LinkState::Up {
                    continue;
                }
                let mut best: Option<(i8, usize)> = None;
                for c in 0..n {
                    if c == i || !level[c] || self.kid_count(c) + joins[c] >= MAX_CONN {
                        continue;
                    }
                    if let Some(r) = rssi(i, c)
                        && best.is_none_or(|(b, _)| r > b)
                    {
                        best = Some((r, c));
                    }
                }
                if let Some((r, p)) = best {
                    let node = &mut self.nodes[i];
                    node.state = LinkState::Up;
                    node.parent = p as i16;
                    node.rssi = Some(r);
                    node.time_synced = true;
                    node.time_rounds = 2;
                    node.node_sync_at = now_ms.wrapping_add(i as u32 * 7_919 % NODE_SYNC_INTERVAL_MS);
                    node.ap_sync_at = node.node_sync_at.wrapping_add(AP_SYNC_LAG_MS);
                    node.time_sync_at = now_ms.wrapping_add(i as u32 * 4_241 % TIME_SYNC_INTERVAL_MS);
                    joined = true;
                    // Count the slot right away (the CSR is rebuilt per round).
                    joins[p] += 1;
                }
            }
            self.rebuild();
            if !joined {
                break;
            }
        }
    }

    fn connect(&mut self, i: usize, p: usize, r: i8, now_ms: u32) {
        let node = &mut self.nodes[i];
        node.state = LinkState::Up;
        node.parent = p as i16;
        node.rssi = Some(r);
        node.bad_since = None;
        node.sync_deadline = None;
        node.node_sync_at = now_ms.wrapping_add(NODE_SYNC_INTERVAL_MS);
        node.ap_sync_at = node.node_sync_at.wrapping_add(AP_SYNC_LAG_MS);
        node.rate_mbps = 0.0;
        node.time_sync_at = now_ms;
        node.time_synced = false;
        node.time_rounds = 0;
        node.up.clear();
        node.down.clear();
        self.changed.push(p as u16);
        self.new_edges.push(i as u16);
    }

    /// Closes `i`'s connection to its parent (both send buffers are lost)
    /// and starts a rescan.
    fn close(&mut self, i: usize, now_ms: u32) {
        let node = &mut self.nodes[i];
        if node.parent >= 0 {
            self.changed.push(node.parent as u16);
            self.changed.push(i as u16);
        }
        node.parent = -1;
        node.rssi = None;
        node.bad_since = None;
        node.sync_deadline = None;
        node.member_scan_at = None;
        node.up.clear();
        node.down.clear();
        let jitter = i as u32 * 373 % 1_000;
        node.state = LinkState::Down { scan_done_ms: now_ms.wrapping_add(SCAN_MS + jitter) };
    }

    
    /// A dead node: its send buffers are gone; its uplink closes when its
    /// parent's side times out ([`NODE_TIMEOUT_MS`] of silence), like its
    /// children's connections to it do on their side. If it comes back it
    /// starts with a scan.
    fn dead(&mut self, i: usize, now_ms: u32) {
        let (s, e) = (self.kid_off[i] as usize, self.kid_off[i + 1] as usize);
        for k in s..e {
            let c = usize::from(self.kids[k]);
            self.nodes[c].down.clear();
        }
        let node = &mut self.nodes[i];
        node.up.clear();
        node.down.clear();
        node.rssi = None;
        node.sync_deadline = None;
        node.member_scan_at = None;
        if node.parent >= 0 {
            let since = *node.bad_since.get_or_insert(now_ms);
            if now_ms.wrapping_sub(since) >= NODE_TIMEOUT_MS {
                self.close(i, now_ms);
            }
        }
        if self.nodes[i].parent < 0 {
            self.nodes[i].state = LinkState::Down { scan_done_ms: now_ms.wrapping_add(SCAN_MS) };
        }
    }

    /// Children lists, depths, tops, hops and the depth order.
    fn rebuild(&mut self) {
        let n = self.nodes.len();
        self.kid_off.clear();
        self.kid_off.resize(n + 1, 0);
        for node in &self.nodes {
            if node.parent >= 0 {
                self.kid_off[node.parent as usize + 1] += 1;
            }
        }
        for i in 0..n {
            self.kid_off[i + 1] += self.kid_off[i];
        }
        self.kids.clear();
        self.kids.resize(self.kid_off[n] as usize, 0);
        // Fill in id order with a running cursor per parent.
        self.cursor.clear();
        self.cursor.extend_from_slice(&self.kid_off[..n]);
        for c in 0..n {
            let p = self.nodes[c].parent;
            if p >= 0 {
                let slot = &mut self.cursor[p as usize];
                self.kids[*slot as usize] = c as u16;
                *slot += 1;
            }
        }

        let root = usize::from(self.root);
        let root_up = root < n && self.nodes[root].alive;
        let mut max_depth = 0u16;
        for i in 0..n {
            // `live`: every node on the chain alive and every link audible
            // right now (a stale association waiting for NODE_TIMEOUT keeps
            // its parent but carries nothing).
            let (mut at, mut depth, mut live) = (i, 0u16, self.nodes[i].alive);
            while self.nodes[at].parent >= 0 && usize::from(depth) <= n {
                live &= self.nodes[at].rssi.is_some();
                at = self.nodes[at].parent as usize;
                live &= self.nodes[at].alive;
                depth += 1;
            }
            let node = &mut self.nodes[i];
            node.depth = depth;
            node.top = at as u16;
            node.hops = if at == root && root_up && live { depth.min(254) as u8 } else { u8::MAX };
            max_depth = max_depth.max(depth);
        }
        // Counting sort, deepest first.
        self.order.clear();
        for d in (0..=max_depth).rev() {
            for i in 0..n {
                if self.nodes[i].depth == d {
                    self.order.push(i as u16);
                }
            }
        }
    }

    /// Node sync requests after topology changes (`syncLayout`), plus the
    /// station side's immediate request on every new connection.
    fn sync_wave(&mut self, now_ms: u32, out: &mut Vec<MeshTx>) {
        if self.changed.is_empty() && self.new_edges.is_empty() {
            return;
        }
        let n = self.nodes.len();
        for node in &mut self.nodes {
            node.wave_up = false;
            node.wave_down = false;
        }
        for k in 0..self.new_edges.len() {
            let c = usize::from(self.new_edges[k]);
            self.nodes[c].wave_up = true;
        }
        for k in 0..self.changed.len() {
            let x = usize::from(self.changed[k]);
            if x >= n || !self.nodes[x].alive {
                continue;
            }
            for node in &mut self.nodes {
                node.mark = false;
            }
            let mut at = x;
            for _ in 0..=n {
                self.nodes[at].mark = true;
                match self.nodes[at].parent {
                    p if p >= 0 => at = p as usize,
                    _ => break,
                }
            }
            let top = self.nodes[x].top;
            for c in 0..n {
                let node = &mut self.nodes[c];
                if node.parent < 0 || node.top != top || !node.alive {
                    continue;
                }
                // The side the change is on tells the other side.
                if node.mark {
                    node.wave_up = true;
                } else {
                    node.wave_down = true;
                }
            }
        }
        for c in 0..n {
            let node = &self.nodes[c];
            if node.parent < 0 || !node.alive || !self.nodes[node.parent as usize].alive {
                continue;
            }
            let (up, down) = (node.wave_up, node.wave_down);
            if up {
                self.request_up(c, now_ms, out);
            }
            if down {
                let p = self.nodes[c].parent as usize;
                let bytes = self.sync_len(p, c);
                self.queue_control(c, Dir::Down, NODE_SYNC_REQUEST, 0, bytes, now_ms, out);
            }
        }
    }

    fn request_up(&mut self, c: usize, now_ms: u32, out: &mut Vec<MeshTx>) {
        let p = self.nodes[c].parent as usize;
        let bytes = self.sync_len(c, p);
        self.queue_control(c, Dir::Up, NODE_SYNC_REQUEST, 0, bytes, now_ms, out);
        let node = &mut self.nodes[c];
        node.sync_deadline.get_or_insert(now_ms.wrapping_add(NODE_TIMEOUT_MS));
        node.node_sync_at = now_ms.wrapping_add(NODE_SYNC_INTERVAL_MS);
    }

    /// Per-connection node sync (every 60 s) and time sync timers.
    fn periodic(&mut self, now_ms: u32, out: &mut Vec<MeshTx>) {
        for c in 0..self.nodes.len() {
            let node = &self.nodes[c];
            if node.parent < 0 || !node.alive || !self.nodes[node.parent as usize].alive {
                continue;
            }
            if reached(now_ms, node.node_sync_at) {
                self.request_up(c, now_ms, out);
            }
            if reached(now_ms, self.nodes[c].ap_sync_at) {
                // The AP side's own periodic request, answered upward.
                let p = self.nodes[c].parent as usize;
                let bytes = self.sync_len(p, c);
                self.queue_control(c, Dir::Down, NODE_SYNC_REQUEST, 0, bytes, now_ms, out);
                self.nodes[c].ap_sync_at = now_ms.wrapping_add(NODE_SYNC_INTERVAL_MS);
            }
            let node = &self.nodes[c];
            if reached(now_ms, node.time_sync_at) {
                let synced = node.time_synced;
                let bytes = self.time_len(c, Dir::Up, 1, now_ms);
                self.queue_control(c, Dir::Up, TIME_SYNC, 1, bytes, now_ms, out);
                let wait = if synced { TIME_SYNC_INTERVAL_MS } else { TIME_SYNC_FAST_MS };
                self.nodes[c].time_sync_at = now_ms.wrapping_add(wait);
            }
        }
    }

    /// Ends of edge `c` for a direction: (sender, receiver).
    fn ends(&self, c: usize, dir: Dir) -> (usize, usize) {
        let p = self.nodes[c].parent.max(0) as usize;
        match dir {
            Dir::Up => (c, p),
            Dir::Down => (p, c),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn queue_control(&mut self, c: usize, dir: Dir, kind: u8, sub: u8, bytes: u32, now_ms: u32, out: &mut Vec<MeshTx>) {
        let (tx, rx) = self.ends(c, dir);
        let pkg = Package { kind, origin: tx as u16, born_ms: now_ms, bytes, sub, ..Package::default() };
        let conn = match dir {
            Dir::Up => &mut self.nodes[c].up,
            Dir::Down => &mut self.nodes[c].down,
        };
        // Priority: in front of everything not yet on the wire. A full
        // buffer discards its newest telemetry to make room.
        if conn.q.len() >= MAX_MESSAGE_QUEUE
            && let Some(last) = conn.q.iter().rposition(|p| !p.is_control())
        {
            conn.q.remove(last);
        }
        let at = usize::from(conn.retries > 0).min(conn.q.len());
        conn.q.insert(at, pkg);
        out.push(MeshTx { src: tx as u16, dst: rx as u16, kind, bytes });
    }

    /// JSON bytes of `x`'s view of the mesh, minus the side behind `exclude`
    /// (`NodeTree::addTo`, after `excludeRoute`).
    fn tree_len(&self, x: usize, exclude: usize) -> u32 {
        let mut len = TREE_NODE_LEN + digits(node_id(x as u16));
        if x == usize::from(self.root) {
            len += ROOT_FLAG_LEN;
        }
        let mut subs = 0u32;
        self.for_each_link(x, exclude, |nb| {
            subs += 1;
            len += self.tree_len(nb, x);
        });
        if subs > 0 {
            len += SUBS_LEN + subs - 1;
        }
        len
    }

    /// Calls `f` for each live connection of `x` except to `exclude`:
    /// its parent first, then its children in id order.
    fn for_each_link(&self, x: usize, exclude: usize, mut f: impl FnMut(usize)) {
        let node = &self.nodes[x];
        if node.parent >= 0 {
            let p = node.parent as usize;
            if p != exclude && self.nodes[p].alive {
                f(p);
            }
        }
        for &k in &self.kids[self.kid_off[x] as usize..self.kid_off[x + 1] as usize] {
            let k = usize::from(k);
            if k != exclude && self.nodes[k].alive {
                f(k);
            }
        }
    }

    /// Bytes on the stream of a node sync request / reply from `x` to `y`.
    pub fn sync_len(&self, x: usize, y: usize) -> u32 {
        let n = self.nodes.len();
        if x >= n || y >= n {
            return 0;
        }
        // tree minus its closing brace, `,"type":5,"dest":D,"from":F}`, '\0'.
        self.tree_len(x, y) - 1 + 9 + 8 + digits(node_id(y as u16)) + 8 + digits(node_id(x as u16)) + 1 + 1
    }

    /// Clears `out` and writes the node sync package `x` sends `y` (exact
    /// text; the model itself only needs [`Self::sync_len`]).
    pub fn encode_node_sync(&self, out: &mut String, kind: u8, x: u16, y: u16) {
        out.clear();
        let (x, y) = (usize::from(x), usize::from(y));
        if x >= self.nodes.len() || y >= self.nodes.len() {
            return;
        }
        self.encode_tree(out, x, y);
        out.pop();
        let _ = write!(out, ",\"type\":{kind},\"dest\":{},\"from\":{}}}", node_id(y as u16), node_id(x as u16));
    }

    fn encode_tree(&self, out: &mut String, x: usize, exclude: usize) {
        let _ = write!(out, "{{\"nodeId\":{}", node_id(x as u16));
        if x == usize::from(self.root) {
            out.push_str(",\"root\":true");
        }
        let mut first = true;
        self.for_each_link(x, exclude, |nb| {
            out.push_str(if first { ",\"subs\":[" } else { "," });
            first = false;
            self.encode_tree(out, nb, x);
        });
        if !first {
            out.push(']');
        }
        out.push('}');
    }

    /// Bytes of a time sync package over edge `c` (node time in µs).
    fn time_len(&mut self, c: usize, dir: Dir, sub: u8, now_ms: u32) -> u32 {
        let (tx, rx) = self.ends(c, dir);
        let t = now_ms.wrapping_mul(1_000);
        encode_time_sync(&mut self.text, TIME_SYNC, node_id(rx as u16), node_id(tx as u16), sub, [t, t, t]);
        self.text.len() as u32 + 1
    }

    // ---- telemetry in ------------------------------------------------------

    /// Hands a telemetry package from `origin` to the mesh:
    /// `mesh.sendSingle(root, ...)` (`broadcast = false`) or
    /// `mesh.sendBroadcast(...)`. `false` when it was refused (no route to
    /// the root in the node's layout / no connection, or a full send
    /// buffer).
    pub fn send(&mut self, origin: u16, pkg: Package, broadcast: bool) -> bool {
        let o = usize::from(origin);
        if o >= self.nodes.len() || !self.nodes[o].alive {
            return false;
        }
        if !broadcast {
            // Routed by the layout the node knows: a connection whose
            // parent went silent still takes packages until NODE_TIMEOUT.
            return o != usize::from(self.root) && self.nodes[o].parent >= 0 && self.rooted(o) && self.push(o, Dir::Up, pkg);
        }
        let mut any = false;
        if self.nodes[o].parent >= 0 {
            any |= self.push(o, Dir::Up, pkg);
        }
        let (s, e) = (self.kid_off[o] as usize, self.kid_off[o + 1] as usize);
        for k in s..e {
            let c = usize::from(self.kids[k]);
            any |= self.push(c, Dir::Down, pkg);
        }
        any
    }

    /// Appends to a send buffer; `false` when it is full.
    fn push(&mut self, c: usize, dir: Dir, pkg: Package) -> bool {
        let conn = match dir {
            Dir::Up => &mut self.nodes[c].up,
            Dir::Down => &mut self.nodes[c].down,
        };
        if conn.q.len() >= MAX_MESSAGE_QUEUE {
            return false;
        }
        conn.q.push_back(pkg);
        true
    }

    // ---- transport ---------------------------------------------------------

    /// Moves packages over the connections for one step of `dt_ms`, spending
    /// airtime on `channel` (when it is on) and drawing its losses. Upward
    /// buffers are served deepest node first and downward ones shallowest
    /// first, so a package can cross several hops in one step, as it does
    /// in a few milliseconds on hardware. Node sync and time sync replies
    /// are queued as their requests arrive, and reported to `ctl_out`.
    pub fn transport(
        &mut self,
        now_ms: u32,
        dt_ms: u32,
        channel: &mut Channel,
        events: &mut Vec<MeshEvent>,
        ctl_out: &mut Vec<MeshTx>,
    ) {
        let n = self.nodes.len();
        let ch_on = channel.enabled();
        let dt_us = dt_ms.min(1_000) as f32 * 1_000.0;
        for i in 0..n {
            self.allowance[i] =
                if ch_on { (1.0 - channel.busy(i as u16)).max(0.05) * dt_us } else { f32::INFINITY };
        }
        for k in 0..self.order.len() {
            let c = usize::from(self.order[k]);
            self.service(c, Dir::Up, now_ms, channel, events, ctl_out);
        }
        for k in (0..self.order.len()).rev() {
            let c = usize::from(self.order[k]);
            self.service(c, Dir::Down, now_ms, channel, events, ctl_out);
        }
    }

    fn conn(&mut self, c: usize, dir: Dir) -> &mut Conn {
        match dir {
            Dir::Up => &mut self.nodes[c].up,
            Dir::Down => &mut self.nodes[c].down,
        }
    }

    fn service(
        &mut self,
        c: usize,
        dir: Dir,
        now_ms: u32,
        channel: &mut Channel,
        events: &mut Vec<MeshEvent>,
        ctl_out: &mut Vec<MeshTx>,
    ) {
        if self.nodes[c].parent < 0 {
            return;
        }
        let (tx, rx) = self.ends(c, dir);
        if !self.nodes[tx].alive {
            return;
        }
        let link_rssi = self.nodes[c].rssi;
        let reachable = self.nodes[rx].alive && link_rssi.is_some();
        let ch_on = channel.enabled();
        // Minstrel-like rate choice from the link's SINR, DSSS 1 Mbps up to
        // HT MCS7 (the mesh leaves 802.11b on); the TCP ACKs go back over
        // the same link.
        let rates = channel.link_rates(tx as u16, rx as u16);
        let back_rates = channel.link_rates(rx as u16, tx as u16);
        if dir == Dir::Up {
            self.nodes[c].rate_mbps = rates.0;
        }
        // Station side: Nagle on (see the module docs); accepted side: NODELAY.
        let nagle = dir == Dir::Up;
        let tcp_ack = |channel: &mut Channel| {
            let f = AirFrame::new(rx as u16, AirKind::Ack, channel::TCP_ACK_MPDU, back_rates.0).with_rates(back_rates);
            channel.unicast(now_ms, &f, Dst::Node(tx as u16), reachable);
        };
        loop {
            let conn = self.conn(c, dir);
            if let Some(t) = conn.ack_at
                && reached(now_ms, t)
            {
                // The receiver's delayed ACK fires on its fast timer (lwIP
                // delays the ACK of a lone segment in both directions).
                conn.ack_at = None;
                conn.ack_odd = false;
                if reachable {
                    self.tcp_acks += 1;
                    tcp_ack(channel);
                }
            }
            let conn = self.conn(c, dir);
            if conn.q.is_empty() {
                conn.backlog = false;
                return;
            }
            if let Some(t) = conn.blocked_until {
                if !reached(now_ms, t) {
                    return;
                }
                conn.blocked_until = None;
            }
            if self.allowance[tx] <= 0.0 {
                self.conn(c, dir).backlog = true;
                return;
            }
            let conn = self.conn(c, dir);
            let mut payload = conn.q[0].bytes;
            let mut k = 1;
            if nagle || conn.backlog || conn.retries > 0 {
                while k < conn.q.len() && payload + conn.q[k].bytes <= TCP_MSS {
                    payload += conn.q[k].bytes;
                    k += 1;
                }
            }
            // Nagle: a segment short of the MSS (nothing left over behind
            // it) waits while one is unacknowledged. Retransmissions don't.
            let full = k < conn.q.len() || payload >= TCP_MSS;
            if nagle && conn.ack_odd && !full && conn.retries == 0 {
                return;
            }
            let segs = payload.div_ceil(TCP_MSS).max(1);
            let kind = if conn.q[0].is_control() { AirKind::MeshControl } else { AirKind::MeshData };
            let retx = conn.retries > 0;
            // Shares of the payload, for the byte ledger.
            let (mut own, mut ctl) = (0u32, 0u32);
            for pkg in conn.q.iter().take(k) {
                if pkg.is_control() {
                    ctl += pkg.bytes;
                } else if usize::from(pkg.origin) == tx {
                    own += pkg.bytes;
                }
            }
            // A frame the WiFi driver refuses (full TX queue) never airs:
            // lwIP gets ERR_MEM and retries on its next fast tick, no RTO.
            if ch_on && !channel.admit(tx as u16) {
                let conn = self.conn(c, dir);
                conn.blocked_until = Some(tick_after(tx, now_ms, TCP_FAST_TMR_MS));
                conn.backlog = true;
                return;
            }
            let seg_hdr = channel::TCP_IP_OVERHEAD + channel::LLC_SNAP + channel::QOS_MAC_OVERHEAD;
            let hdr = segs * seg_hdr;
            let conn = self.conn(c, dir);
            for pkg in conn.q.iter().take(k) {
                if !pkg.is_control() {
                    let air_bytes = pkg.bytes + hdr / k as u32;
                    events.push(MeshEvent::Hop { src: tx as u16, origin: pkg.origin, air_bytes });
                }
            }
            let mut f = AirFrame::new(tx as u16, kind, payload.div_ceil(segs) + seg_hdr, rates.0).with_rates(rates);
            f.own_frac = own as f32 / payload.max(1) as f32;
            f.ctl_frac = ctl as f32 / payload.max(1) as f32;
            f.retx = retx;
            let (mut us, mut lost) = (0.0, false);
            for _ in 0..segs {
                let r = channel.unicast(now_ms, &f, Dst::Node(rx as u16), reachable);
                us += r.us;
                lost |= !r.delivered;
            }
            self.allowance[tx] -= us;
            self.segments += u64::from(segs);
            if lost {
                if reachable {
                    self.rto_on_air += 1;
                } else {
                    self.rto_unreachable += 1;
                }
            }
            let conn = self.conn(c, dir);
            if lost {
                // Lost on air or nobody there: wait for the RTO (one
                // segment in flight gets no duplicate ACKs).
                conn.retries = conn.retries.saturating_add(1);
                conn.blocked_until = Some(conn.rto_deadline(tx, now_ms));
                conn.backlog = true;
                return;
            }
            if conn.retries == 0 {
                // Karn: no RTT sample from a retransmitted segment.
                conn.rtt_samples = conn.rtt_samples.saturating_add(1).min(RTO_INITIAL_TICKS as u8);
            }
            conn.retries = 0;
            let mut acks = 0;
            for _ in 0..segs {
                if conn.ack_odd {
                    // Second unacknowledged segment: ACKed at once.
                    acks += 1;
                    conn.ack_at = None;
                } else {
                    conn.ack_at = Some(tick_after(rx, now_ms, TCP_FAST_TMR_MS));
                }
                conn.ack_odd = !conn.ack_odd;
            }
            self.tcp_acks += acks;
            let mut batch = std::mem::take(&mut self.scratch);
            batch.clear();
            let conn = self.conn(c, dir);
            batch.extend(conn.q.drain(..k).map(|mut p| {
                p.air_us += us;
                p
            }));
            for _ in 0..acks {
                tcp_ack(channel);
            }
            for &pkg in &batch {
                self.arrive(c, dir, pkg, now_ms, events, ctl_out);
            }
            self.scratch = batch;
        }
    }

    /// `pkg` crossed edge `c` in direction `dir`.
    fn arrive(&mut self, c: usize, dir: Dir, pkg: Package, now_ms: u32, events: &mut Vec<MeshEvent>, ctl_out: &mut Vec<MeshTx>) {
        let (_, rx) = self.ends(c, dir);
        let root = usize::from(self.root);
        match pkg.kind {
            SINGLE => {
                if rx == root {
                    events.push(MeshEvent::Arrived(pkg));
                } else if self.nodes[rx].parent < 0 || !self.push(rx, Dir::Up, pkg) {
                    events.push(MeshEvent::Dropped { at: rx as u16, origin: pkg.origin });
                }
            }
            BROADCAST => {
                if rx == root {
                    events.push(MeshEvent::Arrived(pkg));
                }
                if dir == Dir::Up && rx != root && self.nodes[rx].parent >= 0 && !self.push(rx, Dir::Up, pkg) {
                    events.push(MeshEvent::Dropped { at: rx as u16, origin: pkg.origin });
                }
                let (s, e) = (self.kid_off[rx] as usize, self.kid_off[rx + 1] as usize);
                for j in s..e {
                    let k = usize::from(self.kids[j]);
                    if (dir == Dir::Up && k == c) || !self.nodes[k].alive {
                        continue;
                    }
                    if !self.push(k, Dir::Down, pkg) {
                        events.push(MeshEvent::Dropped { at: rx as u16, origin: pkg.origin });
                    }
                }
            }
            NODE_SYNC_REQUEST => {
                let back = if dir == Dir::Up { Dir::Down } else { Dir::Up };
                let (tx2, rx2) = self.ends(c, back);
                let bytes = self.sync_len(tx2, rx2);
                self.queue_control(c, back, NODE_SYNC_REPLY, 0, bytes, now_ms, ctl_out);
            }
            NODE_SYNC_REPLY => {
                if dir == Dir::Down {
                    self.nodes[c].sync_deadline = None;
                }
            }
            TIME_SYNC => {
                if pkg.sub == 1 && dir == Dir::Up {
                    let bytes = self.time_len(c, Dir::Down, 2, now_ms);
                    self.queue_control(c, Dir::Down, TIME_SYNC, 2, bytes, now_ms, ctl_out);
                } else if pkg.sub == 2 && dir == Dir::Down {
                    let node = &mut self.nodes[c];
                    node.time_rounds = node.time_rounds.saturating_add(1);
                    if node.time_synced || node.time_rounds >= 2 {
                        node.time_synced = true;
                        node.time_sync_at = now_ms.wrapping_add(TIME_SYNC_INTERVAL_MS);
                    } else {
                        node.time_sync_at = now_ms.wrapping_add(TIME_RETRY_MS);
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::channel::ChannelConfig;

    #[test]
    fn package_escapes_the_inner_json() {
        let mut s = String::new();
        encode_package(&mut s, SINGLE, node_id(0), node_id(3), r#"{"type":"drone","n":3}"#);
        assert_eq!(
            s,
            r#"{"type":9,"dest":2200000000,"from":2200023757,"msg":"{\"type\":\"drone\",\"n\":3}"}"#
        );
        assert_eq!(escaped_len("a\"b\\c\n\u{1}"), 1 + 2 + 1 + 2 + 1 + 2 + 6);
        let mut e = String::new();
        escape_into(&mut e, "a\"b\\c\n\u{1}");
        assert_eq!(e.len(), escaped_len("a\"b\\c\n\u{1}"));
        assert_eq!(e, "a\\\"b\\\\c\\n\\u0001");
    }

    #[test]
    fn time_sync_shapes() {
        let mut s = String::new();
        encode_time_sync(&mut s, TIME_SYNC, 1, 2, 2, [10, 20, 30]);
        assert_eq!(s, r#"{"type":4,"dest":1,"from":2,"msg":{"type":2,"t0":10,"t1":20,"t2":30}}"#);
        encode_time_sync(&mut s, TIME_SYNC, 1, 2, 0, [0; 3]);
        assert_eq!(s, r#"{"type":4,"dest":1,"from":2,"msg":{"type":0}}"#);
    }

    /// Nodes on a line, `gap` apart: each hears only its neighbours within
    /// `range`, louder when closer.
    fn line(gap: f32, range: f32) -> impl FnMut(usize, usize) -> Option<i8> {
        move |a: usize, b: usize| {
            let d = (a as f32 - b as f32).abs() * gap;
            (d <= range).then(|| (-30.0 - d * 0.5) as i8)
        }
    }

    fn run(m: &mut PainlessMesh, t: &mut u32, ms: u32, alive: &[bool], rssi: &mut dyn FnMut(usize, usize) -> Option<i8>) -> Vec<MeshTx> {
        let mut all = Vec::new();
        let mut ch = Channel::new(ChannelConfig::default(), m.len(), 1);
        let (mut ev, mut ctl) = (Vec::new(), Vec::new());
        for _ in 0..ms / 50 {
            *t += 50;
            let mut out = Vec::new();
            m.step(*t, 50, alive, rssi, &mut out);
            all.extend(out);
            m.transport(*t, 50, &mut ch, &mut ev, &mut ctl);
            all.append(&mut ctl);
        }
        all
    }

    #[test]
    fn boot_forms_a_strongest_rssi_tree() {
        // 0 - 1 - 2 - 3 on a line, 60 m apart, range 130 m: 1 and 2 hear
        // the root, 3 only 1 and 2 (2 is louder).
        let mut m = PainlessMesh::new(4, 0);
        let mut t = 0;
        let mut rssi = line(60.0, 130.0);
        run(&mut m, &mut t, 50, &[true; 4], &mut rssi);
        assert_eq!((m.parent(0), m.hops(0)), (None, Some(0)));
        assert_eq!((m.parent(1), m.hops(1)), (Some(0), Some(1)));
        assert_eq!((m.parent(2), m.hops(2)), (Some(0), Some(1)));
        assert_eq!((m.parent(3), m.hops(3)), (Some(2), Some(2)), "strongest AP that reaches the root");
        assert_eq!(m.children(0), [1, 2]);
        let mut path = Vec::new();
        m.path_to_root(3, &mut path);
        assert_eq!(path, [2, 0]);
        assert!(m.parent_rssi(3).is_some());
    }

    #[test]
    fn max_conn_forces_a_second_hop() {
        let n = MAX_CONN + 3;
        let mut m = PainlessMesh::new(n, 0);
        let mut t = 0;
        // Everybody hears everybody; the root is loudest for all.
        let mut rssi = |a: usize, b: usize| (a != b).then(|| if b == 0 { -40 } else { -50 - (b as i8 % 5) });
        run(&mut m, &mut t, 50, &vec![true; n], &mut rssi);
        assert_eq!(m.children(0).len(), MAX_CONN);
        assert!((1..n as u16).all(|i| m.connected(i)));
        assert_eq!((1..n as u16).filter(|&i| m.hops(i) == Some(2)).count(), 2);
    }

    #[test]
    fn node_timeout_then_rescan_and_rejoin() {
        // 0 - 1 - 2 - 3, range 70: a chain. Kill 1: 2 must time out (10 s),
        // rescan and find nobody that reaches the root (3 is below it).
        let mut m = PainlessMesh::new(4, 0);
        let mut t = 0;
        let mut rssi = line(60.0, 70.0);
        run(&mut m, &mut t, 50, &[true; 4], &mut rssi);
        assert_eq!(m.hops(3), Some(3));
        let mut alive = [true, false, true, true];
        let t0 = t;
        let mut dropped_at = None;
        while t - t0 < 20_000 {
            run(&mut m, &mut t, 50, &alive, &mut rssi);
            if dropped_at.is_none() && m.parent(2).is_none() {
                dropped_at = Some(t - t0);
            }
        }
        let d = dropped_at.expect("2 dropped its dead parent");
        assert!((NODE_TIMEOUT_MS..NODE_TIMEOUT_MS + 100).contains(&d), "timeout after {d} ms");
        assert!(!m.connected(2) && !m.connected(3));
        assert_eq!(m.parent(3), Some(2), "a child stays with its parent while it rejoins");
        // Revive 1 (a new relay arrives): 2 rejoins within one rescan
        // interval plus association.
        alive[1] = true;
        let t1 = t;
        let mut joined_at = None;
        while t - t1 < 40_000 && joined_at.is_none() {
            run(&mut m, &mut t, 50, &alive, &mut rssi);
            if m.connected(3) {
                joined_at = Some(t - t1);
            }
        }
        let j = joined_at.expect("3 is back through 2");
        assert!(j <= SCAN_MS + RESCAN_MS + 2 * CONNECT_MS + 2_000, "rejoined after {j} ms");
    }

    #[test]
    fn cut_off_subtree_moves_to_a_rooted_ap() {
        // Boot: 0 <- 1 <- 2 <- 3 and 0 <- 4. Then 3 starts hearing 4.
        let link = std::cell::Cell::new(false);
        let mut rssi = |a: usize, b: usize| match (a.min(b), a.max(b)) {
            (0, 1) | (1, 2) | (0, 4) => Some(-60),
            (2, 3) => Some(-40),
            (3, 4) if link.get() => Some(-80),
            _ => None,
        };
        let mut m = PainlessMesh::new(5, 0);
        let mut t = 0;
        run(&mut m, &mut t, 50, &[true; 5], &mut rssi);
        assert_eq!((m.parent(2), m.parent(3), m.parent(4)), (Some(1), Some(2), Some(0)));
        link.set(true);
        let alive = [true, false, true, true, true];
        let t0 = t;
        let (mut at3, mut at2) = (None, None);
        while t - t0 < 25_000 {
            run(&mut m, &mut t, 50, &alive, &mut rssi);
            if at3.is_none() && m.connected(3) {
                at3 = Some(t - t0);
            }
            if at2.is_none() && m.connected(2) {
                at2 = Some(t - t0);
            }
        }
        let at3 = at3.expect("3 reaches the root through 4");
        assert_eq!(m.parent(3), Some(4));
        assert!(at3 <= NODE_TIMEOUT_MS + SCAN_MS + 1_000 + CONNECT_MS + 100, "3 after {at3} ms");
        // 2 lost its parent and its child; it joins its former child.
        assert_eq!(m.parent(2), Some(3));
        let at2 = at2.expect("2 rejoined");
        assert!(at2 <= at3 + CONNECT_MS + 100, "2 after {at2} ms, 3 after {at3} ms");
        assert_eq!(m.hops(2), Some(3));
    }

    #[test]
    fn no_root_election() {
        let mut m = PainlessMesh::new(4, 0);
        let mut t = 0;
        let mut rssi = |a: usize, b: usize| (a != b).then_some(-50);
        run(&mut m, &mut t, 50, &[true; 4], &mut rssi);
        run(&mut m, &mut t, 30_000, &[false, true, true, true], &mut rssi);
        assert!((0..4).all(|i| !m.connected(i)), "no root election in painlessMesh");
        // The survivors still find each other (painlessMesh joins any node
        // it doesn't know), as one rootless tree without loops.
        let tops: Vec<u16> = (1..4).map(|i| m.nodes[i].top).collect();
        assert!(tops.iter().all(|&t| t == tops[0]), "{tops:?}");
        assert_eq!((1..4u16).filter(|&i| m.parent(i).is_none()).count(), 1);
    }

    #[test]
    fn node_sync_lengths_match_the_exact_json() {
        let mut m = PainlessMesh::new(5, 0);
        let mut t = 0;
        let mut rssi = line(60.0, 70.0);
        run(&mut m, &mut t, 50, &[true; 5], &mut rssi);
        let mut s = String::new();
        m.encode_node_sync(&mut s, NODE_SYNC_REQUEST, 4, 3);
        assert_eq!(s, format!(r#"{{"nodeId":{},"type":5,"dest":{},"from":{}}}"#, node_id(4), node_id(3), node_id(4)));
        assert_eq!(m.sync_len(4, 3) as usize, s.len() + 1);
        m.encode_node_sync(&mut s, NODE_SYNC_REPLY, 3, 4);
        let want = format!(
            r#"{{"nodeId":{},"subs":[{{"nodeId":{},"subs":[{{"nodeId":{},"subs":[{{"nodeId":{},"root":true}}]}}]}}],"type":6,"dest":{},"from":{}}}"#,
            node_id(3),
            node_id(2),
            node_id(1),
            node_id(0),
            node_id(4),
            node_id(3)
        );
        assert_eq!(s, want);
        assert_eq!(m.sync_len(3, 4) as usize, s.len() + 1);
        // A star: siblings are comma separated.
        let mut star = PainlessMesh::new(4, 0);
        let mut all = |a: usize, b: usize| (a != b).then_some(if b == 0 { -40 } else { -60 });
        run(&mut star, &mut t, 50, &[true; 4], &mut all);
        star.encode_node_sync(&mut s, NODE_SYNC_REQUEST, 0, 3);
        assert!(s.starts_with(&format!(r#"{{"nodeId":{},"root":true,"subs":[{{"nodeId":{}}},{{"nodeId":"#, node_id(0), node_id(1))));
        assert_eq!(star.sync_len(0, 3) as usize, s.len() + 1);
    }

    #[test]
    fn sync_and_time_traffic_follow_the_timers() {
        let mut m = PainlessMesh::new(3, 0);
        let mut t = 0;
        let mut rssi = line(60.0, 70.0);
        run(&mut m, &mut t, 50, &[true; 3], &mut rssi);
        let tx = run(&mut m, &mut t, 120_000, &[true; 3], &mut rssi);
        let count = |kind: u8, src: u16| tx.iter().filter(|x| x.kind == kind && x.src == src).count();
        // Two edges (1-0, 2-1): each end asks once per minute and the
        // other end replies.
        assert_eq!(count(NODE_SYNC_REQUEST, 1), 2 + 2, "1 asks its parent and its child");
        assert_eq!(count(NODE_SYNC_REQUEST, 0), 2, "the root asks its child (AP side)");
        assert_eq!(count(NODE_SYNC_REQUEST, 2), 2);
        assert_eq!(count(NODE_SYNC_REPLY, 0), 2);
        assert_eq!(count(NODE_SYNC_REPLY, 1), 2 + 2);
        assert_eq!(count(NODE_SYNC_REPLY, 2), 2);
        // Synced at boot: one time sync round per minute per edge.
        assert_eq!(count(TIME_SYNC, 2), 2);
        assert_eq!(count(TIME_SYNC, 1), 2 + 2, "1 asks its parent and answers its child");

        // A new connection: immediate node sync, then a wave over the other
        // edge, and two time sync rounds 200 ms apart.
        let mut m = PainlessMesh::new(3, 0);
        let mut t = 0;
        run(&mut m, &mut t, 50, &[true, true, false], &mut rssi);
        let tx = run(&mut m, &mut t, 6_000, &[true; 3], &mut rssi);
        assert_eq!(m.hops(2), Some(2));
        let reqs: Vec<_> = tx.iter().filter(|x| x.kind == NODE_SYNC_REQUEST).map(|x| (x.src, x.dst)).collect();
        assert!(reqs.contains(&(2, 1)) && reqs.contains(&(1, 0)), "{reqs:?}");
        let times = tx.iter().filter(|x| x.kind == TIME_SYNC && x.src == 2).count();
        assert_eq!(times, 2, "two rounds to sync a new connection");
        let reply = tx.iter().find(|x| x.kind == NODE_SYNC_REPLY && x.src == 1 && x.dst == 2).unwrap();
        assert_eq!(reply.bytes, m.sync_len(1, 2));
    }

    #[test]
    fn single_goes_up_broadcast_floods() {
        // Star around the root with one second-level node: 0 <- 1,2 ; 1 <- 3.
        let mut m = PainlessMesh::new(4, 0);
        let mut t = 0;
        let mut rssi = |a: usize, b: usize| match (a.min(b), a.max(b)) {
            (0, 1) | (0, 2) | (1, 3) => Some(-50),
            _ => None,
        };
        run(&mut m, &mut t, 50, &[true; 4], &mut rssi);
        assert_eq!(m.parent(3), Some(1));
        let mut ch = Channel::new(ChannelConfig::default(), 4, 1);
        let (mut ev, mut ctl) = (Vec::new(), Vec::new());
        let pkg = Package { kind: SINGLE, origin: 3, seq: 7, bytes: 300, ..Package::default() };
        assert!(m.send(3, pkg, false));
        m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
        let hops: Vec<u16> = ev.iter().filter_map(|e| if let MeshEvent::Hop { src, .. } = e { Some(*src) } else { None }).collect();
        assert_eq!(hops, [3, 1], "up the tree, one hop per link");
        assert!(ev.iter().any(|e| matches!(e, MeshEvent::Arrived(p) if p.seq == 7 && p.air_us > 0.0)));
        ev.clear();
        let pkg = Package { kind: BROADCAST, ..pkg };
        assert!(m.send(3, pkg, true));
        // Nagle: 3→1 and 1→0 hold it until the single's delayed ACKs.
        m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
        assert!(!ev.iter().any(|e| matches!(e, MeshEvent::Hop { .. })), "held behind the unacked single");
        m.transport(t + TCP_FAST_TMR_MS, 50, &mut ch, &mut ev, &mut ctl);
        let mut hops: Vec<u16> = ev.iter().filter_map(|e| if let MeshEvent::Hop { src, .. } = e { Some(*src) } else { None }).collect();
        hops.sort_unstable();
        assert_eq!(hops, [0, 1, 3], "every edge once: 3→1, 1→0, 0→2");
        assert_eq!(ev.iter().filter(|e| matches!(e, MeshEvent::Arrived(_))).count(), 1);
        // A node with no route refuses sendSingle.
        let mut lone = PainlessMesh::new(2, 0);
        lone.step(50, 50, &[true, true], &mut |_, _| None, &mut Vec::new());
        assert!(!lone.send(1, pkg, false) && !lone.send(1, pkg, true));
    }

    #[test]
    fn unreachable_parent_stalls_tcp_and_the_buffer_fills() {
        let mut m = PainlessMesh::new(2, 0);
        let mut t = 0;
        let mut near = |a: usize, b: usize| (a != b).then_some(-50);
        run(&mut m, &mut t, 50, &[true; 2], &mut near);
        let mut ch = Channel::new(ChannelConfig::default(), 2, 1);
        let (mut ev, mut ctl) = (Vec::new(), Vec::new());
        let mut far = |_: usize, _: usize| None;
        let mut refused = 0;
        for s in 0..120u32 {
            t += 50;
            m.step(t, 50, &[true; 2], &mut far, &mut Vec::new());
            for _ in 0..2 {
                let pkg = Package { kind: SINGLE, origin: 1, seq: s, bytes: 250, ..Package::default() };
                refused += u32::from(!m.send(1, pkg, false));
            }
            m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
        }
        assert!(!ev.iter().any(|e| matches!(e, MeshEvent::Arrived(_))), "nothing gets through");
        assert_eq!(m.queued(1).0, MAX_MESSAGE_QUEUE, "the send buffer is full");
        assert!(refused >= 240 - MAX_MESSAGE_QUEUE as u32 - 2, "{refused} refused");
        // Retransmissions are rate limited by the RTO, not one per step.
        let tries = ev.iter().filter(|e| matches!(e, MeshEvent::Hop { .. })).count();
        assert!(tries < 40, "{tries} transmissions in 6 s of stall");
        // At NODE_TIMEOUT the connection closes and the buffer is dropped.
        for _ in 0..100 {
            t += 50;
            m.step(t, 50, &[true; 2], &mut far, &mut Vec::new());
        }
        assert_eq!((m.parent(1), m.queued(1)), (None, (0, 0)));
    }

    #[test]
    fn backlog_coalesces_into_segments() {
        let mut m = PainlessMesh::new(2, 0);
        let mut t = 0;
        let mut near = |a: usize, b: usize| (a != b).then_some(-50);
        run(&mut m, &mut t, 50, &[true; 2], &mut near);
        // Fill the buffer while the link is blocked by a retransmission.
        m.nodes[1].up.blocked_until = Some(t + 1_000);
        m.nodes[1].up.backlog = true;
        for s in 0..20 {
            assert!(m.send(1, Package { kind: SINGLE, origin: 1, seq: s, bytes: 250, ..Package::default() }, false));
        }
        let mut ch = Channel::new(ChannelConfig::default(), 2, 1);
        let (mut ev, mut ctl) = (Vec::new(), Vec::new());
        t += 1_000;
        m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
        // 5 packages of 250 B per 1436 B segment: each carries a fifth of
        // the segment's TCP/IP + LLC + MAC headers. Three segments go (two
        // full ones are ACKed at once, the third waits for its ACK); the
        // last, short of the MSS, is held by Nagle.
        let hdr = channel::TCP_IP_OVERHEAD + channel::LLC_SNAP + channel::QOS_MAC_OVERHEAD;
        assert!(ev.iter().all(|e| match e {
            MeshEvent::Hop { air_bytes, .. } => *air_bytes == 250 + hdr / 5,
            _ => true,
        }));
        assert_eq!(ev.iter().filter(|e| matches!(e, MeshEvent::Arrived(_))).count(), 15);
        assert_eq!(m.queued(1).0, 5);
        m.transport(t + TCP_FAST_TMR_MS, 50, &mut ch, &mut ev, &mut ctl);
        assert_eq!(ev.iter().filter(|e| matches!(e, MeshEvent::Arrived(_))).count(), 20);
        assert_eq!(m.queued(1).0, 0);
        // Drained and acknowledged, the next package goes alone.
        t += 2 * TCP_FAST_TMR_MS;
        m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
        ev.clear();
        assert!(m.send(1, Package { kind: SINGLE, origin: 1, bytes: 250, ..Package::default() }, false));
        m.transport(t + 50, 50, &mut ch, &mut ev, &mut ctl);
        assert_eq!(ev[0], MeshEvent::Hop { src: 1, origin: 1, air_bytes: 250 + hdr });
    }

    /// 25 Hz of 250 B packages for 10 s over one edge, `broadcast` from the
    /// root (downward) or single from the child (upward). Returns
    /// (segments, TCP ACKs, mean arrival latency at the root in ms).
    fn stream(broadcast: bool) -> (u64, u64, f32) {
        let mut m = PainlessMesh::new(2, 0);
        let mut t = 0;
        let mut near = |a: usize, b: usize| (a != b).then_some(-50);
        run(&mut m, &mut t, 1_000, &[true; 2], &mut near);
        let mut ch = Channel::new(ChannelConfig::default(), 2, 1);
        let (mut ev, mut ctl) = (Vec::new(), Vec::new());
        let (seg0, ack0) = m.tcp_counts();
        let (mut lat, mut n) = (0.0f32, 0u32);
        let mut next = t;
        for _ in 0..200 {
            t += 50;
            m.step(t, 50, &[true; 2], &mut near, &mut Vec::new());
            while reached(t, next) {
                let pkg = Package { kind: SINGLE, origin: 1, born_ms: next, bytes: 250, ..Package::default() };
                if broadcast {
                    assert!(m.send(0, Package { kind: BROADCAST, origin: 0, ..pkg }, true));
                } else {
                    assert!(m.send(1, pkg, false));
                }
                next += 40;
            }
            ev.clear();
            ctl.clear();
            m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
            for e in &ev {
                if let MeshEvent::Arrived(p) = e {
                    lat += t.wrapping_sub(p.born_ms) as f32;
                    n += 1;
                }
            }
        }
        let (seg, ack) = m.tcp_counts();
        // Node/time sync over the edge in these 10 s is a handful at most.
        (seg - seg0, ack - ack0, if n > 0 { lat / n as f32 } else { 0.0 })
    }

    #[test]
    fn upward_tcp_runs_nagle_with_delayed_acks() {
        // Station side, Nagle on: one small segment in flight, the rest
        // coalesced until the parent's delayed ACK (250 ms fast timer).
        // 25 Hz x 250 B is ~1.6 kB per 250 ms tick: a full segment plus a
        // short one per ACK round, ~8 segments a second instead of 25.
        let (segs, acks, lat) = stream(false);
        eprintln!("[nagle] upward: {segs} segments, {acks} TCP ACKs, mean latency {lat:.0} ms for 250 packages");
        assert!((30..=100).contains(&segs), "{segs} segments for 250 packages in 10 s (expect 4-8/s)");
        assert!(acks <= segs + 2 && 2 * acks + 2 >= segs, "{acks} TCP ACKs for {segs} segments");
        assert!((60.0..250.0).contains(&lat), "mean latency {lat:.0} ms: the 0-250 ms delayed-ACK wait");
        // Accepted side, NODELAY: every package its own segment, an ACK per two.
        let (segs, acks, _) = stream(true);
        assert!((245..=260).contains(&segs), "{segs} downward segments for 250 packages");
        assert!((120..=135).contains(&acks), "{acks} downward TCP ACKs");
    }

    #[test]
    fn tcp_timers_follow_lwip() {
        // Timer ticks are strictly after `now`, on the node's own phase.
        for now in [0u32, 1, 249, 250, 12_345] {
            let f = tick_after(3, now, TCP_FAST_TMR_MS);
            assert!(f > now && f - now <= TCP_FAST_TMR_MS, "{now} -> {f}");
            let s = tick_after(3, now, TCP_SLOW_TMR_MS);
            assert!(s > now && s - now <= TCP_SLOW_TMR_MS && (s - f).is_multiple_of(TCP_FAST_TMR_MS), "{now} -> {s}");
        }
        let mut c = Conn::new();
        // Fresh connection: 3 s (6 ticks); settled: 3 ticks, doubling.
        c.retries = 1;
        let d = c.rto_deadline(3, 1_000) - 1_000;
        assert!((2_500..=3_000).contains(&d), "fresh RTO {d} ms");
        c.rtt_samples = RTO_INITIAL_TICKS as u8;
        for (r, lo) in [(1u8, 1_000u32), (2, 2_500), (3, 5_500)] {
            c.retries = r;
            let d = c.rto_deadline(3, 1_000) - 1_000;
            assert!((lo..=lo + TCP_SLOW_TMR_MS).contains(&d), "retry {r}: RTO {d} ms");
        }
        assert_eq!(RTO_MIN_MS, 1_500);
    }

    #[test]
    fn a_lost_segment_waits_for_the_rto() {
        // An established upward stream, then the parent goes out of range
        // for 6 s (< NODE_TIMEOUT): the first retransmission comes 1-1.5 s
        // after the loss, the next 2.5-3 s later, not every 250 ms.
        let mut m = PainlessMesh::new(2, 0);
        let mut t = 0;
        let mut near = |a: usize, b: usize| (a != b).then_some(-50);
        let mut far = |_: usize, _: usize| None;
        run(&mut m, &mut t, 1_000, &[true; 2], &mut near);
        let mut ch = Channel::new(ChannelConfig::default(), 2, 1);
        let (mut ev, mut ctl) = (Vec::new(), Vec::new());
        for s in 0..20u32 {
            t += 50;
            m.step(t, 50, &[true; 2], &mut near, &mut Vec::new());
            assert!(m.send(1, Package { kind: SINGLE, origin: 1, seq: s, bytes: 250, ..Package::default() }, false));
            m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
        }
        assert!(m.send(1, Package { kind: SINGLE, origin: 1, seq: 99, bytes: 250, ..Package::default() }, false));
        let mut sends = Vec::new();
        for _ in 0..120 {
            t += 50;
            m.step(t, 50, &[true; 2], &mut far, &mut Vec::new());
            ev.clear();
            m.transport(t, 50, &mut ch, &mut ev, &mut ctl);
            if ev.iter().any(|e| matches!(e, MeshEvent::Hop { src: 1, .. })) {
                sends.push(t);
            }
        }
        assert!(sends.len() >= 3, "{sends:?}");
        let gaps: Vec<u32> = sends.windows(2).map(|w| w[1] - w[0]).collect();
        assert!((1_000..=1_550).contains(&gaps[0]), "first RTO {gaps:?}");
        assert!((2_500..=3_050).contains(&gaps[1]), "backed-off RTO {gaps:?}");
    }

    #[test]
    fn a_gap_reforms_the_tree() {
        let mut m = PainlessMesh::new(3, 0);
        let mut t = 0;
        let mut all = |a: usize, b: usize| (a != b).then_some(-50);
        run(&mut m, &mut t, 50, &[true; 3], &mut all);
        assert!(m.connected(2));
        // A huge step (or telemetry off for a while) must not leave stale
        // timers that drop everybody.
        m.step(t.wrapping_add(u32::MAX), u32::MAX, &[true; 3], &mut all, &mut Vec::new());
        assert!(m.connected(1) && m.connected(2));
    }
}
