# Swarm Mind: simulation reference

What the simulator models, every function you can call, and the experiments you can run with it. The [README](../README.md) covers setup and the demo controls; this document covers the machinery underneath.

- [1. Architecture](#1-architecture)
- [2. What you can simulate](#2-what-you-can-simulate)
- [3. The drone brain (`swarm-brain`)](#3-the-drone-brain-swarm-brain)
- [4. The world (`swarm-sim`)](#4-the-world-swarm-sim)
- [5. Real telemetry, painlessMesh and shared airtime](#5-real-telemetry-painlessmesh-and-shared-airtime)
- [6. WASM exports (`swarm-wasm`)](#6-wasm-exports-swarm-wasm)
- [7. Web worker protocol](#7-web-worker-protocol)
- [8. Data layouts](#8-data-layouts)
- [9. Showcase demos](#9-showcase-demos)
- [10. Test suites](#10-test-suites)
- [11. Measured numbers](#11-measured-numbers)
- [12. Limits and known gaps](#12-limits-and-known-gaps)

---

## 1. Architecture

```
┌───────────────────────── browser ──────────────────────────┐
│  main thread (three.js)          Web Worker                 │
│  render · HUD · console ◄─msgs─► sim.worker.ts ──C-ABI──►  │
│  link budget · demos             swarm.wasm                 │
└─────────────────────────────────────────┬──────────────────┘
                                          │
  swarm-wasm  (C-ABI exports, input sanitising, null-safe)
      │
  swarm-sim   (the world: terrain, radio, fire, bodies, channel,
      │        painlessMesh, telemetry, metrics, chatter)
      │
  swarm-brain (one per drone: no_std, no allocator, ~2.6 KB state,
               builds for ESP32-C3 riscv32imc-unknown-none-elf)
```

**The rule:** only the simulator knows where everything is. Each brain gets only its own `Sensors` and the frames the radio model delivers to its inbox. There is no shared state, no map and no coordinator. `metrics.rs` looks at the whole world, but only to *judge* the swarm in tests, never to steer it.

Every step (50 ms in the browser) the world:

1. delivers last step's frames (range, line of sight, random loss, channel loss),
2. calls `Brain::tick` for every live drone, collecting its outbox and its flight target,
3. moves bodies toward their targets (cruise 25 m above ground, at most 12 units/s),
4. spreads the fire,
5. if enabled, runs telemetry, the painlessMesh tree and the shared-airtime channel,
6. writes the snapshot, the chatter records and the stats buffers.

With the channel and telemetry off (the `SimConfig` default), the step 5 models never run. The world is bit-identical to the version before they existed (checked by T1 and the S8 determinism digest `8b94cbfcd261ec2f`).

---

## 2. What you can simulate

| Area | What you can do | How |
|---|---|---|
| **Maps** | 5 terrains, each a different radio problem (ridge, canyon walls, two crests, open sea, easy hills) | `Preset`, `sim_new_preset`, `#map=` hash |
| **Swarm size** | 1–256 drones (the base station is drone 0) | `sim_new(seed, drones)` |
| **Operator intents** | Hold, Monitor fire, Map perimeter, Return to base, with an optional target point | `sim_command`, buttons, Shift+click |
| **Failures** | Kill any drone (relay, scout, reserve, the base) singly or in groups | `sim_kill`, click a drone, demos |
| **Radio** | Range 260 units, terrain line of sight, log-distance RSSI, global random loss 0–100 % | `RadioConfig`, `sim_set_loss` |
| **Fire** | Wildfire on a 96×96 grid, driven by wind and slope, stopping at water and above the snow line; spot fires anywhere | `sim_set_wind`, `sim_ignite` |
| **Real telemetry** | Your firmware's exact JSON (XIAO S3, C6 RTK, or mixed) as painlessMesh 1.5.7 packages, Single or Broadcast | `sim_set_telemetry` |
| **Telemetry rate** | Fixed (0.1–50 Hz), your backend's budget rule, or brain-managed (Phase 5) | `rate_mode` 0/1/2 |
| **Shared 2.4 GHz airtime** | Every frame costs airtime; busy air causes collisions, retries, queue drops and loss for everyone | `sim_set_channel` |
| **External interference** | 0–80 % of the air taken by other networks | `external_busy` |
| **Root loss** | Kill the bridge: telemetry stops reaching the ground station (painlessMesh has no root election) while the brains elect a new sink | `sim_kill(0)` |
| **Speed** | 1–8 sim steps per worker tick | `speed` message |
| **Determinism** | Same seed and same inputs give the same world, step for step | `seed` |

Questions you can answer with it:

- How many bytes per second does each drone put on air at 25 Hz, and how many reach the ground station?
- At what fleet size and rate does the channel collapse, and which drones (by tree depth) suffer first?
- How much does 30 % interference from a neighbouring network cost?
- How long does the brain swarm take to heal after a relay dies, compared with painlessMesh's reroute?
- Does the backend's budget rule or the brain's own rate controller keep the fleet healthier?
- Does Broadcast routing cost what you expect compared with Single?

---

## 3. The drone brain (`swarm-brain`)

`no_std`, zero dependencies, fixed buffers (`MAX_NEIGHBORS` 48, `MAX_INBOX` 16, `MAX_OUTBOX` 4). The same code runs in WASM and builds bare-metal with `cargo brain-mcu`. Its state stays under 4096 B (a test enforces it).

### 3.1 Public API

| Item | Purpose |
|---|---|
| `Brain::new(id, is_sink, home, cfg) -> Brain` | One drone's brain. `is_sink` is true for the base station. |
| `Brain::tick(&mut self, sensors, inbox, outbox) -> Actuation` | The whole decision loop, once per control step: read frames, update neighbours, maintain the route, play its formation part, choose what to send and where to fly. |
| `id()`, `is_sink()` | Identity. |
| `hops()`, `parent()`, `sink()` | Current route to the sink (`NO_ROUTE` = 255 when there is none). |
| `role()` | `Reserve`, `Scout`, `Relay` or `Sink`. |
| `slot()` | Formation slot it holds (0 = none). |
| `intent()`, `intent_target()` | Newest operator intent it knows, and its target point. |
| `neighbors()` | Its belief about every drone it hears (`Neighbor`: hops, parent, role, position, RSSI, slot, flags, congestion, intent seq, heard-slot bitmap, ...). |
| `link_loss()` | Its estimate of channel loss from gaps in neighbours' beacon counters. |
| `Config { beacon_interval_ms: 1000, neighbor_timeout_ms: 3500 }` | Defaults. Parked drones beacon slower, the sink faster during missions, carriers faster on lossy channels. |

**Inputs, `Sensors`:**

| Field | Meaning |
|---|---|
| `now_ms` | Clock. |
| `pos` | Own position (GNSS). |
| `ground_agl` | Rangefinder height above ground. |
| `fire: FireReading` | Downward thermal sensor: burning cell count in a 60-unit disc and their centroid. |
| `tx_dropped` | Local sends refused since the last tick: radio TX queue full, plus painlessMesh refusals and buffer drops at this node. |
| `telemetry_requested_ms` | Operator's telemetry interval (0 = telemetry off). |

**Outputs, `Actuation`:** `target` (where to fly; the flight controller does the flying) and `telemetry_interval_ms` (the interval the brain chose for the host's telemetry task, 0 = none).

### 3.2 Wire format (`wire.rs`)

Frame: `[kind u8][src u16][dst u16][payload..]`, little-endian, at most `MAX_FRAME` = 32 bytes (well under ESP-NOW's 250). Only two kinds go on air:

| Kind | Message | Content |
|---|---|---|
| `KIND_HEARTBEAT` (0) | `Payload::Heartbeat(Beacon)`, about 31 B, once per beacon interval | hops, parent, role (+ 2-bit congestion level up), position as `i16`, sink id + freshness `sink_seq`, beacon counter, intent seq, claimed slot, vacancy notice (`need`, `need_hops`), flags, interval in 50 ms units, 32-bit heard-slot bitmap |
| `KIND_INTENT` (1) | `Payload::Intent` (operator → base) or `Payload::Mission` (base flood: intent + seq + target + the sink's origin) | told apart by length |

Flag bits: `FLAG_SETTLED`, `FLAG_LANDED`, `FLAG_FILLING`, `FLAG_CARRYING`, `FLAG_ORPHAN`, a 2-bit congestion *verdict* (bits 5–6) and `FLAG_CUT_EPOCH` (bit 7). `Message::encode` / `Message::decode` and `Frame::{from_bytes, as_bytes, kind, src, dst}` are the codec. Special ids: `BROADCAST` 0xFFFF, `OPERATOR` 0xFFFE.

### 3.3 Behaviours

| Module | Behaviour |
|---|---|
| `plan.rs`: formation | From two points (sink, fire) every drone derives the same template: a **two-rail relay ladder** with rows about 160 apart (range 260), so every relay hears both relays of the neighbouring rows and any single loss leaves a working route; **scouts** in sectors around the fire; **parking spots** for reserves (hot spares), filled middle-first. |
| `mission.rs`: slots | Assignment without messages: each drone ranks the unassigned drones it hears and takes the slot at its rank. Duplicate claims resolve deterministically (in position beats en route, clearly closer beats farther, then lower id); the loser never returns, so roles don't thrash. Vacancies are noticed from expected neighbours going quiet; the **vouching bitmap** separates a dead holder from a missed heartbeat. Reserves fill the gap. |
| `routing.rs`: routes | Loop-free distance-vector gradient (Babel/DSDV feasibility condition: switch only to a parent with newer sink freshness, or the same freshness and strictly fewer hops). Poisoning (`NO_ROUTE`) makes children drop a broken route at once. Hold-down for sideways moves, a hop cap. **Acting-sink election** when the base dies: a witness that last routed to it takes over. **Orphan flag** for drones cut off. |
| `airtime.rs`: telemetry rate (Phase 5) | See [5.5](#55-brain-managed-rate-phase-5). |
| `brain.rs`: radio discipline | Adaptive missed-beacon tolerance from measured loss; intent anti-entropy (a neighbour behind on `intent_seq` gets the mission re-sent); `protect_control` (carriers beacon twice as often on a lossy channel); `routes_via_suspect` (never adopt a route through a relay already suspected dead). |

---

## 4. The world (`swarm-sim`)

### 4.1 Configuration: `SimConfig`

| Field | Default | Meaning |
|---|---|---|
| `seed` | 1 | Terrain, loss and fire randomness. |
| `drones` | 20 | Including the base station (id `BASE_ID` = 0). |
| `extent` | 1200 | Map width (square). |
| `terrain_size` | 129 | Height samples per side. |
| `radio` | `RadioConfig { range: 260, loss: 0 }` | Radio model. |
| `brain` | `Config::default()` | Brain timing. |
| `cruise_agl` | 25 | Cruise height above ground. |
| `max_speed` | 12 | Units per second. |
| `fire_size` | 96 | Fire grid cells per side. |
| `ignition` | per preset | Where the fire starts. |
| `sensor_radius` | 60 | Thermal sensor footprint. |
| `wind` | per preset (Alpine: `(-3, 0)`) | Steers only the fire. |
| `preset` | `AlpineRidge` | Map. |
| `channel` | off | Shared-airtime model ([5.3](#53-shared-24-ghz-channel-channelrs)). |
| `telemetry` | off | Real telemetry ([5.1](#51-telemetry-telemetryrs)). |

`SimConfig::for_preset(preset, seed, drones)` gives the standard setup for a map.

### 4.2 `Sim` methods

| Method | What it does |
|---|---|
| `Sim::new(cfg)` | Build the world: terrain, base, drones landed at the base, fire. |
| `step(dt_ms)` | Advance one step (the pipeline in section 1). |
| `kill(id) -> bool` | Switch a live drone off (it falls and stays down as a wreck). |
| `command(intent, x, z) -> bool` | Hand an operator intent to the base (wired, not radioed). Non-finite points are rejected, off-map ones clamped. |
| `set_loss(p)` | Global random delivery loss, clamped to 0..1. |
| `set_wind(x, z)` | Wind for the fire. |
| `ignite(x, z)` | Spot fire; invalid or off-map points are ignored. |
| `set_channel_enabled(on)`, `set_external_busy(f)`, `set_channel(cfg)` | Channel model on/off, external airtime (0–0.8), full config. |
| `set_telemetry(cfg)` | Telemetry config ([5.1](#51-telemetry-telemetryrs)). |
| `telemetry_stats()`, `channel_summary()`, `tlm_sample(id)` | Stats buffers and the last package text ([8.4](#84-telemetry-stats-and-channel-summary)). |
| `channel()`, `painless()`, `telemetry()` | Read-only access to the three models. |
| `terrain()`, `fire()`, `now_ms()`, `stats()`, `config()`, `preset()` | World state. `stats()` is `StepStats`: frames, bytes and drops of the last step. |
| `drone_count()`, `brain(id)`, `is_alive(id)`, `position(id)`, `in_contact(a, b)` | Per-drone ground truth. |
| `snapshot()`, `chatter()` | The flat render and console buffers ([8](#8-data-layouts)). |
| `base_position(cfg, terrain)` | Where the base stands on a map. |

### 4.3 Maps: `Preset`

The numeric values are part of the web contract and are never renumbered.

| Value | Preset | URL hash | Radio problem |
|---|---|---|---|
| 0 | `AlpineRidge` | `#map=alpine` | One north-south ridge between base and fire: needs a relay chain over it. |
| 1 | `Canyon` | `#map=canyon` | Base in a winding gorge, fire on the far rim: walls hide the floor, so relays climb out. |
| 2 | `TwinValleys` | `#map=twin-valleys` | Two parallel ridges: a longer chain over two crests. |
| 3 | `Archipelago` | `#map=archipelago` | Fire on an island: open sea, hops limited only by range. |
| 4 | `Foothills` | `#map=foothills` | Gentle hills, fire close by: the easy baseline. |

Each preset supplies `name()`, `ignition(extent)`, `wind()`, `base_xz(extent)` and `water_level()`. `Terrain::for_preset` builds the heights. Preset tests check that every map is deterministic, poses its advertised problem and is *fair*: a flyable relay chain with line of sight on every hop exists, with spares left over.

### 4.4 Terrain, radio and fire

| Function | Model |
|---|---|
| `Terrain::height_at(x, z)` / `surface_at` | Bilinear height (surface = water level over sea). |
| `Terrain::line_of_sight(a, b)` | Ray march over the height field; ridges and canyon walls block radio. |
| `Terrain::is_water`, `water_level` | Sea maps. |
| `RadioConfig::receive(terrain, from, to) -> Option<i8>` | Heard only within range **and** with line of sight; returns RSSI. |
| `RadioConfig::rssi_dbm(d)` | Log-distance path loss, scaled so the edge of range reads -95 dBm. |
| `FireGrid::step(dt, terrain, rng)` | Cells go unburnt → burning → burnt; spread is pushed by wind and runs uphill faster; water and ground above `SNOW_LINE` (200) do not burn. |
| `FireGrid::read(pos, radius) -> FireReading` | The thermal sensor: burning cells in the disc under the drone and their centroid. |
| `FireGrid::ignite`, `set_wind`, `cells`, `burning_count` | Control and state. |

### 4.5 Ground-truth metrics (`metrics.rs`, tests only)

`route_ok(sim, id)` (the drone's believed route really reaches a live sink), `with_role(sim, role)`, `scouts_connected(sim)` (every scout has a working route), `routed_count(sim)`.

### 4.6 Chatter (`chatter.rs`)

One record per frame put on air in the latest step, decoded from the wire bytes with `Message::decode`, exactly what a sniffer would see. Operator intents are logged in the step the base reads them. Telemetry adds TLM, painlessMesh sync, time sync and uplink records. Layout in [8.3](#83-chatter-record).

---

## 5. Real telemetry, painlessMesh and shared airtime

This models what your real system sends, based on `swarm_control` (`drone_mavlink_wifi.ino`, UDP build), painlessMesh 1.5.7 and your field measurements. It is all off by default.

### 5.1 Telemetry (`telemetry.rs`)

`TelemetryConfig { enabled, variant, rate, routing, requested_hz, chatter_every_frame }`. Default: off, XIAO S3, Fixed(25), Single, 25 Hz.

| Setting | Values |
|---|---|
| `Variant` | `XiaoS3` (no GNSS keys), `C6Rtk` (adds the UM982 `g*` keys), `Mixed` (odd ids are C6). |
| `RateMode` | `Fixed(hz)`; `BackendBudget`: Hz = clamp(`budget_pps` / online, 5, 25), with AIMD on fleet-median loss: ×0.8 every 10 s while loss > 10 %, +5 pps per 30 s after it stays < 5 %, floor 20 pps; `Brain`: each brain's `telemetry_interval_ms`. |
| `Routing` | `Single` (`sendSingle` to the root, up the tree) or `Broadcast` (`sendBroadcast`, flooded over every link). |

**What a drone sends:**

- A **FAST** frame every interval (25 Hz = 40 ms) on a phase-locked schedule. Every `500 + id % 97` ms the **SLOW** status keys ride along (about 280 B more).
- Values come from the sim's truth:
  - x/z become lat/lon around 47.4979, 19.0402;
  - altitudes: `amsl` with a 105 m datum, `a` above the take-off point;
  - speed, heading and NED velocity from motion;
  - armed while airborne; PX4 modes from what the drone is doing (LOITER, MISSION, LAND);
  - current and battery from a 4S hover-plus-speed draw;
  - `wifiRssi`/`ap` from its mesh parent link; `tf` = its packages the mesh refused.
  - Fields without a model (`cyc`, `dl`/`rb`, RTCM and GNSS link counters) are plausible constants or clocks.
- The frame becomes the escaped `msg` of `{"type":9,"dest":…,"from":…,"msg":"…"}` plus `\0`.
- It crosses the tree hop by hop as TCP. At the root it becomes one UDP datagram `drone/<host>/data\n<json>` to the WiFi station, not retransmitted.
- The **ground station** sees only what arrives: `s` gaps are loss and `t` gives latency.

Helper functions:

- `build_frame(out, state, variant, slow)` writes the firmware JSON.
- `BackendBudget::update(now, median_loss)` and `BackendBudget::hz(online, cap)` implement the backend rule.
- `Telemetry::step(...)` runs the schedule.
- `Telemetry::build_stats(...)` fills the stats buffers.
- `Telemetry::take_refused(i)` feeds `Sensors::tx_dropped`.
- `sample(id)` returns the last package text.

Constants: `MIN_HZ` 5, `MAX_HZ` 25, `HZ_LIMIT` 50, `BUDGET_PPS` 60, `BUDGET_FLOOR_PPS` 20, `SLOW_INTERVAL_MS` 500, `STATION_ID` 0xFFFD.

### 5.2 painlessMesh 1.5.7 (`painless.rs`)

The tree your telemetry rides on. It is separate from the brain's routing.

**Packages:**

- `SINGLE` (9) and `BROADCAST` (8);
- `NODE_SYNC_REQUEST` (5) and `REPLY` (6), carrying the full subtree;
- `TIME_SYNC` (4) and `TIME_DELAY` (3);
- all in ArduinoJson key order, over TCP port 5555, each `\0`-terminated.

**Tree:**

- A node without a parent scans, then associates to the strongest-RSSI node with a free SoftAP slot. A node cannot join below its own subtree, so the tree never forms a loop.
- The root is the bridge (base station, `setRoot` + `stationManual`).
- A lost parent is detected after `NODE_TIMEOUT_MS` = 10 s, then the node rescans.
- NodeSync runs every 60 s. TimeSync runs every 10 s until synced, then every 60 s.
- **No root election:** if the root dies, nothing reaches the ground station.

**Transport:**

- Each tree edge is a TCP connection with send buffers that hold a bounded number of packages.
- A full buffer refuses new telemetry (`MeshEvent::Dropped`).
- A lost segment stalls the stream until lwIP's retransmission timeout, which doubles on each retry.
- Backlogged packages are coalesced into segments of at most `TCP_MSS` (1436 B).
- Closing a connection drops both buffers.

**API:**

- `PainlessMesh::new(nodes, root)` and `step(now, dt, alive, rssi, out)`.
- `parent(id)`, `hops(id)`, `connected(id)`, `path_to_root(id, out)`.
- `node_id(id)` = 2 200 000 000 + id × 7919.
- `encode_package`, `encode_time_sync`, `escape_into`, `escaped_len`.

### 5.3 Shared 2.4 GHz channel (`channel.rs`)

Everything on air costs airtime, and every frame is received (or not) from its own signal, noise and interference:

- brain frames as ESP-NOW at 1 Mbps (broadcasts once; addressed frames are ESP-NOW unicasts with MAC retries, channel on only);
- mesh TCP segments at the link's adapted rate, each with its MAC retries and 802.11 ACK, and the TCP ACKs back (one per two segments, a lone segment ACKed on the receiver's next 250 ms tick, both directions);
- mesh NodeSync (both ends of each connection, every 60 s, the AP side 10 s later) and TimeSync;
- every SoftAP's beacons: 190 B every 102.4 ms at 1 Mbps, about 1.6 % of airtime per node;
- the root's uplink, at the router's rate (12 Mbps minimum, adapted from the root ↔ router SNR), crossing the air twice when the ground-station laptop is a WiFi client of the same router (`gcs_on_wifi`, default on).

**Airtime** (802.11-2016, ESP-IDF):

| Item | Value |
|---|---|
| Slot / SIFS | 9 µs (all-ESP32 BSS, short slot even for DSSS) / 10 µs |
| AIFS / CW, best-effort data (HT QoS) | 37 µs (SIFS + 3 slots) / 15…1023, doubling per retry |
| AIFS / CW, ESP-NOW action frames (AC_VO) | 28 µs / 3…7 |
| Beacons | DIFS 28 µs, no backoff |
| PLCP | DSSS 192 µs at 1 Mbps, short 96 µs at 2/5.5/11; ERP-OFDM 20 µs + 4 µs symbols + 6 µs extension; HT-mixed 36 µs + 4 µs symbols + 6 µs |
| ACK | 14 B at the highest basic rate ≤ the data rate (DSSS: the rate; OFDM/HT: 6/12/24 Mbps) |
| MAC header + FCS | QoS data 26 + 4 B; management / action 24 + 4 B; LLC/SNAP 8, IPv4+TCP 40, IPv4+UDP 28, ESP-NOW 15 |
| Retries | at most 7 attempts (dot11ShortRetryLimit); each attempt drawn and charged as it happens; the backoff beyond the first attempt's is queue time, not busy medium |

**Reception (SINR)** for a frame `s → r`:

| Term | Model | Source |
|---|---|---|
| Mean signal | radio.rs log-distance (−95 dBm at range on the brains' RSSI scale) + `LINK_BUDGET_DB` 10 → −85 dBm at the edge of range | 12 dB fade margin over the 1 Mbps sensitivity at the planned range; radio.rs itself is unchanged (brain-visible) |
| Shadowing | static per pair, log-normal σ 5 dB, symmetric, seeded from the channel seed | UAV LOS links 2–6 dB |
| Fading | Rician, K 6 dB, per attempt | UAV links K ≈ 2–15 dB |
| Noise floor | −174 dBm/Hz + 73 dB (20 MHz) + NF 5 dB = −96 dBm, + up to 3 dB × external busy | thermal + ESP32 NF (DSSS 1 Mbps then needs ≈ −1 dB SNR, matching −97 dBm sensitivity) |
| Rate thresholds | ESP32 datasheet sensitivity − noise: 1 −97, 2 −94, 5.5 −92, 11 −88, OFDM 6 −92 … 54 −74, HT MCS0 −92 … MCS7 −72 dBm (MCS1–6 interpolated) | ESP32 datasheet |
| PER | `1 − (1 − PER₁₀₀₀)^(L/1000)`, `PER₁₀₀₀ = 1/(1 + 9·e^(2·(SINR − γ)))` (10 % at threshold, 90→10 % in 2.2 dB) | 802.11 sensitivity criterion, AWGN waterfalls |
| Contention overlap | contenders as a group overlap with `1 − e^(−K_COLL·b)` at their busy share `b` (beacons excluded: TBTT, no backoff), split by airtime; the receiver itself transmitting = lost (half-duplex) | `K_COLL` = **1.2**, fitted on T5 |
| Hidden overlap | `1 − e^(−h·(1 + T/T_k))` per sender `r` hears and `s` cannot sense (h its airtime share, T our frame time, T_k its mean transmission time) | unslotted ALOHA vulnerable window `(−T_k, T)`; equal lengths give the old `1 − e^(−2h)` |
| External (slider) | carrier-sensed time share; overlaps with `1 − e^(−K_COLL·ext)`, power N(−85, 6) dB at the receiver (as strong as a node at the edge of range) | assumption |
| Capture | an overlapped frame survives if `S − I` ≥ 5 dB (DSSS) / 10 dB (OFDM, HT), then PER at the SINR | 4–6 / ≥10 dB |

**Rates (Minstrel-like).** Per link and geometry refresh (250 ms), the model's mean success probability at every rate (fading and current interference included) gives the best-throughput rate (first two attempts) and, for the remaining attempts (multi-rate retry), the rate that delivers in the least expected channel time while keeping the loss after all 7 attempts ≤ 10⁻³ (`MRR_RESIDUAL_LOSS`; a lost TCP segment costs a 1–3 s RTO). An SNR-limited link falls back to a robust rate; a collision-limited one does not drop to 1 Mbps, where a 1.4 kB segment is 40× longer on air and more exposed to hidden senders. Mesh links use DSSS 1 … 11 and HT MCS0 … 7; the uplink HT ≥ 12 Mbps. A weak or interfered link falls back and costs more air. Brain ESP-NOW stays at 1 Mbps.

**TX queue.** 8 frames per sender (`tx_queue_frames`); a new frame is refused with the M/M/1/K blocking probability of its carrier-sense domain; refused frames never reach the air and are counted for `Sensors::tx_dropped`.

API: `admit(src)`, `broadcast(now, &AirFrame)` + `lost_broadcast(rx)`, `unicast(now, &AirFrame, Dst, reachable) -> TxResult`, `beacons(now, dt, nodes)`, `link_rates(tx, rx)`, `station_rates(node)`, `link_loss(src, rx, unicast)`, `unicast_failure(..)`, `busy(node)`, `airtime_frac(node)`, `take_tx_dropped(node)`, `queue_drops_per_s(node)`, `kind_frac(kind)`, `pps()`, `ledger()`, `busy_mean_max(nodes)`. The byte ledger (`airlog.rs`) books every frame's MPDU bytes to the node that sends it, whether or not the airtime model is on.

**Calibration** against `UDP_VS_MQTT.md` (one AP, ~400 B frames crossing the air twice, 2 Mbps DSSS before radio hygiene):

| Load | Field | Sim (T5) |
|---|---|---|
| ~70 pps | ~1 % loss | 0.1 % (AP busy 0.22) |
| ~150 pps (6 × 25 Hz) | 26–32 % loss | 24.6 % (AP busy 0.51) |
| ~75 pps, a different session | 12–18 % | not reproduced: 0.2 % at external 0, 0.5–1.3 % at external 0.2–0.3 |
| 150 pps at 12 Mbps | (target after hygiene) | 0.0 % |

`K_COLL` = 1.2 is the one fitted constant (it was 0.8 for the old all-or-nothing collisions; capture now saves some overlaps, and the AIFS/preamble/header changes moved the cell in both directions). Everything else is an 802.11, ESP-IDF or datasheet figure, or one of the assumptions marked above.

### 5.4 Everything on the same channel

The busy fraction a node sees comes from its own frames, its neighbours' frames (brain + mesh + ACKs + beacons + uplink) and external traffic. When it rises:

- brain heartbeats collide or are refused, so routing and repair slow down;
- TCP segments retry, then stall, then buffers fill and refuse telemetry;
- deep drones suffer most, because their packages cross every hop on the way.

### 5.5 Brain-managed rate (Phase 5)

Each brain picks its own telemetry interval from local evidence only:

- **its own refused sends** (`tx_dropped`, which includes painlessMesh refusals at relays);
- **its loss estimate**, from gaps in neighbours' beacon counters;
- **congestion levels carried in spare heartbeat bits**, with no extra bytes or frames. A 2-bit level is reported *up* toward the sink (the maximum seen beyond each drone). A 2-bit verdict is passed *down*, set by the sink and copied from each drone's parent.

The controller is AIMD with hysteresis, evaluated every 500 ms:

- **Cuts:** at level 2 or above the sink cuts the rate above the floor by ×0.6 (squared at level 3), then waits 6 s before cutting again.
- **Fair cuts:** each cut flips `FLAG_CUT_EPOCH`. Every drone cuts exactly once per epoch change, so near and far drones cut the same number of times.
- **Raises:** after 4 s at level 0 a drone adds 1 Hz every 2 s, and probes 4 times slower near the rate of the last cut.
- **Level 1:** holds the rate for at most 30 s, then the slow climb resumes.
- **Bounds:** the controller starts at 5 Hz, never exceeds the operator's rate and never goes below `MIN_HZ` = 5 (unless the operator asked for less).
- **Telemetry off:** none of this runs and heartbeats are bit-identical.

---

## 6. WASM exports (`swarm-wasm`)

Plain `extern "C"` functions; no wasm-bindgen. Every function tolerates a null `sim` (it does nothing and returns 0/null), and every argument is sanitised, so no input can trap. Pointers stay valid until the next call that mutates the sim.

### Lifecycle

| Export | Signature | Behaviour |
|---|---|---|
| `sim_new` | `(seed: u32, drones: u32) -> *mut Sim` | Alpine world; `drones` clamped to 1..=256. |
| `sim_new_preset` | `(preset: u32, seed: u32, drones: u32) -> *mut Sim` | World on map `preset`; null if unknown. `sim_new_preset(0, s, n)` = `sim_new(s, n)`. |
| `sim_free` | `(sim)` | Destroy. |
| `sim_preset` | `(sim) -> u32` | The map it was built from. |
| `sim_step` | `(sim, dt_ms: u32)` | Advance; `dt` capped at 1000 ms (a backgrounded tab can't teleport the world). |

### Control

| Export | Signature | Behaviour |
|---|---|---|
| `sim_kill` | `(sim, id: u32) -> u32` | 1 if a live drone was switched off. |
| `sim_command` | `(sim, intent: u32, x: f32, z: f32) -> u32` | Intent 0 Hold, 1 Monitor fire, 2 Map perimeter, 3 Return to base. 1 if it reached the base. |
| `sim_set_loss` | `(sim, loss: f32)` | Random delivery loss 0..1 (NaN = 0). |
| `sim_set_wind` | `(sim, x: f32, z: f32)` | Wind; non-finite = calm. |
| `sim_ignite` | `(sim, x: f32, z: f32)` | Spot fire; invalid points ignored. |
| `sim_set_channel` | `(sim, enabled: u32, external_busy: f32)` | Airtime model on/off; external clamped 0..=0.8. |
| `sim_set_telemetry` | `(sim, enabled, variant, rate_mode, hz: f32, routing) -> u32` | Variant 0 S3 / 1 C6 / 2 mixed; mode 0 fixed / 1 budget / 2 brain; `hz` 0.1..=50 (non-finite = 25); routing 0 single / 1 broadcast. Returns 0 and changes nothing if a code is unknown. |

### Read-out

| Export | Returns |
|---|---|
| `sim_snapshot_ptr` / `sim_snapshot_len` | Render snapshot, `f32` ([8.1](#81-snapshot)). |
| `sim_terrain_ptr` / `sim_terrain_size` / `sim_terrain_extent` | Height field (`size²` floats) and map width. |
| `sim_water_level` | Sea height, NaN for dry maps. |
| `sim_fire_ptr` / `sim_fire_size` | Fire cells, `size²` bytes: 0 unburnt, 1 burning, 2 burnt. |
| `sim_chatter_ptr` / `sim_chatter_len` | Decoded air traffic, 12 floats per record ([8.3](#83-chatter-record)); length in floats. |
| `sim_telemetry_ptr` / `sim_telemetry_len` | Per-drone telemetry stats, 20 floats per drone ([8.4](#84-telemetry-stats-and-channel-summary)). |
| `sim_channel_ptr` / `sim_channel_len` | Fleet channel summary, 16 floats. |
| `sim_tlm_sample_ptr` / `sim_tlm_sample_len` | UTF-8 text of the last painlessMesh package drone `id` sent. |

---

## 7. Web worker protocol

`web/src/protocol.ts`. The worker owns the WASM instance and steps it every 50 ms.

**Main → worker**

| Message | Effect |
|---|---|
| `{type:'init', seed, drones, preset}` | New world (also used by Reset and the map picker). |
| `{type:'kill', id}` | Kill a drone. |
| `{type:'command', intent, x, z}` | Operator intent. |
| `{type:'loss', value}` | Random packet loss. |
| `{type:'ignite', x, z}` | Spot fire. |
| `{type:'speed', value}` | 1–8 sim steps per tick (fast-forward). Counters sum the tick's steps. |
| `{type:'channel', enabled, external}` | Airtime model. |
| `{type:'telemetry', enabled, variant, rateMode, hz, routing}` | Telemetry config. |
| `{type:'tlmsample-sub', on}` | Subscribe to package-text samples. |

**Worker → main**

| Message | Content |
|---|---|
| `terrain` | Heights, size, extent, preset (once per world). |
| `snapshot` | Snapshot buffer + step time. |
| `fire` | Fire cells. |
| `chatter` | Batched chatter records (at least every 100 ms). |
| `tlmstats` | Telemetry stats + channel summary. |
| `tlmsample` | `{id, seq, text}`: one package text per drone per second. |
| `error` | Worker failure. |

---

## 8. Data layouts

### 8.1 Snapshot

Version 3 (`snapshot::VERSION`):

```
header  [version, tick, time_s, drones, links, pulses, frames, bytes,
         dropped, burning_cells, base_intent, reserved]
drone   [id, x, y, z, alive, role, hops, parent]  × drones   (parent -1 = none)
link    [a, b, kind, quality]                      × links    (kind 1 = route to parent)
pulse   [src, dst, kind]                           × pulses   (dst -1 = broadcast)
```

Links come from what the drones *believe* (their neighbour tables), not from the radio model, so a link to a dead drone lingers until its neighbours time it out. `quality` (0..1, new in v3) is what the radio really delivers on it: the share of brain frames each end heard of those the other end sent it (it alive, the frame broadcast or addressed to it), over the last 3–4 s, the worse direction (`linkq.rs`). It is exactly 1 with no loss; a link that is physically gone (out of range, behind terrain, a dead node: each dead node is charged one unheard frame per second) decays to 0 while the tables still list it. With the channel model off it reflects range, line of sight and `sim_set_loss`; with it on, every frame's SINR. The renderer fades lines by it.

### 8.2 Fire

`fire_size²` bytes, row-major, row along +z and column along +x: 0 unburnt, 1 burning, 2 burnt.

### 8.3 Chatter record

12 floats (`CHATTER_STRIDE`): `[t_ms, src, dst (-1 broadcast), kind, f0..f7]`. `src` 65534 is the operator console.

| Kind | Value | Shown as | `f0..f7` |
|---|---|---|---|
| `CHATTER_HEARTBEAT` | 0 | HB | hops, parent, role, sink, slot, flags, need, intent_seq |
| `CHATTER_INTENT` | 1 | INTENT | operator intent to the base |
| `CHATTER_MISSION` | 2 | MISSION | the base's flood |
| `CHATTER_TLM` | 3 | TLM | telemetry package (sampled; the text comes from `tlmsample`) |
| `CHATTER_PM_SYNC` | 4 | PM-SYNC | NodeSync request / reply |
| `CHATTER_PM_TIME` | 5 | PM-TIME | TimeSync / TimeDelay |
| `CHATTER_ROOT_UPLINK` | 6 | UPLINK | root → WiFi station datagram |
| `CHATTER_UNDECODABLE` | 255 | | bytes that did not decode |

### 8.4 Telemetry stats and channel summary

**Per drone**, `TLM_STRIDE` = 20 floats, in id order (offsets in `telemetry::tlm`). Rates are over a 2 s window unless noted. Bytes are MPDU bytes (MAC header + FCS + LLC/SNAP + IP/TCP/UDP + payload) of every transmission, MAC retries and TCP retransmissions included.

| Off | Field | Meaning |
|---|---|---|
| 0 | `id` | |
| 1 | `requested_hz` | Rate it sends at now (fixed / budget share / brain choice). |
| 2 | `delivered_hz` | Its frames reaching the ground station, by `t`, read 0.75 s late. |
| 3 | `loss_pct` | From `s` gaps at the ground station, **10 s window**. |
| 4 | `latency_ms` | Mean `t` → ground-station latency. |
| 5 | `tx_bytes_per_s` | Its own packages' share of the mesh segments it sends. |
| 6 | `relay_bytes_per_s` | Others' packages' share of the segments it relays (root: its uplink datagrams, both crossings). |
| 7 | `airtime_frac` | Share of channel time it transmits. |
| 8 | `busy_frac` | Channel busy as it hears it, external included. |
| 9 | `queue_drops_per_s` | Refused by its TX queue + full mesh buffers. |
| 10 | `tree_parent` | painlessMesh parent (-1 none). |
| 11 | `tree_hops` | Hops to root (-1 not connected). |
| 12 | `last_frame_bytes` | Last telemetry JSON payload. |
| 13 | `last_package_bytes` | Last package text incl. `\0`. |
| 14 | `brain_bytes_per_s` | Brain frames (ESP-NOW MPDUs). |
| 15 | `variant` | 0 S3, 1 C6. |
| 16 | `operator_hz` | Rate the operator asked for. |
| 17 | `tx_all_bytes_per_s` | Everything it transmits: its packages, relayed packages, NodeSync/TimeSync, TCP ACKs and 802.11 ACKs it sends, beacons, retries, brain frames, uplink. |
| 18 | `ack_bytes_per_s` | ACKs it sends: 802.11 ACKs (14 B) + TCP ACKs (78 B MPDU). |
| 19 | `parent_rate_mbps` | PHY rate of its link to its painlessMesh parent (0: none, or channel off). |

**Fleet summary**, `CHANNEL_LEN` = 28 floats (offsets in `telemetry::chan`):

| Off | Field | Meaning |
|---|---|---|
| 0 | `pps_on_air` | Frames on air per second except beacons (data, ACKs, retries, brain, uplink). |
| 1–2 | `busy_frac_mean`, `busy_frac_max` | Over live nodes. |
| 3 | `external_busy` | The slider. |
| 4–5 | `gcs_pps`, `gcs_bytes_per_s` | Datagrams reaching the ground station, and their UDP payload bytes. |
| 6–11 | `budget_pps`, `mode`, `channel_on`, `telemetry_on`, `median_loss_pct`, `bridge_up` | |
| 12–14 | `air_brain`, `air_mesh`, `air_uplink` | Channel-time fractions. |
| 15 | `online` | Drones heard in the last 10 s. |
| 16 | `bytes_on_air` | All bytes on air / s (= 17 + 18 + 19 + 20 + 21 + 22). |
| 17 | `bytes_brain` | ESP-NOW brain frames. |
| 18 | `bytes_tlm_data` | Mesh segments carrying telemetry, every hop. |
| 19 | `bytes_ack` | 802.11 ACKs + TCP ACKs. |
| 20 | `bytes_mesh_control` | NodeSync + TimeSync. |
| 21 | `bytes_uplink` | Root → station datagrams. |
| 22 | `bytes_beacon` | SoftAP beacons. |
| 23 | `bytes_retry` | The part of 16 that is retransmissions (MAC + TCP). |
| 24 | `frames_on_air` | All frames / s, beacons included. |
| 25 | `link_quality_mean` | Mean `quality` over the snapshot's links. |
| 26 | `links_degraded` | Snapshot links with `quality` < 0.5. |
| 27 | `bytes_mesh_control_60s` | 20 averaged over 60 s: NodeSync / TimeSync run once a minute per connection, so the 2 s figure is usually 0 (show this one; 16 still sums the 2 s figures). |

Every byte is booked to the node that sends it, so the per-drone `tx_all_bytes_per_s` sum to `bytes_on_air` (test T12). The WiFi router and the ground station are not drones: their crossing of the uplink and their 802.11 ACKs are booked to the root (the base site).

The UI labels the two windows ("@GCS 2s", "LOSS 10s"), because a stall that ended a few seconds ago still shows in loss while the delivered rate has already recovered.

---

## 9. Showcase demos

`web/src/ui/demos.ts`. Each demo is a timed script of actions, captions, camera moves and waits on measured conditions. <kbd>Esc</kbd> cancels.

| Demo | What happens | Shows |
|---|---|---|
| **Relay down** | Deploy, then kill the busiest relay. | Heal time ("Healed in …"). |
| **Base destroyed** | Kill the base station. | Acting-sink election ("New sink in …"). |
| **Radio storm** | 45 % of every delivery lost, then cleared. | Adaptive tolerance; the formation holds. |
| **Fire jumps** | Spot fires light ahead of the front, then a new Monitor command. | Re-planning and re-deployment time. |
| **Mass casualty** | Half the swarm killed at once (the base is spared). | Recovery, or partial recovery. |
| **Channel saturation** | 1. Backend budget (5 Hz each). 2. Everyone forced to 25 Hz until the channel collapses. 3. Kill the drone that forwards the most telemetry: brain heal time vs painlessMesh reroute. 4. Brain-managed rate with 30 % external airtime. | Real Hz at the GCS, loss and busy, measured only after each 10 s loss window settles. |
| **Full tour** | Several of the above in sequence. | |

---

## 10. Test suites

Run from the repo root (`source "$HOME/.cargo/env"` first):

```bash
cargo test --workspace --release                       # everything
cargo test --release -p swarm-sim --test scenarios -- --nocapture
cargo test --release -p swarm-sim --test telemetry -- --nocapture
cargo brain-mcu                                        # ESP32-C3 build
```

**Brain unit tests** (56): wire codec, routing feasibility, repair, slot claims, airtime controller.

**Scenario suite** (`tests/scenarios.rs`, 59 tests across all 5 maps):

| Test | Checks |
|---|---|
| S1 deploy to fire | Formation reaches the fire and every scout is routed. |
| S2 single relay kill (repeated) | Heal within the limit, every time. |
| S3 multiple simultaneous kills | Recovery from several losses at once. |
| S4 partition then merge | Split swarm reunites. |
| S5 base offline | Acting sink elected. |
| S6 new intent mid-repair | A fresh command during a repair converges. |
| S7 packet loss | Works under heavy random loss. |
| S8 determinism | Same seed gives the same world (digest). |
| S9 idle efficiency | Low traffic when nothing happens. |
| S10 return to base | Everyone comes home and lands. |
| map sanity, preset matrix, chatter checks | Maps are valid; chatter is a faithful decode of the air. |

**Telemetry suite** (`tests/telemetry.rs`, 20 tests):

| Test | Checks |
|---|---|
| T1 telemetry off parity | With telemetry off the world is unchanged (digest). |
| T2 byte budget, one drone | Exact frame / package / on-air bytes at 25 Hz for both variants. |
| T3 escaping overhead; broadcast cost | Escaping overhead; Broadcast ≥ (N−1) × Single airtime. |
| T4 backend budget rule | clamp(60/N, 5, 25) and AIMD cuts. |
| T5 field calibration; external busy | The table in 5.3; external airtime costs delivery. |
| T6 fixed 25 Hz saturates Alpine | 20 × 25 Hz collapses deep drones. |
| T7 backend budget keeps Alpine healthy | All routed at 5 Hz, 0 % loss. |
| T8 relay kill under telemetry (5 maps); tree is physical | Brain heal vs mesh reroute; the tree follows radio reach. |
| T9 brain managed (Alpine, Canyon) | Phase 5 at 25 Hz requested + 30 % external: every routed drone ≥ 4.75 Hz at the GCS, then a relay kill heals ≤ 5 s. |
| T10 root death, no bridge | Nothing reaches the GCS; the brains still elect a sink. |
| T11 step time, everything on | Under 2 ms per step at 20 drones. |

---

## 11. Measured numbers

From the last full run (20 drones, Alpine unless stated):

| Measurement | Value |
|---|---|
| S3 fast frame / package | 176 B / 268 B (C6: 245 B / 347 B; `n` is the 10-digit node id); with slow keys ≈ 620 B |
| Lone drone at 25 Hz | 25.0 Hz at GCS; its own packages on air 9.3 kB/s (S3), 12.8 kB/s (C6), MPDUs with retries |
| Broadcast vs Single (6 nodes) | 8.5 × the mesh airtime |
| Fixed 25 Hz × 20 (T6) | window after deployment: busy max 0.72, mean loss 38 %, 15.5 Hz mean at GCS; steady state (150–180 s, all drones): 12.1 Hz median / 12.5 mean, pooled loss 50 %, busy 0.48 / 0.65, link quality 0.67, 16 links < 0.5 |
| Backend budget (T7) | budget 43 pps, 0 % median loss, busy 37 % |
| Brain-managed, no external | Brains choose 10–12 Hz, 10 Hz at GCS, 0 % loss (not re-measured with the SINR model) |
| Brain-managed, 30 % external (T9) | every brain at the 5 Hz floor; fleet loss 34–36 %, deep drones 1.7–4.7 Hz; both Alpine and Canyon fail T9 |
| Relay kill: brain heal / mesh reroute | 1.3–4.9 s / 14.7–17.7 s (5 maps) |
| Root death | 0 packages reach the GCS; acting sink after 4.3 s |
| Step time | 0.25 ms at 20 drones, 4.4 ms at 100 (everything on) |
| WASM | 223 KB (86 KB gzipped) |
| Browser | 50 fps at FX HIGH (headless cap) |

---

## 12. Limits and known gaps

- **T9 fails on Alpine and Canyon.** With 30 % external airtime every brain is at the 5 Hz floor, yet many drones deliver 1.7–4.7 Hz. Before the SINR model the mesh hops were (wrongly) floored at the router's 12 Mbps and only Alpine's 5–6-hop drones failed; with per-link rates (DSSS 1 … HT MCS7), real retries and 7-attempt chains, the long and interfered hops near the root are the bottleneck. A channel limit, not a brain defect. Options: accept it, allow a floor below 5 Hz, or disable 802.11b on the mesh.
- **25 Hz × 20 never settles.** Tree links fail ~30–38 % of attempts (contention at the calibrated `K_COLL`), so about one segment a second exhausts its 7 attempts; lwIP then stalls that connection for a 1–1.5 s RTO (one segment in flight, no fast retransmit), and a hub's 50-package buffer overflows with its subtree's traffic (200–300 packages/s dropped). painlessMesh joins the strongest rooted AP, usually a near neighbour, so the tree is 5 hops deep where the brain's relay ladder has 2–3 rows: every package crosses more stalled hops. `collapse_timeline` (world.rs, ignored) logs it every 10 s.
- **Field row 3 not reproduced.** The field's ~75 pps → 12–18 % session (`UDP_VS_MQTT.md`) gives 0.2 % here, and 0.5–1.3 % with 20–30 % external airtime.
- **Link quality counts transmitted frames only.** Frames the TX queue refuses never air, so saturation shows as fewer heartbeats (and neighbour-table timeouts) more than as lower `quality`.
- **Simplifications.** The channel draws every attempt (fading, one dominant overlapping frame, capture, PER) but its overlap probabilities come from smoothed busy fractions, not per-slot DCF; the queue uses a blocking probability. All senders in a saturated domain are cut alike. Legacy-only stations, HT40 and MIMO are not modelled; mesh packages are not split across TCP segments. Drones hold position against wind. Firmware fields without a physical model are constants.
- **Console sampling.** At 25 Hz, when a drone sends two packages in one 50 ms step, only the later text is kept, so some TLM lines show only `s=<seq>`.
- **60 fps** is unverified: the headless test browser caps at 50.
- **Foothills seeds 3/12:** once the swarm leaves, the base has no radio contact.
