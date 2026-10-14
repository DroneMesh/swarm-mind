# Graph Report - swarm-mind  (2026-10-11)

## Corpus Check
- 75 files · ~144,283 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 29 file(s) not represented in the graph (top: (none) 13, .d 4, .timestamp 3)

## Summary
- 2225 nodes · 5202 edges · 96 communities (86 shown, 10 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 202 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Snapshot
- World
- Brain
- terrain.rs
- _
- FixedVec<T, N>
- Swarm Mind — Self-Healing Wildfire Drone Swarm
- package.json
- SwarmExports
- compilerOptions
- fire.rs
- snapshot.rs
- src/telemetry.rs
- swarm-brain
- painless.rs
- scenarios.rs
- LinkMeter
- tests.rs
- mod.rs
- main.ts
- TelemetryConfig
- _
- link-style.ts
- channel/tests.rs
- selectDrone
- Swarm Mind: simulation reference
- onWorkerMessage
- swarm.ts
- Sim
- .suspect
- web_src_render_terrain_buildterrain
- Brain
- Channel
- airtime.rs
- _
- layout.test.ts
- .transmit
- .new
- .new
- routing.rs
- Swarm Mind
- Telemetry
- tests/telemetry.rs
- telemetry.ts
- preset_tests.rs
- .new
- .new
- t9
- hostile_inputs_never_panic
- tlmField
- .describe
- RadioConsole
- t8
- PainlessMesh
- world.rs
- tree_is_physical
- 2. What you can simulate
- AirLedger
- .service
- Option
- sim.worker.ts
- TlmDrone
- scenario_label
- .updateStatus
- .mean_failure
- Traffic
- .run
- .new
- SmokePool
- Window<N>
- swarm-brain/src/lib.rs
- super
- RadioConfig
- base-station.ts
- 4. The world (`swarm-sim`)
- Preset
- fire.ts
- fx.ts
- NodeRadio
- .unicast
- BackendBudget
- Lifecycle
- vec3
- .new
- Vec3
- console.ts
- chatter.rs
- window.rs
- .new
- json_unescape
- ConsoleHost
- demos.ts
- three
- metrics.rs
- SwarmView
- channel.rs

## God Nodes (most connected - your core abstractions)
1. `Sim` - 117 edges
2. `Channel` - 67 edges
3. `Vec3` - 65 edges
4. `PainlessMesh` - 58 edges
5. `Brain` - 48 edges
6. `_` - 47 edges
7. `Snapshot` - 47 edges
8. `RadioConsole` - 41 edges
9. `_` - 34 edges
10. `_` - 33 edges

## Surprising Connections (you probably didn't know these)
- `5.1 Telemetry (`telemetry.rs`)` --references--> `s()`  [INFERRED]
  docs/SIMULATION.md → web/src/ui/demos.ts
- `5.3 Shared 2.4 GHz channel (`channel.rs`)` --references--> `s()`  [INFERRED]
  docs/SIMULATION.md → web/src/ui/demos.ts
- `Telemetry & airtime model` --references--> `s()`  [INFERRED]
  README.md → web/src/ui/demos.ts
- `Control` --references--> `hz()`  [INFERRED]
  docs/SIMULATION.md → web/src/ui/demos.ts
- `5.1 Telemetry (`telemetry.rs`)` --references--> `Variant`  [INFERRED]
  docs/SIMULATION.md → web/src/telemetry.ts

## Import Cycles
- 2-file cycle: `crates/swarm-brain/src/brain.rs -> crates/swarm-brain/src/mission.rs -> crates/swarm-brain/src/brain.rs`
- 2-file cycle: `crates/swarm-brain/src/airtime.rs -> crates/swarm-brain/src/brain.rs -> crates/swarm-brain/src/airtime.rs`

## Communities (96 total, 10 thin omitted)

### Community 0 - "Snapshot"
Cohesion: 0.09
Nodes (12): callsign(), actingSinkName(), fitViewport(), kill(), onSnapshot(), pickRelay(), syncQuality(), updateHud() (+4 more)

### Community 1 - "World"
Cohesion: 0.21
Nodes (7): nearest_burning_xz(), FnMut, Intent, Option, String, summary_line(), World

### Community 2 - "Brain"
Cohesion: 0.09
Nodes (19): _, Brain, Config, JITTER_DIV, LANDED_AGL, MIN_BEACON_GAP_MS, NO_ROUTE, PARKED_INTERVAL_MS (+11 more)

### Community 3 - "terrain.rs"
Cohesion: 0.11
Nodes (17): archipelago(), canyon(), canyon_centre(), CANYON_FLOOR, CANYON_RIM, foothills(), height_at_matches_grid_samples(), lerp() (+9 more)

### Community 4 - "_"
Cohesion: 0.06
Nodes (41): _, Beacon, BROADCAST, CONGESTION_UP_SHIFT, FLAG_CARRYING, FLAG_CONGESTION_MASK, FLAG_CONGESTION_SHIFT, FLAG_CUT_EPOCH (+33 more)

### Community 5 - "FixedVec<T, N>"
Cohesion: 0.15
Nodes (11): FixedVec<T, N>, push_rejects_when_full(), retain_keeps_matching(), Default, FnMut, Result, Self, Deref (+3 more)

### Community 6 - "Swarm Mind — Self-Healing Wildfire Drone Swarm"
Cohesion: 0.10
Nodes (20): Core Capabilities (MoSCoW), Decisions Log, Evidence, Implementation Phases, Key Hypothesis, MVP Scope, Open Questions, Parallelism Notes (+12 more)

### Community 7 - "package.json"
Cohesion: 0.09
Nodes (20): @types/three, typescript, vite, dependencies, three, devDependencies, @types/three, typescript (+12 more)

### Community 9 - "compilerOptions"
Cohesion: 0.13
Nodes (14): compilerOptions, isolatedModules, lib, module, moduleResolution, noEmit, noFallthroughCasesInSwitch, noUnusedLocals (+6 more)

### Community 10 - "fire.rs"
Cohesion: 0.08
Nodes (37): BASE_RATE, BURN_MS, BURNING, BURNT, calm_wind_ignores_non_finite_input(), caught_span(), caught_west_east(), cells_burn_out() (+29 more)

### Community 11 - "snapshot.rs"
Cohesion: 0.33
Nodes (5): DRONE_STRIDE, HEADER_LEN, LINK_STRIDE, PULSE_STRIDE, VERSION

### Community 12 - "src/telemetry.rs"
Cohesion: 0.03
Nodes (68): chatter, ACK_BYTES_PER_S, AIR_BRAIN, AIR_MESH, AIR_UPLINK, AIRBORNE_AGL, AIRTIME_FRAC, AMSL_DATUM_M (+60 more)

### Community 13 - "swarm-brain"
Cohesion: 1.00
Nodes (3): swarm-brain, swarm-sim, swarm-wasm

### Community 14 - "painless.rs"
Cohesion: 0.06
Nodes (35): AP_SYNC_LAG_MS, BROADCAST, CONNECT_MS, encode_package(), encode_time_sync(), escape_into(), FORM_GAP_MS, MAX_CONN (+27 more)

### Community 15 - "scenarios.rs"
Cohesion: 0.06
Nodes (41): BASE_OFFLINE_LIMIT_MS, BASE_OFFLINE_MAX_RATE, BASE_OFFLINE_WINDOW_MS, CONNECTED_HOLD_MS, DEPLOY_LIMIT_MS, DETERMINISM_MS, expect_restored(), IDLE_MAX_RATE (+33 more)

### Community 16 - "LinkMeter"
Cohesion: 0.21
Nodes (7): BUCKET_MS, BUCKETS, LinkMeter, ratio_window_and_dead_decay(), Fn, Self, Vec

### Community 17 - "tests.rs"
Cohesion: 0.11
Nodes (44): a_cut_off_drone_poisons_its_route_instead_of_counting_up(), a_drone_that_missed_the_flood_catches_up(), a_far_holder_waits_its_turn_before_moving_up(), a_filler_that_arrived_and_then_loses_to_the_holder_stays_away(), a_filler_that_loses_to_another_filler_stays_away(), a_group_that_elected_its_own_sink_heads_home_and_merges(), a_scout_over_a_cliff_edge_holds_the_height_of_its_link(), a_swarm_taking_off_together_does_not_beacon_in_lockstep() (+36 more)

### Community 18 - "mod.rs"
Cohesion: 0.14
Nodes (19): base_alive(), burning_cells(), deployed(), Desc, descendants(), fire_centroid(), MAX_CHAIN, relays_by_descendants() (+11 more)

### Community 19 - "main.ts"
Cohesion: 0.04
Nodes (57): air, airLine(), airtime, applyLook(), camera, controls, demoHost, demoMenu (+49 more)

### Community 20 - "TelemetryConfig"
Cohesion: 0.21
Nodes (6): clamp_hz(), RateMode, Routing, Self, sanitize(), TelemetryConfig

### Community 21 - "_"
Cohesion: 0.09
Nodes (28): _, demo(), DIRS, fill_order_puts_connectivity_first(), H, HOP_SPACING, ladder_neighbours(), ladder_spans_the_gap_with_margin() (+20 more)

### Community 22 - "link-style.ts"
Cohesion: 0.13
Nodes (16): BAD, LINK_HEX, linkStyle(), NEIGHBOR, NEIGHBOR_ALPHA, Q_BAD, Q_HEALTHY, Q_WARN (+8 more)

### Community 23 - "channel/tests.rs"
Cohesion: 0.18
Nodes (23): at(), carrier_sense_hears_neighbours_within_cs_range_only(), cell(), channel_is_deterministic(), config_is_sanitized_and_external_busy_counts(), field_calibration_at_12_mbps_is_clean(), field_calibration_reproduces_the_measured_ceiling(), field_loss() (+15 more)

### Community 24 - "selectDrone"
Cohesion: 0.18
Nodes (4): linkBudget, radio, selectDrone(), SelectionMarker

### Community 25 - "Swarm Mind: simulation reference"
Cohesion: 0.18
Nodes (11): 10. Test suites, 11. Measured numbers, 12. Limits and known gaps, 1. Architecture, 3.1 Public API, 3.2 Wire format (`wire.rs`), 3.3 Behaviours, 3. The drone brain (`swarm-brain`) (+3 more)

### Community 26 - "onWorkerMessage"
Cohesion: 0.08
Nodes (19): clearTlmStats(), command(), commandTarget(), fail(), groundAt(), issue(), log, onWorkerMessage() (+11 more)

### Community 27 - "swarm.ts"
Cohesion: 0.11
Nodes (30): antennaGeometry(), ARM_REACH, bodyGeometry(), DRONE_SPAN, frameGeometry(), LIGHT, LIGHT_OFFSETS, LIGHT_SLOTS (+22 more)

### Community 28 - "Sim"
Cohesion: 0.17
Nodes (42): Sim, abi_survives_hostile_input(), command_and_kill_report_success(), MAX_DRONES, MAX_STEP_MS, null_sim_is_harmless(), presets_build_and_stream_chatter(), sim_channel_len() (+34 more)

### Community 29 - ".suspect"
Cohesion: 0.17
Nodes (3): Neighbor, seq_newer(), Brain

### Community 31 - "Brain"
Cohesion: 0.10
Nodes (11): FireReading, Sensors, Brain, fly_to(), is_park(), Intent, Option, Role (+3 more)

### Community 32 - "Channel"
Cohesion: 0.11
Nodes (5): Channel, collision_prob(), LastBroadcast, Option, Vec

### Community 33 - "airtime.rs"
Cohesion: 0.09
Nodes (38): a_held_level_does_not_flicker(), a_lingering_verdict_cuts_once_per_sink_decision(), Airtime, CEILING_MARGIN, CLEAN_BEFORE_RAISE_MS, congestion_goes_up_to_the_sink_and_its_verdict_comes_down(), CUT, CUT_HOLD_MS (+30 more)

### Community 34 - "_"
Cohesion: 0.07
Nodes (28): FixedVec, _, ASSEMBLY_MS, CEILING_AGL, CLIMB_MARGIN, CLIMB_RATE, FILLING_OR_LANDED, MARGINAL_RSSI_DBM (+20 more)

### Community 35 - "layout.test.ts"
Cohesion: 0.08
Nodes (28): ref_node_assert, ref_node_test, Recovery, Tone, DOWN, dynamic(), Style, STYLES (+20 more)

### Community 36 - ".transmit"
Cohesion: 0.16
Nodes (6): Brain, flags_level(), level_of(), Actuation, reached(), Outbox

### Community 37 - ".new"
Cohesion: 0.23
Nodes (13): Bench, broadcast_floods_and_relays_count_bytes(), frame_matches_the_firmware_byte_for_byte(), frame_sizes_match_the_field(), ground_station_measures_loss_from_seq_gaps(), keys(), on(), phase_locked_25hz_with_two_slow_frames_per_second() (+5 more)

### Community 38 - ".new"
Cohesion: 0.23
Nodes (14): a_gap_reforms_the_tree(), backlog_coalesces_into_segments(), boot_forms_a_strongest_rssi_tree(), cut_off_subtree_moves_to_a_rooted_ap(), line(), max_conn_forces_a_second_hop(), no_root_election(), node_sync_lengths_match_the_exact_json() (+6 more)

### Community 39 - "routing.rs"
Cohesion: 0.12
Nodes (16): crate, DEAD_MEMORY_MS, DEAD_SLACK, DeadSink, ELECT_GRACE_MS, LINK_PENALTY, MAX_HOPS, ORIGIN_REACHED (+8 more)

### Community 40 - "Swarm Mind"
Cohesion: 0.15
Nodes (10): All commands, Deploying, Maps, Prerequisites, Project layout, Quick start, Swarm Mind, Telemetry & airtime model (+2 more)

### Community 41 - "Telemetry"
Cohesion: 0.15
Nodes (8): DroneView, median(), reached(), record_mesh_tx(), FnMut, Vec, state_for(), Telemetry

### Community 42 - "tests/telemetry.rs"
Cohesion: 0.05
Nodes (41): CONNECTED_HOLD_MS, DEPLOY_LIMIT_MS, GCS_WINDOW_MS, S8_DIGEST, S8_RUN_MS, S8_WORLD_DIGEST, T10_SINK_LIMIT_MS, T10_STATS_DRAIN_MS (+33 more)

### Community 43 - "telemetry.ts"
Cohesion: 0.06
Nodes (42): 5.1 Telemetry (`telemetry.rs`), linkHealth, linksLine(), airHeader, airMixText(), CH, fleetLossText(), fleetLossTone() (+34 more)

### Community 44 - "preset_tests.rs"
Cohesion: 0.16
Nodes (24): alpine_ridge_is_bit_identical(), archipelago_has_sea_gaps_shorter_than_a_hop(), at_agl(), bases_and_fires_sit_on_dry_burnable_ground(), burn(), canyon_walls_block_the_floor(), every_preset_fire_spreads_on_land_only(), every_preset_is_deterministic_and_seed_dependent() (+16 more)

### Community 45 - ".new"
Cohesion: 0.26
Nodes (7): ChannelConfig, clamp_busy(), dbm_to_mw(), queue_block_prob(), rician_db(), Default, sanitize()

### Community 46 - ".new"
Cohesion: 0.23
Nodes (15): cfg_on(), fnv(), mesh_air(), row(), Routing, Self, s8_script(), STEP_MS (+7 more)

### Community 47 - "t9"
Cohesion: 0.20
Nodes (14): FleetLoss, GcsMeter, label(), meter_run(), one_drone(), summ(), t10_root_death_no_bridge(), t2_byte_budget_one_drone() (+6 more)

### Community 48 - "hostile_inputs_never_panic"
Cohesion: 0.23
Nodes (6): chatter_heartbeats_match_the_senders_brain(), dead_base_station_rejects_commands(), hostile_inputs_never_panic(), killed_drone_falls_to_the_ground_and_stays(), neighbors_forget_a_killed_drone(), Option

### Community 49 - "tlmField"
Cohesion: 0.17
Nodes (10): AirtimeView, GroundFn, heat(), HEAT_HIGH, HEAT_LOW, HEAT_MID, TREE_COLOR, TREE_STALE (+2 more)

### Community 50 - ".describe"
Cohesion: 0.22
Nodes (5): chatterCount(), flagNames(), slotName(), LogEvent, fmtXZ()

### Community 52 - "t8"
Cohesion: 0.15
Nodes (15): deploy_or_fail(), painless_subtree(), pick_victim(), Item, Iterator, Option, Vec, t8() (+7 more)

### Community 53 - "PainlessMesh"
Cohesion: 0.17
Nodes (4): a_lost_segment_waits_for_the_rto(), PainlessMesh, FnMut, String

### Community 54 - "world.rs"
Cohesion: 0.08
Nodes (28): ladder_relays(), a_dead_neighbours_link_decays_to_zero_before_it_is_forgotten(), BASE_ID, base_position(), CEILING, chatter_decodes_heartbeats_intents_and_missions(), Drone, fall() (+20 more)

### Community 55 - "tree_is_physical"
Cohesion: 0.29
Nodes (7): ap_loss(), channel_on(), t5_external_busy_costs_delivery(), t5_field_calibration(), t8_tree_is_physical_alpine(), t8_tree_is_physical_canyon(), tree_is_physical()

### Community 56 - "2. What you can simulate"
Cohesion: 0.25
Nodes (8): 2. What you can simulate, 6. WASM exports (`swarm-wasm`), 8.1 Snapshot, 8.2 Fire, 8.3 Chatter record, 8. Data layouts, Control, apply()

### Community 57 - "AirLedger"
Cohesion: 0.18
Nodes (11): AirLedger, ByteKind, FleetRates, KINDS, node_rows_sum_to_the_fleet(), NodeBytes, NodeRates, Short (+3 more)

### Community 58 - ".service"
Cohesion: 0.21
Nodes (10): digits(), Dir, MeshEvent, MeshTx, node_id(), Package, reached(), Vec (+2 more)

### Community 59 - "Option"
Cohesion: 0.19
Nodes (6): Conn, LinkState, Node, Option, Self, VecDeque

### Community 60 - "sim.worker.ts"
Cohesion: 0.22
Nodes (17): ref_wasm_swarm_wasm_url, chatterKind, chatterBuf, checkLayout(), collectChatter(), copyF32(), COUNTERS, counterSums (+9 more)

### Community 61 - "TlmDrone"
Cohesion: 0.18
Nodes (14): BySent, build_frame(), digits(), Option, Short, String, TelemetryState, TlmDrone (+6 more)

### Community 62 - "scenario_label"
Cohesion: 0.13
Nodes (20): dist_xz(), live_count(), alpine_s0_map_sanity(), c1(), c1_chatter_matches_air(), deploy(), describe(), String (+12 more)

### Community 64 - ".mean_failure"
Cohesion: 0.32
Nodes (9): capture_db(), Interferer, mw_to_dbm(), per(), Phy, rate(), RateInfo, RATES (+1 more)

### Community 65 - "Traffic"
Cohesion: 0.29
Nodes (4): Result, Traffic, Display, Formatter

### Community 66 - ".run"
Cohesion: 0.09
Nodes (12): demoButtons, lossAgreesWithRate(), tlmCount(), treeDescendants(), alive(), DemoHost, DemoRunner, freshContext() (+4 more)

### Community 69 - "Window<N>"
Cohesion: 0.27
Nodes (3): Self, Window<N>, .SPAN_S

### Community 70 - "swarm-brain/src/lib.rs"
Cohesion: 0.29
Nodes (6): MAX_INBOX, MAX_NEIGHBORS, MAX_OUTBOX, fixedvec, max_hops, min_hz_as_telemetry_min_hz

### Community 71 - "super"
Cohesion: 0.22
Nodes (6): canyon_centre, channelconfig, firegrid, radioconfig, super, terrain

### Community 72 - "RadioConfig"
Cohesion: 0.25
Nodes (6): RadioConfig, Default, Option, Self, Terrain, rssi_falls_with_distance_to_minus_95_at_range()

### Community 73 - "base-station.ts"
Cohesion: 0.33
Nodes (6): BaseStation, dishGeometry(), glowTexture(), mergeAll(), strut(), tint()

### Community 74 - "4. The world (`swarm-sim`)"
Cohesion: 0.22
Nodes (8): 4.1 Configuration: `SimConfig`, 4.2 `Sim` methods, 4.3 Maps: `Preset`, 4.4 Terrain, radio and fire, 4.5 Ground-truth metrics (`metrics.rs`, tests only), 4.6 Chatter (`chatter.rs`), 4. The world (`swarm-sim`), stats()

### Community 75 - "Preset"
Cohesion: 0.29
Nodes (3): Preset, .ALL, Option

### Community 76 - "fire.ts"
Cohesion: 0.10
Nodes (11): PresetInfo, FireCell, EMBER_SHADER, FireLayer, Particles, Place, SMOKE_SHADER, clamp() (+3 more)

### Community 77 - "fx.ts"
Cohesion: 0.32
Nodes (4): createFx(), Fx, Quality, renderQuality()

### Community 78 - "NodeRadio"
Cohesion: 0.18
Nodes (7): beacon_airtime_us(), NodeRadio, normals(), Terrain, shadow_db(), Shadowing, shadowing_is_static_symmetric_and_about_5_db()

### Community 79 - ".unicast"
Cohesion: 0.15
Nodes (17): ack_rate_mbps(), AirFrame, AirKind, backoff_us(), byte_kind(), Dst, espnow_airtime_us(), packet_error_rate() (+9 more)

### Community 80 - "BackendBudget"
Cohesion: 0.40
Nodes (3): BackendBudget, budget_rule_and_aimd(), Default

### Community 83 - ".new"
Cohesion: 0.14
Nodes (31): alpine_preset_is_the_default_config(), brain_loss(), channel_costs_brain_frames_airtime_and_loss(), collapse_timeline(), demo_channel_report(), drones_stay_on_the_map_and_above_ground(), fire_spreads_in_the_world(), full_fleet_at_25_hz_congests_the_channel() (+23 more)

### Community 84 - "Vec3"
Cohesion: 0.19
Nodes (8): Add, Self, sqrt(), sqrt_is_accurate(), Vec3, .ZERO, Mul, Sub

### Community 85 - "console.ts"
Cohesion: 0.07
Nodes (29): cache, NATO, parseCallsign(), C, CHATTER_STRIDE, Flag, HB, Heartbeat (+21 more)

### Community 86 - "chatter.rs"
Cohesion: 0.12
Nodes (19): CHATTER_HEARTBEAT, CHATTER_INTENT, CHATTER_MISSION, CHATTER_PM_SYNC, CHATTER_PM_TIME, CHATTER_ROOT_UPLINK, CHATTER_STRIDE, CHATTER_TLM (+11 more)

### Community 87 - "window.rs"
Cohesion: 0.50
Nodes (3): BUCKET_MS, SENT_LAG, Window

### Community 89 - "json_unescape"
Cohesion: 0.67
Nodes (3): json_unescape(), OneDrone, String

### Community 91 - "demos.ts"
Cohesion: 0.06
Nodes (36): 5.2 painlessMesh 1.5.7 (`painless.rs`), 5.3 Shared 2.4 GHz channel (`channel.rs`), 5.4 Everything on the same channel, 5.5 Brain-managed rate (Phase 5), 5. Real telemetry, painlessMesh and shared airtime, 8.4 Telemetry stats and channel summary, fleetTlm, median() (+28 more)

### Community 94 - "three"
Cohesion: 0.26
Nodes (4): three, CameraDirector, ease(), ShotKind

### Community 97 - "metrics.rs"
Cohesion: 0.27
Nodes (9): MAX_CHAIN, route_ok(), routed_count(), Item, Iterator, Role, scouts_connected(), with_role() (+1 more)

### Community 98 - "SwarmView"
Cohesion: 0.18
Nodes (6): clamp(), clamp01(), droneColor(), lerp(), planeSampler(), SwarmView

### Community 103 - "channel.rs"
Cohesion: 0.04
Nodes (54): ACK_BYTES, ACTIVE_EPS, AIFS_BE_US, AUTO_RATES, BEACON_BYTES, BEACON_INTERVAL_MS, BUSY_CAP, CAPTURE_DSSS_DB (+46 more)

## Knowledge Gaps
- **572 isolated node(s):** `MIN_HZ`, `START_HZ`, `EVAL_MS`, `CUT_HOLD_MS`, `CLEAN_BEFORE_RAISE_MS` (+567 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 762 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **10 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Vec3` connect `Vec3` to `World`, `Brain`, `terrain.rs`, `_`, `fire.rs`, `src/telemetry.rs`, `tests.rs`, `_`, `.suspect`, `Brain`, `Channel`, `_`, `.transmit`, `routing.rs`, `Telemetry`, `preset_tests.rs`, `hostile_inputs_never_panic`, `world.rs`, `scenario_label`, `RadioConfig`, `NodeRadio`, `channel.rs`?**
  _High betweenness centrality (0.134) - this node is a cross-community bridge._
- **Why does `Sim` connect `Sim` to `World`, `_`, `fire.rs`, `LinkMeter`, `mod.rs`, `Channel`, `Telemetry`, `.new`, `t9`, `hostile_inputs_never_panic`, `t8`, `PainlessMesh`, `world.rs`, `scenario_label`, `.new`, `Preset`, `NodeRadio`, `.new`, `metrics.rs`?**
  _High betweenness centrality (0.080) - this node is a cross-community bridge._
- **Why does `Channel` connect `Channel` to `fire.rs`, `src/telemetry.rs`, `painless.rs`, `channel/tests.rs`, `Sim`, `.new`, `.new`, `Telemetry`, `.new`, `world.rs`, `AirLedger`, `.service`, `.mean_failure`, `RadioConfig`, `NodeRadio`, `.unicast`, `.new`, `Vec3`, `channel.rs`?**
  _High betweenness centrality (0.066) - this node is a cross-community bridge._
- **What connects `MIN_HZ`, `START_HZ`, `EVAL_MS` to the rest of the system?**
  _572 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Snapshot` be split into smaller, more focused modules?**
  _Cohesion score 0.09061224489795919 - nodes in this community are weakly interconnected._
- **Should `Brain` be split into smaller, more focused modules?**
  _Cohesion score 0.0915915915915916 - nodes in this community are weakly interconnected._
- **Should `terrain.rs` be split into smaller, more focused modules?**
  _Cohesion score 0.112375533428165 - nodes in this community are weakly interconnected._