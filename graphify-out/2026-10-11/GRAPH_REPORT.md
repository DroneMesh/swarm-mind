# Graph Report - swarm-mind  (2026-10-11)

## Corpus Check
- 71 files · ~142,696 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 2 file(s) not represented in the graph (top: .toml 1, (none) 1)

## Summary
- 2216 nodes · 5172 edges · 94 communities (89 shown, 5 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 201 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Snapshot
- World
- Brain
- terrain.rs
- _
- super
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
- .run
- tests.rs
- mod.rs
- main.ts
- TelemetryConfig
- _
- link-style.ts
- channel/tests.rs
- PulseView
- Swarm Mind: simulation reference
- onWorkerMessage
- swarm.ts
- Sim
- Brain
- web_src_render_terrain_buildterrain
- Brain
- Channel
- airtime.rs
- _
- layout.test.ts
- Brain
- .new
- .new
- _
- Swarm Mind
- Telemetry
- tests/telemetry.rs
- telemetry.ts
- preset_tests.rs
- .new
- .new
- t9
- mesh_link_report
- tlmField
- LinkBudgetPanel
- RadioConsole
- t8
- .step
- .step
- label
- 2. What you can simulate
- AirLedger
- PainlessMesh
- Option
- sim.worker.ts
- VARIANT
- SimConfig
- resetSession
- RateInfo
- Traffic
- DemoRunner
- .new
- three
- Window<N>
- swarm-brain/src/lib.rs
- .sync_len
- NodeRadio
- base-station.ts
- 4. The world (`swarm-sim`)
- Preset
- fire.ts
- fx.ts
- shadow_db
- .unicast
- BackendBudget
- Lifecycle
- vec3
- world.rs
- Vec3
- console.ts
- chatter.rs
- window.rs
- Terrain
- demos.ts
- CameraDirector
- metrics.rs
- SwarmView
- channel.rs

## God Nodes (most connected - your core abstractions)
1. `Sim` - 117 edges
2. `Channel` - 67 edges
3. `Vec3` - 65 edges
4. `PainlessMesh` - 55 edges
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
- `Control` --references--> `hz()`  [INFERRED]
  docs/SIMULATION.md → web/src/ui/demos.ts
- `5.1 Telemetry (`telemetry.rs`)` --references--> `Variant`  [INFERRED]
  docs/SIMULATION.md → web/src/telemetry.ts
- `5.1 Telemetry (`telemetry.rs`)` --references--> `RateMode`  [INFERRED]
  docs/SIMULATION.md → web/src/telemetry.ts

## Import Cycles
- 2-file cycle: `crates/swarm-brain/src/brain.rs -> crates/swarm-brain/src/mission.rs -> crates/swarm-brain/src/brain.rs`
- 2-file cycle: `crates/swarm-brain/src/airtime.rs -> crates/swarm-brain/src/brain.rs -> crates/swarm-brain/src/airtime.rs`

## Communities (94 total, 5 thin omitted)

### Community 0 - "Snapshot"
Cohesion: 0.13
Nodes (7): actingSinkName(), pickRelay(), syncQuality(), updateHud(), SwarmMonitor, Snapshot, routed()

### Community 1 - "World"
Cohesion: 0.21
Nodes (7): nearest_burning_xz(), FnMut, Intent, Option, String, summary_line(), World

### Community 2 - "Brain"
Cohesion: 0.11
Nodes (9): Actuation, Brain, Radio, Intent, NodeId, Option, Outbox, Role (+1 more)

### Community 3 - "terrain.rs"
Cohesion: 0.11
Nodes (17): archipelago(), canyon(), canyon_centre(), CANYON_FLOOR, CANYON_RIM, foothills(), height_at_matches_grid_samples(), lerp() (+9 more)

### Community 4 - "_"
Cohesion: 0.06
Nodes (41): _, Beacon, BROADCAST, CONGESTION_UP_SHIFT, FLAG_CARRYING, FLAG_CONGESTION_MASK, FLAG_CONGESTION_SHIFT, FLAG_CUT_EPOCH (+33 more)

### Community 5 - "super"
Cohesion: 0.06
Nodes (24): canyon_centre, channelconfig, FixedVec<T, N>, push_rejects_when_full(), retain_keeps_matching(), Default, FnMut, Result (+16 more)

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
Nodes (67): chatter, ACK_BYTES_PER_S, AIR_BRAIN, AIR_MESH, AIR_UPLINK, AIRBORNE_AGL, AIRTIME_FRAC, AMSL_DATUM_M (+59 more)

### Community 13 - "swarm-brain"
Cohesion: 1.00
Nodes (3): swarm-brain, swarm-sim, swarm-wasm

### Community 14 - "painless.rs"
Cohesion: 0.06
Nodes (31): AP_SYNC_LAG_MS, BROADCAST, CONNECT_MS, encode_time_sync(), FORM_GAP_MS, MAX_CONN, MAX_MESSAGE_QUEUE, MESH_PORT (+23 more)

### Community 15 - "scenarios.rs"
Cohesion: 0.05
Nodes (40): BASE_OFFLINE_LIMIT_MS, BASE_OFFLINE_MAX_RATE, BASE_OFFLINE_WINDOW_MS, c1_chatter_matches_air(), CONNECTED_HOLD_MS, DEPLOY_LIMIT_MS, DETERMINISM_MS, IDLE_MAX_RATE (+32 more)

### Community 17 - "tests.rs"
Cohesion: 0.11
Nodes (44): a_cut_off_drone_poisons_its_route_instead_of_counting_up(), a_drone_that_missed_the_flood_catches_up(), a_far_holder_waits_its_turn_before_moving_up(), a_filler_that_arrived_and_then_loses_to_the_holder_stays_away(), a_filler_that_loses_to_another_filler_stays_away(), a_group_that_elected_its_own_sink_heads_home_and_merges(), a_scout_over_a_cliff_edge_holds_the_height_of_its_link(), a_swarm_taking_off_together_does_not_beacon_in_lockstep() (+36 more)

### Community 18 - "mod.rs"
Cohesion: 0.12
Nodes (23): base_alive(), burning_cells(), deployed(), Desc, descendants(), dist_xz(), fire_centroid(), live_count() (+15 more)

### Community 19 - "main.ts"
Cohesion: 0.04
Nodes (55): callsign(), air, airLine(), airtime, camera, controls, demoHost, demoMenu (+47 more)

### Community 20 - "TelemetryConfig"
Cohesion: 0.23
Nodes (6): clamp_hz(), RateMode, Routing, Self, sanitize(), TelemetryConfig

### Community 21 - "_"
Cohesion: 0.08
Nodes (30): _, demo(), DIRS, fill_order_puts_connectivity_first(), H, heard_bits_are_distinct(), HOP_SPACING, ladder_neighbours() (+22 more)

### Community 22 - "link-style.ts"
Cohesion: 0.11
Nodes (18): ref_node_assert, ref_node_test, BAD, LINK_HEX, linkStyle(), NEIGHBOR, NEIGHBOR_ALPHA, Q_BAD (+10 more)

### Community 23 - "channel/tests.rs"
Cohesion: 0.18
Nodes (23): at(), carrier_sense_hears_neighbours_within_cs_range_only(), cell(), channel_is_deterministic(), config_is_sanitized_and_external_busy_counts(), field_calibration_at_12_mbps_is_clean(), field_calibration_reproduces_the_measured_ceiling(), field_loss() (+15 more)

### Community 24 - "PulseView"
Cohesion: 0.22
Nodes (3): onSnapshot(), dynamic(), PulseView

### Community 25 - "Swarm Mind: simulation reference"
Cohesion: 0.18
Nodes (11): 10. Test suites, 11. Measured numbers, 12. Limits and known gaps, 1. Architecture, 3.1 Public API, 3.2 Wire format (`wire.rs`), 3.3 Behaviours, 3. The drone brain (`swarm-brain`) (+3 more)

### Community 26 - "onWorkerMessage"
Cohesion: 0.15
Nodes (18): command(), commandTarget(), fail(), groundAt(), issue(), kill(), log, onWorkerMessage() (+10 more)

### Community 27 - "swarm.ts"
Cohesion: 0.11
Nodes (30): antennaGeometry(), ARM_REACH, bodyGeometry(), DRONE_SPAN, frameGeometry(), LIGHT, LIGHT_OFFSETS, LIGHT_SLOTS (+22 more)

### Community 28 - "Sim"
Cohesion: 0.16
Nodes (43): Vec, Sim, abi_survives_hostile_input(), command_and_kill_report_success(), MAX_DRONES, MAX_STEP_MS, null_sim_is_harmless(), presets_build_and_stream_chatter() (+35 more)

### Community 29 - "Brain"
Cohesion: 0.13
Nodes (8): Neighbor, seq_newer(), Brain, DeadSink, Routing, NodeId, Option, Self

### Community 31 - "Brain"
Cohesion: 0.16
Nodes (3): Brain, is_park(), Intent

### Community 32 - "Channel"
Cohesion: 0.11
Nodes (7): Channel, collision_prob(), Interferer, LastBroadcast, rate_index(), Option, Vec

### Community 33 - "airtime.rs"
Cohesion: 0.09
Nodes (38): a_held_level_does_not_flicker(), a_lingering_verdict_cuts_once_per_sink_decision(), Airtime, CEILING_MARGIN, CLEAN_BEFORE_RAISE_MS, congestion_goes_up_to_the_sink_and_its_verdict_comes_down(), CUT, CUT_HOLD_MS (+30 more)

### Community 34 - "_"
Cohesion: 0.07
Nodes (28): FixedVec, _, ASSEMBLY_MS, CEILING_AGL, CLIMB_MARGIN, CLIMB_RATE, FILLING_OR_LANDED, MARGINAL_RSSI_DBM (+20 more)

### Community 35 - "layout.test.ts"
Cohesion: 0.16
Nodes (15): DOWN, Style, STYLES, UNKNOWN, DEGRADED_QUALITY, DRONE_STRIDE, HEADER_LEN, L (+7 more)

### Community 36 - "Brain"
Cohesion: 0.22
Nodes (4): Brain, flags_level(), level_of(), reached()

### Community 37 - ".new"
Cohesion: 0.31
Nodes (9): Bench, broadcast_floods_and_relays_count_bytes(), ground_station_measures_loss_from_seq_gaps(), on(), phase_locked_25hz_with_two_slow_frames_per_second(), rate_modes(), Item, Iterator (+1 more)

### Community 38 - ".new"
Cohesion: 0.23
Nodes (14): a_gap_reforms_the_tree(), a_lost_segment_waits_for_the_rto(), backlog_coalesces_into_segments(), boot_forms_a_strongest_rssi_tree(), cut_off_subtree_moves_to_a_rooted_ap(), line(), max_conn_forces_a_second_hop(), no_root_election() (+6 more)

### Community 39 - "_"
Cohesion: 0.09
Nodes (24): crate, _, Config, JITTER_DIV, LANDED_AGL, MIN_BEACON_GAP_MS, NO_ROUTE, PARKED_INTERVAL_MS (+16 more)

### Community 40 - "Swarm Mind"
Cohesion: 0.18
Nodes (9): All commands, Deploying, Maps, Prerequisites, Project layout, Quick start, Swarm Mind, Troubleshooting (+1 more)

### Community 41 - "Telemetry"
Cohesion: 0.11
Nodes (13): BySent, DroneView, keys(), median(), reached(), record_mesh_tx(), FnMut, Short (+5 more)

### Community 42 - "tests/telemetry.rs"
Cohesion: 0.05
Nodes (39): CONNECTED_HOLD_MS, DEPLOY_LIMIT_MS, GCS_WINDOW_MS, S8_DIGEST, S8_RUN_MS, S8_WORLD_DIGEST, T10_SINK_LIMIT_MS, T10_STATS_DRAIN_MS (+31 more)

### Community 43 - "telemetry.ts"
Cohesion: 0.09
Nodes (35): 5.1 Telemetry (`telemetry.rs`), airHeader, airMixText(), CH, CHANNEL_LEN, fleetLossText(), fleetLossTone(), fmtByteRate() (+27 more)

### Community 44 - "preset_tests.rs"
Cohesion: 0.16
Nodes (25): alpine_ridge_is_bit_identical(), archipelago_has_sea_gaps_shorter_than_a_hop(), at_agl(), bases_and_fires_sit_on_dry_burnable_ground(), burn(), canyon_walls_block_the_floor(), every_preset_fire_spreads_on_land_only(), every_preset_is_deterministic_and_seed_dependent() (+17 more)

### Community 45 - ".new"
Cohesion: 0.27
Nodes (8): ChannelConfig, clamp_busy(), dbm_to_mw(), mw_to_dbm(), queue_block_prob(), rician_db(), Default, sanitize()

### Community 46 - ".new"
Cohesion: 0.21
Nodes (17): cfg_on(), fnv(), mesh_air(), row(), Routing, Self, s8_script(), STEP_MS (+9 more)

### Community 47 - "t9"
Cohesion: 0.17
Nodes (17): deploy_or_fail(), FleetLoss, GcsMeter, meter_run(), one_drone(), painless_subtree(), pick_victim(), Option (+9 more)

### Community 48 - "mesh_link_report"
Cohesion: 0.15
Nodes (12): brain_loss(), channel_costs_brain_frames_airtime_and_loss(), drones_stay_on_the_map_and_above_ground(), hostile_inputs_never_panic(), ids(), mesh_link_report(), neighbors_forget_a_killed_drone(), Intent (+4 more)

### Community 49 - "tlmField"
Cohesion: 0.14
Nodes (13): AirtimeView, GroundFn, heat(), HEAT_HIGH, HEAT_LOW, HEAT_MID, TREE_COLOR, TREE_STALE (+5 more)

### Community 50 - "LinkBudgetPanel"
Cohesion: 0.12
Nodes (10): linkHealth, linksLine(), linkHealthText(), el(), LinkBudgetHost, LinkBudgetPanel, pct(), tag() (+2 more)

### Community 51 - "RadioConsole"
Cohesion: 0.13
Nodes (4): ConsoleHost, el(), fmtTime(), RadioConsole

### Community 52 - "t8"
Cohesion: 0.22
Nodes (9): Item, Iterator, t8(), t8_relay_kill_under_telemetry_alpine(), t8_relay_kill_under_telemetry_archipelago(), t8_relay_kill_under_telemetry_canyon(), t8_relay_kill_under_telemetry_foothills(), t8_relay_kill_under_telemetry_twin_valleys() (+1 more)

### Community 53 - ".step"
Cohesion: 0.24
Nodes (4): reached(), FnMut, stream(), upward_tcp_runs_nagle_with_delayed_acks()

### Community 54 - ".step"
Cohesion: 0.16
Nodes (10): base_position(), chatter_decodes_heartbeats_intents_and_missions(), chatter_heartbeats_match_the_senders_brain(), Drone, fall(), fly(), Brain, Terrain (+2 more)

### Community 55 - "label"
Cohesion: 0.18
Nodes (11): ap_loss(), channel_on(), json_unescape(), label(), OneDrone, String, t5_external_busy_costs_delivery(), t5_field_calibration() (+3 more)

### Community 56 - "2. What you can simulate"
Cohesion: 0.25
Nodes (8): 2. What you can simulate, 6. WASM exports (`swarm-wasm`), 8.1 Snapshot, 8.2 Fire, 8.3 Chatter record, 8. Data layouts, Control, apply()

### Community 57 - "AirLedger"
Cohesion: 0.17
Nodes (11): AirLedger, ByteKind, FleetRates, KINDS, node_rows_sum_to_the_fleet(), NodeBytes, NodeRates, Self (+3 more)

### Community 58 - "PainlessMesh"
Cohesion: 0.26
Nodes (6): Dir, MeshEvent, MeshTx, Package, PainlessMesh, Vec

### Community 59 - "Option"
Cohesion: 0.17
Nodes (8): Conn, LinkState, Node, Option, Self, tcp_timers_follow_lwip(), tick_after(), VecDeque

### Community 60 - "sim.worker.ts"
Cohesion: 0.19
Nodes (19): ref_wasm_swarm_wasm_url, CHATTER_STRIDE, chatterKind, chatterBuf, checkLayout(), collectChatter(), copyF32(), COUNTERS (+11 more)

### Community 61 - "VARIANT"
Cohesion: 0.21
Nodes (13): build_frame(), digits(), frame_matches_the_firmware_byte_for_byte(), frame_sizes_match_the_field(), Option, String, state(), TelemetryState (+5 more)

### Community 62 - "SimConfig"
Cohesion: 0.27
Nodes (17): Default, SimConfig, c1(), deploy(), expect_restored(), kill_top_relay(), s1(), s10() (+9 more)

### Community 63 - "resetSession"
Cohesion: 0.12
Nodes (5): clearTlmStats(), fitViewport(), resetSession(), FireLayer, TargetMarker

### Community 64 - "RateInfo"
Cohesion: 0.26
Nodes (9): capture_db(), normals(), packet_error_rate(), per(), Phy, rate(), RateInfo, RATES (+1 more)

### Community 65 - "Traffic"
Cohesion: 0.29
Nodes (4): Result, Traffic, Display, Formatter

### Community 66 - "DemoRunner"
Cohesion: 0.17
Nodes (7): demoButtons, alive(), DemoRunner, freshContext(), readMs(), select(), throughput()

### Community 67 - ".new"
Cohesion: 0.36
Nodes (3): RoleWatch, Role, Self

### Community 68 - "three"
Cohesion: 0.26
Nodes (3): three, dyn(), SmokePool

### Community 69 - "Window<N>"
Cohesion: 0.27
Nodes (3): Self, Window<N>, .SPAN_S

### Community 70 - "swarm-brain/src/lib.rs"
Cohesion: 0.22
Nodes (8): MAX_INBOX, MAX_NEIGHBORS, MAX_OUTBOX, sqrt(), sqrt_is_accurate(), fixedvec, max_hops, min_hz_as_telemetry_min_hz

### Community 71 - ".sync_len"
Cohesion: 0.29
Nodes (6): digits(), encode_package(), escape_into(), node_id(), package_escapes_the_inner_json(), String

### Community 72 - "NodeRadio"
Cohesion: 0.17
Nodes (9): beacon_airtime_us(), NodeRadio, Terrain, RadioConfig, Default, Option, Self, Terrain (+1 more)

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
Cohesion: 0.17
Nodes (5): FireCell, EMBER_SHADER, Particles, Place, SMOKE_SHADER

### Community 77 - "fx.ts"
Cohesion: 0.32
Nodes (4): createFx(), Fx, Quality, renderQuality()

### Community 78 - "shadow_db"
Cohesion: 0.33
Nodes (3): shadow_db(), Shadowing, shadowing_is_static_symmetric_and_about_5_db()

### Community 79 - ".unicast"
Cohesion: 0.15
Nodes (15): ack_rate_mbps(), AirFrame, AirKind, backoff_us(), byte_kind(), Dst, espnow_airtime_us(), phy_of() (+7 more)

### Community 80 - "BackendBudget"
Cohesion: 0.40
Nodes (3): BackendBudget, budget_rule_and_aimd(), Default

### Community 83 - "world.rs"
Cohesion: 0.12
Nodes (38): a_dead_neighbours_link_decays_to_zero_before_it_is_forgotten(), alpine_preset_is_the_default_config(), BASE_ID, CEILING, dead_base_station_rejects_commands(), demo_channel_report(), fire_spreads_in_the_world(), full_fleet_at_25_hz_congests_the_channel() (+30 more)

### Community 84 - "Vec3"
Cohesion: 0.13
Nodes (11): Add, FireReading, Sensors, Self, Vec3, .ZERO, fly_to(), Option (+3 more)

### Community 85 - "console.ts"
Cohesion: 0.06
Nodes (32): cache, NATO, parseCallsign(), C, chatterCount(), Flag, flagNames(), HB (+24 more)

### Community 86 - "chatter.rs"
Cohesion: 0.12
Nodes (19): CHATTER_HEARTBEAT, CHATTER_INTENT, CHATTER_MISSION, CHATTER_PM_SYNC, CHATTER_PM_TIME, CHATTER_ROOT_UPLINK, CHATTER_STRIDE, CHATTER_TLM (+11 more)

### Community 87 - "window.rs"
Cohesion: 0.50
Nodes (3): BUCKET_MS, SENT_LAG, Window

### Community 90 - "Terrain"
Cohesion: 0.14
Nodes (12): applyLook(), picker, switchPreset(), PresetInfo, PRESETS, clamp(), paint(), paintStrata() (+4 more)

### Community 91 - "demos.ts"
Cohesion: 0.06
Nodes (37): 5.2 painlessMesh 1.5.7 (`painless.rs`), 5.3 Shared 2.4 GHz channel (`channel.rs`), 5.4 Everything on the same channel, 5.5 Brain-managed rate (Phase 5), 5. Real telemetry, painlessMesh and shared airtime, 8.4 Telemetry stats and channel summary, Telemetry & airtime model, fleetTlm (+29 more)

### Community 94 - "CameraDirector"
Cohesion: 0.13
Nodes (8): linkBudget, overview(), radio, selectDrone(), CameraDirector, ease(), ShotKind, SelectionMarker

### Community 97 - "metrics.rs"
Cohesion: 0.27
Nodes (9): MAX_CHAIN, route_ok(), routed_count(), Item, Iterator, Role, scouts_connected(), with_role() (+1 more)

### Community 98 - "SwarmView"
Cohesion: 0.15
Nodes (7): clamp(), clamp01(), droneColor(), lerp(), planeSampler(), SwarmView, wrapAngle()

### Community 103 - "channel.rs"
Cohesion: 0.04
Nodes (54): ACK_BYTES, ACTIVE_EPS, AIFS_BE_US, AUTO_RATES, BEACON_BYTES, BEACON_INTERVAL_MS, BUSY_CAP, CAPTURE_DSSS_DB (+46 more)

## Knowledge Gaps
- **569 isolated node(s):** `MIN_HZ`, `START_HZ`, `EVAL_MS`, `CUT_HOLD_MS`, `CLEAN_BEFORE_RAISE_MS` (+564 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 757 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **5 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Vec3` connect `Vec3` to `World`, `Brain`, `terrain.rs`, `_`, `src/telemetry.rs`, `tests.rs`, `mod.rs`, `_`, `Brain`, `Brain`, `Channel`, `_`, `_`, `Telemetry`, `preset_tests.rs`, `mesh_link_report`, `.step`, `swarm-brain/src/lib.rs`, `NodeRadio`, `channel.rs`?**
  _High betweenness centrality (0.139) - this node is a cross-community bridge._
- **Why does `Sim` connect `Sim` to `World`, `_`, `super`, `fire.rs`, `mod.rs`, `Channel`, `Telemetry`, `.new`, `t9`, `mesh_link_report`, `t8`, `.step`, `PainlessMesh`, `SimConfig`, `.new`, `NodeRadio`, `Preset`, `world.rs`, `metrics.rs`?**
  _High betweenness centrality (0.076) - this node is a cross-community bridge._
- **Why does `Channel` connect `Channel` to `fire.rs`, `src/telemetry.rs`, `painless.rs`, `channel/tests.rs`, `Sim`, `.new`, `.new`, `Telemetry`, `.new`, `mesh_link_report`, `AirLedger`, `PainlessMesh`, `RateInfo`, `NodeRadio`, `shadow_db`, `.unicast`, `world.rs`, `Vec3`, `channel.rs`?**
  _High betweenness centrality (0.060) - this node is a cross-community bridge._
- **What connects `MIN_HZ`, `START_HZ`, `EVAL_MS` to the rest of the system?**
  _569 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Snapshot` be split into smaller, more focused modules?**
  _Cohesion score 0.12773109243697478 - nodes in this community are weakly interconnected._
- **Should `Brain` be split into smaller, more focused modules?**
  _Cohesion score 0.11174242424242424 - nodes in this community are weakly interconnected._
- **Should `terrain.rs` be split into smaller, more focused modules?**
  _Cohesion score 0.112375533428165 - nodes in this community are weakly interconnected._