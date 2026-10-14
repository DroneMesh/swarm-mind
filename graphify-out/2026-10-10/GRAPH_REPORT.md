# Graph Report - swarm-mind  (2026-10-10)

## Corpus Check
- 63 files · ~121,313 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 2 file(s) not represented in the graph (top: .toml 1, (none) 1)

## Summary
- 1994 nodes · 4678 edges · 90 communities (85 shown, 5 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 142 edges (avg confidence: 0.82)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Snapshot
- World
- Brain
- Preset
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
- .run
- tests.rs
- mod.rs
- main.ts
- TelemetryConfig
- onWorkerMessage
- FireLayer
- Telemetry
- PulseView
- DemoRunner
- _
- swarm.ts
- Sim
- .suspect
- web_src_render_terrain_buildterrain
- Brain
- Channel
- airtime.rs
- _
- monitor.ts
- .transmit
- RadioConfig
- run
- routing.rs
- Swarm Mind
- .step
- tests/telemetry.rs
- linkbudget.ts
- preset_tests.rs
- .new
- .new
- t9
- hostile_inputs_never_panic
- airtime.ts
- LinkBudgetPanel
- RadioConsole
- t8
- PainlessMesh
- world.rs
- tree_is_physical
- .account
- t10_root_death_no_bridge
- .service
- Conn
- sim.worker.ts
- TlmDrone
- scenario_label
- resetSession
- .new
- Traffic
- swarm-brain/src/lib.rs
- .new
- three
- ChannelConfig
- fire.ts
- .sync_len
- .end_step
- base-station.ts
- .describe
- switchPreset
- Routing
- fx.ts
- pulses.ts
- vec3
- .new
- Vec3
- console.ts
- chatter.rs
- Terrain
- demos.ts
- CameraDirector
- metrics.rs
- SwarmView
- channel.rs

## God Nodes (most connected - your core abstractions)
1. `Sim` - 113 edges
2. `Vec3` - 64 edges
3. `PainlessMesh` - 54 edges
4. `Brain` - 48 edges
5. `_` - 47 edges
6. `Channel` - 47 edges
7. `Snapshot` - 44 edges
8. `RadioConsole` - 39 edges
9. `_` - 34 edges
10. `_` - 33 edges

## Surprising Connections (you probably didn't know these)
- `Telemetry & airtime model` --references--> `s()`  [INFERRED]
  README.md → web/src/ui/demos.ts
- `Brain` --references--> `MAX_NEIGHBORS`  [EXTRACTED]
  crates/swarm-brain/src/brain.rs → crates/swarm-brain/src/lib.rs
- `stale_or_foreign_intents_are_ignored()` --calls--> `mission()`  [INFERRED]
  crates/swarm-brain/src/tests.rs → crates/swarm-brain/src/wire.rs
- `every_preset_is_solvable_with_spares()` --calls--> `base_position()`  [INFERRED]
  crates/swarm-sim/src/preset_tests.rs → crates/swarm-sim/src/world.rs
- `bases_and_fires_sit_on_dry_burnable_ground()` --calls--> `base_position()`  [INFERRED]
  crates/swarm-sim/src/preset_tests.rs → crates/swarm-sim/src/world.rs

## Import Cycles
- 2-file cycle: `crates/swarm-brain/src/brain.rs -> crates/swarm-brain/src/mission.rs -> crates/swarm-brain/src/brain.rs`

## Communities (90 total, 5 thin omitted)

### Community 0 - "Snapshot"
Cohesion: 0.13
Nodes (9): actingSinkName(), pickRelay(), updateHud(), SwarmMonitor, Snapshot, alive(), routed(), select() (+1 more)

### Community 1 - "World"
Cohesion: 0.21
Nodes (7): nearest_burning_xz(), FnMut, Intent, Option, String, summary_line(), World

### Community 2 - "Brain"
Cohesion: 0.09
Nodes (20): Airtime, _, Brain, Config, JITTER_DIV, LANDED_AGL, MIN_BEACON_GAP_MS, NO_ROUTE (+12 more)

### Community 3 - "Preset"
Cohesion: 0.07
Nodes (25): canyon_centre, channelconfig, Preset, .ALL, Option, archipelago(), canyon(), canyon_centre() (+17 more)

### Community 4 - "_"
Cohesion: 0.06
Nodes (41): _, Beacon, BROADCAST, CONGESTION_UP_SHIFT, FLAG_CARRYING, FLAG_CONGESTION_MASK, FLAG_CONGESTION_SHIFT, FLAG_CUT_EPOCH (+33 more)

### Community 5 - "FixedVec<T, N>"
Cohesion: 0.14
Nodes (12): FixedVec, FixedVec<T, N>, push_rejects_when_full(), retain_keeps_matching(), Default, FnMut, Result, Self (+4 more)

### Community 6 - "Swarm Mind — Self-Healing Wildfire Drone Swarm"
Cohesion: 0.10
Nodes (20): Core Capabilities (MoSCoW), Decisions Log, Evidence, Implementation Phases, Key Hypothesis, MVP Scope, Open Questions, Parallelism Notes (+12 more)

### Community 7 - "package.json"
Cohesion: 0.09
Nodes (20): @types/three, typescript, vite, dependencies, three, devDependencies, @types/three, typescript (+12 more)

### Community 8 - "SwarmExports"
Cohesion: 0.10
Nodes (11): apply(), collectChatter(), copyF32(), init(), post(), postFireIfChanged(), postTelemetry(), postTlmSamples() (+3 more)

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
Cohesion: 0.04
Nodes (55): chatter, AIR_BRAIN, AIR_MESH, AIR_UPLINK, AIRBORNE_AGL, AIRTIME_FRAC, AMSL_DATUM_M, BRAIN_BYTES_PER_S (+47 more)

### Community 13 - "swarm-brain"
Cohesion: 1.00
Nodes (3): swarm-brain, swarm-sim, swarm-wasm

### Community 14 - "painless.rs"
Cohesion: 0.06
Nodes (33): BROADCAST, CONNECT_MS, encode_time_sync(), FORM_GAP_MS, MAX_CONN, MAX_MESSAGE_QUEUE, MESH_PORT, NODE_SYNC_INTERVAL_MS (+25 more)

### Community 15 - "scenarios.rs"
Cohesion: 0.06
Nodes (41): BASE_OFFLINE_LIMIT_MS, BASE_OFFLINE_MAX_RATE, BASE_OFFLINE_WINDOW_MS, CONNECTED_HOLD_MS, DEPLOY_LIMIT_MS, DETERMINISM_MS, expect_restored(), IDLE_MAX_RATE (+33 more)

### Community 17 - "tests.rs"
Cohesion: 0.11
Nodes (44): a_cut_off_drone_poisons_its_route_instead_of_counting_up(), a_drone_that_missed_the_flood_catches_up(), a_far_holder_waits_its_turn_before_moving_up(), a_filler_that_arrived_and_then_loses_to_the_holder_stays_away(), a_filler_that_loses_to_another_filler_stays_away(), a_group_that_elected_its_own_sink_heads_home_and_merges(), a_scout_over_a_cliff_edge_holds_the_height_of_its_link(), a_swarm_taking_off_together_does_not_beacon_in_lockstep() (+36 more)

### Community 18 - "mod.rs"
Cohesion: 0.14
Nodes (19): base_alive(), burning_cells(), deployed(), Desc, descendants(), fire_centroid(), MAX_CHAIN, relays_by_descendants() (+11 more)

### Community 19 - "main.ts"
Cohesion: 0.05
Nodes (43): airLine(), airtime, camera, controls, demoHost, demoMenu, demos, director (+35 more)

### Community 20 - "TelemetryConfig"
Cohesion: 0.13
Nodes (10): BackendBudget, budget_rule_and_aimd(), clamp_hz(), RateMode, Routing, Default, Option, Self (+2 more)

### Community 21 - "onWorkerMessage"
Cohesion: 0.14
Nodes (17): command(), commandTarget(), fail(), groundAt(), issue(), kill(), log, onWorkerMessage() (+9 more)

### Community 22 - "FireLayer"
Cohesion: 0.18
Nodes (3): fitViewport(), FireLayer, Particles

### Community 23 - "Telemetry"
Cohesion: 0.19
Nodes (4): Telemetry, Window<N>, .SPAN_S, windows_slide()

### Community 25 - "DemoRunner"
Cohesion: 0.20
Nodes (4): demoButtons, DemoRunner, freshContext(), readMs()

### Community 26 - "_"
Cohesion: 0.08
Nodes (30): _, demo(), DIRS, fill_order_puts_connectivity_first(), H, heard_bits_are_distinct(), HOP_SPACING, ladder_neighbours() (+22 more)

### Community 27 - "swarm.ts"
Cohesion: 0.11
Nodes (30): antennaGeometry(), ARM_REACH, bodyGeometry(), DRONE_SPAN, frameGeometry(), LIGHT, LIGHT_OFFSETS, LIGHT_SLOTS (+22 more)

### Community 28 - "Sim"
Cohesion: 0.15
Nodes (43): Vec, Sim, abi_survives_hostile_input(), command_and_kill_report_success(), MAX_DRONES, MAX_STEP_MS, null_sim_is_harmless(), presets_build_and_stream_chatter() (+35 more)

### Community 29 - ".suspect"
Cohesion: 0.19
Nodes (3): Neighbor, seq_newer(), Brain

### Community 31 - "Brain"
Cohesion: 0.15
Nodes (4): Brain, is_park(), Intent, Role

### Community 32 - "Channel"
Cohesion: 0.13
Nodes (7): Channel, collision_prob(), loss_is_monotone_in_load_and_external_busy(), queue_block_prob(), rates_retries_and_queue_formulas(), Option, wifi_rate_mbps()

### Community 33 - "airtime.rs"
Cohesion: 0.09
Nodes (38): a_held_level_does_not_flicker(), a_lingering_verdict_cuts_once_per_sink_decision(), Airtime, CEILING_MARGIN, CLEAN_BEFORE_RAISE_MS, congestion_goes_up_to_the_sink_and_its_verdict_comes_down(), CUT, CUT_HOLD_MS (+30 more)

### Community 34 - "_"
Cohesion: 0.08
Nodes (26): _, ASSEMBLY_MS, CEILING_AGL, CLIMB_MARGIN, CLIMB_RATE, FILLING_OR_LANDED, MARGINAL_RSSI_DBM, MAX_LINK_SLOPE (+18 more)

### Community 35 - "monitor.ts"
Cohesion: 0.19
Nodes (12): LogEvent, Recovery, Tone, D, DRONE_STRIDE, HEADER_LEN, LINK_STRIDE, NO_ROUTE (+4 more)

### Community 36 - ".transmit"
Cohesion: 0.16
Nodes (6): Brain, flags_level(), level_of(), Actuation, reached(), Outbox

### Community 37 - "RadioConfig"
Cohesion: 0.27
Nodes (6): RadioConfig, Default, Option, Self, Terrain, rssi_falls_with_distance_to_minus_95_at_range()

### Community 38 - "run"
Cohesion: 0.28
Nodes (14): a_gap_reforms_the_tree(), a_lost_segment_waits_for_the_rto(), backlog_coalesces_into_segments(), boot_forms_a_strongest_rssi_tree(), cut_off_subtree_moves_to_a_rooted_ap(), line(), max_conn_forces_a_second_hop(), no_root_election() (+6 more)

### Community 39 - "routing.rs"
Cohesion: 0.17
Nodes (11): crate, DEAD_MEMORY_MS, DEAD_SLACK, ELECT_GRACE_MS, LINK_PENALTY, MAX_HOPS, ORIGIN_REACHED, ORPHAN_ELECT_MS (+3 more)

### Community 40 - "Swarm Mind"
Cohesion: 0.20
Nodes (9): All commands, Deploying, Maps, Prerequisites, Project layout, Quick start, Swarm Mind, Troubleshooting (+1 more)

### Community 41 - ".step"
Cohesion: 0.23
Nodes (9): digits(), DroneView, keys(), median(), reached(), record_mesh_tx(), FnMut, Vec (+1 more)

### Community 42 - "tests/telemetry.rs"
Cohesion: 0.05
Nodes (38): CONNECTED_HOLD_MS, DEPLOY_LIMIT_MS, GCS_WINDOW_MS, S8_DIGEST, S8_RUN_MS, T10_SINK_LIMIT_MS, T10_STATS_DRAIN_MS, T11_BUDGET_100_MS (+30 more)

### Community 43 - "linkbudget.ts"
Cohesion: 0.08
Nodes (33): ref_node_assert, ref_node_test, CH, CHANNEL_LEN, fleetLossText(), fleetLossTone(), HEARD_HZ, MAX_EXTERNAL_BUSY (+25 more)

### Community 44 - "preset_tests.rs"
Cohesion: 0.16
Nodes (24): alpine_ridge_is_bit_identical(), archipelago_has_sea_gaps_shorter_than_a_hop(), at_agl(), bases_and_fires_sit_on_dry_burnable_ground(), burn(), canyon_walls_block_the_floor(), every_preset_fire_spreads_on_land_only(), every_preset_is_deterministic_and_seed_dependent() (+16 more)

### Community 45 - ".new"
Cohesion: 0.24
Nodes (13): carrier_sense_hears_neighbours_within_cs_range_only(), channel_is_deterministic(), config_is_sanitized_and_external_busy_counts(), field_calibration_at_12_mbps_is_clean(), field_calibration_reproduces_the_measured_ceiling(), field_loss(), hidden_terminals_hurt_only_senders_that_cannot_sense_them(), mesh_links_use_the_mesh_floor_not_the_router_minimum() (+5 more)

### Community 46 - ".new"
Cohesion: 0.19
Nodes (18): cfg_on(), json_unescape(), mesh_air(), one_drone(), OneDrone, row(), Routing, Self (+10 more)

### Community 47 - "t9"
Cohesion: 0.24
Nodes (10): FleetLoss, GcsMeter, label(), meter_run(), summ(), t6_fixed_25hz_saturates_alpine(), t7_backend_budget_keeps_alpine_healthy(), t9() (+2 more)

### Community 48 - "hostile_inputs_never_panic"
Cohesion: 0.15
Nodes (11): chatter_heartbeats_match_the_senders_brain(), dead_base_station_rejects_commands(), drones_stay_on_the_map_and_above_ground(), hostile_inputs_never_panic(), ids(), killed_drone_falls_to_the_ground_and_stays(), lossy_radio_still_builds_a_mesh(), neighbors_forget_a_killed_drone() (+3 more)

### Community 49 - "airtime.ts"
Cohesion: 0.14
Nodes (12): AirtimeView, GroundFn, heat(), HEAT_HIGH, HEAT_LOW, HEAT_MID, TREE_COLOR, TREE_STALE (+4 more)

### Community 50 - "LinkBudgetPanel"
Cohesion: 0.26
Nodes (3): el(), LinkBudgetPanel, tag()

### Community 51 - "RadioConsole"
Cohesion: 0.15
Nodes (4): callsign(), el(), fmtTime(), RadioConsole

### Community 52 - "t8"
Cohesion: 0.20
Nodes (12): deploy_or_fail(), painless_subtree(), pick_victim(), Option, Vec, t8(), t8_relay_kill_under_telemetry_alpine(), t8_relay_kill_under_telemetry_archipelago() (+4 more)

### Community 53 - "PainlessMesh"
Cohesion: 0.19
Nodes (3): PainlessMesh, FnMut, Option

### Community 54 - "world.rs"
Cohesion: 0.11
Nodes (23): ladder_relays(), BASE_ID, base_position(), CEILING, Drone, fall(), fly(), GRAVITY (+15 more)

### Community 55 - "tree_is_physical"
Cohesion: 0.29
Nodes (7): ap_loss(), channel_on(), t5_external_busy_costs_delivery(), t5_field_calibration(), t8_tree_is_physical_alpine(), t8_tree_is_physical_canyon(), tree_is_physical()

### Community 56 - ".account"
Cohesion: 0.40
Nodes (3): AirKind, Airtime, expected_attempts()

### Community 57 - "t10_root_death_no_bridge"
Cohesion: 0.40
Nodes (5): Item, Iterator, t10_root_death_no_bridge(), tlm_records(), tlm_routed()

### Community 58 - ".service"
Cohesion: 0.31
Nodes (5): Dir, MeshEvent, MeshTx, Package, Vec

### Community 59 - "Conn"
Cohesion: 0.22
Nodes (7): Conn, LinkState, Node, Self, tcp_timers_follow_lwip(), tick_after(), VecDeque

### Community 60 - "sim.worker.ts"
Cohesion: 0.11
Nodes (15): ref_wasm_swarm_wasm_url, C, CHATTER_STRIDE, chatterKind, Flag, HB, Heartbeat, MI (+7 more)

### Community 61 - "TlmDrone"
Cohesion: 0.23
Nodes (12): BySent, build_frame(), String, state_for(), TelemetryState, TlmDrone, uplink_datagram_shape(), VARIANT (+4 more)

### Community 62 - "scenario_label"
Cohesion: 0.13
Nodes (20): dist_xz(), live_count(), alpine_s0_map_sanity(), c1(), c1_chatter_matches_air(), deploy(), describe(), String (+12 more)

### Community 63 - "resetSession"
Cohesion: 0.15
Nodes (3): clearTlmStats(), resetSession(), TargetMarker

### Community 64 - ".new"
Cohesion: 0.23
Nodes (11): Bench, broadcast_floods_and_relays_count_bytes(), frame_matches_the_firmware_byte_for_byte(), frame_sizes_match_the_field(), ground_station_measures_loss_from_seq_gaps(), on(), phase_locked_25hz_with_two_slow_frames_per_second(), rate_modes() (+3 more)

### Community 65 - "Traffic"
Cohesion: 0.29
Nodes (4): Result, Traffic, Display, Formatter

### Community 66 - "swarm-brain/src/lib.rs"
Cohesion: 0.29
Nodes (6): MAX_INBOX, MAX_NEIGHBORS, MAX_OUTBOX, fixedvec, max_hops, min_hz_as_telemetry_min_hz

### Community 68 - "three"
Cohesion: 0.26
Nodes (3): three, dyn(), SmokePool

### Community 69 - "ChannelConfig"
Cohesion: 0.40
Nodes (4): ChannelConfig, clamp_busy(), Default, sanitize()

### Community 70 - "fire.ts"
Cohesion: 0.18
Nodes (8): FireCell, FromWorker, Intent, INTENT_NAMES, ToWorker, EMBER_SHADER, Place, SMOKE_SHADER

### Community 71 - ".sync_len"
Cohesion: 0.29
Nodes (6): digits(), encode_package(), escape_into(), node_id(), package_escapes_the_inner_json(), String

### Community 72 - ".end_step"
Cohesion: 0.36
Nodes (6): beacon_airtime_us(), cell(), NodeRadio, Terrain, Vec, softap_beacons_cost_airtime_per_alive_node()

### Community 73 - "base-station.ts"
Cohesion: 0.33
Nodes (6): BaseStation, dishGeometry(), glowTexture(), mergeAll(), strut(), tint()

### Community 74 - ".describe"
Cohesion: 0.25
Nodes (4): chatterCount(), flagNames(), slotName(), fmtXZ()

### Community 75 - "switchPreset"
Cohesion: 0.31
Nodes (4): applyLook(), picker, switchPreset(), ScenePicker

### Community 76 - "Routing"
Cohesion: 0.36
Nodes (5): DeadSink, Routing, NodeId, Option, Self

### Community 77 - "fx.ts"
Cohesion: 0.32
Nodes (4): createFx(), Fx, Quality, renderQuality()

### Community 78 - "pulses.ts"
Cohesion: 0.25
Nodes (6): DOWN, dynamic(), Style, STYLES, UNKNOWN, PulseKind

### Community 83 - ".new"
Cohesion: 0.15
Nodes (23): alpine_preset_is_the_default_config(), channel_costs_brain_frames_airtime_and_loss(), chatter_decodes_heartbeats_intents_and_missions(), demo_channel_report(), fire_spreads_in_the_world(), full_fleet_at_25_hz_congests_the_channel(), gradient_reaches_every_drone(), ignite_starts_a_spot_fire_and_ignores_bad_input() (+15 more)

### Community 84 - "Vec3"
Cohesion: 0.10
Nodes (14): Add, FireReading, Sensors, Self, sqrt(), sqrt_is_accurate(), Vec3, .ZERO (+6 more)

### Community 85 - "console.ts"
Cohesion: 0.09
Nodes (22): cache, NATO, parseCallsign(), OPERATOR_ID, ConsoleHost, defaults(), EntryKind, fmtRate() (+14 more)

### Community 86 - "chatter.rs"
Cohesion: 0.12
Nodes (19): CHATTER_HEARTBEAT, CHATTER_INTENT, CHATTER_MISSION, CHATTER_PM_SYNC, CHATTER_PM_TIME, CHATTER_ROOT_UPLINK, CHATTER_STRIDE, CHATTER_TLM (+11 more)

### Community 90 - "Terrain"
Cohesion: 0.22
Nodes (8): PresetInfo, PRESETS, clamp(), paint(), paintStrata(), Terrain, presetFromHash(), writeHash()

### Community 91 - "demos.ts"
Cohesion: 0.07
Nodes (27): Telemetry & airtime model, fleetTlm, median(), Action, BASE_DESTROYED, CHANNEL_SATURATION, Demo, DemoContext (+19 more)

### Community 94 - "CameraDirector"
Cohesion: 0.14
Nodes (7): linkBudget, overview(), selectDrone(), CameraDirector, ease(), ShotKind, SelectionMarker

### Community 97 - "metrics.rs"
Cohesion: 0.27
Nodes (9): MAX_CHAIN, route_ok(), routed_count(), Item, Iterator, Role, scouts_connected(), with_role() (+1 more)

### Community 98 - "SwarmView"
Cohesion: 0.17
Nodes (7): clamp(), clamp01(), droneColor(), lerp(), planeSampler(), SwarmView, wrapAngle()

### Community 103 - "channel.rs"
Cohesion: 0.06
Nodes (34): ACK_BYTES, ACTIVE_EPS, airtime_formulas(), BEACON_BYTES, BEACON_INTERVAL_MS, BUSY_CAP, DIFS_US, DSSS_PLCP_US (+26 more)

## Knowledge Gaps
- **498 isolated node(s):** `MIN_HZ`, `START_HZ`, `EVAL_MS`, `CUT_HOLD_MS`, `CLEAN_BEFORE_RAISE_MS` (+493 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 680 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **5 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Vec3` connect `Vec3` to `World`, `Brain`, `Preset`, `_`, `src/telemetry.rs`, `tests.rs`, `_`, `.suspect`, `Brain`, `.transmit`, `RadioConfig`, `.step`, `preset_tests.rs`, `hostile_inputs_never_panic`, `world.rs`, `scenario_label`, `.end_step`, `Routing`, `channel.rs`?**
  _High betweenness centrality (0.135) - this node is a cross-community bridge._
- **Why does `Sim` connect `Sim` to `World`, `Preset`, `_`, `fire.rs`, `mod.rs`, `Telemetry`, `Channel`, `.step`, `.new`, `t9`, `hostile_inputs_never_panic`, `t8`, `PainlessMesh`, `world.rs`, `t10_root_death_no_bridge`, `scenario_label`, `.new`, `.end_step`, `.new`, `metrics.rs`?**
  _High betweenness centrality (0.064) - this node is a cross-community bridge._
- **Why does `_` connect `_` to `airtime.rs`, `Brain`, `_`, `swarm-brain/src/lib.rs`, `Preset`, `routing.rs`, `tests.rs`, `Vec3`?**
  _High betweenness centrality (0.041) - this node is a cross-community bridge._
- **What connects `MIN_HZ`, `START_HZ`, `EVAL_MS` to the rest of the system?**
  _498 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Snapshot` be split into smaller, more focused modules?**
  _Cohesion score 0.1349206349206349 - nodes in this community are weakly interconnected._
- **Should `Brain` be split into smaller, more focused modules?**
  _Cohesion score 0.09041835357624832 - nodes in this community are weakly interconnected._
- **Should `Preset` be split into smaller, more focused modules?**
  _Cohesion score 0.06801346801346801 - nodes in this community are weakly interconnected._