# Swarm Mind — Self-Healing Wildfire Drone Swarm

## Problem Statement

Portfolio viewers (recruiters, engineers) can't easily judge whether someone truly understands decentralized/distributed systems — static write-ups and 2D flocking demos don't prove it. Claims like "the swarm self-heals" are unconvincing unless the viewer can break the system themselves and watch it recover. Without a hands-on, visually legible demo, that skill stays invisible.

## Evidence

- Existing browser swarm tools are research-oriented and visually minimal: IoFT web visualizer (spheres on black mesh, Python backend), Pera-Swarm visualizer, SwarmJS (2D), DroneMD (Three.js flocking, no comms visualization). None combine terrain, a mission, visible messaging, and user-induced failure.
- Research supports the premise that fully distributed swarms can approach centralized performance (arXiv 1905.00492) and that fault-tolerant reassignment matters (Hu et al. 2022).
- Assumption — that an interactive "break the relay" moment is more persuasive than video — needs validation through informal viewer feedback.

## Proposed Solution

A static web app: a Three.js scene of procedural mountain terrain with a spreading wildfire, monitored by a drone swarm. Each drone runs an **independent Rust `no_std` brain** (no heap, bounded buffers, ESP32-portable) compiled to WASM and executed in a Web Worker. Brains share no memory; the only coupling is a simulated radio that enforces range, terrain line-of-sight, loss, and latency. The operator issues high-level **intents** ("Monitor fire", "Map perimeter", "Hold", "Return to base"); drones decide roles (scout / relay / reserve) themselves. Every message is rendered as a pulse along a link, and the user can click any drone to kill it. Chosen over a JS-only sim (not portable, less credible) and over a native Rust server (breaks static hosting).

## Key Hypothesis

We believe a visibly decentralized swarm that self-heals after the viewer kills its relay will demonstrate distributed-systems competence to portfolio viewers.
We'll know we're right when any single-relay loss is repaired (base↔scout connectivity restored) in ≤ 5 s sim-time with ≤ 2 msgs/drone/s average, at 60 fps.

## What We're NOT Building

- Real hardware flashing — the brain is portable by construction, but the demo doesn't ship firmware.
- Realistic flight dynamics / RF propagation — kinematic motion + LOS/range radio is enough to make the logic honest.
- Mobile support — laptop browser only.
- Multiplayer / server-hosted simulation — static hosting only.

## Success Metrics

| Metric | Target | How Measured |
|--------|--------|--------------|
| Relay-loss recovery time | ≤ 5 s sim-time | Scenario test harness (native Rust) + in-app HUD |
| Message efficiency | ≤ 2 msgs/drone/s steady state | Radio counters in sim |
| Frame rate | 60 fps, 20+ drones (stretch 100) | Chrome perf trace on mid-range laptop |
| Brain footprint | ≤ 16 KB state per drone, no alloc | `no_std` build for `riscv32imc-unknown-none-elf` + `size_of` asserts |
| Scenario pass rate | 100% of defined scenarios | `cargo test` scenario suite |

## Open Questions

- [ ] Drone count target for the default scene (20 vs 50)?
- [ ] Should the base station be killable, and does the swarm then elect a sink?
- [ ] Do we show a "how it works" panel citing algorithms/research?
- [ ] Fixed-point vs f32 in the brain (ESP32-C3 has no FPU)?

---

## Users & Context

**Primary User**
- **Who**: Recruiter or engineer browsing a portfolio on a laptop.
- **Current behavior**: Skims READMEs/videos for 30–90 s.
- **Trigger**: Lands on the project link.
- **Success state**: Has killed a relay, watched recovery, and understood why it worked.

**Job to Be Done**
When I review this portfolio, I want to break something and see it recover, so I can judge whether this person understands distributed systems.

**Non-Users**
Researchers needing physics/RF-accurate simulation; real drone operators; mobile viewers.

---

## Solution Detail

### Core Capabilities (MoSCoW)

| Priority | Capability | Rationale |
|----------|------------|-----------|
| Must | Rust `no_std` per-drone brain (`tick(sensors, inbox) -> (cmd, outbox)`) | Core credibility claim; ESP32-portable |
| Must | Radio sim: range, terrain LOS, loss, latency; the only inter-drone channel | Makes relays necessary and logic honest |
| Must | Kill-a-drone + automatic relay repair | The demo's money moment |
| Must | Intent commands propagated through the mesh | "We give the command, they figure it out" |
| Must | Terrain + spreading fire (slope/wind CA) + orbit camera | Mission context |
| Must | Visualized links + message pulses, color by message type | Shows communication |
| Should | Partition detection & merge, sink re-election | Covers multi-failure scenarios |
| Should | HUD: msgs/s, connectivity, recovery timer, per-drone role | Quantifies "efficient" |
| Could | Packet-loss slider, wind control, speed control | Lets viewers stress it |
| Could | Event log narrating decisions ("D7: parent lost → requesting relay") | Explains reasoning |
| Won't | Firmware build / ESP-NOW driver | Out of scope for v1 |

### MVP Scope

~20 drones + base station, procedural mountains, spreading fire, "Monitor fire" intent, visible links/pulses, click-to-kill, single-relay self-heal, 60 fps.

### User Flow

Open page → swarm launches from base and auto-forms a relay chain over the ridge to the fire → viewer clicks the highlighted relay → links drop, orphan alerts pulse → a reserve drone claims the gap → chain restored, HUD shows recovery time.

### Required Scenarios

1. Single relay killed
2. Multiple simultaneous kills
3. Partition → merge
4. Base station offline
5. Fire spreads beyond coverage
6. New intent issued mid-repair
7. Packet-loss spike

---

## Technical Approach

**Feasibility**: MEDIUM-HIGH

**Architecture Notes**
- Cargo workspace: `swarm-brain` (`no_std`, `heapless`, zero deps beyond that), `swarm-sim` (world, radio, fire, physics; std), `swarm-wasm` (wasm-bindgen bridge).
- Web: Vite + TypeScript + Three.js. Sim runs in a Web Worker at 10–20 Hz; renderer interpolates at 60 fps. Snapshots transferred as ArrayBuffers (no SharedArrayBuffer — static hosts can't set COOP/COEP).
- Rendering: InstancedMesh drones, one LineSegments buffer for links, instanced points for pulses, heightmap terrain, emissive fire texture from CA grid.
- Brain algorithms: adaptive heartbeats; hop-count gradient to sink with parent pointer; timeout-based parent-loss detection; local repair request + auction for relay slot; sector auctions for perimeter coverage; lowest-ID leader per partition; intent flooding with sequence numbers + dedupe (bounded).
- Message wire format: fixed ≤ 64-byte packed structs (fits ESP-NOW 250 B).

**Technical Risks**

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| "All scenarios" scope creep | H | Fixed scenario list above, each a `cargo test` |
| Emergent behavior oscillates (relay thrash) | M | Hysteresis, claim timeouts, deterministic seeded tests |
| Worker↔main transfer cost at 100 drones | L | Packed Float32 snapshots, transferables |
| ESP32-C3 lacks FPU | M | Keep math simple; evaluate fixed-point later |
| WASM toolchain friction | L | wasm-pack + vite-plugin or prebuilt pkg |

---

## Implementation Phases

| # | Phase | Description | Status | Parallel | Depends | PRP Plan |
|---|-------|-------------|--------|----------|---------|----------|
| 1 | Scaffold & contracts | Workspace, crates, Vite app, message/snapshot formats | complete | - | - | - |
| 2 | Brain core | Neighbors, heartbeats, gradient routing, intents (native tests) | complete | with 3, 4 | 1 | - |
| 3 | World sim | Terrain heightmap, LOS radio, fire CA, kinematics | complete | with 2, 4 | 1 | - |
| 4 | Renderer | Terrain, fire, drones, links, pulses, camera (mock data) | complete | with 2, 3 | 1 | - |
| 5 | Integration | WASM worker bridge, click-to-kill, intent UI | complete | - | 2, 3, 4 | - |
| 6 | Swarm intelligence | Relay repair, auctions, partitions, scenario suite | complete | - | 5 | - |
| 7 | Polish & ship | HUD, event log, perf pass, static deploy | in-progress | - | 6 | - |

### Phase Details

**Phase 1: Scaffold & contracts** — Goal: buildable skeleton. Scope: Cargo workspace, `Brain::tick` signature, `Message` enum, snapshot layout, Vite+Three page. Success: `cargo test` + `npm run dev` both run.

**Phase 2: Brain core** — Goal: drones know neighbors and route to sink. Success: native tests show gradient converges on a line/grid topology.

**Phase 3: World sim** — Goal: honest physical environment. Success: LOS blocks across ridges; fire spreads uphill faster.

**Phase 4: Renderer** — Goal: the look. Success: 60 fps with 100 mock drones + links + pulses.

**Phase 5: Integration** — Goal: real brains drive visuals. Success: clicking a drone kills it in the sim.

**Phase 6: Swarm intelligence** — Goal: self-healing + mission. Success: all 7 scenarios pass in tests and look right in-app.

**Phase 7: Polish & ship** — Goal: portfolio-ready. Success: deployed static URL, HUD metrics meet targets.

### Parallelism Notes

After contracts are fixed in Phase 1, brain (pure logic), world sim (environment), and renderer (consumes snapshot format with mock data) touch disjoint code and can proceed concurrently.

---

## Decisions Log

| Decision | Choice | Alternatives | Rationale |
|----------|--------|--------------|-----------|
| Brain language | Rust `no_std` | JS, std Rust | ESP32-portable, credible "real firmware logic" |
| "Backend" | WASM in Web Worker | Native Rust server + WebSocket | Keeps static hosting |
| Render stack | Vite + TS + vanilla Three.js | React Three Fiber | Less overhead, direct perf control |
| Inter-drone coupling | Simulated radio only | Shared world state | Prevents cheating; honest decentralization |
| Control model | Intents, not waypoints | Direct drone control | "Mind of their own" |

---

## Research Summary

**Market Context**
No interactive browser demo combines terrain, mission, visible comms, and user-induced failure. Closest: IoFT visualizer (arXiv 2403.12753), Pera-Swarm visualizer, SwarmJS, DroneMD. Relevant research: distributed wildfire coalitions (arXiv 1905.00492), graph-based fault-tolerant UAV navigation (Hu et al. 2022), BYU perimeter monitoring with limited comms.

**Technical Context**
Greenfield (empty repo). Node 24 + pnpm available; Rust toolchain not yet installed. ESP-NOW payload limit (250 B) informs ≤ 64 B message design.

---

*Generated: 2026-10-10*
*Status: DRAFT - needs validation*
