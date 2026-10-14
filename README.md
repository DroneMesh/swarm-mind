# Swarm Mind

A browser demo of a self-healing drone swarm monitoring a wildfire over 3D terrain.

Every drone runs its own small Rust brain: `no_std`, no allocator, about 2.6 KB of state, and it builds for an ESP32-C3. It knows only its own sensors and the radio frames it hears. You give one high-level command ("monitor the fire"); the drones work out roles, relay chains, repairs and a new base on their own. Click any drone to kill it and watch the swarm route around the loss.

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| Rust (via [rustup](https://rustup.rs)) | stable, edition 2024 (1.85+) | `rustup target add wasm32-unknown-unknown` |
| Node.js | 22+ (Vite 8) | npm comes with it |
| ESP32 target (optional) | — | `rustup target add riscv32imc-unknown-none-elf` |

```bash
# One-time setup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh   # if Rust is missing
source "$HOME/.cargo/env"
rustup target add wasm32-unknown-unknown riscv32imc-unknown-none-elf
cd web && npm install
```

## Quick start

```bash
cd web
npm run dev        # builds the Rust brain + sim to WASM, then starts Vite
```

Open the URL Vite prints (usually http://localhost:5173). The dev and preview servers listen on `0.0.0.0`, so other devices on your local network can use the **Network** URL Vite prints too (allow port 5173 through your firewall if needed). After a couple of seconds the swarm launches automatically with **Monitor fire**.

## All commands

Run from `web/`:

| Command | What it does |
|---|---|
| `npm run dev` | Build WASM, start the dev server with hot reload |
| `npm run build` | Build WASM, type-check, produce a static site in `web/dist/` |
| `npm run preview` | Serve the built `web/dist/` locally |
| `npm run wasm` | Rebuild only the WASM (`web/src/wasm/swarm.wasm`) |
| `npm run typecheck` | TypeScript check only |
| `npm test` | Web unit tests (`web/tests`, node --test) |

Run from the repo root (`source "$HOME/.cargo/env"` first if `cargo` isn't found):

| Command | What it does |
|---|---|
| `cargo test --workspace --release` | All unit tests plus the scenario suite (every map) |
| `cargo test --release -p swarm-sim --test scenarios -- --nocapture` | Scenario suite with its recovery-time and traffic numbers |
| `cargo clippy --workspace --all-targets` | Lints (expected: 0 warnings) |
| `cargo brain-mcu` | Build the drone brain for bare-metal ESP32-C3 (`riscv32imc-unknown-none-elf`) |
| `cargo test --release -p swarm-sim step_timing -- --ignored --nocapture` | Simulation step timing at 20 / 100 / 256 drones |

## Using the demo

- **Orbit / zoom / pan:** drag, scroll, right-drag.
- **Kill a drone:** click it (the base station is protected), or press **Kill a relay**.
- **Commands** (top right): Hold, Monitor fire, Map perimeter, Return to base. **Shift+click** the ground to set a custom target.
- **Packet loss** slider: 0–60%. **Reset** starts a fresh swarm.
- **FX HIGH / LOW:** toggles bloom and searchlights. It drops to LOW automatically if fps falls.
- **Radio console** (bottom, toggle with <kbd>`</kbd>): live decoded radio traffic between drones by call sign (OVERWATCH is the base; ALPHA-1, BRAVO-1, … are drones).
  - **Filters:** message type (HB / MISSION / INTENT / SYSTEM, and AIR: TLM / PM-SYNC / PM-TIME / UPLINK), role, a call-sign search box, **Changes only**, Pause and Clear.
  - **Follow a drone:** click a call sign to follow that drone with the camera.
- **Showcase** bar: one-click demos with captions and camera moves: Relay down, Base destroyed, Radio storm, Fire jumps, Mass casualty, Channel saturation, Full tour.
- **Link budget** (left): telemetry and airtime per drone, with controls. See [Telemetry & airtime model](#telemetry--airtime-model). <kbd>Esc</kbd> cancels a demo or a follow.

### Maps

Pick one from the **MAP** selector, or open the URL with a hash:

| Hash | Map | Radio challenge |
|---|---|---|
| `#map=alpine` | Alpine Ridge | relays must climb over a snowy ridge |
| `#map=canyon` | Canyon | canyon walls block the radio, so relays hold the rim |
| `#map=twin-valleys` | Twin Valleys | two ridges, so a longer relay chain |
| `#map=archipelago` | Archipelago | fire on an island across open water |
| `#map=foothills` | Foothills | easy baseline |

## Telemetry & airtime model

The brain's frames are small. A real drone also streams its firmware telemetry, and on a real 2.4 GHz channel that stream is most of the traffic. The demo models it so you can read off the byte size and update rate per drone, and see what happens when the air runs out.

**What is sent.** Each drone builds the same telemetry JSON as `drone_mavlink_wifi.ino`. A FAST frame goes out every telemetry interval (25 Hz = 40 ms). Every 500 + (id % 97) ms, the SLOW keys (battery, modes, link counters) ride inside one of the fast frames. There are two firmware variants: **XIAO S3** (no GNSS keys) and **C6 RTK** (adds the `g*` GNSS keys). **Mixed** makes odd ids C6. The frame is wrapped in a painlessMesh 1.5.7 package, `{"type":9,"dest":…,"from":…,"msg":"<escaped JSON>"}`, which is about 260 B for an S3 fast frame. **Single** sends it to the root. **Broadcast** (type 8) is flooded by every node. The packages travel over TCP hop by hop along painlessMesh's own tree, which is separate from the brain's routing. OVERWATCH is the root and the bridge: it republishes each package as one UDP datagram (`drone/drone-<id>/data`) to a WiFi station on the same channel. If the root dies, telemetry has no bridge, because painlessMesh has no root election.

**Shared airtime.** The following all use the same channel:

- brain frames (ESP-NOW style)
- telemetry TCP segments, plus 802.11 and TCP ACKs
- painlessMesh NODE_SYNC and TIME_SYNC traffic
- the root's uplink

Each node's busy fraction is the airtime of everything it can hear, plus the **External airtime** slider (0–80 %, airtime used by other networks). Loss, backoff and queue drops follow from the busy fraction and apply to brain frames and telemetry alike. The calibration targets come from the field: ~70 pps on one AP gives ~1 % loss, and 6 nodes × 25 Hz (~150 pps of ~400 B frames) gives 26–32 % loss.

**Rate modes.**

- **Fixed:** every drone sends at the slider's rate.
- **Budget:** the backend rule. Hz = clamp(60 pps / online nodes, 5, 25). AIMD on the fleet-median loss: ×0.8 every 10 s while loss is above 10 %, +5 pps per 30 s after it stays below 5 %.
- **Brain:** Phase 5. Each drone's brain sets its own telemetry interval.

**In the demo** (all on by default: XIAO S3, Single, 25 Hz fixed, channel on, external 0 %):

- **Link budget** panel (left; ▾ folds it, CTL hides the controls). The fleet header shows the telemetry packages/s offered to the WiFi station (senders × Hz, the unit the field measured) against the 60 pps clean budget and the ~150 pps collapse point, and separately every frame on air (mesh hops, ACKs, sync, brain frames, uplink), mean and max busy, ground-station B/s and pps, the backend budget, the rate mode and whether the bridge is up. The sortable per-drone table shows: requested Hz, Hz at the GCS, loss (from gaps in `s`; `none` when no package arrives, so there is nothing to measure; such drones are left out of the fleet median and counted as "not heard", which turns the header amber or red), latency, telemetry and relay B/s, own airtime, busy and tree hops. Green, amber and red mark healthy, strained and failing values. Click a row to follow that drone. With the channel model off, the airtime figures show `—` (airtime is only counted with the model on); with both models off the panel and HUD say so instead of keeping the last numbers. TLM console lines show a package's text only when it is that very package (same `s`), otherwise just `s=<seq>`.
- **Map:** dashed orange lines show the painlessMesh tree. A dashed red line means the parent is already down but painlessMesh has not timed it out yet. Ground discs grow and redden with the busy fraction. **MAP** toggles the tree, the brain links and the heat. The drone tooltip adds the telemetry stats.
- **Radio console:** the **AIR** filters are TLM, PM-SYNC, PM-TIME and UPLINK. A TLM line shows the exact package text, sampled once per second per drone, and hovering the line shows all of it.
- **HUD:** the `radio` line shows brain bytes per drone. The `tlm` line shows telemetry bytes per drone, Hz at the GCS and channel busy.
- **Showcase · Channel saturation:**
  1. The backend budget is in force, then everyone is forced to 25 Hz.
  2. The captions quote the measured Hz at the GCS, loss and busy.
  3. The drone that forwards the most telemetry is killed. The captions compare the brain's heal time with painlessMesh's reroute (NODE_TIMEOUT 10 s plus rescan).
  4. The demo switches to brain-managed rate and adds 30 % external airtime, then reports the result.

For every function, data layout, model and test, see [docs/SIMULATION.md](docs/SIMULATION.md).

## Deploying

The site is fully static: no server, no special headers. Run `npm run build` and upload `web/dist/` to any static host (GitHub Pages, Netlify, S3 and so on). Asset paths are relative, so it also works from a subfolder.

## Project layout

```
crates/
  swarm-brain/   no_std per-drone brain: wire format, routing, roles, repair, formation planning
  swarm-sim/     world simulation: terrain presets, radio (range, line of sight, loss), fire, metrics
    tests/scenarios.rs   acceptance suite (relay kills, partitions, base loss, packet loss…) on every map
  swarm-wasm/    C-ABI WASM exports used by the web worker
web/
  src/sim.worker.ts      runs the simulation in a Web Worker (50 ms steps)
  src/main.ts            scene, HUD, controls
  src/render/            drones, base station, terrain, fire, radio pulses, post-effects
  src/ui/                radio console, demos, camera director, map picker
.claude/PRPs/prds/swarm-mind.prd.md   product requirements and success metrics
```

## Troubleshooting

- **`cargo: not found` during `npm run dev`:** run `source "$HOME/.cargo/env"` in that shell, or restart the terminal after installing rustup.
- **`can't find crate for core` / wasm target errors:** run `rustup target add wasm32-unknown-unknown`.
- **Stale behaviour after editing Rust:** `npm run dev` rebuilds the WASM only at startup. Restart it, or run `npm run wasm`.
- **Port already in use:** `npm run dev -- --port 5174`.
- **Disk space:** the `target*/` folders at the root are Rust build caches; any of them can be deleted safely.
