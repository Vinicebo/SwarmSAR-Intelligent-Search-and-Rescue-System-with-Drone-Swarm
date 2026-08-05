# Phase 1 Retrospective — Single Drone Simulation

Phase 1 goal (per the [project roadmap](../README.md#development-roadmap)): set up ROS 2, Gazebo, and a single drone. This document records what was actually built, and — more importantly — what had to be revised after the initial commits and why, so the same mistakes aren't repeated in Phase 2.

## What Was Delivered

- **Workspace and package scaffolding** — 9 ROS 2 packages under `src/` (`communication`, `navigation`, `formation`, `mapping`, `planning`, `search`, `battery`, `collision`, `simulator`), each with a valid `package.xml`/`setup.py` so `colcon build` recognizes them.
- **Documentation** — [README.md](../README.md), [docs/setup.md](setup.md), [docs/build.md](build.md), [docs/architecture.md](architecture.md), [docs/algorithms.md](algorithms.md).
- **Simulation world** — [worlds/earthquake_city.world](../worlds/earthquake_city.world): ground plane, sun, and the `gz-sim` system plugins (Physics, Sensors, Imu, NavSat, ApplyLinkWrench, SceneBroadcaster, UserCommands) needed for sensors and force actuation to work at all.
- **Drone model** — [models/quadrotor/model.sdf.template](../models/quadrotor/model.sdf.template): a box-body quadrotor with IMU, GPS (navsat), LiDAR (gpu_lidar), and camera sensors.
- **Hover controller** — [src/navigation/navigation/hover_controller.py](../src/navigation/navigation/hover_controller.py): a closed-loop PID on altitude (fed by GPS), applying force via `EntityWrench`.
- **Launch infrastructure** — [launch/single_drone.launch.py](../launch/single_drone.launch.py) and [launch/swarm.launch.py](../launch/swarm.launch.py), plus [src/simulator/simulator/spawn_helpers.py](../src/simulator/simulator/spawn_helpers.py) for per-drone SDF/bridge templating.

## What Changed After the Initial Commits, and Why

The initial commits (`644a21d` workspace init, `4a01df5` Phase 1 single-drone setup) targeted **ROS 2 Humble + Gazebo Classic**, since that was the best-documented, most stable combination at the time of writing. Three rounds of revision followed once real constraints surfaced:

### 1. Wrong ROS 2 / Gazebo distro for the actual OS (`009f2ba`)

The user's machine runs **Ubuntu 26.04 "Resolute Raccoon"**, which pairs with **ROS 2 Lyrical Luth + Gazebo Jetty (`gz-sim`)** — not Humble/Gazebo Classic, which is deprecated. Everything Gazebo-Classic-specific had to be rewritten:

| Humble / Gazebo Classic | Lyrical / gz-sim (Jetty) |
|---|---|
| `gazebo_ros_pkgs` plugins per sensor (`libgazebo_ros_imu_sensor.so`, etc.) | Native `<sensor>` tags, processed by world-level systems (`Imu`, `NavSat`, `Sensors`) |
| `libgazebo_ros_force.so` + `geometry_msgs/Wrench` | `ApplyLinkWrench` system + `ros_gz_interfaces/EntityWrench` |
| `gazebo.launch.py` + `spawn_entity.py` | `ros_gz_sim`'s `gz_sim.launch.py` + `create` executable |
| Separate OSRF apt repo | Gazebo ships as ROS vendor packages (`ros-lyrical-ros-gz*`) — no separate repo needed |

**Lesson**: don't assume the best-documented stack matches the user's actual environment — check the OS/tool versions first. This is now standard practice for this project (see [docs/setup.md](setup.md)).

### 2. Force control was silently broken (`928685e`)

The first working `gz-sim` version of the hover controller applied force via the **instantaneous** `wrench` topic, which only holds for a single physics step. At 20 Hz control vs 250 Hz physics, gravity went unopposed most of the time and the drone fell instead of hovering.

Fix required three coordinated changes, not just one:
- Switch to `wrench/persistent` (force stays applied across steps).
- Realize `wrench/persistent` is **additive**, not absolute — track the last commanded force and publish the *delta*, or the applied force runs away unboundedly.
- Add `<spherical_coordinates>` to the world, because `NavSatFix.altitude` reads `0` without a WGS84 origin — the PID had nothing real to control against.

Also fixed in the same pass: `use_sim_time` wasn't set on the controller (wall clock doesn't track sim time reliably), and the PID's derivative term was computed against a fixed timer period instead of the real measured time between GPS readings (the sensor's actual rate is the loop's real sample rate).

Full technical detail lives in [docs/simulation_notes.md](simulation_notes.md) — read it before writing any new force- or timing-sensitive controller.

### 3. Drone identity was hardcoded, which would have broken Phase 2 silently

After the hover fix, `drone_1` was still hardcoded in three places: the model's sensor topics, the bridge config, and the controller's target entity name. Spawning more than one drone this way would have every controller apply force to the same `drone_1::base_link`, and every drone's sensors publish on the same topic.

Fixed by templating:
- `model.sdf` → `model.sdf.template` with `{drone_id}` placeholders.
- `ros_gz_bridge.yaml` → `ros_gz_bridge.yaml.template`, with the world-level `clock` bridge split out into its own static `config/clock_bridge.yaml`.
- `spawn_helpers.py` renders both per drone at launch time.
- `hover_controller.py` now reads `drone_id` as a ROS parameter instead of a module constant.
- `launch/swarm.launch.py` added, spawning N drones (`num_drones:=` launch arg, default 30) each with its own rendered SDF/bridge/controller.

### 4. Only `drone_1` actually hovered in a real `num_drones:=30` run (`6a2cbcf` and follow-up)

The first real end-to-end run surfaced two more issues the paper review above missed:

- **Per-drone force topics didn't scale.** Templating gave every drone its own `{drone_id}/force` ROS topic, but all of them bridged to the same world-level `wrench/persistent` gz topic. `parameter_bridge` can only advertise a given gz topic once per process — every bridge entry after the first failed silently (`Node::Advertise(): Error advertising topic ...`), so only `drone_1`'s force commands ever reached Gazebo. Fixed by moving to one shared `drone_force` ROS topic (bridged once, in `config/world_bridge.yaml.template`), the same pattern `/swarm_state` already used — Gazebo routes each `EntityWrench` to the right drone via the message's `entity` field, not the ROS topic name. See [docs/simulation_notes.md](simulation_notes.md#a-gz-topic-can-only-be-bridged-once-per-process).
- **The force bridge's world name was still hardcoded** to `earthquake_city` in the template even after drone-id templating, so `swarm.launch.py world:=<anything else>` would have silently broken force actuation the same way. Now templated as `{world}` and rendered from the launch file's actual `world` argument.

Fixing the shared-topic issue then surfaced two more, only visible with two real controllers running against the same live Gazebo instance:

- **A restarted controller inherited a stale force baseline.** One drone's controller crashed (see next point) and was relaunched; its fresh `applied_force_z = 0.0` no longer matched what was actually still standing on its entity in Gazebo, so its delta math was permanently offset and the drone settled into an arbitrary frozen altitude instead of 3 m. Fixed by clearing the entity's standing force at the start of every controller instance (repeated for a short window, not a single message, to survive bridge/discovery startup latency) — see `STARTUP_CLEAR_TICKS` in `hover_controller.py`.
- **A `ZeroDivisionError` crashed a controller outright** when two GPS readings landed with the same (or an out-of-order) simulated timestamp — something that didn't show up with only one drone's timing on the host. Fixed with a `dt <= 0` guard that skips the update instead of dividing.

See [docs/simulation_notes.md](simulation_notes.md) for the technical detail on both.

## What's Still Untested

The fixes above have been verified against two real controllers running concurrently against one live (headless) Gazebo instance, including one deliberately crashing and restarting mid-flight — both converged cleanly to 3 m with no oscillation. Not yet verified: the full 30-drone scale, the full sensor suite (camera/lidar) under GUI rendering, and any of this on the user's actual machine rather than this sandboxed environment. Before starting further Phase 2 communication work, the practical next step is:

```bash
ros2 launch simulator single_drone.launch.py     # confirm hover still works after templating
ros2 launch simulator swarm.launch.py num_drones:=5   # small-scale swarm smoke test before 30
```

Any errors from these runs should be reported back before building the P2P communication layer on top.
