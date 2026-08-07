# Gazebo / gz-sim Simulation Notes

Lessons learned while building the Phase 1 hover controller. Read this before writing any new controller that applies forces, reads sensors, or spawns entities — the same mistakes are easy to repeat when scaling to multiple drones.

## Wrench Topics: `wrench` vs `wrench/persistent`

`gz-sim`'s `ApplyLinkWrench` system exposes two topics per world:

- `world/<world>/wrench` — applies the wrench for a **single physics step only**. If your control loop runs slower than the physics step (it almost always does — e.g. 20 Hz control vs 250 Hz physics), gravity goes unopposed most of the time and the entity falls.
- `world/<world>/wrench/persistent` — the wrench **stays applied** across steps until changed. Use this for continuous force control (hover, thrust, etc.).

**Critical gotcha**: `wrench/persistent` is *additive* — each message adds to the entity's existing standing force, it does not replace it. Publishing the same absolute force every tick makes the applied force grow without bound. Always track the last commanded value and publish the **delta**:

```python
delta = new_force - self.applied_force
self.applied_force = new_force
# publish delta, not new_force
```

## GPS Altitude Requires `spherical_coordinates`

The bridged `NavSatFix.altitude` field reads `0` for every position unless the world defines `<spherical_coordinates>` (with a reference `EARTH_WGS84` origin). Without it, any altitude-based controller silently gets a constant zero and never converges.

## Control Loops Need `use_sim_time`

Wall-clock (`ROS_TIME`) does not reliably track simulated time, especially when Gazebo runs slower/faster than real time. Any node that computes a derivative or integral against elapsed time must:

1. Set `use_sim_time: true` as a node parameter (in the launch file).
2. Have a `/clock` topic bridged from Gazebo (`GZ_TO_ROS`, `rosgraph_msgs/msg/Clock`).

Without both, `self.get_clock().now()` silently falls back to wall time and the loop's timing assumptions break.

## Derivative Terms: Use Real Sensor Timing, Not the Publish Timer

A control loop's *effective* sample rate is set by its slowest input, not by how often you publish output. The hover PID computes its integral/derivative on GPS message arrival (10 Hz) using the actual measured `dt` between readings — not the 20 Hz publish timer. Using the timer period instead exaggerates the derivative whenever the sensor and timer rates drift out of sync, which destabilizes the loop.

## Hardcoded Entity Names Don't Survive Multiple Drones

The original hover controller hardcoded `'drone_1::base_link'`, and the model/bridge config hardcoded `drone_1/*` topic names. That breaks silently the moment more than one drone is spawned — every controller instance targets the same entity, and sensor topics collide across drones.

Fixed by templating:

- `models/quadrotor/model.sdf.template` — sensor `<topic>` tags use `{drone_id}` instead of a fixed name.
- `config/ros_gz_bridge.yaml.template` — same `{drone_id}` templating for the per-drone bridge entries; world-level bridges (`clock`, `drone_force`) live in their own `config/world_bridge.yaml.template`, rendered once, since they're per-world, not per-drone.
- `src/simulator/simulator/spawn_helpers.py` — renders a filled-in SDF/bridge-config per drone at launch time (`render_drone_sdf`, `render_bridge_yaml`), used by both `launch/single_drone.launch.py` and `launch/swarm.launch.py`.
- `hover_controller.py` — reads `drone_id` as a ROS parameter and builds the target `Entity` from it in `__init__`, instead of a module-level constant.

Any new controller that needs to know which drone it's running on should follow the same pattern: a `drone_id` ROS parameter (or namespace-derived value), never a hardcoded string.

## A gz Topic Can Only Be Bridged Once Per Process

The first swarm attempt gave every drone its own `{drone_id}/force` ROS topic, each bridged to the *same* `world/<world>/wrench/persistent` gz topic. Only `drone_1` ever hovered — every other drone's bridge entry failed with `Node::Advertise(): Error advertising topic ... Did you forget to start the discovery service?`, logged but easy to miss among 30 drones' worth of startup output. `parameter_bridge` can't create more than one ROS→GZ advertisement for the same gz topic within one process, no matter how many different ROS-side topic names point at it.

Fixed the same way `/swarm_state` already worked: **one shared ROS topic** (`drone_force`, bridged once in `config/world_bridge.yaml.template`) that every drone's hover controller publishes to (remapped from the node-local `force` to the absolute `/drone_force` in the launch files). Gazebo routes each `EntityWrench` to the right drone via the message's own `entity` field — the ROS topic doesn't need to be per-drone for that to work, only the bridge complained.

**Rule of thumb**: any world-level gz resource (wrench topics, and probably world control/marker topics later) needs exactly one bridge entry, shared across all drones, with per-entity routing handled by message content — not one bridge entry per drone.

## Delta Tracking Breaks Across Controller Restarts

Publishing deltas onto `wrench/persistent` (see above) only stays correct if the same process has been the *sole* publisher for that entity since its standing force was last zero. A second drone crashing (e.g. from the `ZeroDivisionError` below) and being relaunched starts a fresh `applied_force_z = 0.0` that no longer matches whatever force the crashed instance actually left standing in Gazebo — the new instance's deltas are now offset from reality by an unknown amount, and the drone settles into a random, uncontrolled equilibrium instead of the setpoint.

Fixed by clearing the entity's standing force at the start of every controller instance's life (`world/<world>/wrench/clear`, `gz.msgs.Entity`/`ros_gz_interfaces/msg/Entity`) before trusting `applied_force_z = 0.0`, repeated for `STARTUP_CLEAR_TICKS` ticks rather than sent once — a single message published in `__init__` has no guarantee of arriving before bridge/discovery finishes connecting.

## Guard Against Zero/Negative `dt` in Timing-Based Control

Two GPS readings can land back-to-back with the same (or, if queued out of order, an earlier) simulated timestamp — this showed up as a `ZeroDivisionError` in the derivative term as soon as a second drone was added to the same process/host, not with a single drone. Any control loop dividing by measured `dt` needs `if dt <= 0: return` (skip the update, keep the last good `previous_error`/`previous_time`) before doing the division — don't assume consecutive sensor messages always have strictly increasing timestamps.

## Spawn-vs-Controller Race With Delta-Tracked Force

Spawning a drone (`ros_gz_sim create`) and starting its controller node happen in parallel launch actions with no ordering guarantee. With 30 drones, Gazebo processes the spawn requests one at a time, so the last few entities can take a noticeable moment to actually exist.

This collided badly with the delta-tracking pattern above: the hover controller used to start its force-publish timer immediately in `__init__`. If the very first (largest) message — the one compensating gravity — was sent before the entity existed, it was silently dropped. But the controller's internal `applied_force_z` was still updated as if it had landed, permanently desyncing the node's belief about the standing force from what Gazebo actually had (zero). Every following delta was computed against that false baseline, so the real applied force stayed near zero and the drone never left the ground — intermittently, for whichever few drones happened to lose the race that run.

**Fix**: don't start the force timer until you have positive proof the entity is alive. A sensor reading (GPS, IMU — anything attached to the entity) only exists once the entity and its sensors are live in the scene graph, so gate the timer on the first sensor callback instead of starting it in `__init__`. Any future controller that both (a) depends on the target entity already existing and (b) tracks cumulative/delta state must use the same gate — starting stateful, fire-and-forget publishing before the target is confirmed to exist is the general failure mode here, not something specific to hovering.

## A Single Bridge Process Has a Bandwidth Ceiling — Unused Sensors Still Cost Bandwidth

Below ~19 drones, everything hovered reliably. At 30, several drones never left the ground at all — same symptom as the spawn race above, but that race was already fixed, and the failure now scaled with drone count rather than being a one-off timing fluke. That points at a resource ceiling, not an ordering bug.

The camera sensor was the culprit: uncompressed `sensor_msgs/Image` at 640×480/30Hz is roughly 27 MB/s per drone, uncompressed, all flowing through the single `parameter_bridge` process (one process bridges every drone's topics — see "A gz Topic Can Only Be Bridged Once Per Process" above for why it's one process at all). At 30 drones that's on the order of 800 MB/s through one process. Past some drone count that process falls behind on *everything* it relays, not just images — including GPS. If a drone's very first GPS message is among the casualties, the hover controller's spawn-race gate (which is correctly waiting for proof the entity exists) never fires, and that drone sits on the ground indefinitely. Nothing in the project subscribes to the camera feed yet — victim detection doesn't land until a later phase — so this bandwidth was being spent for zero benefit.

**Fix**: dropped the camera sensor to 320×240 at 2 Hz in `models/quadrotor/model.sdf.template` (roughly a 60x bandwidth cut) until the search module actually needs real frames. The sensor and its bridge entry still exist and work, just at a resolution/rate that doesn't choke the bridge at swarm scale.

**Rule of thumb**: a sensor's simulated cost doesn't wait for something to subscribe to it — camera/LiDAR generation and bridging happen regardless of whether any node reads the result. Don't run a high-bandwidth sensor at full rate/resolution before something downstream actually consumes it, especially anything that gets multiplied by the drone count.

**If this doesn't fully resolve it**: check whether `parameter_bridge`'s CPU usage is pegged (`top`/`htop`) during a large run, and whether a stuck drone's GPS topic ever publishes at all (`ros2 topic hz /drone_N/gps`) — if it's the LiDAR (also per-drone, though far lighter than camera) or general DDS discovery overhead at high node counts instead, those would need a different fix (lower LiDAR rate/sample count, or splitting the swarm across multiple `parameter_bridge` processes).

## Applying These Lessons Going Forward

Any new module that commands forces/velocities (navigation, collision avoidance) or reads timed sensor data (battery estimation, mapping) should:

- Use `wrench/persistent` + delta tracking for any continuous force actuation.
- Set `use_sim_time` and rely on the bridged `/clock`.
- Drive control-loop timing off the actual sensor message arrival, not a fixed timer.
- Be written so the drone identity (entity name, topic namespace) is a parameter, never a hardcoded constant — this is the one that will bite hardest once Phase 2 spawns the full swarm.
- Never bridge the same gz topic under more than one ROS topic name — share one bridge entry and route by message content instead.
- Clear any entity's standing persistent force at controller startup before trusting a local "applied force" baseline of zero.
- Guard any `dt`-based division against `dt <= 0` — don't assume sensor timestamps are always strictly increasing between consecutive messages.
- Gate any stateful, fire-and-forget publishing (deltas, accumulators) behind proof the target entity already exists — don't start it unconditionally in `__init__`.
