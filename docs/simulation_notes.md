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
- `config/ros_gz_bridge.yaml.template` — same `{drone_id}` templating for the per-drone bridge entries; the shared `clock` bridge lives in its own `config/clock_bridge.yaml` since it's world-level, not per-drone.
- `src/simulator/simulator/spawn_helpers.py` — renders a filled-in SDF/bridge-config per drone at launch time (`render_drone_sdf`, `render_bridge_yaml`), used by both `launch/single_drone.launch.py` and `launch/swarm.launch.py`.
- `hover_controller.py` — reads `drone_id` as a ROS parameter and builds the target `Entity` from it in `__init__`, instead of a module-level constant.

Any new controller that needs to know which drone it's running on should follow the same pattern: a `drone_id` ROS parameter (or namespace-derived value), never a hardcoded string.

## Applying These Lessons Going Forward

Any new module that commands forces/velocities (navigation, collision avoidance) or reads timed sensor data (battery estimation, mapping) should:

- Use `wrench/persistent` + delta tracking for any continuous force actuation.
- Set `use_sim_time` and rely on the bridged `/clock`.
- Drive control-loop timing off the actual sensor message arrival, not a fixed timer.
- Be written so the drone identity (entity name, topic namespace) is a parameter, never a hardcoded constant — this is the one that will bite hardest once Phase 2 spawns the full swarm.
