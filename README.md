# SwarmSAR – Intelligent Search and Rescue System with Drone Swarm

A simulation of a fully decentralized drone swarm performing autonomous search and rescue (SAR) operations in a post-earthquake urban environment, built with ROS 2 and Gazebo.

---

## Table of Contents

- [Overview](#overview)
- [Scenario](#scenario)
- [Architecture](#architecture)
- [Algorithms](#algorithms)
- [Sensors](#sensors)
- [Communication Protocol](#communication-protocol)
- [Project Structure](#project-structure)
- [Setup and Installation](#setup-and-installation)
- [Build Guide](docs/build.md)
- [Simulation Notes](docs/simulation_notes.md)
- [Phase 1 Retrospective](docs/phase1_retrospective.md)
- [Running the Simulation](#running-the-simulation)
- [Dashboard Interface](#dashboard-interface)
- [Metrics](#metrics)
- [Development Roadmap](#development-roadmap)
- [Technologies](#technologies)

---

## Overview

SwarmSAR deploys a swarm of 30 autonomous drones over a disaster area with no prior map and no central controller. Each drone operates independently, sharing information with neighbors via peer-to-peer communication to collectively explore the area, detect victims, avoid collisions, and manage battery life.

This is a classic distributed robotics problem — solved here through emergent collective behavior, decentralized planning, and consensus-based map sharing.

---

## Scenario

> A city has been struck by an earthquake. A rescue team deploys 30 drones over the rubble.

The drones have **no pre-built map** of the area. Each one must:

- Explore unknown terrain autonomously
- Detect and geolocate survivors (victims)
- Avoid buildings, debris, and other drones
- Share discovered information with the swarm
- Decide independently which drone investigates each location
- Return to base when battery is critically low
- Compensate when another drone fails mid-mission

All coordination happens through local communication — no central computer, no global state.

---

## Architecture

Each drone is an independent ROS 2 node composed of the following modules:

```
Drone Node
│
├── Navigation      — Waypoint following, A* path planning, local frame transforms
├── Communication   — P2P broadcast/receive, neighbor discovery, message queue
├── Planner         — Frontier selection, task auction, goal arbitration
├── Mapping         — Local occupancy grid, consensus merge with neighbors
├── Sensors         — GPS, IMU, LiDAR, Camera, Victim Detector (simulated)
├── Battery         — State of charge estimation, return-to-home trigger
├── Collision       — Reactive obstacle avoidance, Boids separation rule
├── Formation       — Swarm spacing, coverage pattern maintenance
└── Search          — Victim detection logic, confirmation protocol
```

### Module Responsibilities

| Module        | Description |
|---------------|-------------|
| `Navigation`  | Converts high-level goals into velocity commands. Uses A* on the local map for path planning. |
| `Communication` | Broadcasts drone state (ID, pose, battery, map diff, victims) to neighbors within range. No central broker. |
| `Planner`     | Uses the Auction Algorithm to bid on unexplored frontiers. Selects the highest-utility goal. |
| `Mapping`     | Maintains a local 2D occupancy grid. Merges neighbor maps using a Weighted Consensus protocol. |
| `Sensors`     | Publishes simulated GPS, IMU, LiDAR point clouds, and camera frames on standard ROS 2 topics. |
| `Battery`     | Monitors energy consumption. Triggers return-to-home at a configurable threshold (default: 20%). |
| `Collision`   | Runs at high frequency. Combines LiDAR readings and neighbor positions to generate repulsion forces. |
| `Formation`   | Enforces minimum inter-drone spacing using Boids rules. Adapts dynamically during exploration. |
| `Search`      | Processes camera and victim detector output. Confirms victim detection with a multi-frame filter. |

---

## Algorithms

### Boids (Collective Behavior)
Governs the swarm's emergent spatial behavior through three rules applied to each drone:
- **Separation** — avoid getting too close to neighbors
- **Alignment** — match the average velocity of neighbors
- **Cohesion** — move toward the average position of the local group

Used in: `Formation`, `Collision`

### A* (Pathfinding)
Classic heuristic search algorithm for planning collision-free paths on the local occupancy grid. Runs onboard each drone independently.

Used in: `Navigation`

### Frontier Exploration
Identifies boundaries between known free space and unknown space on the local map. These "frontiers" become candidate exploration targets.

Used in: `Planner`, `Mapping`

### Consensus Algorithm (Map Sharing)
When two drones are within communication range, they exchange local map segments. Each cell value is updated as a weighted average, propagating information through the swarm without a central database.

Used in: `Mapping`, `Communication`

### Auction Algorithm (Task Allocation)
When multiple frontiers exist, drones broadcast bids based on distance, battery level, and estimated utility. The drone with the highest bid claims the target. Prevents redundant exploration.

Used in: `Planner`, `Communication`

---

## Sensors

All sensors are simulated in Gazebo and published on standard ROS 2 topics:

| Sensor          | ROS 2 Topic              | Type                          | Purpose |
|-----------------|--------------------------|-------------------------------|---------|
| GPS             | `/drone_N/gps`           | `sensor_msgs/NavSatFix`       | Global position estimate |
| IMU             | `/drone_N/imu`           | `sensor_msgs/Imu`             | Orientation and acceleration |
| LiDAR           | `/drone_N/scan`          | `sensor_msgs/LaserScan`       | Obstacle detection and mapping |
| Camera          | `/drone_N/camera/image`  | `sensor_msgs/Image`           | Visual victim detection |
| Victim Detector | `/drone_N/victim_signal` | `std_msgs/Float32`            | Proximity signal to simulated victims |

---

## Communication Protocol

Drones communicate peer-to-peer using ROS 2 topics with a simulated range constraint. No message broker or central node.

Every drone's `swarm_state_node` (see [src/communication/communication/swarm_state_node.py](src/communication/communication/swarm_state_node.py)) publishes and subscribes on the **same absolute topic**, `/swarm_state` — there is no per-drone relay or central broker deciding who receives what. Each drone locally discards any broadcast from outside its simulated radio range, the same way a real receiver would simply fail to pick up a weak signal; nothing outside the receiving drone itself decides that.

Each drone periodically broadcasts a **SwarmState** message (defined in [src/swarm_interfaces/msg/SwarmState.msg](src/swarm_interfaces/msg/SwarmState.msg) — a separate `ament_cmake` package, since the pure-Python module packages can't generate custom messages):

```
SwarmState:
  drone_id:       uint32
  position:       geometry_msgs/Point
  battery_pct:    float32
  status:         enum {EXPLORING, INVESTIGATING, RETURNING, IDLE, FAILED}
  map_diff:       nav_msgs/OccupancyGrid   # local map delta since last broadcast
  victims_found:  geometry_msgs/Point[]    # list of confirmed victim positions
  timestamp:      builtin_interfaces/Time
```

Messages outside simulated radio range (configurable, default: 50 m) are dropped locally by the receiving drone's own node — filtering happens at the edge, not in the middle.

`battery_pct`, `status`, `map_diff`, and `victims_found` are placeholders until the battery, planning, mapping, and search modules land in later phases; `position` is already live, computed from the bridged GPS reading via a local equirectangular projection around the world's `spherical_coordinates` origin.

Each drone also keeps a local table of in-range neighbors (`SwarmStateNode.neighbors`), pruning any neighbor it hasn't heard from in 5 seconds — this is currently the swarm's only way to notice a drone has moved out of range or failed, a first building block toward the fault-detection and task-redistribution features.

---

## Project Structure

```
SwarmSAR/
├── src/
│   ├── communication/      # P2P messaging, range filter, SwarmState publisher/subscriber
│   ├── navigation/         # Waypoint controller, A* planner, velocity command publisher
│   ├── formation/          # Boids rules, formation controller
│   ├── mapping/            # Occupancy grid manager, frontier detector, consensus merger
│   ├── planning/           # Goal arbitration, auction bidding, frontier ranker
│   ├── search/             # Victim detection, confirmation filter, alert publisher
│   ├── battery/            # Charge estimator, return-to-home trigger
│   ├── collision/          # Reactive avoidance, LiDAR-based repulsion
│   ├── simulator/          # Drone spawner, world manager, per-drone templating
│   └── swarm_interfaces/   # Custom messages (SwarmState) — ament_cmake, not ament_python
├── config/
│   ├── swarm_params.yaml   # Swarm size, comm range, battery thresholds
│   ├── boids_params.yaml   # Separation/alignment/cohesion weights
│   └── sensor_params.yaml  # Sensor noise, update rates
├── launch/
│   ├── single_drone.launch.py
│   ├── swarm.launch.py
│   └── dashboard.launch.py
├── worlds/
│   └── earthquake_city.world
├── models/
│   └── quadrotor/          # URDF/SDF drone model with sensor plugins
├── results/
│   └── .gitkeep
├── docs/
│   ├── architecture.md
│   ├── algorithms.md
│   └── setup.md
└── README.md
```

---

## Setup and Installation

### Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| Ubuntu | 26.04 LTS "Resolute Raccoon" | Native partition recommended |
| ROS 2 | Lyrical Luth | LTS release, supported until May 2031 |
| Gazebo | Jetty (`gz-sim`) | Official simulator pairing for ROS 2 Lyrical, via `ros_gz` |
| Python | 3.12+ | For ROS 2 nodes and scripts |

### 1. Install ROS 2 Lyrical Luth

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repo
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 desktop (includes RViz)
sudo apt update && sudo apt install ros-lyrical-desktop
```

### 2. Install Gazebo Jetty (via ROS vendor packages)

Since ROS 2 Jazzy, Gazebo ships as ROS vendor packages — no separate OSRF repository is needed.

```bash
sudo apt install ros-lyrical-ros-gz ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-interfaces
```

### 3. Clone and Build This Project

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/SwarmSAR.git

cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Install Python Dependencies

```bash
pip install numpy scipy matplotlib
```

---

## Running the Simulation

### Phase 1 — Single drone in empty world

```bash
ros2 launch simulator single_drone.launch.py
```

### Phase 2 — Full swarm (30 drones)

```bash
ros2 launch simulator swarm.launch.py num_drones:=30 world:=earthquake_city
```

### Phase 3 — With dashboard

```bash
ros2 launch simulator dashboard.launch.py
```

---

## Dashboard Interface

A real-time monitoring panel (RViz + optional web dashboard) displays:

| Field | Description |
|-------|-------------|
| Mission Time | Elapsed time since launch |
| Active Drones | Count of drones currently exploring |
| Returning Drones | Count of drones heading back to charge |
| Victims Found | Number of confirmed detections |
| Area Coverage | Percentage of search area mapped |
| Map | Live 2D occupancy grid, merged from swarm |
| Flight Paths | Drone trajectories overlaid on the map |

---

## Metrics

The system logs the following metrics at the end of each simulation run (saved to `results/`):

| Metric | Description |
|--------|-------------|
| Mission Time | Total time to achieve target coverage or find all victims |
| Area Coverage | Percentage of search zone explored |
| Energy Consumption | Total watt-hours consumed across all drones |
| Collisions Avoided | Count of near-miss events handled by the collision module |
| Messages Exchanged | Total inter-drone messages sent |
| Efficiency | Victims found per unit of energy consumed |
| Fault Events | Number of drone failures and how the swarm recovered |

---

## Development Roadmap

| Phase | Objective | Status |
|-------|-----------|--------|
| 1 | Environment setup: ROS 2, Gazebo, single drone hovering | Planned |
| 2 | Multi-drone spawning, P2P communication, SwarmState messages | Planned |
| 3 | Frontier exploration, consensus mapping, auction-based task allocation | Planned |
| 4 | Victim detection, battery management, fault tolerance | Planned |
| 5 | Dashboard, metrics logging, documentation, algorithm comparison | Planned |

---

## Technologies

| Technology | Role |
|------------|------|
| ROS 2 Lyrical Luth | Middleware: topics, nodes, launch files, parameter server |
| Gazebo | Physics simulation, sensor plugins, world rendering |
| RViz 2 | Real-time visualization of topics, maps, and trajectories |
| Python 3 | All nodes and algorithms (Phase 1–4) |
| C++ (future) | Performance-critical modules (collision, navigation at high frequency) |
| NumPy / SciPy | Numerical computation for algorithms |
| Matplotlib | Offline result plotting |

---

*SwarmSAR is an academic project exploring decentralized multi-robot coordination in emergency scenarios.*
