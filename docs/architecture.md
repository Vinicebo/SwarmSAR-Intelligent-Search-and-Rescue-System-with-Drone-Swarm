# Architecture

## Overview

SwarmSAR uses a fully decentralized architecture. There is no master node or global state. Each drone is an independent ROS 2 node that communicates only with drones within its simulated radio range.

## Node Graph

```
[Gazebo World]
      |
      | sensor data (LiDAR, GPS, IMU, Camera)
      v
[Drone Node: drone_N]
  ├── /drone_N/sensors     → reads GPS, IMU, LiDAR, Camera
  ├── /drone_N/mapping     → builds and merges occupancy grid
  ├── /drone_N/planning    → selects next frontier goal
  ├── /drone_N/navigation  → executes path to goal
  ├── /drone_N/collision   → reactive avoidance layer
  ├── /drone_N/battery     → monitors charge, triggers return-to-home
  ├── /drone_N/search      → detects and confirms victims
  ├── /drone_N/formation   → Boids-based swarm spacing
  └── /drone_N/comm        ←→ /drone_M/comm  (P2P within range)
```

## Data Flow

1. Sensors publish raw data to per-drone topics.
2. `mapping` subscribes to LiDAR and GPS to update the local occupancy grid.
3. `planning` reads the occupancy grid to extract frontiers and runs the auction.
4. `navigation` receives the goal from `planning` and publishes velocity commands.
5. `collision` intercepts velocity commands and adds repulsion forces before they reach the motors.
6. `comm` broadcasts `SwarmState` and receives neighbor states periodically.
7. `mapping` merges neighbor map diffs received from `comm`.
8. `battery` monitors energy; when low, overrides `planning` goal with return-to-home.
9. `search` monitors the camera feed; on victim detection it publishes an alert and notifies `comm`.

## Decentralization

No node outside the drone itself subscribes to its internal topics. All coordination emerges from local rules and neighbor-to-neighbor message exchange. The swarm has no single point of failure.
