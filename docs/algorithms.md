# Algorithms

## Boids (Reynolds, 1987)

Governs the swarm's emergent spatial behavior. Each drone computes a steering force from three components applied to neighbors within `neighbor_radius_m`:

- **Separation**: `F_sep = sum(-normalize(pos_j - pos_i) / dist(i,j))` for neighbors within `separation_radius_m`
- **Alignment**: `F_ali = normalize(avg(vel_j)) - vel_i`
- **Cohesion**: `F_coh = normalize(avg(pos_j) - pos_i)`

Final force: `F = w_sep * F_sep + w_ali * F_ali + w_coh * F_coh`

Weights are configured in `config/boids_params.yaml`.

Used in: `formation`, `collision`

---

## A* (Pathfinding)

Runs on the local 2D occupancy grid (8-connected neighborhood). Heuristic: Euclidean distance to goal. Cells with occupancy value > 50 are treated as obstacles. Replanned whenever the map is updated or the path is blocked.

Used in: `navigation`

---

## Frontier Exploration

A frontier cell is any free cell (occupancy < 50) that is adjacent to at least one unknown cell (occupancy = -1). Frontier cells are clustered by proximity. The centroid of each cluster becomes a candidate exploration goal passed to the planner.

Used in: `planning`, `mapping`

---

## Consensus Map Merge

When drone `i` receives a map diff from drone `j` over the communication channel, each overlapping cell is updated as a weighted average:

```
grid[cell] = (confidence_i * val_i + confidence_j * val_j) / (confidence_i + confidence_j)
```

Confidence decays with the number of hops since the cell was directly observed by a sensor, preventing stale data from overriding fresh observations.

Used in: `mapping`, `communication`

---

## Auction Algorithm (Task Allocation)

Prevents multiple drones from exploring the same frontier. For each frontier `f`, drone `i` computes a bid:

```
bid(i, f) = utility(f) / (dist(i, f) + epsilon)
utility(f) = frontier_size(f) * (battery_pct / 100)
```

Each drone broadcasts its bids. The drone with the highest bid for frontier `f` claims it and sets it as its navigation goal. Ties are broken by drone ID. Bids are recomputed whenever the swarm state changes or a drone fails.

Used in: `planning`, `communication`
