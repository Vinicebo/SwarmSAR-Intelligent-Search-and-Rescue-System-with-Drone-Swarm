# Algorithms

## Boids (Reynolds, 1987)

Each drone computes a steering force from three components applied to neighbors within `neighbor_radius_m`:

- **Separation**: `F_sep = sum(-normalize(pos_j - pos_i) / dist(i,j))` for neighbors within `separation_radius_m`
- **Alignment**: `F_ali = normalize(avg(vel_j)) - vel_i`
- **Cohesion**: `F_coh = normalize(avg(pos_j) - pos_i)`

Final force: `F = w_sep * F_sep + w_ali * F_ali + w_coh * F_coh`

## A* Pathfinding

Runs on the local 2D occupancy grid (8-connected). Heuristic: Euclidean distance. Cells with occupancy > 50 are treated as obstacles.

## Frontier Exploration

A frontier is any free cell (occupancy < 50) adjacent to an unknown cell (occupancy = -1). Frontier candidates are clustered. The centroid of each cluster is a candidate exploration goal.

## Consensus Map Merge

When drone i receives a map diff from drone j:
```
grid[cell] = (confidence_i * val_i + confidence_j * val_j) / (confidence_i + confidence_j)
```
Confidence decays with the number of hops since the cell was directly observed.

## Auction Algorithm (Task Allocation)

For each frontier f, drone i computes a bid:
```
bid(i, f) = utility(f) / (dist(i, f) + epsilon)
utility(f) = frontier_size(f) * (battery_pct / 100)
```
Drone i broadcasts its bids. The drone with the highest bid for frontier f claims it. Ties broken by drone ID.
