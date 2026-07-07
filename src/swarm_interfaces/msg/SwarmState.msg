# Periodic broadcast each drone sends to neighbors within communication
# range. See README.md#communication-protocol for the field-by-field spec.

uint8 EXPLORING = 0
uint8 INVESTIGATING = 1
uint8 RETURNING = 2
uint8 IDLE = 3
uint8 FAILED = 4

uint32 drone_id
geometry_msgs/Point position
float32 battery_pct
uint8 status
nav_msgs/OccupancyGrid map_diff
geometry_msgs/Point[] victims_found
builtin_interfaces/Time timestamp
