import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
from swarm_interfaces.msg import SwarmState

EARTH_RADIUS_M = 6371000.0
DEFAULT_COMM_RANGE_M = 50.0
BROADCAST_PERIOD_S = 1.0

# A neighbor not heard from in this many broadcast periods is presumed out
# of range or failed, and pruned. Generous relative to BROADCAST_PERIOD_S so
# a couple of dropped messages don't read as a fault.
NEIGHBOR_TIMEOUT_S = 5.0
PRUNE_PERIOD_S = 1.0

# The battery module lands in Phase 4; until then every drone reports full
# charge so downstream consumers (mapping, planning) have a well-formed
# SwarmState to work with instead of an undefined field.
PLACEHOLDER_BATTERY_PCT = 100.0


def latlon_to_local_xy(lat_deg, lon_deg, origin_lat_deg, origin_lon_deg):
    """Equirectangular approximation, accurate enough at the scale of a
    single mission area (hundreds of meters) around a fixed local origin —
    matches the world file's <spherical_coordinates> reference point."""
    lat_rad = math.radians(origin_lat_deg)
    x = math.radians(lon_deg - origin_lon_deg) * EARTH_RADIUS_M * math.cos(lat_rad)
    y = math.radians(lat_deg - origin_lat_deg) * EARTH_RADIUS_M
    return x, y


class SwarmStateNode(Node):
    """Broadcasts this drone's state on a single topic shared by the whole
    swarm and locally discards any neighbor's broadcast outside simulated
    radio range. There is no central node deciding who talks to whom — every
    drone publishes the same way and filters what it receives on its own,
    the way a real radio receiver would just fail to pick up a weak signal.

    In-range neighbors are kept in self.neighbors (drone_id -> (SwarmState,
    last_seen)), pruned once they go quiet for NEIGHBOR_TIMEOUT_S — this is
    the swarm's only way to notice a drone has dropped out (moved out of
    range or failed), since nothing else is watching."""

    def __init__(self):
        super().__init__('swarm_state_node')
        self.declare_parameter('drone_id', 'drone_1')
        self.declare_parameter('comm_range_m', DEFAULT_COMM_RANGE_M)
        self.declare_parameter('origin_lat_deg', 0.0)
        self.declare_parameter('origin_lon_deg', 0.0)

        drone_id_str = self.get_parameter('drone_id').get_parameter_value().string_value
        self.numeric_id = int(drone_id_str.split('_')[-1])
        self.comm_range_m = self.get_parameter('comm_range_m').get_parameter_value().double_value
        self.origin_lat_deg = self.get_parameter('origin_lat_deg').get_parameter_value().double_value
        self.origin_lon_deg = self.get_parameter('origin_lon_deg').get_parameter_value().double_value

        self.position = Point()
        self.have_position = False
        self.neighbors = {}  # numeric drone_id -> (SwarmState, last_seen: rclpy.time.Time)

        self.gps_sub = self.create_subscription(NavSatFix, 'gps', self.on_gps, 10)
        self.state_pub = self.create_publisher(SwarmState, 'swarm_state', 10)
        self.state_sub = self.create_subscription(SwarmState, 'swarm_state', self.on_swarm_state, 10)
        self.broadcast_timer = self.create_timer(BROADCAST_PERIOD_S, self.broadcast_state)
        self.prune_timer = self.create_timer(PRUNE_PERIOD_S, self.prune_stale_neighbors)

        self.get_logger().info(f'Swarm state node started for {drone_id_str}.')

    def on_gps(self, msg):
        x, y = latlon_to_local_xy(msg.latitude, msg.longitude, self.origin_lat_deg, self.origin_lon_deg)
        self.position.x = x
        self.position.y = y
        self.position.z = msg.altitude
        self.have_position = True

    def broadcast_state(self):
        if not self.have_position:
            return

        msg = SwarmState()
        msg.drone_id = self.numeric_id
        msg.position = self.position
        msg.battery_pct = PLACEHOLDER_BATTERY_PCT
        msg.status = SwarmState.EXPLORING
        msg.map_diff = OccupancyGrid()
        msg.victims_found = []
        msg.timestamp = self.get_clock().now().to_msg()
        self.state_pub.publish(msg)

    def on_swarm_state(self, msg):
        if msg.drone_id == self.numeric_id or not self.have_position:
            return

        distance = math.dist(
            (self.position.x, self.position.y),
            (msg.position.x, msg.position.y),
        )
        if distance > self.comm_range_m:
            return

        is_new = msg.drone_id not in self.neighbors
        self.neighbors[msg.drone_id] = (msg, self.get_clock().now())

        if is_new:
            self.get_logger().info(
                f'Neighbor drone_{msg.drone_id} entered range ({distance:.1f} m).'
            )

    def prune_stale_neighbors(self):
        now = self.get_clock().now()
        stale_ids = [
            drone_id for drone_id, (_, last_seen) in self.neighbors.items()
            if (now - last_seen).nanoseconds / 1e9 > NEIGHBOR_TIMEOUT_S
        ]
        for drone_id in stale_ids:
            del self.neighbors[drone_id]
            self.get_logger().info(
                f'Neighbor drone_{drone_id} lost (no broadcast for over '
                f'{NEIGHBOR_TIMEOUT_S:.0f}s) — out of range or failed.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SwarmStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
