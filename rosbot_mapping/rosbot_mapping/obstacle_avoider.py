#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TwistStamped

# class ObstacleAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoidance_node')

#         # Subscribers
#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

#         # Publisher for velocity commands
#         self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

#         # Timer to compute motion commands periodically
#         self.timer = self.create_timer(0.5, self.control_loop)

#         self.latest_map = None

#     def map_callback(self, msg: OccupancyGrid):
#         self.latest_map = msg
#         self.get_logger().info("Received updated map")

#     def control_loop(self):
#         if self.latest_map is None:
#             self.get_logger().warn("No map data yet!")
#             return

#         # Extract map info
#         width = self.latest_map.info.width
#         height = self.latest_map.info.height
#         resolution = self.latest_map.info.resolution
#         data = self.latest_map.data

#         # Find robot's center cell in map (assume centered at middle of map)
#         center_x = width // 2
#         center_y = height // 2

#         # Define a "safe radius" in cells (0.5 m)
#         safe_radius = int(0.5 / resolution)

#         obstacle_detected = False

#         # Check if any cell in radius has an obstacle (value > 50)
#         for dy in range(-safe_radius, safe_radius + 1):
#             for dx in range(-safe_radius, safe_radius + 1):
#                 x = center_x + dx
#                 y = center_y + dy
#                 index = y * width + x
#                 if 0 <= index < len(data):
#                     if data[index] > 50:
#                         obstacle_detected = True
#                         break
#             if obstacle_detected:
#                 break

#         msg = TwistStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()

#         if obstacle_detected:
#             # Stop if obstacle detected
#             msg.twist.linear.x = 0.0
#             msg.twist.angular.z = 0.5  # Rotate in place to turn away
#             self.get_logger().info("Obstacle detected! Turning to avoid.")
#         else:
#             # Move forward if no obstacle
#             msg.twist.linear.x = 0.2
#             msg.twist.angular.z = 0.0
#             self.get_logger().info("Path is clear, moving forward.")

#         self.cmd_vel_pub.publish(msg)

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TwistStamped

# class ObstacleAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoidance_node')

#         # Subscribers
#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

#         # Publisher for velocity commands
#         self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

#         # Timer to compute motion commands periodically
#         self.timer = self.create_timer(0.5, self.control_loop)

#         self.latest_map = None

#     def map_callback(self, msg: OccupancyGrid):
#         self.latest_map = msg
#         self.get_logger().info("Received updated map")

#     # def control_loop(self):
#     #     if self.latest_map is None:
#     #         self.get_logger().warn("No map data yet!")
#     #         return

#     #     msg = TwistStamped()
#     #     msg.header.stamp = self.get_clock().now().to_msg()
#     #     msg.twist.linear.x = 1.0
#     #     msg.twist.angular.z = 0.5

#     #     self.cmd_vel_pub.publish(msg)
#     def control_loop(self):
#             if self.latest_map is None:
#                 self.get_logger().warn("No map data yet!")
#                 return

#         # Extract map info
#         width = self.latest_map.info.width
#         height = self.latest_map.info.height
#         resolution = self.latest_map.info.resolution
#         data = self.latest_map.data

#         # Find robot's center cell in map (assume centered at middle of map)
#         center_x = width // 2
#         center_y = height // 2

#         # Define a "safe radius" in cells (0.5 m)
#         safe_radius = int(0.5 / resolution)

#         obstacle_detected = False

#         # Check if any cell in radius has an obstacle (value > 50)
#         for dy in range(-safe_radius, safe_radius + 1):
#             for dx in range(-safe_radius, safe_radius + 1):
#                 x = center_x + dx
#                 y = center_y + dy
#                 index = y * width + x
#                 if 0 <= index < len(data):
#                     if data[index] > 50:
#                         obstacle_detected = True
#                         break
#             if obstacle_detected:
#                 break

#         msg = TwistStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()

#         if obstacle_detected:
#             # Stop if obstacle detected
#             msg.twist.linear.x = 0.0
#             msg.twist.angular.z = 0.5  # Rotate in place to turn away
#             self.get_logger().info
# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleAvoidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# zone based 
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TwistStamped

# class ObstacleAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoidance_node')

#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
#         self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.1, self.control_loop)

#         self.latest_map = None

#         # States
#         self.state = "MOVING_FORWARD"
#         self.rotate_start_time = None
#         self.rotate_duration = 3.0  # seconds to rotate when obstacle detected

#     def map_callback(self, msg: OccupancyGrid):
#         self.latest_map = msg

#     def control_loop(self):
#         if self.latest_map is None:
#             self.get_logger().warn("No map data yet!")
#             return

#         width = self.latest_map.info.width
#         height = self.latest_map.info.height
#         resolution = self.latest_map.info.resolution
#         data = self.latest_map.data

#         center_x = width // 2
#         center_y = height // 2
#         safe_radius = int(0.5 / resolution)

#         # Zones lists
#         front_cells = []
#         left_cells = []
#         right_cells = []
#         back_cells = []

#         # Detect obstacles in zones
#         for dy in range(-safe_radius, safe_radius + 1):
#             for dx in range(-safe_radius, safe_radius + 1):
#                 x = center_x + dx
#                 y = center_y + dy
#                 index = y * width + x
#                 if 0 <= index < len(data):
#                     if data[index] > 50:
#                         # Front zone: ahead in negative y (assuming robot facing up)
#                         if dy < 0 and abs(dx) <= safe_radius // 2:
#                             front_cells.append((x, y))
#                         # Left zone: dx negative (left side)
#                         elif dx < 0 and abs(dy) <= safe_radius // 2:
#                             left_cells.append((x, y))
#                         # Right zone: dx positive (right side)
#                         elif dx > 0 and abs(dy) <= safe_radius // 2:
#                             right_cells.append((x, y))
#                         # Back zone: dy positive
#                         elif dy > 0 and abs(dx) <= safe_radius // 2:
#                             back_cells.append((x, y))

#         msg = TwistStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()

#         # Decide action based on state machine and zones
#         if self.state == "MOVING_FORWARD":
#             if len(front_cells) > 0:
#                 self.get_logger().info("Obstacle detected in front! Moving backward.")
#                 # Move backward a bit
#                 msg.twist.linear.x = -0.1
#                 msg.twist.angular.z = 0.0
#                 self.state = "BACKING_UP"
#                 self.back_start_time = self.get_clock().now()
#                 self.back_duration = 2.0  # seconds to back up
#             elif len(left_cells) > 0:
#                 self.get_logger().info("Obstacle detected on left! Rotating right.")
#                 # Rotate right to avoid
#                 msg.twist.linear.x = 0.0
#                 msg.twist.angular.z = -0.5
#                 self.state = "ROTATING"
#                 self.rotate_start_time = self.get_clock().now()
#             elif len(right_cells) > 0:
#                 self.get_logger().info("Obstacle detected on right! Rotating left.")
#                 # Rotate left to avoid
#                 msg.twist.linear.x = 0.0
#                 msg.twist.angular.z = 0.5
#                 self.state = "ROTATING"
#                 self.rotate_start_time = self.get_clock().now()
#             elif len(back_cells) > 0:
#                 self.get_logger().info("Obstacle detected at back! Moving forward.")
#                 msg.twist.linear.x = 0.2
#                 msg.twist.angular.z = 0.0
#             else:
#                 # No obstacles detected, move forward
#                 msg.twist.linear.x = 0.2
#                 msg.twist.angular.z = 0.0
#                 self.get_logger().info("Path clear, moving forward.")

#         elif self.state == "ROTATING":
#             now = self.get_clock().now()
#             elapsed = (now - self.rotate_start_time).nanoseconds / 1e9
#             if elapsed >= self.rotate_duration:
#                 self.get_logger().info("Rotation done, moving forward.")
#                 self.state = "MOVING_FORWARD"
#                 msg.twist.linear.x = 0.2
#                 msg.twist.angular.z = 0.0
#             else:
#                 # Keep rotating in same direction
#                 if len(left_cells) > 0:
#                     # rotate right
#                     msg.twist.linear.x = 0.0
#                     msg.twist.angular.z = -0.5
#                 else:
#                     # rotate left
#                     msg.twist.linear.x = 0.0
#                     msg.twist.angular.z = 0.5
#                 self.get_logger().info(f"Rotating... {elapsed:.2f}s elapsed")

#         elif self.state == "BACKING_UP":
#             now = self.get_clock().now()
#             elapsed = (now - self.back_start_time).nanoseconds / 1e9
#             if elapsed >= self.back_duration:
#                 self.get_logger().info("Backing up done, rotating to avoid obstacle.")
#                 self.state = "ROTATING"
#                 self.rotate_start_time = self.get_clock().now()
#                 msg.twist.linear.x = 0.0
#                 msg.twist.angular.z = 0.5  # rotate left by default after backing up
#             else:
#                 # Keep backing up
#                 msg.twist.linear.x = -0.1
#                 msg.twist.angular.z = 0.0
#                 self.get_logger().info(f"Backing up... {elapsed:.2f}s elapsed")

#         else:
#             # Unknown state fallback
#             msg.twist.linear.x = 0.0
#             msg.twist.angular.z = 0.0
#             self.state = "MOVING_FORWARD"
#             self.get_logger().warn("Unknown state! Resetting to MOVING_FORWARD.")

#         self.cmd_vel_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleAvoidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TwistStamped

# class ObstacleAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoidance_node')

#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
#         self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.5, self.control_loop)

#         self.latest_map = None
#         self.state = 'forward'
#         self.state_timer = 0

#     def map_callback(self, msg: OccupancyGrid):
#         self.latest_map = msg

#     def control_loop(self):
#         if self.latest_map is None:
#             self.get_logger().warn("No map data yet!")
#             return

#         # Get map info
#         width = self.latest_map.info.width
#         height = self.latest_map.info.height
#         resolution = self.latest_map.info.resolution
#         data = self.latest_map.data

#         center_x = width // 2
#         center_y = height // 2
#         safe_radius = int(0.5 / resolution)

#         obstacle_detected = False

#         for dy in range(-safe_radius, safe_radius + 1):
#             for dx in range(-safe_radius, safe_radius + 1):
#                 x = center_x + dx
#                 y = center_y + dy
#                 index = y * width + x
#                 if 0 <= index < len(data):
#                     if data[index] > 50:
#                         obstacle_detected = True
#                         break
#             if obstacle_detected:
#                 break

#         # Create the TwistStamped message
#         msg = TwistStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()

#         # Basic state machine
#         if not obstacle_detected:
#             self.state = 'forward'
#             self.state_timer = 0
#         else:
#             if self.state == 'forward':
#                 self.state = 'backward'
#                 self.state_timer = 4  # Back up for 4 cycles (~2s)
#             elif self.state == 'backward' and self.state_timer <= 0:
#                 self.state = 'side'
#                 self.state_timer = 4  # Sideways for 4 cycles
#             elif self.state == 'side' and self.state_timer <= 0:
#                 self.state = 'rotate'
#                 self.state_timer = 4  # Rotate in place for 4 cycles

#         # Decrement state timer
#         if obstacle_detected:
#             self.state_timer -= 1

#         # Command according to state
#         if self.state == 'forward':
#             msg.twist.linear.x = 0.2
#             msg.twist.angular.z = 0.0
#             self.get_logger().info("Moving forward.")
#         elif self.state == 'backward':
#             msg.twist.linear.x = -0.1
#             msg.twist.angular.z = 0.0
#             self.get_logger().info("Obstacle detected! Moving backward.")
#         elif self.state == 'side':
#             msg.twist.linear.x = 0.0
#             msg.twist.angular.z = 0.0
#             msg.twist.linear.y = 0.1  # Move sideways if supported
#             self.get_logger().info("Obstacle detected! Moving sideways.")
#         elif self.state == 'rotate':
#             msg.twist.linear.x = 0.0
#             msg.twist.angular.z = 0.5
#             self.get_logger().info("Obstacle detected! Rotating to find new path.")

#         self.cmd_vel_pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleAvoidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import TwistStamped, TransformStamped
# import tf2_ros
# from tf2_ros import TransformListener, Buffer


# class ObstacleAvoidanceNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoidance_node')

#         # Subscribers and publishers
#         self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
#         self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

#         # Timer for control loop (every 0.5s)
#         self.timer = self.create_timer(0.5, self.control_loop)

#         # Map and state
#         self.latest_map = None
#         self.state = 'forward'
#         self.state_timer = 0

#         # TF buffer and listener
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#     def map_callback(self, msg: OccupancyGrid):
#         self.latest_map = msg

#     def control_loop(self):
#         if self.latest_map is None:
#             self.get_logger().warn("No map data yet!")
#             return

#         # Lookup transform from map -> base_link
#         try:
#             transform: TransformStamped = self.tf_buffer.lookup_transform(
#                 'map', 'base_link', rclpy.time.Time(),
#                 timeout=rclpy.duration.Duration(seconds=0.5)
#             )
#             robot_x = transform.transform.translation.x
#             robot_y = transform.transform.translation.y
#         except Exception as e:
#             self.get_logger().warn(f"TF lookup failed: {e}")
#             return

#         # Map info
#         origin_x = self.latest_map.info.origin.position.x
#         origin_y = self.latest_map.info.origin.position.y
#         resolution = self.latest_map.info.resolution
#         width = self.latest_map.info.width
#         height = self.latest_map.info.height
#         data = self.latest_map.data

#         # Convert robot (x, y) in map frame to cell indices
#         center_x = int((robot_x - origin_x) / resolution)
#         center_y = int((robot_y - origin_y) / resolution)
#         safe_radius = int(0.5 / resolution)  # 0.5m radius

#         obstacle_detected = False

#         # Check cells in safe_radius
#         for dy in range(-safe_radius, safe_radius + 1):
#             for dx in range(-safe_radius, safe_radius + 1):
#                 x = center_x + dx
#                 y = center_y + dy
#                 if 0 <= x < width and 0 <= y < height:
#                     index = y * width + x
#                     if data[index] > 50:
#                         obstacle_detected = True
#                         break
#             if obstacle_detected:
#                 break

#         # Create TwistStamped message
#         msg = TwistStamped()
#         msg.header.stamp = self.get_clock().now().to_msg()

#         # State machine
#         if not obstacle_detected:
#             self.state = 'forward'
#             self.state_timer = 0
#         else:
#             if self.state == 'forward':
#                 self.state = 'backward'
#                 self.state_timer = 4  # Back up for 4 cycles (~2s)
#             elif self.state == 'backward' and self.state_timer <= 0:
#                 self.state = 'rotate'
#                 self.state_timer = 4  # Rotate in place for 4 cycles

#         # Decrement timer
#         if obstacle_detected:
#             self.state_timer -= 1

#         # Motion commands
#         if self.state == 'forward':
#             msg.twist.linear.x = 0.2
#             msg.twist.angular.z = 0.0
#             self.get_logger().info("Moving forward.")
#         elif self.state == 'backward':
#             msg.twist.linear.x = -0.1
#             msg.twist.angular.z = 0.0
#             self.get_logger().info("Obstacle detected! Moving backward.")
#         elif self.state == 'rotate':
#             msg.twist.linear.x = 0.0
#             msg.twist.angular.z = 0.5
#             self.get_logger().info("Obstacle detected! Rotating to find new path.")

#         # Publish command
#         self.cmd_vel_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleAvoidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import TwistStamped
# import math

# class ObstacleAvoider(Node):
#     def __init__(self):
#         super().__init__('obstacle_avoider')

#         # Parameters
#         self.declare_parameter('min_obstacle_distance', 0.4)
#         self.declare_parameter('max_obstacle_distance', 2.0)
#         self.declare_parameter('obstacle_threshold', 0.5)
#         self.declare_parameter('emergency_stop_distance', 0.2)
#         self.declare_parameter('forward_speed', 0.2)
#         self.declare_parameter('turn_speed', 0.6)
#         self.declare_parameter('laser_scan_topic', '/scan')
#         self.declare_parameter('cmd_vel_topic', '/cmd_vel')
#         self.declare_parameter('robot_frame', 'base_link')
#         self.declare_parameter('obstacle_angle_range', 90)  # Degrees

#         # Get parameters
#         self.min_dist = self.get_parameter('min_obstacle_distance').value
#         self.max_dist = self.get_parameter('max_obstacle_distance').value
#         self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
#         self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
#         self.forward_speed = self.get_parameter('forward_speed').value
#         self.turn_speed = self.get_parameter('turn_speed').value
#         self.scan_topic = self.get_parameter('laser_scan_topic').value
#         self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
#         self.obstacle_angle_range = math.radians(self.get_parameter('obstacle_angle_range').value)

#         # Publishers and Subscribers
#         self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
#         self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

#         self.get_logger().info("✅ Obstacle Avoider node ready.")

#     def scan_callback(self, scan: LaserScan):
#         # Determine the obstacle region indices
#         angle_min = scan.angle_min
#         angle_max = scan.angle_max
#         angle_increment = scan.angle_increment
#         ranges = scan.ranges

#         num_points = len(ranges)
#         mid_index = num_points // 2
#         angle_range_points = int(self.obstacle_angle_range / angle_increment / 2)

#         start_index = max(0, mid_index - angle_range_points)
#         end_index = min(num_points, mid_index + angle_range_points)

#         obstacle_distances = [r for r in ranges[start_index:end_index] if self.min_dist < r < self.max_dist]

#         if not obstacle_distances:
#             self.move_forward()
#             return

#         min_distance = min(obstacle_distances)

#         if min_distance < self.emergency_stop_distance:
#             self.stop()
#         elif min_distance < self.obstacle_threshold:
#             self.turn()
#         else:
#             self.move_forward()

#     def move_forward(self):
#         twist = TwistStamped()
#         twist.twist.linear.x = self.forward_speed
#         twist.twist.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist)

#     def turn(self):
#         twist = TwistStamped()
#         twist.twist.linear.x = 0.0
#         twist.twist.angular.z = self.turn_speed
#         self.cmd_vel_pub.publish(twist)

#     def stop(self):
#         twist = TwistStamped()
#         twist.twist.linear.x = 0.0
#         twist.twist.angular.z = 0.0
#         self.cmd_vel_pub.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObstacleAvoider()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Parameters
        self.declare_parameter('min_obstacle_distance', 0.4)
        self.declare_parameter('max_obstacle_distance', 2.0)
        self.declare_parameter('obstacle_threshold', 0.6)
        self.declare_parameter('emergency_stop_distance', 0.2)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('laser_scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('obstacle_angle_range', 120)  # Wider view

        # Get parameters
        self.min_dist = self.get_parameter('min_obstacle_distance').value
        self.max_dist = self.get_parameter('max_obstacle_distance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.scan_topic = self.get_parameter('laser_scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.obstacle_angle_range = math.radians(self.get_parameter('obstacle_angle_range').value)

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info("✅ Obstacle Avoider node ready.")

    def scan_callback(self, scan: LaserScan):
        # Determine obstacle region indices
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        num_points = len(ranges)

        mid_index = num_points // 2
        angle_range_points = int(self.obstacle_angle_range / angle_increment / 2)

        start_index = max(0, mid_index - angle_range_points)
        end_index = min(num_points, mid_index + angle_range_points)

        # Find the closest obstacle and where it is (left or right)
        closest_distance = float('inf')
        closest_angle_index = -1

        for i in range(start_index, end_index):
            dist = ranges[i]
            if self.min_dist < dist < self.max_dist:
                if dist < closest_distance:
                    closest_distance = dist
                    closest_angle_index = i

        if closest_angle_index == -1:
            # No obstacles: move forward
            self.move_forward()
            return

        if closest_distance < self.emergency_stop_distance:
            self.stop()
        elif closest_distance < self.obstacle_threshold:
            # Determine if obstacle is to the left or right of center
            if closest_angle_index < mid_index:
                self.turn_right()
            else:
                self.turn_left()
        else:
            self.move_forward()

    def move_forward(self):
        twist = TwistStamped()
        twist.twist.linear.x = self.forward_speed
        twist.twist.angular.z = 0.0
        self.get_logger().info("🟢 Moving forward.")
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = self.turn_speed
        self.get_logger().warn("🔄 Obstacle detected! Turning left.")
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = -self.turn_speed
        self.get_logger().warn("🔄 Obstacle detected! Turning right.")
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.get_logger().error("🛑 Emergency stop!")
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
