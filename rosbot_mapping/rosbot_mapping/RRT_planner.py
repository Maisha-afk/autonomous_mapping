#!/usr/bin/env python3
#global planner
#RAPDILY EXPLORING RANDOM TREE
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException
import random
import math


class RRTNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


class RRTPlanner(Node):
    def __init__(self):
        super().__init__("RRT_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=10)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/costmap", self.map_callback, map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_callback, 10
        )
        self.path_pub = self.create_publisher(Path, "/rrt/path", 10)

        self.map_ = None

    def map_callback(self, map_msg: OccupancyGrid):
        self.map_ = map_msg

    def goal_callback(self, pose: PoseStamped):
        if self.map_ is None:
            self.get_logger().error("No map received!")
            return

        try:
            map_to_base_tf = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
            )
        except LookupException:
            self.get_logger().error("Could not transform from map to base_footprint")
            return

        start_pose = Pose()
        start_pose.position.x = map_to_base_tf.transform.translation.x
        start_pose.position.y = map_to_base_tf.transform.translation.y

        path = self.plan(start_pose, pose.pose)
        if path.poses:
            self.get_logger().info("RRT path found!")
            self.path_pub.publish(path)
        else:
            self.get_logger().warn("No path found.")

    def plan(self, start: Pose, goal: Pose, max_iter=5000, step_size=0.2, goal_threshold=0.5):
        start_node = self.world_to_grid(start)
        goal_node = self.world_to_grid(goal)

        tree = [start_node]

        for i in range(max_iter):
            rand_node = self.sample_random_node()
            nearest_node = self.get_nearest_node(tree, rand_node)
            new_node = self.steer(nearest_node, rand_node, step_size)

            if self.is_free(new_node):
                tree.append(new_node)

                # Check if goal is reached
                if self.euclidean_distance(new_node, goal_node) < goal_threshold / self.map_.info.resolution:
                    self.get_logger().info(f"Goal reached at iteration {i}")
                    goal_node.parent = new_node
                    return self.reconstruct_path(goal_node)

        # Failed to find path
        return Path()

    def sample_random_node(self):
        x = random.randint(0, self.map_.info.width - 1)
        y = random.randint(0, self.map_.info.height - 1)
        return RRTNode(x, y)

    def get_nearest_node(self, tree, node):
        return min(tree, key=lambda n: self.euclidean_distance(n, node))

    def steer(self, from_node, to_node, step_size):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.hypot(dx, dy)

        if dist == 0:
            return from_node

        scale = min(step_size / self.map_.info.resolution, dist) / dist
        new_x = int(from_node.x + dx * scale)
        new_y = int(from_node.y + dy * scale)
        return RRTNode(new_x, new_y, from_node)

    def is_free(self, node):
        if not (0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height):
            return False
        cell_value = self.map_.data[self.pose_to_cell(node)]
        return cell_value == 0

    def euclidean_distance(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def reconstruct_path(self, goal_node):
        path = Path()
        path.header.frame_id = self.map_.header.frame_id
        current = goal_node
        while current:
            pose = self.grid_to_world(current)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.map_.header.frame_id
            pose_stamped.pose = pose
            path.poses.append(pose_stamped)
            current = current.parent
        path.poses.reverse()
        return path

    def world_to_grid(self, pose: Pose) -> RRTNode:
        grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
        grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
        return RRTNode(grid_x, grid_y)

    def grid_to_world(self, node: RRTNode) -> Pose:
        pose = Pose()
        pose.position.x = node.x * self.map_.info.resolution + self.map_.info.origin.position.x
        pose.position.y = node.y * self.map_.info.resolution + self.map_.info.origin.position.y
        return pose

    def pose_to_cell(self, node: RRTNode):
        return node.y * self.map_.info.width + node.x


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
