#!/usr/bin/env python3

"""
Simple Navigation with Nav2: Sends sequential goals for continuous movement.
Obstacle avoidance is handled by Nav2 (costmaps, local planner).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import TransformListener, Buffer
import tf2_ros
import math

class Nav2ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('nav2_obstacle_avoider')

        # Create the navigator
        self.navigator = BasicNavigator()

        # Set initial pose (required by Nav2)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2 to activate
        self.navigator.waitUntilNav2Active()

        # Define a simple square path (or random walk)
        self.waypoints = self.create_square_waypoints()

        self.get_logger().info("Nav2 is active. Sending goals...")

        # Start following waypoints
        self.send_next_goal()

    def create_square_waypoints(self):
        """Create a simple square path around (0,0)"""
        waypoints = []
        side_length = 2.0
        corners = [
            (side_length, 0.0),
            (side_length, side_length),
            (0.0, side_length),
            (0.0, 0.0)
        ]

        for x, y in corners:
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0  # Facing forward
            waypoints.append(goal)

        return waypoints

    def send_next_goal(self):
        """Send goals in a loop"""
        import time

        for goal in self.waypoints:
            self.get_logger().info(f"Going to point: ({goal.pose.position.x}, {goal.pose.position.y})")
            self.navigator.goToPose(goal)

            # Wait until the goal is reached or failed
            while not self.navigator.isTaskComplete():
                time.sleep(0.1)  # Allow interruption

            result = self.navigator.getTaskResult()
            if result == 4:  # SUCCEEDED
                self.get_logger().info("Goal reached!")
            else:
                self.get_logger().warn(f"Goal failed with result: {result}")

        # Loop forever
        self.send_next_goal()  # Restart after completing square


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
