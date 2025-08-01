#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random


def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


class Nav2ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('nav2_obstacle_avoider')
        self.navigator = BasicNavigator()

        # Set initial pose
        initial_pose = create_pose_stamped(self.navigator, 0.0, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2 to activate fully
        self.navigator.waitUntilNav2Active()

        self.get_logger().info("Nav2 is active. Starting navigation...")

        # Define a list of possible waypoints in the standard world (e.g., turtlebot3_world)
        # These values are for turtlebot3_world (small warehouse) - adjust if using different world
        self.waypoints = [
            create_pose_stamped(self.navigator, 1.5, 0.0, 0.0),
            create_pose_stamped(self.navigator, 1.5, -1.0, -1.57),
            create_pose_stamped(self.navigator, 0.0, -1.0, 3.14),
            create_pose_stamped(self.navigator, -1.5, -1.0, 1.57),
            create_pose_stamped(self.navigator, -1.5, 0.0, 0.0),
            create_pose_stamped(self.navigator, -1.5, 1.0, 1.57),
            create_pose_stamped(self.navigator, 0.0, 1.0, 3.14),
            create_pose_stamped(self.navigator, 1.5, 1.0, -1.57),
        ]

        # Start continuous navigation
        self.navigate_loop()

    def navigate_loop(self):
        """Send goals in a loop to keep moving forever"""
        i = 0
        while rclpy.ok():
            goal_pose = self.waypoints[i % len(self.waypoints)]
            i += 1

            self.get_logger().info(f"Going to waypoint #{i % len(self.waypoints) + 1}")

            # Send goal
            self.navigator.goToPose(goal_pose)

            # Wait until goal reached or failed
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # Optional: log feedback or cancel on timeout
                pass

            result = self.navigator.getResult()
            if result == 4:  # SUCCEEDED
                self.get_logger().info("Reached goal successfully!")
            elif result == 5:  # CANCELED
                self.get_logger().warn("Goal was canceled!")
            elif result == 6:  # FAILED
                self.get_logger().error("Goal failed!")

            # Optional: slight delay before next goal
            # time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Nav2ObstacleAvoider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
