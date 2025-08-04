#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import tf_transformations
import random
import math

class SimpleObstacleAvoider(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoider')
        
        # Initialize navigator
        self.navigator = BasicNavigator()
        
        # Subscribe to laser scan for obstacle detection
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Obstacle detection variables
        self.obstacle_detected = False
        self.min_distance = 0.6  # Minimum safe distance
        
        # Waypoint management
        self.current_waypoint_index = 0
        self.waypoints = []
        
        self.get_logger().info('Simple Obstacle Avoider with Nav2 Started!')

    def create_pose_stamped(self, position_x, position_y, orientation_z):
        """Create a PoseStamped message"""
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        try:
            ranges = msg.ranges
            # Check front area (front 60 degrees)
            front_ranges = ranges[330:360] + ranges[0:30]
            
            # Filter valid readings
            valid_ranges = [r for r in front_ranges if 0.1 < r < 10.0]
            
            # Check if obstacle is too close
            if valid_ranges:
                min_distance = min(valid_ranges)
                self.obstacle_detected = min_distance < self.min_distance
                
                if self.obstacle_detected:
                    self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m - Nav2 will replan')
            
        except Exception as e:
            self.get_logger().error(f'Laser processing error: {e}')

    def generate_safe_waypoint(self):
        """Generate a random safe waypoint"""
        x = random.uniform(-3.0, 3.0)
        y = random.uniform(-3.0, 3.0)
        theta = random.uniform(-math.pi, math.pi)
        return self.create_pose_stamped(x, y, theta)

    def setup_waypoints(self):
        """Setup predefined waypoints for navigation"""
        self.waypoints = [
            self.create_pose_stamped(2.0, 0.0, 0.0),
            self.create_pose_stamped(2.0, 2.0, 1.57),
            self.create_pose_stamped(0.0, 2.0, 3.14),
            self.create_pose_stamped(-2.0, 0.0, -1.57),
            self.create_pose_stamped(0.0, 0.0, 0.0),
        ]

    def run_navigation(self):
        """Main navigation loop with obstacle avoidance"""
        # Set initial pose
        initial_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)
        
        # Wait for Nav2 to activate
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 activated! Starting navigation...')
        
        # Setup waypoints
        self.setup_waypoints()
        
        while rclpy.ok():
            try:
                # Check if we need to send a new goal
                if not self.navigator.isTaskComplete():
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue
                
                # Get next waypoint
                if self.current_waypoint_index < len(self.waypoints):
                    goal_pose = self.waypoints[self.current_waypoint_index]
                    self.get_logger().info(f'Going to waypoint {self.current_waypoint_index + 1}')
                else:
                    # Generate random waypoint when all predefined waypoints are visited
                    goal_pose = self.generate_safe_waypoint()
                    self.get_logger().info('Going to random waypoint')
                
                # Send navigation goal
                self.navigator.goToPose(goal_pose)
                
                # Wait for task completion
                while not self.navigator.isTaskComplete():
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                # Check result
                result = self.navigator.getResult()
                if result == 3:  # SUCCEEDED
                    self.get_logger().info('Goal reached successfully!')
                    self.current_waypoint_index += 1
                else:
                    self.get_logger().warn(f'Navigation failed, trying next waypoint')
                    self.current_waypoint_index += 1
                
            except Exception as e:
                self.get_logger().error(f'Navigation error: {e}')
                break

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleObstacleAvoider()
        node.run_navigation()
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
