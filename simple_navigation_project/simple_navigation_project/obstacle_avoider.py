#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import random

class SimpleObstacleAvoider(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoider')

        # Publisher to send velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for navigation goals (Nav2 style)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscriber to receive laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan',
            self.laser_callback,
            10 
        )

        # Nav2 Action Client (optional - for future use)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer to publish commands regularly
        self.timer = self.create_timer(0.1, self.move_robot)
        
        # Timer to publish Nav2-style goals
        self.goal_timer = self.create_timer(5.0, self.publish_nav_goal)

        # Robot state variables
        self.obstacle_detected = False
        self.min_distance = 0.5
        self.min_lane_width = 0.8
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.left_lane_width = 0.0
        self.right_lane_width = 0.0

        # Speed settings
        self.normal_speed = 0.2  # Reduced for safety
        self.turn_speed = 0.5

        self.get_logger().info('Simple Obstacle Avoider with Nav2 concepts started!')
        self.get_logger().info('Robot will navigate using Nav2 patterns...')

    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles and measure lane widths"""
        try:
            # Get ranges (distances) from laser scan
            ranges = msg.ranges

            # Check different directions with wider scanning
            if len(ranges) > 360:
                front_ranges = ranges[340:360] + ranges[0:20]  # Front 40 degrees
                left_ranges = ranges[60:120]
                right_ranges = ranges[240:300]
                left_wide_ranges = ranges[45:135]
                right_wide_ranges = ranges[225:315]
            else:
                # Handle different laser configurations
                total = len(ranges)
                front_idx = total // 12
                front_ranges = ranges[0:front_idx] + ranges[-front_idx:]
                left_ranges = ranges[front_idx:total//3]
                right_ranges = ranges[2*total//3:-front_idx]
                left_wide_ranges = left_ranges
                right_wide_ranges = right_ranges

            # Filter out invalid readings (inf, nan, 0)
            front_distances = [r for r in front_ranges if 0.1 < r < 10.0]
            left_distances = [r for r in left_ranges if 0.1 < r < 10.0]
            right_distances = [r for r in right_ranges if 0.1 < r < 10.0]
            left_wide_distances = [r for r in left_wide_ranges if 0.1 < r < 10.0]
            right_wide_distances = [r for r in right_wide_ranges if 0.1 < r < 10.0]

            # Check if path is clear 
            self.front_clear = len(front_distances) == 0 or min(front_distances) > self.min_distance
            self.left_clear = len(left_distances) == 0 or min(left_distances) > self.min_distance
            self.right_clear = len(right_distances) == 0 or min(right_distances) > self.min_distance

            # Measure lane widths
            self.left_lane_width = min(left_wide_distances) if left_wide_distances else 0.0
            self.right_lane_width = min(right_wide_distances) if right_wide_distances else 0.0

            # Overall obstacle detection
            self.obstacle_detected = not self.front_clear

        except Exception as e:
            self.get_logger().warn(f'Laser processing error: {e}')

    def move_robot(self):
        """Main movement logic with Nav2-style obstacle avoidance"""
        cmd = Twist()

        try:
            if self.front_clear:
                # Move forward at normal speed
                cmd.linear.x = self.normal_speed
                cmd.angular.z = 0.0
                self.get_logger().debug('Moving forward - path clear')
  
            else:
                # Obstacle detected - use Nav2-style decision making
                cmd.linear.x = 0.0

                # Check if lanes are wide enough (Nav2 costmap concept)
                right_safe = self.right_clear and self.right_lane_width > self.min_distance
                left_safe = self.left_clear and self.left_lane_width > self.min_distance

                if right_safe and left_safe:
                    # Both lanes safe - choose wider (Nav2 planner concept)
                    if self.right_lane_width > self.left_lane_width:
                        cmd.angular.z = -self.turn_speed
                        self.get_logger().info(f'Nav2 decision: Turn right (width: {self.right_lane_width:.2f}m)')
                    else:
                        cmd.angular.z = self.turn_speed
                        self.get_logger().info(f'Nav2 decision: Turn left (width: {self.left_lane_width:.2f}m)')

                elif right_safe:
                    cmd.angular.z = -self.turn_speed
                    self.get_logger().info(f'Nav2 decision: Turn right (clear path)')

                elif left_safe:
                    cmd.angular.z = self.turn_speed
                    self.get_logger().info(f'Nav2 decision: Turn left (clear path)')
               
                else:
                    # No clear path - recovery behavior (Nav2 recovery concept)
                    cmd.angular.z = self.turn_speed
                    self.get_logger().info('Nav2 recovery: Rotating to find path')
                    
            # Publish the command
            self.cmd_vel_pub.publish(cmd)

        except Exception as e:
            self.get_logger().error(f'Movement error: {e}')
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

    def publish_nav_goal(self):
        """Publish navigation goals in Nav2 format"""
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Generate goal based on current state
        if self.front_clear:
            # Goal ahead
            goal.pose.position.x = 2.0
            goal.pose.position.y = random.uniform(-0.5, 0.5)
        else:
            # Goal to the side
            if self.left_clear:
                goal.pose.position.x = 1.0
                goal.pose.position.y = 1.0
            else:
                goal.pose.position.x = 1.0
                goal.pose.position.y = -1.0
        
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Nav2 Goal published: x={goal.pose.position.x:.1f}, y={goal.pose.position.y:.1f}')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimpleObstacleAvoider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        # Stop the robot
        if 'node' in locals():
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
        rclpy.shutdown()

if __name__ == '__main__':
    main()