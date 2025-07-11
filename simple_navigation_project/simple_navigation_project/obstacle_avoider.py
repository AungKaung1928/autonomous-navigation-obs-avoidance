#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SimpleObstacleAvoider(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoider')
        
        # Publisher to send velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to receive laser scan data
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            10
        )
        
        # Timer to publish commands regularly
        self.timer = self.create_timer(0.1, self.move_robot)
        
        # Robot state variables
        self.obstacle_detected = False
        self.min_distance = 0.5  # Stop if obstacle closer than 50cm
        self.min_lane_width = 0.8  # Minimum lane width to enter (80cm)
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.left_lane_width = 0.0
        self.right_lane_width = 0.0
        
        # Speed settings - reduced for smoother movement
        self.normal_speed = 0.3  # Reduced from 0.8 to 0.3
        self.turn_speed = 0.3    # Smooth turning speed
        
        self.get_logger().info('Simple Obstacle Avoider Started!')
        
    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles and measure lane widths"""
        try:
            # Get ranges (distances) from laser
            ranges = msg.ranges
            
            # Check different directions with wider scanning
            # Front: indices around middle of scan
            front_ranges = ranges[340:360] + ranges[0:20]  # Front 40 degrees
            left_ranges = ranges[60:120]   # Left side
            right_ranges = ranges[240:300] # Right side
            
            # Extended scanning for lane width measurement
            left_wide_ranges = ranges[45:135]   # Wider left scan
            right_wide_ranges = ranges[225:315] # Wider right scan
            
            # Filter out invalid readings (inf, nan, 0)
            front_distances = [r for r in front_ranges if 0.1 < r < 10.0]
            left_distances = [r for r in left_ranges if 0.1 < r < 10.0]
            right_distances = [r for r in right_ranges if 0.1 < r < 10.0]
            
            left_wide_distances = [r for r in left_wide_ranges if 0.1 < r < 10.0]
            right_wide_distances = [r for r in right_wide_ranges if 0.1 < r < 10.0]
            
            # Check if path is clear (no obstacles within min_distance)
            self.front_clear = len(front_distances) == 0 or min(front_distances) > self.min_distance
            self.left_clear = len(left_distances) == 0 or min(left_distances) > self.min_distance
            self.right_clear = len(right_distances) == 0 or min(right_distances) > self.min_distance
            
            # Measure lane widths to determine if they're safe to enter
            self.left_lane_width = min(left_wide_distances) if left_wide_distances else 0.0
            self.right_lane_width = min(right_wide_distances) if right_wide_distances else 0.0
            
            # Overall obstacle detection
            self.obstacle_detected = not self.front_clear
            
        except Exception as e:
            self.get_logger().warn(f'Laser processing error: {e}')
            
    def move_robot(self):
        """Main movement logic - think ahead and avoid narrow lanes"""
        cmd = Twist()
        
        try:
            if self.front_clear:
                # Move forward at normal speed
                cmd.linear.x = self.normal_speed
                cmd.angular.z = 0.0
                self.get_logger().info('Moving forward - path clear')
                
            else:
                # Obstacle detected - think about available options
                cmd.linear.x = 0.0  # Stop forward motion
                
                # Check if lanes are wide enough and clear
                right_safe = self.right_clear and self.right_lane_width > self.min_lane_width
                left_safe = self.left_clear and self.left_lane_width > self.min_lane_width
                
                if right_safe and left_safe:
                    # Both lanes are safe - choose the wider one
                    if self.right_lane_width >= self.left_lane_width:
                        cmd.angular.z = -self.turn_speed
                        self.get_logger().info(f'Turning right - lane width: {self.right_lane_width:.2f}m')
                    else:
                        cmd.angular.z = self.turn_speed
                        self.get_logger().info(f'Turning left - lane width: {self.left_lane_width:.2f}m')
                        
                elif right_safe:
                    # Only right lane is safe
                    cmd.angular.z = -self.turn_speed
                    self.get_logger().info(f'Turning right - safe lane width: {self.right_lane_width:.2f}m')
                    
                elif left_safe:
                    # Only left lane is safe
                    cmd.angular.z = self.turn_speed
                    self.get_logger().info(f'Turning left - safe lane width: {self.left_lane_width:.2f}m')
                    
                else:
                    # No safe lanes - continue rotating to find better option
                    cmd.angular.z = self.turn_speed
                    if self.right_clear or self.left_clear:
                        self.get_logger().info('Lanes too narrow - continuing to search for wider path')
                    else:
                        self.get_logger().info('No clear path - rotating to find opening')
            
            # Publish the command
            self.cmd_vel_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Movement error: {e}')
            # Stop robot on error
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

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