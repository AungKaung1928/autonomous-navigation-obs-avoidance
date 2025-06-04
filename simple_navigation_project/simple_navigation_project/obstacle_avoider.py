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
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        
        self.get_logger().info('Simple Obstacle Avoider Started!')
        
    def laser_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        try:
            # Get ranges (distances) from laser
            ranges = msg.ranges
            
            # Check different directions
            # Front: indices around middle of scan
            front_ranges = ranges[340:360] + ranges[0:20]  # Front 40 degrees
            left_ranges = ranges[60:120]   # Left side
            right_ranges = ranges[240:300] # Right side
            
            # Filter out invalid readings (inf, nan, 0)
            front_distances = [r for r in front_ranges if 0.1 < r < 10.0]
            left_distances = [r for r in left_ranges if 0.1 < r < 10.0]
            right_distances = [r for r in right_ranges if 0.1 < r < 10.0]
            
            # Check if path is clear (no obstacles within min_distance)
            self.front_clear = len(front_distances) == 0 or min(front_distances) > self.min_distance
            self.left_clear = len(left_distances) == 0 or min(left_distances) > self.min_distance
            self.right_clear = len(right_distances) == 0 or min(right_distances) > self.min_distance
            
            # Overall obstacle detection
            self.obstacle_detected = not self.front_clear
            
        except Exception as e:
            self.get_logger().warn(f'Laser processing error: {e}')
            
    def move_robot(self):
        """Main movement logic"""
        cmd = Twist()
        
        try:
            if self.front_clear:
                # Move forward
                cmd.linear.x = 0.8  # Forward speed can be adjusted.
                cmd.angular.z = 0.0
                self.get_logger().info('Moving forward - path clear')
                
            else:
                # Obstacle detected - turn
                cmd.linear.x = 0.0  # Stop forward motion
                
                if self.right_clear:
                    # Turn right
                    cmd.angular.z = -0.5
                    self.get_logger().info('Turning right - avoiding obstacle')
                elif self.left_clear:
                    # Turn left
                    cmd.angular.z = 0.5
                    self.get_logger().info('Turning left - avoiding obstacle')
                else:
                    # Both sides blocked - turn around
                    cmd.angular.z = 1.0
                    self.get_logger().info('Turning around - obstacle on both sides')
            
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
