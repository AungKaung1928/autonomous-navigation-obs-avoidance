#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class SimpleObstacleAvoider(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoider')
        
        # Declare parameters with validation
        self.declare_parameter('normal_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('min_distance', 0.5)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Timer for robot movement
        self.create_timer(0.1, self.move_robot)
        
        # Get and validate parameters
        self.speed = max(0.1, min(0.5, self.get_parameter('normal_speed').value))
        self.turn = max(0.3, min(1.5, self.get_parameter('turn_speed').value))
        self.min_dist = max(0.3, min(1.0, self.get_parameter('min_distance').value))
        
        # State variables
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.scan_received = False
        
        self.get_logger().info(f'🤖 Obstacle Avoider Started!')
        self.get_logger().info(f'⚡ Speed: {self.speed:.2f} m/s')
        self.get_logger().info(f'🔄 Turn: {self.turn:.2f} rad/s') 
        self.get_logger().info(f'📏 Min Distance: {self.min_dist:.2f} m')

    def laser_callback(self, msg):
        if len(msg.ranges) == 0:
            return
            
        ranges = msg.ranges
        total = len(ranges)
        self.scan_received = True
        
        # Define sectors based on laser configuration
        if total > 360:
            # Full 360-degree scan
            front = ranges[350:360] + ranges[0:10]
            left = ranges[20:90]
            right = ranges[270:340]
        else:
            # Smaller scan range
            idx = max(1, total // 12)
            front = ranges[0:idx] + ranges[-idx:]
            left = ranges[idx:total//3]
            right = ranges[2*total//3:-idx]
        
        # Check clearance (ignore readings too close which might be noise)
        def is_clear(sector):
            valid_ranges = [r for r in sector if 0.1 < r < 10.0]
            if not valid_ranges:
                return True
            return all(r > self.min_dist for r in valid_ranges)
        
        self.front_clear = is_clear(front)
        self.left_clear = is_clear(left)
        self.right_clear = is_clear(right)

    def move_robot(self):
        if not self.scan_received:
            return  # Wait for laser data
            
        cmd = Twist()
        
        if self.front_clear:
            cmd.linear.x = self.speed
            # Add slight variation to make movement more natural
            cmd.linear.x += random.uniform(-0.02, 0.02)
        else:
            # Choose turn direction based on clearance
            if self.right_clear and self.left_clear:
                cmd.angular.z = random.choice([-self.turn, self.turn])
            elif self.right_clear:
                cmd.angular.z = -self.turn  # Turn right
            elif self.left_clear:
                cmd.angular.z = self.turn   # Turn left
            else:
                cmd.angular.z = self.turn   # Default turn left
            self.get_logger().info(f'🚧 Obstacle detected! Turning {"right" if cmd.angular.z < 0 else "left"}')
        
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Stopping robot...')
    finally:
        try:
            # Send stop command
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()