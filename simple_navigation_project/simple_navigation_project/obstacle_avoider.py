#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import random

class SimpleObstacleAvoider(Node):
    def __init__(self):
        super().__init__('simple_obstacle_avoider')
        
        # Declare parameters
        self.declare_parameter('normal_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('min_distance', 0.5)
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timers
        self.create_timer(0.1, self.move_robot)
        self.create_timer(5.0, self.publish_goal)
        
        # Get parameters
        self.speed = self.get_parameter('normal_speed').value
        self.turn = self.get_parameter('turn_speed').value
        self.min_dist = self.get_parameter('min_distance').value
        
        # State variables
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        
        self.get_logger().info(f'Started! Speed={self.speed}m/s, Turn={self.turn}rad/s')

    def laser_callback(self, msg):
        ranges = msg.ranges
        total = len(ranges)
        
        # Define sectors based on laser configuration
        if total > 360:
            front = ranges[350:360] + ranges[0:10]
            left = ranges[20:90]
            right = ranges[270:340]
        else:
            idx = total // 12
            front = ranges[0:idx] + ranges[-idx:]
            left = ranges[idx:total//3]
            right = ranges[2*total//3:-idx]
        
        # Check clearance
        self.front_clear = all(r > self.min_dist or r < 0.1 for r in front)
        self.left_clear = all(r > self.min_dist or r < 0.1 for r in left)
        self.right_clear = all(r > self.min_dist or r < 0.1 for r in right)

    def move_robot(self):
        cmd = Twist()
        
        if self.front_clear:
            cmd.linear.x = self.speed
            self.get_logger().debug('Moving forward')
        else:
            if self.right_clear and self.left_clear:
                cmd.angular.z = random.choice([-self.turn, self.turn])
            elif self.right_clear:
                cmd.angular.z = -self.turn
            elif self.left_clear:
                cmd.angular.z = self.turn
            else:
                cmd.angular.z = self.turn
            self.get_logger().info(f'Obstacle! Turning {"right" if cmd.angular.z < 0 else "left"}')
        
        self.cmd_vel_pub.publish(cmd)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 2.0 if self.front_clear else 1.0
        goal.pose.position.y = random.uniform(-0.5, 0.5)
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Goal: x={goal.pose.position.x:.1f}, y={goal.pose.position.y:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())  # Stop robot
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()