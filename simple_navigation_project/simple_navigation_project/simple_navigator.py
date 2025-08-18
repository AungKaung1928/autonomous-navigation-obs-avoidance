#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Goal publisher for visualization
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Timer to send random goals
        self.create_timer(10.0, self.send_random_goal)
        
        self.get_logger().info('Simple Navigator started!')

    def send_random_goal(self):
        """Send a random navigation goal"""
        try:
            # Generate random goal
            goal_x = random.uniform(-2.0, 2.0)
            goal_y = random.uniform(-2.0, 2.0)
            goal_yaw = random.uniform(-math.pi, math.pi)
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
            
            # Publish goal
            self.goal_pub.publish(goal_pose)
            
            self.get_logger().info(f'Published goal: x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.2f}')
            
        except Exception as ex:
            self.get_logger().error(f'Error sending goal: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()