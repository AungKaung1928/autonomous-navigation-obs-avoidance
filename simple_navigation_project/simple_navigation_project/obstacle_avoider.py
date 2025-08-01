#!/usr/bin/env python3
"""
Simple Navigation with Nav2: Sends sequential goals for continuous movement.
Obstacle avoidance is handled by Nav2 (costmaps, local planner).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class Nav2ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('nav2_obstacle_avoider')
        
        # Create the navigator
        self.navigator = BasicNavigator()
        
        # Wait for Nav2 to activate
        self.get_logger().info("Waiting for Nav2 to activate...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active!")
        
        # Define waypoints
        self.waypoints = self.create_square_waypoints()
        self.current_goal_index = 0
        
        # Timer for non-blocking goal sending
        self.timer = self.create_timer(0.5, self.check_navigation_status)
        
        # Send first goal
        self.send_current_goal()

    def create_square_waypoints(self):
        """Create a simple square path around (0,0)"""
        waypoints = []
        side_length = 1.5  # Reduced size for better compatibility
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

    def send_current_goal(self):
        """Send the current goal from waypoints list"""
        if not self.waypoints:
            return
            
        goal = self.waypoints[self.current_goal_index]
        self.get_logger().info(f"Going to waypoint {self.current_goal_index}: "
                              f"({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
        
        self.navigator.goToPose(goal)

    def check_navigation_status(self):
        """Non-blocking check of navigation status"""
        if not self.navigator.isTaskComplete():
            return  # Still navigating
            
        # Task completed - check result
        result = self.navigator.getResult()
        
        if result == 3:  # TaskResult.SUCCEEDED
            self.get_logger().info(f"Reached waypoint {self.current_goal_index}!")
        elif result == 4:  # TaskResult.CANCELED  
            self.get_logger().warn("Navigation was canceled")
        elif result == 5:  # TaskResult.ABORTED
            self.get_logger().warn("Navigation aborted - trying next waypoint")
        else:
            self.get_logger().warn(f"Navigation failed with result: {result}")
        
        # Move to next waypoint
        self.current_goal_index = (self.current_goal_index + 1) % len(self.waypoints)
        
        # Send next goal after a brief pause
        self.create_timer(1.0, self.send_current_goal)  # One-shot timer

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Nav2ObstacleAvoider()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
