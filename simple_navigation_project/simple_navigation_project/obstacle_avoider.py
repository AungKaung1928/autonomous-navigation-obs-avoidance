#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    """Create a PoseStamped message for navigation goals"""
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

def main():
    # Initialize ROS2
    rclpy.init()
    nav = BasicNavigator()
    
    print("Setting initial pose...")
    # Set initial pose (robot starting position)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    
    print("Waiting for Nav2...")
    # Wait for Navigation 2 to become active
    nav.waitUntilNav2Active()
    
    print("Creating waypoints...")
    # Create simple waypoints for obstacle avoidance navigation
    waypoints = []
    waypoints.append(create_pose_stamped(nav, 2.0, 0.0, 0.0))      # Move forward
    waypoints.append(create_pose_stamped(nav, 2.0, 1.0, 1.57))     # Turn left
    waypoints.append(create_pose_stamped(nav, 0.0, 1.0, 3.14))     # Move back
    waypoints.append(create_pose_stamped(nav, 0.0, -1.0, -1.57))   # Turn right
    waypoints.append(create_pose_stamped(nav, 1.0, -1.0, 0.0))     # Move forward
    
    print("Starting waypoint navigation...")
    # Follow waypoints - Nav2 will handle obstacle avoidance
    nav.followWaypoints(waypoints)
    
    # Monitor navigation progress
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"Navigation feedback: {feedback.current_waypoint}/{len(waypoints)}")
    
    # Get final result
    result = nav.getResult()
    if result:
        print(f"Navigation completed with result: {result}")
    else:
        print("Navigation failed!")
    
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
