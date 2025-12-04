#!/usr/bin/env python3
"""
Simple autonomous patrol with obstacle avoidance.
Using Lifecycle Node for production deployment.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class PatrolController(LifecycleNode):
    
    def __init__(self):
        super().__init__('patrol_controller')
        
        # Parameters
        self.declare_parameter('patrol_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('safe_distance', 0.8)
        self.declare_parameter('min_lane_width', 0.8)
        
        self.patrol_speed = self.get_parameter('patrol_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.min_lane_width = self.get_parameter('min_lane_width').value
        
        # Sensor data
        self.front_clear = True
        self.left_clear = True
        self.right_clear = True
        self.left_lane_width = 0.0
        self.right_lane_width = 0.0
        self.latest_scan = None
        
        # Will be initialized in configure
        self.cmd_vel_pub = None
        self.laser_sub = None
        self.timer = None
        
        self.get_logger().info('🤖 Patrol Controller Created')
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle state"""
        self.get_logger().info('Configuring...')
        
        try:
            # Create publishers and subscribers
            self.cmd_vel_pub = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self._laser_callback, 10
            )
            
            self.get_logger().info('✅ Configuration complete')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'❌ Configuration failed: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle state"""
        self.get_logger().info('Activating...')
        
        try:
            # Activate publisher
            self.cmd_vel_pub.on_activate(state)
            
            # Start control timer
            self.timer = self.create_timer(0.1, self._move_robot)
            
            self.get_logger().info('✅ Patrol controller activated - Starting patrol')
            self.get_logger().info(f'Safe distance: {self.safe_distance}m')
            self.get_logger().info(f'Min lane width: {self.min_lane_width}m')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'❌ Activation failed: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle state"""
        self.get_logger().info('Deactivating...')
        
        try:
            # Stop robot
            self._send_stop_command()
            
            # Stop timer
            if self.timer:
                self.timer.cancel()
                self.timer = None
            
            # Deactivate publisher
            self.cmd_vel_pub.on_deactivate(state)
            
            self.get_logger().info('✅ Deactivated')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'❌ Deactivation failed: {e}')
            return TransitionCallbackReturn.ERROR
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle state"""
        self.get_logger().info('Cleaning up...')
        
        try:
            # Destroy publishers and subscriptions
            if self.cmd_vel_pub:
                self.destroy_lifecycle_publisher(self.cmd_vel_pub)
            if self.laser_sub:
                self.destroy_subscription(self.laser_sub)
            
            self.get_logger().info('✅ Cleanup complete')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'❌ Cleanup failed: {e}')
            return TransitionCallbackReturn.ERROR
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle state"""
        self.get_logger().info('Shutting down...')
        self._send_stop_command()
        return TransitionCallbackReturn.SUCCESS
    
    def _laser_callback(self, msg):
        """Process laser scan"""
        self.latest_scan = msg
        
        ranges = msg.ranges
        total = len(ranges)
        
        if total == 0:
            return
        
        # Filter valid ranges
        def get_valid_ranges(indices):
            valid = []
            for i in indices:
                if 0 <= i < total:
                    r = ranges[i]
                    if math.isfinite(r) and 0.1 < r < 10.0:
                        valid.append(r)
            return valid
        
        # Front sector: 340-360 and 0-20 degrees (±20 degrees from center)
        front_indices = list(range(340, 360)) + list(range(0, 20))
        front_distances = get_valid_ranges(front_indices)
        
        # Left sector: 60-120 degrees
        left_indices = list(range(60, 120))
        left_distances = get_valid_ranges(left_indices)
        
        # Right sector: 240-300 degrees
        right_indices = list(range(240, 300))
        right_distances = get_valid_ranges(right_indices)
        
        # Wide left for lane width: 45-135 degrees
        left_wide_indices = list(range(45, 135))
        left_wide_distances = get_valid_ranges(left_wide_indices)
        
        # Wide right for lane width: 225-315 degrees
        right_wide_indices = list(range(225, 315))
        right_wide_distances = get_valid_ranges(right_wide_indices)
        
        # Check if paths are clear
        self.front_clear = (
            len(front_distances) == 0 or 
            min(front_distances) > self.safe_distance
        )
        self.left_clear = (
            len(left_distances) == 0 or 
            min(left_distances) > self.safe_distance
        )
        self.right_clear = (
            len(right_distances) == 0 or 
            min(right_distances) > self.safe_distance
        )
        
        # Measure lane widths
        self.left_lane_width = (
            0.0 if len(left_wide_distances) == 0 
            else min(left_wide_distances)
        )
        self.right_lane_width = (
            0.0 if len(right_wide_distances) == 0 
            else min(right_wide_distances)
        )
    
    def _move_robot(self):
        """Move robot - match C++ logic exactly"""
        if self.latest_scan is None:
            return
        
        cmd = Twist()
        
        if self.front_clear:
            # PATH IS CLEAR - MOVE FORWARD
            cmd.linear.x = self.patrol_speed
            cmd.angular.z = 0.0
            self.get_logger().info('Moving forward - path clear', throttle_duration_sec=2.0)
        
        else:
            # OBSTACLE DETECTED - STOP AND TURN
            cmd.linear.x = 0.0
            
            # Check if lanes are safe (clear AND wide enough)
            right_safe = self.right_clear and self.right_lane_width > self.min_lane_width
            left_safe = self.left_clear and self.left_lane_width > self.min_lane_width
            
            if right_safe and left_safe:
                # Both lanes safe - pick wider one
                if self.right_lane_width >= self.left_lane_width:
                    cmd.angular.z = -self.turn_speed
                    self.get_logger().info(
                        f'Turning RIGHT - lane width: {self.right_lane_width:.2f}m',
                        throttle_duration_sec=1.0
                    )
                else:
                    cmd.angular.z = self.turn_speed
                    self.get_logger().info(
                        f'Turning LEFT - lane width: {self.left_lane_width:.2f}m',
                        throttle_duration_sec=1.0
                    )
            
            elif right_safe:
                # Only right is safe
                cmd.angular.z = -self.turn_speed
                self.get_logger().info(
                    f'Turning RIGHT - safe lane: {self.right_lane_width:.2f}m',
                    throttle_duration_sec=1.0
                )
            
            elif left_safe:
                # Only left is safe
                cmd.angular.z = self.turn_speed
                self.get_logger().info(
                    f'Turning LEFT - safe lane: {self.left_lane_width:.2f}m',
                    throttle_duration_sec=1.0
                )
            
            else:
                # No safe lanes - keep rotating to find one
                cmd.angular.z = self.turn_speed
                if self.right_clear or self.left_clear:
                    self.get_logger().info(
                        'Lanes too narrow - searching for wider path',
                        throttle_duration_sec=1.0
                    )
                else:
                    self.get_logger().info(
                        'No clear path - rotating to find opening',
                        throttle_duration_sec=1.0
                    )
        
        self.cmd_vel_pub.publish(cmd)
    
    def _send_stop_command(self):
        """Stop robot safely"""
        try:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    node = PatrolController()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Transition to configured state
        node.trigger_configure()
        # Transition to active state
        node.trigger_activate()
        
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Keyboard interrupt - shutting down')
    except Exception as e:
        node.get_logger().error(f'❌ Fatal error: {e}')
    finally:
        # Proper lifecycle shutdown
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.trigger_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
