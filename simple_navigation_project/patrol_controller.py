#!/usr/bin/env python3
"""
Simple autonomous patrol with obstacle avoidance.
Using Lifecycle Node for production deployment.
"""

import math

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


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
            # BEST_EFFORT matches both the sim bridge and the real LDS driver.
            sensor_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self._laser_callback, sensor_qos
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
    
    def _sector_min(self, msg, lo_deg: float, hi_deg: float) -> float:
        """Minimum valid range over a bearing sector in the robot frame (0 = front, +90 = left).

        Indices are derived from angle_min / angle_increment, so this works for both the
        Classic LDS convention (angle_min = 0) and gpu_lidar (angle_min = -pi).
        """
        n = len(msg.ranges)
        two_pi = 2.0 * math.pi
        start = (math.radians(lo_deg) - msg.angle_min) % two_pi
        count = int(round(math.radians(hi_deg - lo_deg) / msg.angle_increment))
        first = int(round(start / msg.angle_increment))
        best = math.inf
        for k in range(count + 1):
            r = msg.ranges[(first + k) % n]
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                best = min(best, r)
        return best

    def _laser_callback(self, msg):
        """Reduce the scan to front / side clearances and side lane widths."""
        self.latest_scan = msg
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        front = self._sector_min(msg, -20.0, 20.0)
        left = self._sector_min(msg, 60.0, 120.0)
        right = self._sector_min(msg, -120.0, -60.0)
        left_wide = self._sector_min(msg, 45.0, 135.0)
        right_wide = self._sector_min(msg, -135.0, -45.0)

        # inf (no return in the sector) counts as clear / unbounded lane
        self.front_clear = front > self.safe_distance
        self.left_clear = left > self.safe_distance
        self.right_clear = right > self.safe_distance
        self.left_lane_width = left_wide if math.isfinite(left_wide) else msg.range_max
        self.right_lane_width = right_wide if math.isfinite(right_wide) else msg.range_max

    def _move_robot(self):
        """10 Hz reactive step: forward if clear, else stop and turn toward the wider safe lane."""
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
        if self.cmd_vel_pub is not None and self.cmd_vel_pub.is_activated and rclpy.ok():
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    # Keep the context alive through SIGINT so the lifecycle deactivate/stop below can
    # still publish; rclpy's own handler would invalidate it first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    
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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
