"""Launch test: lifecycle deactivate stops the robot. The installed node drives forward on an open
synthetic scan; the test then calls /patrol_controller/change_state (deactivate) while scans keep
coming: the last /cmd_vel must be zero, and at most that one stop may arrive after the call. The installed node with the shipped params drives forward on
an open synthetic scan, then the scans stop; within about 1 s every /cmd_vel must be zero.
Run: launch_test test/test_launch_stale_scan.py (after sourcing the workspace).
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import LaserScan

PKG = 'simple_navigation_project'
EXE = 'patrol_controller'
PARAMS = os.path.join(get_package_share_directory(PKG), 'config', 'patrol_params.yaml')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package=PKG, executable=EXE, name='patrol_controller',
                                   output='screen', parameters=[PARAMS, {'use_sim_time': False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


def open_scan():
    scan = LaserScan()
    scan.angle_min, scan.angle_increment = 0.0, 2.0 * math.pi / 360
    scan.angle_max = scan.angle_increment * 359
    scan.range_min, scan.range_max = 0.12, 3.5
    scan.ranges = [3.0] * 360
    return scan


class TestStaleScan(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_deactivate_stops_the_robot(self):
        probe = rclpy.create_node('deactivate_probe')
        cmds = []
        probe.create_subscription(Twist, '/cmd_vel', cmds.append, 10)
        pub = probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        cli = probe.create_client(ChangeState, '/patrol_controller/change_state')
        scan = open_scan()
        try:
            start = time.monotonic()
            while time.monotonic() - start < 20.0 and not (cmds and cmds[-1].linear.x > 0.05):
                pub.publish(scan)
                rclpy.spin_once(probe, timeout_sec=0.05)
            self.assertTrue(cmds and cmds[-1].linear.x > 0.05, 'never drove forward on an open scan')
            self.assertTrue(cli.wait_for_service(timeout_sec=5.0))
            fut = cli.call_async(ChangeState.Request(transition=Transition(id=Transition.TRANSITION_DEACTIVATE)))
            while not fut.done():
                pub.publish(scan)
                rclpy.spin_once(probe, timeout_sec=0.05)
            self.assertTrue(fut.result().success)
            n = len(cmds)
            end = time.monotonic() + 1.5  # scans keep coming; an inactive node must stay silent
            while time.monotonic() < end:
                pub.publish(scan)
                rclpy.spin_once(probe, timeout_sec=0.05)
        finally:
            probe.destroy_node()
        last = cmds[-1]
        self.assertEqual((last.linear.x, last.angular.z), (0.0, 0.0), 'last command before going inactive is not a stop')
        after = cmds[n:]  # the stop itself may arrive after the service reply
        self.assertTrue(all(c.linear.x == 0.0 and c.angular.z == 0.0 for c in after),
                        f'non-zero command after deactivate: {[(c.linear.x, c.angular.z) for c in after]}')
        self.assertLessEqual(len(after), 1, f'{len(after)} commands after deactivate')
