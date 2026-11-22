"""Launch test: the installed lifecycle node runs as its own process with the shipped parameters.

The test process publishes synthetic TurtleBot3 LDS scans on /scan and reads /cmd_vel:
open space -> drive forward; obstacle ahead with the left side blocked -> turn right;
obstacle ahead with the right side blocked -> turn left. After shutdown the process must exit cleanly.
Run: launch_test test/test_launch_scenes.py (after sourcing the workspace).
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PARAMS = os.path.join(
    get_package_share_directory('simple_navigation_project'), 'config', 'patrol_params.yaml')


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='simple_navigation_project', executable='patrol_controller',
        name='patrol_controller', output='screen',
        # The shipped file sets use_sim_time for Gazebo; there is no /clock here.
        parameters=[PARAMS, {'use_sim_time': False}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {'node': node}


def make_scan(range_at_bearing):
    """360 beams, angle_min 0 (LDS convention); bearing in degrees, left positive."""
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 2.0 * math.pi / 360
    scan.angle_max = scan.angle_increment * 359
    scan.range_min = 0.12
    scan.range_max = 3.5
    scan.ranges = [float(range_at_bearing(i if i <= 180 else i - 360)) for i in range(360)]
    return scan


def blocked(*sectors):
    """0.5 m returns inside the given (lo, hi) bearing sectors, 3 m elsewhere."""
    return make_scan(lambda b: 0.5 if any(lo <= b <= hi for lo, hi in sectors) else 3.0)


OPEN = make_scan(lambda b: 3.0)
AHEAD_AND_LEFT = blocked((-20, 20), (45, 135))
AHEAD_AND_RIGHT = blocked((-20, 20), (-135, -45))


class TestScenes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.probe = rclpy.create_node('launch_scene_probe')
        self.cmds = []
        self.probe.create_subscription(Twist, '/cmd_vel', self.cmds.append, 10)
        self.pub = self.probe.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

    def tearDown(self):
        self.probe.destroy_node()

    def drive(self, scan, expect, timeout=10.0):
        """Publish scan until the last 3 /cmd_vel messages all satisfy expect(cmd)."""
        self.cmds.clear()
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self.pub.publish(scan)
            rclpy.spin_once(self.probe, timeout_sec=0.05)
            if len(self.cmds) >= 3 and all(expect(c) for c in self.cmds[-3:]):
                return
        last = [(round(c.linear.x, 2), round(c.angular.z, 2)) for c in self.cmds[-3:]]
        self.fail(f'expected command not seen within {timeout} s; last (v, w): {last}')

    def test_scene_sequence(self):
        # Shipped params: patrol_speed 0.2 m/s, turn_speed 0.5 rad/s.
        seen = []
        for name, scan, expect, timeout in [
                ('open', OPEN, lambda c: c.linear.x == 0.2 and c.angular.z == 0.0, 20.0),
                ('ahead + left', AHEAD_AND_LEFT, lambda c: c.linear.x == 0.0 and c.angular.z == -0.5, 10.0),
                ('ahead + right', AHEAD_AND_RIGHT, lambda c: c.linear.x == 0.0 and c.angular.z == 0.5, 10.0)]:
            self.drive(scan, expect, timeout=timeout)
            seen.append((name, scan, self.cmds[-1]))
        out = os.environ.get('SCENE_CSV')  # optional, for scripts/plot_scenes.py
        if out:
            with open(out, 'w') as f:
                f.write('scene,v,w,ranges\n')
                for name, scan, c in seen:
                    f.write(f'{name},{c.linear.x:.3f},{c.angular.z:.3f},{" ".join(f"{r:.2f}" for r in scan.ranges)}\n')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, node):
        launch_testing.asserts.assertExitCodes(proc_info, process=node)
