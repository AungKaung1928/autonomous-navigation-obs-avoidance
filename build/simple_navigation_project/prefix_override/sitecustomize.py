import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/allkg/demo_robotics/src/install/simple_navigation_project'
