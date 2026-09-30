import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/huyen/drone_project/ros2_ws/install/precision_landing'
