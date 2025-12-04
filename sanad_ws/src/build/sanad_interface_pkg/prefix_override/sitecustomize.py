import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zero/sanad/ROS/sanad_ws/src/install/sanad_interface_pkg'
