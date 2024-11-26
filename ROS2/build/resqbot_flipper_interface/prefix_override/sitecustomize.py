import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/paul/resqbot_dev/ROS2/install/resqbot_flipper_interface'
