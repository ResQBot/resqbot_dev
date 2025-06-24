import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/paul/resqbot_dev/control_ws/install/lotti_teleop'
