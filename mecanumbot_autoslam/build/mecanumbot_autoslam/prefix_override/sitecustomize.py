import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/csenge/Documents/mecanumbot_ws/src/mecanumbot_behaviours/mecanumbot_autoslam/install/mecanumbot_autoslam'
