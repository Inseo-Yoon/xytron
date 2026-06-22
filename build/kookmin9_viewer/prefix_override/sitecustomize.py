import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wlwnstn2396/xycar_ws/install/kookmin9_viewer'
