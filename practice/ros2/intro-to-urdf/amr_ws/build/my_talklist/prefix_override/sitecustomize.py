import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/phant0mz3ro/Documents/aurora/practice/ros2/intro-to-urdf/amr_ws/install/my_talklist'
