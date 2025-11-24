import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david/AprilTag_ROS2_intel-D435/install/apriltag_detector'
